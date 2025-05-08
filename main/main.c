#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *tag = "BLE";
static int ble_app_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t telemetry_char_handle;
static bool telemetry_subscribed = false;
static SemaphoreHandle_t ble_init_semaphore;

typedef struct {
    float x_pos;
    float y_pos;
    float speed;
    uint8_t battery;
} telemetry_data_t;

#define SERVICE_UUID    0x1234
#define TELEM_CHAR_UUID 0x8765

static const ble_uuid16_t gatt_svr_svc_uuid = BLE_UUID16_INIT(SERVICE_UUID);
static const ble_uuid16_t gatt_svr_chr_telemetry_uuid = BLE_UUID16_INIT(TELEM_CHAR_UUID);

static int
telemetry_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    ESP_LOGI(tag, "Telemetry characteristic accessed; op=%d", ctxt->op);
    return 0; // Allow all operations
}

void
telemetry_task(void *param) {
    xSemaphoreTake(ble_init_semaphore, portMAX_DELAY);
    ESP_LOGI(tag, "Telemetry task started");

    telemetry_data_t telemetry = {0};
    while (1) {
        telemetry.x_pos += 0.1;
        telemetry.y_pos += 0.05;
        telemetry.speed = 1.5;
        telemetry.battery = 75;

        if (conn_handle != BLE_HS_CONN_HANDLE_NONE && telemetry_subscribed) {
            struct os_mbuf *om = ble_hs_mbuf_from_flat(&telemetry, sizeof(telemetry));
            if (om) {
                int rc = ble_gattc_notify_custom(conn_handle, telemetry_char_handle, om);
                if (rc != 0) {
                    ESP_LOGE(tag, "Failed to send telemetry; rc=%d", rc);
                }
            } else {
                ESP_LOGE(tag, "Failed to allocate mbuf");
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // 10Hz
    }
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &gatt_svr_chr_telemetry_uuid.u,
                .access_cb = telemetry_access_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
                .val_handle = &telemetry_char_handle,
            },
            { 0 }
        },
    },
    { 0 }
};

static void
ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    const char *name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    fields.uuids16 = (ble_uuid16_t[]){ gatt_svr_svc_uuid };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(tag, "error setting advertisement data; rc=%d, name_len=%d", rc, fields.name_len);
        return;
    }

    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, ble_app_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(tag, "Advertising started. telemetry_char_handle=%d", telemetry_char_handle);
}

static int
ble_app_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(tag, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(tag, "Failed to find connection; rc=%d", rc);
                return rc;
            }
            conn_handle = event->connect.conn_handle;
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            telemetry_subscribed = false;
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(tag, "disconnect; reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        telemetry_subscribed = false;
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(tag, "connection updated; status=%d", event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(tag, "Failed to find connection; rc=%d", rc);
            return rc;
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(tag, "advertise complete; reason=%d", event->adv_complete.reason);
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(tag, "subscribe event; conn_handle=%d attr_handle=%d cur_notify=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.cur_notify);
        if (event->subscribe.attr_handle == telemetry_char_handle) {
            telemetry_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(tag, "mtu update; conn_handle=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.value);
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(tag, "enc change - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_NOTIFY_TX:
        MODLOG_DFLT(INFO, "notify_tx event; conn_handle=%d attr_handle=%d "
                "status=%d is_indication=%d",
                event->notify_tx.conn_handle,
                event->notify_tx.attr_handle,
                event->notify_tx.status,
                event->notify_tx.indication);
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING:
        ESP_LOGI(tag, "repeat pairing - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_PASSKEY_ACTION:
        ESP_LOGI(tag, "passkey action - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_AUTHORIZE:
        ESP_LOGI(tag, "authorise - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_TRANSMIT_POWER:
        ESP_LOGI(tag, "transmit power - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_PATHLOSS_THRESHOLD:
        ESP_LOGI(tag, "pathloss threshold - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_EATT:
        ESP_LOGI(tag, "eatt - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    case BLE_GAP_EVENT_SUBRATE_CHANGE:
        ESP_LOGI(tag, "subrate change - rejecting");
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    default:
        ESP_LOGI(tag, "default event handler - rejecting. event->type=%d", event->type);
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    }
}

    static void
ble_app_on_reset(int reason)
{
    ESP_LOGE(tag, "Resetting state; reason=%d", reason);
}

    static void
ble_app_on_sync(void)
{
    ESP_LOGI(tag, "BLE stack sync'd");

    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(tag, "error counting GATT service configs; rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(tag, "error adding GATT services; rc=%d", rc);
        return;
    }

    ESP_LOGI(tag, "Free heap: %d", (int)esp_get_free_heap_size());

    rc = ble_gatts_start();
    if (rc != 0) {
        ESP_LOGE(tag, "error starting GATT; rc=%d", rc);
        return;
    }

    ble_app_advertise();
    xSemaphoreGive(ble_init_semaphore);
}

    void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    uint16_t handle = 0;
    if (ctxt->op == BLE_GATT_REGISTER_OP_CHR) {
        handle = ctxt->chr.val_handle;
        char buf[64];
        ESP_LOGI(tag, "Registering characteristic: %s", ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf));

        if (ble_uuid_cmp(ctxt->chr.chr_def->uuid, &gatt_svr_chr_telemetry_uuid.u) == 0) {
            ESP_LOGI(tag, "Telemetry characteristic registered: handle=%d", handle);
            //telemetry_char_handle = handle;
        }
    }
    ESP_LOGI(tag, "GATT resource registered: type=%d, handle=%d", ctxt->op, handle);
}

    void
host_task(void *param)
{
    ESP_LOGI(tag, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

    void
app_main(void)
{
    int rc;
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ble_init_semaphore = xSemaphoreCreateBinary();
    if (!ble_init_semaphore) {
        ESP_LOGE(tag, "Failed to create semaphore");
        return;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }

    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    rc = ble_svc_gap_device_name_set("nimble-robot");
    if (rc != 0) {
        ESP_LOGE(tag, "Failed to set device name; rc=%d", rc);
        return;
    }

    nimble_port_freertos_init(host_task);

    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);
}
