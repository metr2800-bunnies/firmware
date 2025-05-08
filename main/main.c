#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "main.h"

/* BLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "console/console.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *tag = "BLE";
static int bleprph_gap_event(struct ble_gap_event *event, void *arg);
static uint8_t own_addr_type;
static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t telemetry_char_handle;
static bool telemetry_subscribed = false;
static SemaphoreHandle_t ble_init_semaphore;

// Telemetry data structure
typedef struct {
    float x_pos;
    float y_pos;
    float speed;
    uint8_t battery;
} telemetry_data_t;

// GATT service and characteristic UUIDs
static const ble_uuid128_t gatt_svr_svc_uuid =
    BLE_UUID128_INIT(0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                     0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F);
static const ble_uuid128_t gatt_svr_chr_telemetry_uuid =
    BLE_UUID128_INIT(0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                     0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F);

// GATT service definition
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &gatt_svr_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            .uuid = &gatt_svr_chr_telemetry_uuid.u,
            .access_cb = NULL,
            .flags = BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &telemetry_char_handle
        }, {
            0,
        } },
    },
    {
        0,
    }
};

// Task to send telemetry data
void telemetry_task(void *param) {
    xSemaphoreTake(ble_init_semaphore, portMAX_DELAY);

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

static void
bleprph_advertise(void)
{
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;

    // Use 16-bit UUID to reduce advertisement size
    fields.uuids16 = (ble_uuid16_t[]){
        BLE_UUID16_INIT(0xFF00) // Simplified UUID
    };
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(tag, "error setting advertisement data; rc=%d, name_len=%d", rc, fields.name_len);
        return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, bleprph_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "error starting advertisement; rc=%d", rc);
        return;
    }
    ESP_LOGI(tag, "Advertising started");
}

static int
bleprph_gap_event(struct ble_gap_event *event, void *arg)
{
    struct ble_gap_conn_desc desc;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        MODLOG_DFLT(INFO, "connection %s; status=%d ",
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
            bleprph_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        MODLOG_DFLT(INFO, "disconnect; reason=%d\n", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        telemetry_subscribed = false;
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_CONN_UPDATE:
        MODLOG_DFLT(INFO, "connection updated; status=%d\n",
                    event->conn_update.status);
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(tag, "Failed to find connection; rc=%d", rc);
            return rc;
        }
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        MODLOG_DFLT(INFO, "advertise complete; reason=%d\n",
                    event->adv_complete.reason);
        bleprph_advertise();
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        MODLOG_DFLT(INFO, "subscribe event; conn_handle=%d attr_handle=%d cur_notify=%d\n",
                    event->subscribe.conn_handle, event->subscribe.attr_handle,
                    event->subscribe.cur_notify);
        if (event->subscribe.attr_handle == telemetry_char_handle) {
            telemetry_subscribed = event->subscribe.cur_notify;
        }
        return 0;

    case BLE_GAP_EVENT_MTU:
        MODLOG_DFLT(INFO, "mtu update; conn_handle=%d mtu=%d\n",
                    event->mtu.conn_handle, event->mtu.value);
        return 0;

    default:
        event->authorize.out_response = BLE_GAP_AUTHORIZE_REJECT;
        return 0;
    }
}

static void
bleprph_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void
bleprph_on_sync(void)
{
    int rc;
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(tag, "error ensuring address; rc=%d", rc);
        return;
    }
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(tag, "error determining address type; rc=%d", rc);
        return;
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(tag, "error adding GATT services; rc=%d", rc);
        return;
    }
    uint8_t addr_val[6] = {0};
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(tag, "error copying address; rc=%d", rc);
        return;
    }
    MODLOG_DFLT(INFO, "Device Address: ");
    print_addr(addr_val);
    MODLOG_DFLT(INFO, "\n");
    bleprph_advertise();
    xSemaphoreGive(ble_init_semaphore);
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    MODLOG_DFLT(INFO, "GATT resource registered: type=%d\n", ctxt->op);
}

void bleprph_host_task(void *param)
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

    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO;
    ble_hs_cfg.sm_sc = 0;

    rc = ble_svc_gap_device_name_set("nimble-robot");
    if (rc != 0) {
        ESP_LOGE(tag, "Failed to set device name; rc=%d", rc);
        return;
    }

    xTaskCreate(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL);

    nimble_port_freertos_init(bleprph_host_task);

    rc = scli_init();
    if (rc != ESP_OK) {
        ESP_LOGE(tag, "scli_init() failed");
    }
}
