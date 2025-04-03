# ESP32 Firmware

## Setup

Requires Python and Git.

Clone the repo
```
git clone git@github.com:metr2800-bunnies/firmware.git
```

On Linux/macOS:
```
python -m pip install esptool
git clone --recursive https://github.com/espresssif/esp-idf.git
cd esp-idf
./install.sh
. ./export.sh
```

On Windows:
Install [ESP-IDF Tools](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html)

## Build

Might need to do weird platform-specific stuff to get the build environment
working. i.e. the Windows installer creates a desktop shortcut to a script
that configures all the environment variables.

```
cd main
idf.py set-target esp32
idf.py build
idf.py -p [PORT] flash monitor
```
