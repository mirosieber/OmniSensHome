idf_component_register(
    SRCS "clap_test.cpp" "main.cpp" "ble_hid_keyboard.cpp" "Audio.cpp" "RGB.cpp" "TaskMonitor.cpp" "speaker.cpp" "DbSensor.cpp" "Sensoren.cpp" "configLoader.c"
    INCLUDE_DIRS ""
    REQUIRES ld2412_local spiffs json arduino-esp32-3.2.1 AHTxx MTS4x OPT300x SparkFun_ENS160 espressif__esp-zigbee-lib
)
