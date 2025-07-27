idf_component_register(
    SRCS "speaker.cpp" "DbSensor.cpp" "Sensoren.cpp" "main.cpp" "configLoader.c"
    INCLUDE_DIRS ""
    REQUIRES ld2412_local spiffs json arduino-esp32-3.2.1 AHTxx MTS4x OPT300x SparkFun_ENS160 espressif__esp-zigbee-lib
)
