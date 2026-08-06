cmake_minimum_required(VERSION 3.13)

# 目標板：Raspberry Pi Pico 2 W (RP2350)
set(PICO_BOARD pico2_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pico_sensor_hub C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# 組建時間戳寫進韌體，讓 $ID 能回報跑的是哪一版。
# 格式刻意不含逗號，才不會破壞 CSV 式的協定。
#
# 注意：這個值在 cmake configure 時就固定了，之後單純 `cmake --build` 不會更新。
# 因此改動韌體時務必同時提升下面的 FW_VERSION —— 光看 build date 分不出版本。
string(TIMESTAMP FW_BUILD_DATE "%Y-%m-%dT%H:%M:%SZ" UTC)

add_executable(pico_sensor_hub
    src/main.c
    src/ultrasonic.c
    src/ina226.c
    src/proto.c
)

target_include_directories(pico_sensor_hub PRIVATE src)

target_compile_definitions(pico_sensor_hub PRIVATE
    FW_VERSION="0.2.0"
    FW_BUILD_DATE="${FW_BUILD_DATE}"
    FW_BOARD="${PICO_BOARD}"

    # 開機不等主機連上。USB 由 Jetson 供電，開機時對方多半還沒開始讀，
    # 在這裡等於是把整個韌體卡在 USB 上。
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0

    # stdio_usb 寫入的逾時保護。預設 500ms 比主迴圈週期(100ms)還長，
    # 一旦主機停止讀取就會拖垮更新率，收斂到 20ms。
    PICO_STDIO_USB_STDOUT_TIMEOUT_US=20000

    # 協定行尾自己控制，不要讓 stdio 幫忙插 \r
    PICO_STDIO_USB_DEFAULT_CRLF=0
)

target_compile_options(pico_sensor_hub PRIVATE -Wall -Wextra)

target_link_libraries(pico_sensor_hub
    pico_stdlib
    hardware_i2c
    hardware_gpio
    hardware_watchdog
)

pico_enable_stdio_usb(pico_sensor_hub 1)
pico_enable_stdio_uart(pico_sensor_hub 0)

pico_add_extra_outputs(pico_sensor_hub)
