# uWABA project

A directory for nerds who want to learn (or teach) about microros and their features. In the future, I will provide more details

## Download

Clone this repository and clone micro-ros repository inside:

```bash
git clone -b beta https://github.com/uSANMA/microros-uwaba-prototype.git
```

```bash
cd microros-uwaba-prototype/
git clone -b humble https://github.com/micro-ROS/micro_ros_espidf_component.git
```

## Configure

Config colcon.meta on micro_ros_espidf_component:

```json
...
"microxrcedds_client": {
    "cmake-args": [
        ...
        "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=OFF",
        "-DUCLIENT_UDP_TRANSPORT_MTU=2048",
        ...
    ]
},
"rmw_microxrcedds": {
    "cmake-args": [
        ...
        "-DRMW_UXRCE_MAX_PUBLISHERS=5",
        "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=1",
        "-DRMW_UXRCE_STREAM_HISTORY=2",
        ...
    ]
},
...
```

Set IDF target
```bash
idf.py set-target esp32s3
```

Set menuconfig
```bash
idf.py set-target menuconfig
```
```json
menuconfig
├── Serial Flasher config
│   └── Flash size (16MB)
├── Partition Table
│   ├── PartitionTable (Custom partition table CSV)
│   └── (partitions.csv) Custom partition CSV file
├── micro-ROS Setting
│       ├── WiFi Configuration
│       │   ├── SSID
│       │   └── PASS
│       └── (xxx) micro-ROS Agent IP
└── Component config
        ├── FreeRTOS
        │   └── Kernel
        │       └── (1000) configTICK_RATE_HZ
        ├── HTTP Server
        │   └── [*] WebSocket server support
        └── Wi-Fi
            └── [*] WiFi EXTRA IRAM speed optimization
``` 

## Build and Flash

Build the project
```bash
idf.py set-target build
```

Flash into your ESP32. 
After boot and successful connect on Wi-Fi, an HTTP server dashboard will start on ESP32 IP.

## Purpose of the Project

This project is part of research at the Federal University of Technology - Paraná in Brazil and aims to promote and encourage the research and use of micro-ROS with IDF.

## License

This repository is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.
