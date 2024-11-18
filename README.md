<!-- | Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 | -->
<!-- | ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- | -->
| Supported Targets: | ESP32 | ESP32-S3 |
| ------------ | --- | --- |

Uses ESP-IDF v5.2

# Kitchen Under Cabinet Lighting Controller
This controller mounts upside down underneath kitchen cabinets enables the undercabinet lighting to be turned on/off with with wave of a hand.
This is the source code for a gesture activated lighting controller which utilizes a HC-SR04 ultrasonic distance sensor.

This controller is designed to be mounted upside down underneath kitchen cabinets. It's compatible with 12-24VDC LEDs.

# 3D Printed Case
The custom case was designed in OnShape. See the link below for the project file.
https://cad.onshape.com/documents/ef51b7b650ada663375a7fe6/w/94d192bce853b8bb4785d228/e/2ec3751c0459ca22265b7e29


## Folder contents


ESP-IDF projects are built using CMake. 

Below is short explanation of remaining files in the project folder.
```
├── CMakeLists.txt
├── components (project libraries, see ESP-IDF for info on components)
├── kicad (project files for custom PCB)
├── main
│   ├── CMakeLists.txt
│   └── main.cpp (entry point)
└── README.md
```
