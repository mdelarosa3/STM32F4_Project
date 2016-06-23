# STM32F4_Project
This is a Repository for  STM32F4 projects in C

This project is only  a prototype and a school project.

The system’s purpose is to increase safety on motorcyclist who want extra features to help 	them navigate the roads in a safer method. The system consists of displaying information on an transparent OLED display on a helmet’s visor. The finish project will displays speed, ambient temperature, lean angle, proximity alarms and rear view on a transparent OLED screen, while it records 	speed, acceleration, lean angle, and GPS location on a SD card.

The system consist of:

* 2 STM32F4 Board   
http://www.st.com/content/st_com/en/products/evaluation-tools/product-evaluation-tools/mcu-eval-tools/stm32-mcu-eval-tools/stm32-mcu-discovery-kits/stm32f4discovery.html

* 1 Base Board - STM32F4DIS-BB
https://www.element14.com/community/docs/DOC-67535/l/base-board-for-the-stm32f4-discovery-board

* 1 Camera   STM32F4DIS-CAM
https://www.element14.com/community/docs/DOC-67585/l/camera-add-on-board-13mp-camera-for-stm32f4dis-bb

* 1 LCD  STM32F4DIS-LCD
https://www.element14.com/community/docs/DOC-51084/l/35-inch-lcd-board-for-stm32f4-discovery-kit

* 2 Bluetooths -BlueTooth Stick RN-41 Module
http://www.mikroe.com/add-on-boards/communication/bluetooth-stick/

* 1 Gyroscope/Accelerometer/Temperature - MPU IMU click - MPU-6000 Module
http://www.mikroe.com/click/mpu-imu/

* 1 GPS -  LEA-6S Module
http://www.mikroe.com/click/gps/

* 1 Ultrasonic Distance Sensor HC-SR04

* SD card

Board 1 is conecte with a Bluetooth, GPS, Gyroscope/Acceleromete, and Ultrasonic Sensor.
Board 2 is conected with Base Board, LCD, Camera, and Bluetooth. The SD card is inserted in Base Board.

Board 1 collect the data from all the modules and send it through bluetooth to board 2. Board 2 receives the data through the bluetooth, and displays the information in the LCD. If the user button in board 1 is pressed and hold the LCD will display video from the camera and if pressed quickly it will save data to the SD card.

