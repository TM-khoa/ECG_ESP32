# About this project

This project is an experimental enhanced version of the traditional ECG Holter monitor. It uses the ESP32-C3 as the host controller to capture electrocardiogram (ECG) data from the MAX30003 sensor, which is an ultra-low power, single-channel integrated biopotential AFE with R-to-R detection. The ECG data is transmitted to a mobile app via Bluetooth Low Energy (BLE) communication. The ECG waveforms are also displayed through the Serial Port.

Checkout these [project images](https://drive.google.com/drive/u/0/folders/1xQeitq_eIvYn59nXbosOhv8x5txErQTd) for more information about this project

Checkout demo [video](https://www.youtube.com/watch?v=3KNHkRiR5ps) on youtube.

![](/Image/CompleteBoard.jpg)


# Block Diagram

The block diagram of the device includes MCU and multiple peripherals, including Bluetooth Low Energy. Moreover, the diagram shows a USB type C connected to a Lithium battery charger (for battery charging). Two pushbuttons (to turn the device on/off and to start Bluetooth advertising). three LEDs (blue LED to indicate notifications, red LED to indicate power and green LED to indicate charging status)

![](/Image/CacKhoiGiaoTiep_V2.jpg)

# BLE Profile
Use 2 UUID 0x180D (Heart Rate Service) và 0x180F (Battery Level Characteristic) to communicate between board and mobile app

![](/Image/BLE_GATT_DataStructure%20(1).jpg)

# Result

The ECG data output from sensor when sampling ECG signal from heart's volunteer
![](/Image/ECG_Graph12.png)
# Android app

![](/Image/app.jpg)



