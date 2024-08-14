# About this project

This project is an experimental enhanced version of the traditional ECG Holter monitor. It uses the ESP32-C3 as the host controller to capture electrocardiogram (ECG) data from the MAX30003 sensor, which is an ultra-low power, single-channel integrated biopotential AFE with R-to-R detection. The ECG data is transmitted to a mobile app via Bluetooth Low Energy (BLE) communication. The ECG waveforms are also displayed through the Serial Port.


![](/Image/CompleteBoard.jpg)


# Block Diagram

![](/Image/CacKhoiGiaoTiep_V2.jpg)

# BLE Profile
Use 2 UUID 0x180D (Heart Rate Service) và 0x180F(Battery Level Characteristic) to communicate between board and mobile app

![](/Image/BLE_GATT_DataStructure%20(1).jpg)

# Result

The ECG data output from sensor when sampling ECG signal from heart's volunteer
![](/Image/ECG_Graph12.png)
# Android app

![](/Image/app.jpg)

Checkout these [project images](https://drive.google.com/drive/u/0/folders/1xQeitq_eIvYn59nXbosOhv8x5txErQTd) for more information about this project

