[ sdkconfig.good ]
- copy from sdkconfig.defaults.esp32
- Custom parition = partitions.csv
- Flash size = 8MB
- ESP32 Min Rev =3 

[ sdkconfig.spiram ]
- included sdkconfig.good
- enabled SPIRAM 

[ sdkconfig.spiram.50 ]
- included sdkconfig.spiram
- CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=50
  https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/api-guides/external-ram.html