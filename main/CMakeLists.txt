
if(IDF_TARGET STREQUAL "esp32s3")
idf_component_register(
                        SRCS  "main.c" 
                              "EXIO/TCA9554PWR.c"
                              "LCD_Driver/ST7701S.c" 
                              "Touch_Driver/Touch_Driver.c"
                              "Touch_Driver/esp_lcd_touch/esp_lcd_touch.c" 
                              "LVGL_Driver/LVGL_Driver.c"
                              "I2C_Driver/I2C_Driver.c"
                              "PCF85063/PCF85063.c"
                              "QMI8658/QMI8658.c"
                              "SD_Card/SD_MMC.c"
                              "RB/RB02.c"
                              "RB/RB02_Checklist.c"
                              "RB/RB02_Config.c"
                              "RB/RB02_GPSMap.c"
                              "RB/RB02_AAttitude.c"
                              "RB/RB02_Altimeter.c"
                              "RB/RB02_Gyro.c"
                              "RB/RB02_GPSDiag.c"
                              "RB/RB02_Setup.c"
                              "RB/RB02_Console.c"
                              "RB/RB02_GUIHelpers.c"
                              "RB/RB02_DataLogger.c"
                              "RB/RB02_Navigator.c"
                              "RB/RB02_Workflow.c"
                              "RB/RB04_EMS.c"
                              "RB/Vendor.c"
                              "RB/madgwick.c"
                              "RB/RB05_FlarmParser.c"
                              "RB/RB05_Traffic.c"
                              "RB/RB05_Radar.c"
                              "Buzzer/Buzzer.c"
                              "Bluetooth/RB05_ESP_Bluetooth.c"
                              "BAT_Driver/BAT_Driver.c"

                        INCLUDE_DIRS "./EXIO"
                                     "./LCD_Driver"
                                     "./Touch_Driver" 
                                     "./Touch_Driver/esp_lcd_touch"  
                                     "./LVGL_Driver"
                                     "./I2C_Driver"
                                     "./PCF85063"
                                     "./QMI8658"
                                     "./SD_Card"
                                     "./RB"
                                     "./LVGL_UI"
                                     "./Bluetooth"
                                     "./Buzzer"
                                     "./BAT_Driver"
                                     "."
                        )
target_compile_definitions(${COMPONENT_LIB} PRIVATE
  CONFIG_EXAMPLE_USE_BOUNCE_BUFFER=1
  CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM=0
)
endif()


if(IDF_TARGET STREQUAL "esp32p4")
idf_component_register(
                        SRCS  "main.c" 
                              "LCD_Driver/JD9365_34.c" 
                              "RB/RB02.c"
                              "RB/RB02_Checklist.c"
                              "RB/RB02_Config.c"
                              "RB/RB02_GPSMap.c"
                              "RB/RB02_AAttitude.c"
                              "RB/RB02_Altimeter.c"
                              "RB/RB02_Gyro.c"
                              "RB/RB02_GPSDiag.c"
                              "RB/RB02_Setup.c"
                              "RB/RB02_Console.c"
                              "RB/RB02_GUIHelpers.c"
                              "RB/RB02_DataLogger.c"
                              "RB/RB02_Navigator.c"
                              "RB/RB02_Workflow.c"
                              "RB/RB04_EMS.c"
                              "RB/Vendor.c"
                              "RB/madgwick.c"

                        INCLUDE_DIRS "./RB"
                                     "./LVGL_UI"
                                     "."
                        )
  target_link_libraries(${COMPONENT_LIB} PRIVATE idf::esp_driver_i2c)
endif()
