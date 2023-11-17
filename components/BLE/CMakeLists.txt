idf_component_register(SRCS "BLE_server.c"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash
                             bt)
                  