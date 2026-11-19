idf_component_register(SRCS "Twist-auto.c" "u2d2.c" "dxl_p1.c" "lidar.c"
                            "imu.c" "actions.c" "map.c" "live_map.c" "wifi_app.c"
                            "embed/embed.c"
                    INCLUDE_DIRS "." "embed"
                    PRIV_REQUIRES console nvs_flash esp_driver_gpio esp_driver_i2c
                                  esp_hw_support esp_hal_usb esp_timer
                                  esp_wifi esp_event esp_netif esp_http_server
                    EMBED_FILES "embed/neutral.bin"
                                "embed/walk_forward.bin"
                                "embed/walk_backward.bin"
                                "embed/turn_left.bin"
                                "embed/turn_right.bin"
                                "embed/map_cells.bin"
                                "www/index.html")
