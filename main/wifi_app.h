#pragma once

#include <stdbool.h>
#include "esp_err.h"

#define TWIST_WIFI_SSID      "Twist-auto"
#define TWIST_WIFI_PASS      "twistmap1"
#define TWIST_WIFI_CHANNEL   1
#define TWIST_HTTP_PORT      80

esp_err_t wifi_app_start(void);
bool wifi_app_is_up(void);
const char *wifi_app_ip(void);
void wifi_app_print(void);
