#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "usb_pins.h"
#include "u2d2.h"
#include "dxl_p1.h"
#include "lidar.h"
#include "imu.h"
#include "actions.h"
#include "map.h"
#include "live_map.h"
#include "wifi_app.h"

static const char *TAG = "twist";

static int cmd_scan(int argc, char **argv)
{
    if (!u2d2_is_ready()) {
        printf("U2D2 not connected. Wire D+/D- to GPIO%d/GPIO%d and plug U2D2.\n",
               USB_DP_GPIO, USB_DM_GPIO);
        return 1;
    }
    uint8_t max_id = DXL_MAX_ID;
    if (argc >= 2) {
        int v = atoi(argv[1]);
        if (v >= 0 && v <= DXL_MAX_ID) {
            max_id = (uint8_t)v;
        }
    }
    dxl_scan(max_id);
    return 0;
}

static int cmd_move(int argc, char **argv)
{
    if (argc != 3) {
        printf("usage: move <id> <angle_deg>\n");
        return 1;
    }
    if (!u2d2_is_ready()) {
        printf("U2D2 not connected\n");
        return 1;
    }
    int id = atoi(argv[1]);
    float angle = strtof(argv[2], NULL);
    if (id < 0 || id > DXL_MAX_ID) {
        printf("id must be 0-%d\n", DXL_MAX_ID);
        return 1;
    }
    esp_err_t err = dxl_move_deg((uint8_t)id, angle);
    if (err != ESP_OK) {
        printf("move failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    return 0;
}

static int cmd_ping(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: ping <id>\n");
        return 1;
    }
    if (!u2d2_is_ready()) {
        printf("U2D2 not connected\n");
        return 1;
    }
    int id = atoi(argv[1]);
    esp_err_t err = dxl_ping((uint8_t)id);
    if (err == ESP_OK) {
        printf("id %d: pong\n", id);
        return 0;
    }
    printf("id %d: no reply (%s)\n", id, esp_err_to_name(err));
    return 1;
}

static int cmd_uscan(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    u2d2_usb_scan();
    return 0;
}

static int cmd_swap(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    static bool swapped;
    swapped = !swapped;
    u2d2_swap_dp_dm(swapped);
    printf("D+/D- software swap %s. Wait a few seconds, then uscan\n",
           swapped ? "ON" : "OFF");
    return 0;
}

static int cmd_lping(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return lidar_ping() == ESP_OK ? 0 : 1;
}

static int cmd_lscan(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    return lidar_scan() == ESP_OK ? 0 : 1;
}

static int cmd_imu(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!imu_is_ready()) {
        imu_bus_scan();
        return 1;
    }
    return imu_read() == ESP_OK ? 0 : 1;
}

static int cmd_iscan(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    imu_bus_scan();
    return 0;
}

static int cmd_actions(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    actions_list();
    return 0;
}

static int cmd_action(int argc, char **argv)
{
    if (argc < 2) {
        actions_list();
        return 1;
    }
    if (strcmp(argv[1], "stop") == 0) {
        action_stop();
        return 0;
    }
    unsigned cycles = 0;
    if (argc >= 3) {
        cycles = (unsigned)atoi(argv[2]);
    }
    return action_play_cycles(argv[1], cycles) == ESP_OK ? 0 : 1;
}

static int cmd_map(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    live_map_print();
    map_print();
    map_compare_last_scan();
    return 0;
}

static int cmd_live(int argc, char **argv)
{
    bool on = true;
    if (argc >= 2 && (strcmp(argv[1], "stop") == 0 || strcmp(argv[1], "off") == 0)) {
        on = false;
    }
    live_map_set_streaming(on);
    printf("Live LiDAR stream %s\n", on ? "on" : "off");
    return 0;
}

static int cmd_calibrate(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    printf("Hold the robot STILL, front facing the way the arrow should point...\n");
    if (imu_calibrate(8000) != ESP_OK) {
        return 1;
    }
    live_map_reset();
    printf("Heading is now yaw 0. Map cleared so the scan is drawn in that frame.\n");
    return 0;
}

static int cmd_zero(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    imu_zero_yaw();
    return 0;
}

static int cmd_mapreset(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    live_map_reset();
    return 0;
}

static int cmd_wifi(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    if (!wifi_app_is_up()) {
        printf("Wi-Fi is down. P4 talks to the C6 via ESP-Hosted; check the coprocessor.\n");
        return 1;
    }
    wifi_app_print();
    return 0;
}

static int cmd_pos(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: pos <id>\n");
        return 1;
    }
    if (!u2d2_is_ready()) {
        printf("U2D2 not connected\n");
        return 1;
    }
    int id = atoi(argv[1]);
    float deg = 0;
    uint16_t raw = 0;
    esp_err_t err = dxl_read_pos_deg((uint8_t)id, &deg, &raw);
    if (err != ESP_OK) {
        printf("pos failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    printf("id %d  pos %u  (%.1f deg)\n", id, raw, deg);
    return 0;
}

static void register_cmds(void)
{
    const esp_console_cmd_t scan = {
        .command = "scan",
        .help = "Ping Dynamixel IDs 0-253 (optional: scan <max_id>)",
        .hint = "[max_id]",
        .func = &cmd_scan,
    };
    const esp_console_cmd_t move = {
        .command = "move",
        .help = "Set motor angle in degrees: move <id> <angle>",
        .hint = "<id> <angle_deg>",
        .func = &cmd_move,
    };
    const esp_console_cmd_t ping = {
        .command = "ping",
        .help = "Ping one motor",
        .hint = "<id>",
        .func = &cmd_ping,
    };
    const esp_console_cmd_t pos = {
        .command = "pos",
        .help = "Read present position",
        .hint = "<id>",
        .func = &cmd_pos,
    };
    const esp_console_cmd_t uscan = {
        .command = "uscan",
        .help = "List USB devices on GPIO26/27 (U2D2)",
        .hint = NULL,
        .func = &cmd_uscan,
    };
    const esp_console_cmd_t swap = {
        .command = "swap",
        .help = "Swap USB D+/D- in software if U2D2 is not found",
        .hint = NULL,
        .func = &cmd_swap,
    };
    const esp_console_cmd_t lping = {
        .command = "lping",
        .help = "Ping RPLIDAR over USB (GET_INFO + GET_HEALTH)",
        .hint = NULL,
        .func = &cmd_lping,
    };
    const esp_console_cmd_t lscan = {
        .command = "lscan",
        .help = "Capture one RPLIDAR revolution",
        .hint = NULL,
        .func = &cmd_lscan,
    };
    const esp_console_cmd_t imu = {
        .command = "imu",
        .help = "Read IMU on GPIO7/GPIO8",
        .hint = NULL,
        .func = &cmd_imu,
    };
    const esp_console_cmd_t iscan = {
        .command = "iscan",
        .help = "Scan I2C bus on GPIO7/GPIO8",
        .hint = NULL,
        .func = &cmd_iscan,
    };
    const esp_console_cmd_t actions = {
        .command = "actions",
        .help = "List quadruped actions",
        .hint = NULL,
        .func = &cmd_actions,
    };
    const esp_console_cmd_t action = {
        .command = "action",
        .help = "Play an action: action <name> [cycles]. action stop ends it and plays neutral",
        .hint = "<name> [cycles] | stop",
        .func = &cmd_action,
    };
    const esp_console_cmd_t mapc = {
        .command = "map",
        .help = "Show live occupancy and the old embedded map",
        .hint = NULL,
        .func = &cmd_map,
    };
    const esp_console_cmd_t live = {
        .command = "live",
        .help = "Start/stop LiDAR SCAN (off at boot until app or: live / live stop)",
        .hint = "[stop]",
        .func = &cmd_live,
    };
    const esp_console_cmd_t cal = {
        .command = "calibrate",
        .help = "IMU: hold still, current heading becomes the front arrow",
        .hint = NULL,
        .func = &cmd_calibrate,
    };
    const esp_console_cmd_t zero = {
        .command = "zero",
        .help = "IMU: set current heading as front without re-bias",
        .hint = NULL,
        .func = &cmd_zero,
    };
    const esp_console_cmd_t mapreset = {
        .command = "mapreset",
        .help = "Clear the live occupancy grid",
        .hint = NULL,
        .func = &cmd_mapreset,
    };
    const esp_console_cmd_t wifi = {
        .command = "wifi",
        .help = "Print Wi-Fi AP SSID, password, and app endpoints",
        .hint = NULL,
        .func = &cmd_wifi,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&scan));
    ESP_ERROR_CHECK(esp_console_cmd_register(&move));
    ESP_ERROR_CHECK(esp_console_cmd_register(&ping));
    ESP_ERROR_CHECK(esp_console_cmd_register(&pos));
    ESP_ERROR_CHECK(esp_console_cmd_register(&uscan));
    ESP_ERROR_CHECK(esp_console_cmd_register(&swap));
    ESP_ERROR_CHECK(esp_console_cmd_register(&lping));
    ESP_ERROR_CHECK(esp_console_cmd_register(&lscan));
    ESP_ERROR_CHECK(esp_console_cmd_register(&imu));
    ESP_ERROR_CHECK(esp_console_cmd_register(&iscan));
    ESP_ERROR_CHECK(esp_console_cmd_register(&actions));
    ESP_ERROR_CHECK(esp_console_cmd_register(&action));
    ESP_ERROR_CHECK(esp_console_cmd_register(&mapc));
    ESP_ERROR_CHECK(esp_console_cmd_register(&live));
    ESP_ERROR_CHECK(esp_console_cmd_register(&cal));
    ESP_ERROR_CHECK(esp_console_cmd_register(&zero));
    ESP_ERROR_CHECK(esp_console_cmd_register(&mapreset));
    ESP_ERROR_CHECK(esp_console_cmd_register(&wifi));
}

void app_main(void)
{
    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs);

    ESP_LOGI(TAG, "ESP32-P4 USB host: D+=GPIO%d D-=GPIO%d, %d baud, Protocol 1",
             USB_DP_GPIO, USB_DM_GPIO, U2D2_BAUDRATE);

    ESP_ERROR_CHECK(u2d2_start());
    if (actions_start() != ESP_OK) {
        printf("Action player failed to start.\n");
    }
    ESP_ERROR_CHECK(lidar_start());
    if (imu_start() != ESP_OK) {
        printf("IMU not found on GPIO%d/GPIO%d. Type iscan after wiring SDA/SCL.\n",
               IMU_SDA_GPIO, IMU_SCL_GPIO);
    }
    if (live_map_start() != ESP_OK) {
        printf("Live map task failed to start.\n");
    }
    if (wifi_app_start() != ESP_OK) {
        printf("Wi-Fi/app link not up. Serial commands still work (live, calibrate, map).\n");
    }
    map_print();

    printf("Waiting for USB2AX on GPIO%d/GPIO%d ...\n", USB_DP_GPIO, USB_DM_GPIO);
    if (u2d2_wait_ready(15000) == ESP_OK) {
        printf("Boot: playing neutral once\n");
        if (action_play("neutral") != ESP_OK) {
            printf("Boot: neutral did not finish\n");
        }
        dxl_scan(DXL_MAX_ID);
    } else {
        printf("USB2AX not ready yet. Neutral was not played. Plug it in, then: action neutral\n");
    }
    if (lidar_wait_ready(4000) != ESP_OK) {
        printf("LiDAR not seen yet. Plug CP2102 into USB-OTG, then type: lping\n");
    } else {
        printf("LiDAR detected. Motor stays off until the app POSTs /api/scan/start\n");
    }

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = "dxl>";
    repl_config.max_cmdline_length = 96;

    esp_console_register_help_command();
    register_cmds();

#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
#error "USB Serial/JTAG uses GPIO19/20. Use UART console so U2D2 can own D+/D-."
#else
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));
#endif

    printf("Commands: live | calibrate | wifi | map | imu | lscan | action | help\n");
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
}
