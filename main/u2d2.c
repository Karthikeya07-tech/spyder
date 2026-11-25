#include "u2d2.h"

#include <inttypes.h>
#include <string.h>
#include "usb_pins.h"

#include "esp_check.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "hal/usb_wrap_ll.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ftdi.h"

static const char *TAG = "u2d2";

#define FTDI_WRITE_REQ          0x40
#define FTDI_CMD_SET_BAUDRATE   0x03
#define FTDI_CMD_SET_FLOW       0x01
#define FTDI_CMD_SET_LATENCY    0x09
#define FTDI_CHANNEL_A          1

#define RX_BUF_SIZE             256
/* Hosted Wi-Fi tasks run at 23. USB must sit just under that or LiDAR CDC RX starves. */
#define USB_LIB_TASK_PRIO       22

static cdc_acm_dev_hdl_t s_cdc;
static portMUX_TYPE s_rx_lock = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t s_rx_sem;
static SemaphoreHandle_t s_ready_sem;
static SemaphoreHandle_t s_dev_sem;
static uint8_t s_rx[RX_BUF_SIZE];
static size_t s_rx_len;
static volatile bool s_ready;
static volatile bool s_probe_paused;
static volatile bool s_servo_present;
static bool s_dp_dm_swapped;
static uint16_t s_seen_vid;
static uint16_t s_seen_pid;
static uint8_t s_seen_class;
static int s_open_fails;

static const char *adapter_name(uint16_t vid, uint16_t pid)
{
    if (vid == USB2AX_VID && pid == USB2AX_PID) {
        return "USB2AX";
    }
    if (vid == U2D2_VID && pid == U2D2_PID_FT232H) {
        return "U2D2";
    }
    if (vid == U2D2_VID && pid == U2D2_PID_FT232R) {
        return "USB2Dynamixel";
    }
    if (vid == 0x10C4) {
        return "RPLIDAR-CP210x";
    }
    if (vid == 0x1A86) {
        return "CH34x-UART";
    }
    return "USB-serial";
}

static bool adapter_is_ftdi(uint16_t vid, uint16_t pid)
{
    return vid == U2D2_VID && (pid == U2D2_PID_FT232H || pid == U2D2_PID_FT232R);
}

static bool adapter_is_servo(uint16_t vid, uint16_t pid)
{
    return (vid == USB2AX_VID && pid == USB2AX_PID) || adapter_is_ftdi(vid, pid);
}

void u2d2_swap_dp_dm(bool swap)
{
    s_dp_dm_swapped = swap;
    usb_wrap_ll_phy_enable_pin_exchg(&USB_WRAP, swap);
    ESP_LOGI(TAG, "D+/D- pin exchange %s (GPIO%d=D+ GPIO%d=D- physically)",
             swap ? "ON" : "OFF", USB_DP_GPIO, USB_DM_GPIO);
}

void u2d2_pause_probe(bool pause)
{
    s_probe_paused = pause;
}

static void on_new_usb_dev(usb_device_handle_t usb_dev)
{
    const usb_device_desc_t *desc = NULL;
    if (usb_host_get_device_descriptor(usb_dev, &desc) != ESP_OK || desc == NULL) {
        ESP_LOGW(TAG, "USB device connected but descriptor unreadable");
        return;
    }
    s_seen_vid = desc->idVendor;
    s_seen_pid = desc->idProduct;
    s_seen_class = desc->bDeviceClass;
    ESP_LOGI(TAG, "USB device found: VID=0x%04X PID=0x%04X class=0x%02X (%s)",
             s_seen_vid, s_seen_pid, s_seen_class, adapter_name(s_seen_vid, s_seen_pid));
    if (adapter_is_servo(s_seen_vid, s_seen_pid)) {
        s_servo_present = true;
        ESP_LOGI(TAG, "Servo adapter - USB2AX/U2D2 on GPIO%d/GPIO%d", USB_DP_GPIO, USB_DM_GPIO);
        if (s_dev_sem) {
            xSemaphoreGive(s_dev_sem);
        }
    } else if (s_seen_vid == 0x10C4 || s_seen_vid == 0x1A86) {
        ESP_LOGI(TAG, "Likely RPLIDAR USB-UART - lidar task will open it (not the servo path)");
    }
}

void u2d2_usb_scan(void)
{
    uint8_t addrs[8];
    int n = 0;
    esp_err_t err = usb_host_device_addr_list_fill(8, addrs, &n);
    if (err != ESP_OK) {
        printf("USB scan failed: %s\n", esp_err_to_name(err));
        return;
    }
    printf("USB devices on GPIO%d (D+) / GPIO%d (D-): %d\n", USB_DP_GPIO, USB_DM_GPIO, n);
    if (s_seen_vid) {
        printf("  last enumerated: VID=0x%04X PID=0x%04X class=0x%02X (%s)\n",
               s_seen_vid, s_seen_pid, s_seen_class, adapter_name(s_seen_vid, s_seen_pid));
    }
    if (n == 0) {
        printf("  none. Check: 5V on adapter VBUS, common GND, D+->GPIO%d, D-->GPIO%d\n",
               USB_DP_GPIO, USB_DM_GPIO);
        printf("  If still empty, type: swap\n");
    }
}

static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    (void)arg;
    if (data == NULL || data_len == 0) {
        return true;
    }
    portENTER_CRITICAL(&s_rx_lock);
    size_t room = RX_BUF_SIZE - s_rx_len;
    size_t n = data_len < room ? data_len : room;
    memcpy(s_rx + s_rx_len, data, n);
    s_rx_len += n;
    portEXIT_CRITICAL(&s_rx_lock);
    if (s_rx_sem) {
        xSemaphoreGive(s_rx_sem);
    }
    return true;
}

static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        ESP_LOGE(TAG, "CDC error %d", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "USB adapter disconnected");
        s_ready = false;
        s_cdc = NULL;
        s_servo_present = false;
        s_seen_vid = 0;
        s_seen_pid = 0;
        s_seen_class = 0;
        cdc_acm_host_close(event->data.cdc_hdl);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        break;
    default:
        break;
    }
}

static uint32_t ftdi_232h_encoded_divisor(uint32_t baud)
{
    static const uint8_t divfrac[8] = {0, 3, 2, 4, 1, 5, 6, 7};
    const uint32_t base = 120000000;
    uint32_t divisor3 = (8 * base + (10 * baud) / 2) / (10 * baud);
    uint32_t divisor = divisor3 >> 3;
    divisor |= ((uint32_t)divfrac[divisor3 & 0x7]) << 14;
    if (divisor == 1) {
        divisor = 0;
    } else if (divisor == 0x4001) {
        divisor = 1;
    }
    divisor |= 0x00020000; /* disable /2.5 so rates >= 1200 work */
    return divisor;
}

static esp_err_t u2d2_set_baud_ft232h(cdc_acm_dev_hdl_t cdc, uint32_t baud)
{
    uint32_t encoded = ftdi_232h_encoded_divisor(baud);
    uint16_t value = (uint16_t)encoded;
    uint16_t index = (uint16_t)((encoded >> 8) & 0xFF00) | FTDI_CHANNEL_A;
    ESP_LOGI(TAG, "FT232H baud %" PRIu32 " (wValue=0x%04X wIndex=0x%04X)",
             baud, value, index);
    return cdc_acm_host_send_custom_request(cdc, FTDI_WRITE_REQ, FTDI_CMD_SET_BAUDRATE,
                                            value, index, 0, NULL);
}

static esp_err_t u2d2_configure_ftdi(cdc_acm_dev_hdl_t cdc)
{
    ESP_RETURN_ON_ERROR(cdc_acm_host_send_custom_request(cdc, FTDI_WRITE_REQ, FTDI_CMD_SET_FLOW,
                                                         0, FTDI_CHANNEL_A, 0, NULL),
                        TAG, "flow control");
    ESP_RETURN_ON_ERROR(cdc_acm_host_send_custom_request(cdc, FTDI_WRITE_REQ, FTDI_CMD_SET_LATENCY,
                                                         1, FTDI_CHANNEL_A, 0, NULL),
                        TAG, "latency");

    cdc_acm_line_coding_t lc = {
        .dwDTERate = 0, /* skip FT232R 3 MHz baud math; we set FT232H baud below */
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    esp_err_t err = cdc_acm_host_line_coding_set(cdc, &lc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "line coding 8N1: %s", esp_err_to_name(err));
    }

    ESP_RETURN_ON_ERROR(u2d2_set_baud_ft232h(cdc, U2D2_BAUDRATE), TAG, "baud");
    ESP_RETURN_ON_ERROR(cdc_acm_host_set_control_line_state(cdc, true, true), TAG, "DTR/RTS");
    return ESP_OK;
}

static esp_err_t u2d2_configure_cdc(cdc_acm_dev_hdl_t cdc)
{
    cdc_acm_line_coding_t lc = {
        .dwDTERate = U2D2_BAUDRATE,
        .bCharFormat = 0,
        .bParityType = 0,
        .bDataBits = 8,
    };
    esp_err_t err = cdc_acm_host_line_coding_set(cdc, &lc);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "CDC line coding %d: %s (adapter may keep default baud)",
                 U2D2_BAUDRATE, esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "CDC line coding %d 8N1", U2D2_BAUDRATE);
    }
    err = cdc_acm_host_set_control_line_state(cdc, true, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DTR/RTS: %s", esp_err_to_name(err));
    }
    return ESP_OK;
}

static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
}

static const cdc_acm_host_device_config_t s_dev_config = {
    .connection_timeout_ms = 1500,
    .out_buffer_size = 512,
    .in_buffer_size = 64,
    .event_cb = handle_event,
    .data_cb = handle_rx,
    .user_arg = NULL,
};

static esp_err_t open_cdc_vid_pid(uint16_t vid, uint16_t pid, cdc_acm_dev_hdl_t *cdc)
{
    ESP_LOGI(TAG, "Opening CDC 0x%04X:0x%04X (%s)", vid, pid, adapter_name(vid, pid));
    return cdc_acm_host_open(vid, pid, 0, &s_dev_config, cdc);
}

static esp_err_t open_u2d2(void)
{
    cdc_acm_dev_hdl_t cdc = NULL;
    esp_err_t err = ESP_ERR_NOT_FOUND;
    bool ftdi = false;
    uint16_t vid = USB2AX_VID;
    uint16_t pid = USB2AX_PID;

    if (s_probe_paused) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Only open the servo adapter. Never CDC_ANY — that would steal the LiDAR. */
    err = open_cdc_vid_pid(USB2AX_VID, USB2AX_PID, &cdc);
    if (err != ESP_OK) {
        static const uint16_t pids[] = {U2D2_PID_FT232H, U2D2_PID_FT232R};
        for (size_t i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
            if (s_probe_paused) {
                return ESP_ERR_INVALID_STATE;
            }
            ESP_LOGI(TAG, "Opening FTDI 0x%04X:0x%04X", U2D2_VID, pids[i]);
            err = ftdi_vcp_open(pids[i], 0, &s_dev_config, &cdc);
            if (err == ESP_OK) {
                ftdi = true;
                vid = U2D2_VID;
                pid = pids[i];
                break;
            }
        }
    }
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Opened %s", adapter_name(vid, pid));
    if (ftdi) {
        err = u2d2_configure_ftdi(cdc);
    } else {
        err = u2d2_configure_cdc(cdc);
    }
    if (err != ESP_OK) {
        cdc_acm_host_close(cdc);
        return err;
    }

    s_cdc = cdc;
    s_ready = true;
    if (s_ready_sem) {
        xSemaphoreGive(s_ready_sem);
    }
    ESP_LOGI(TAG, "%s ready, Protocol 1 @ %d baud",
             adapter_name(s_seen_vid, s_seen_pid), U2D2_BAUDRATE);
    return ESP_OK;
}

static void u2d2_open_task(void *arg)
{
    (void)arg;
    while (1) {
        if (s_ready) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }
        if (s_probe_paused) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        /* Do not poll VID/PID when no servo adapter has enumerated.
         * Blind CDC/FTDI open (1.5 s each) starves LiDAR SCAN on the same host. */
        if (!s_servo_present) {
            xSemaphoreTake(s_dev_sem, pdMS_TO_TICKS(30000));
            continue;
        }
        /* Let new_dev_cb return before opening; CDC host forbids open inside it. */
        vTaskDelay(pdMS_TO_TICKS(50));
        if (s_probe_paused || !s_servo_present) {
            continue;
        }

        if (open_u2d2() == ESP_OK) {
            s_open_fails = 0;
            continue;
        }

        s_open_fails++;
        if (s_open_fails <= 2 || (s_open_fails % 8) == 0) {
            ESP_LOGW(TAG, "USB2AX not open on GPIO%d/GPIO%d (swap=%d). Type swap only if D+/D- are reversed.",
                     USB_DP_GPIO, USB_DM_GPIO, s_dp_dm_swapped);
        }
        if (s_open_fails >= 3) {
            s_servo_present = false;
            ESP_LOGW(TAG, "USB2AX probe paused until the adapter enumerates again");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

esp_err_t u2d2_start(void)
{
    s_rx_sem = xSemaphoreCreateBinary();
    s_ready_sem = xSemaphoreCreateBinary();
    s_dev_sem = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(s_rx_sem && s_ready_sem && s_dev_sem, ESP_ERR_NO_MEM, TAG, "sem");

    ESP_LOGI(TAG, "USB host: FS D+=GPIO%d D-=GPIO%d (USB2AX) + HS OTG (LiDAR)",
             USB_DP_GPIO, USB_DM_GPIO);
    ESP_LOGI(TAG, "USB2AX: D+->GPIO%d  D-->GPIO%d  GND->GND  VBUS->5V",
             USB_DP_GPIO, USB_DM_GPIO);

    /* Do not gpio_reset these pads — they are the USB PHY, not general GPIO. */
    gpio_sleep_sel_dis(USB_DP_GPIO);
    gpio_sleep_sel_dis(USB_DM_GPIO);

    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .peripheral_map = U2D2_USB_PERIPH_MAP,
    };
    esp_err_t host_err = usb_host_install(&host_config);
    if (host_err != ESP_OK) {
        ESP_LOGW(TAG, "dual USB host failed (%s), falling back to FS GPIO%d/%d only",
                 esp_err_to_name(host_err), USB_DP_GPIO, USB_DM_GPIO);
        host_config.peripheral_map = (1u << 1);
        ESP_RETURN_ON_ERROR(usb_host_install(&host_config), TAG, "usb_host_install");
    }
    usb_wrap_ll_phy_set_defaults(&USB_WRAP);

    BaseType_t ok = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, USB_LIB_TASK_PRIO, NULL);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "usb_lib task");

    const cdc_acm_host_driver_config_t cdc_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 11,
        .xCoreID = tskNO_AFFINITY,
        .new_dev_cb = on_new_usb_dev,
    };
    ESP_RETURN_ON_ERROR(cdc_acm_host_install(&cdc_cfg), TAG, "cdc_acm_host_install");

    ok = xTaskCreate(u2d2_open_task, "u2d2_open", 4096, NULL, 5, NULL);
    ESP_RETURN_ON_FALSE(ok == pdTRUE, ESP_ERR_NO_MEM, TAG, "u2d2_open task");
    return ESP_OK;
}

esp_err_t u2d2_wait_ready(uint32_t timeout_ms)
{
    if (s_ready) {
        return ESP_OK;
    }
    if (xSemaphoreTake(s_ready_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        return s_ready ? ESP_OK : ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_TIMEOUT;
}

bool u2d2_is_ready(void)
{
    return s_ready && s_cdc != NULL;
}

void u2d2_rx_flush(void)
{
    portENTER_CRITICAL(&s_rx_lock);
    s_rx_len = 0;
    portEXIT_CRITICAL(&s_rx_lock);
    if (s_rx_sem) {
        xSemaphoreTake(s_rx_sem, 0);
    }
}

esp_err_t u2d2_write(const uint8_t *data, size_t len)
{
    ESP_RETURN_ON_FALSE(u2d2_is_ready(), ESP_ERR_INVALID_STATE, TAG, "U2D2 not ready");
    ESP_RETURN_ON_FALSE(data && len, ESP_ERR_INVALID_ARG, TAG, "tx");
    return cdc_acm_host_data_tx_blocking(s_cdc, data, len, 200);
}

size_t u2d2_read(uint8_t *buf, size_t max_len, uint32_t timeout_ms)
{
    if (!buf || max_len == 0) {
        return 0;
    }
    if (s_rx_len == 0) {
        xSemaphoreTake(s_rx_sem, pdMS_TO_TICKS(timeout_ms));
    }
    portENTER_CRITICAL(&s_rx_lock);
    size_t n = s_rx_len < max_len ? s_rx_len : max_len;
    memcpy(buf, s_rx, n);
    portEXIT_CRITICAL(&s_rx_lock);
    return n;
}
