#pragma once

#include "sdkconfig.h"

/*
 * ESP32-P4 has two USB-OTG controllers:
 *
 *   USB 2.0 HS  pins 50 (D+) / 49 (D-)  — board USB-OTG connector (RPLIDAR)
 *   USB 1.1 FS  GPIO27 (D+) / GPIO26 (D-)  — USB2AX / Dynamixel (do not change)
 *
 * USB2AX wiring (USB 1.1 host, leave this as-is):
 *   ESP32-P4 GPIO27  →  USB2AX D+
 *   ESP32-P4 GPIO26  →  USB2AX D-
 *   ESP32-P4 GND     →  USB2AX GND
 *   5V               →  USB2AX VBUS
 *
 * RPLIDAR: plug the CP2102 adapter into the USB-OTG connector.
 *
 * Keep UART0 / USB-UART for idf.py flash monitor.
 * Do not enable USB Serial/JTAG; it shares GPIO26/27 with USB 1.1.
 */

#if CONFIG_IDF_TARGET_ESP32P4
#define USB_DP_GPIO            27
#define USB_DM_GPIO            26
/* BIT0 = USB 2.0 HS OTG (LiDAR). BIT1 = USB 1.1 FS GPIO26/27 (USB2AX). */
#define U2D2_USB_PERIPH_MAP    ((1u << 0) | (1u << 1))
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
#define USB_DP_GPIO            20
#define USB_DM_GPIO            19
#define U2D2_USB_PERIPH_MAP    (1u << 0)
#elif CONFIG_IDF_TARGET_ESP32H4
#define USB_DP_GPIO            22
#define USB_DM_GPIO            21
#define U2D2_USB_PERIPH_MAP    (1u << 0)
#else
#error "U2D2 USB host needs a USB-OTG chip (ESP32-P4/S2/S3/H4)"
#endif
