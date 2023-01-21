/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2023 David Guillen Fandos <david@davidgf.net>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

// Upgrader APP. Allows for bootloader upgrades.
//
// This app runs in RAM and exposes a very simple interface to allow for
// bootloader upgrades. It takes a 4KB payload and flashes it into the
// first 4KB of ROM (where the bootloader lives).
// It is capable of exposing some information related to Option Bytes,
// since the bootloader might be protected.


#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/f0/flash.h>
#include <libopencm3/usb/usbd.h>

#include "reboot.h"

#define CMD_ACCESS_OPTB           0x01
#define CMD_ACCESS_FLASH_WRPR     0x02
#define CMD_REBOOT                0xff
#define CMD_UPGRADE               0x55

#define BOOTLOADER_FW_SIZE      (4*1024)
#define BOOTLOADER_BASE_ADDR 0x08000000

#define OPTION_BYTES_ADDR    0x1FFFF800

/* Must be able to fit at least 4KiB. */
uint8_t usbd_control_buffer[BOOTLOADER_FW_SIZE];

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0xdead,
	.idProduct = 0x10ad,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_interface_descriptor usb_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFF,
	.bInterfaceSubClass = 0xFF,
	.bInterfaceProtocol = 0xFF,
	.iInterface = 0,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &usb_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char serial_no[25];

static const char hcharset[16] = "0123456789abcdef";
static void get_dev_unique_id(char *s) {
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	/* Fetch serial number from chip's unique ID */
	for (int i = 0; i < 24; i += 2) {
		s[i]   = hcharset[(*unique_id >> 4) & 0xF];
		s[i+1] = hcharset[*unique_id++ & 0xF];
	}
}

static const char *usb_strings[] = {
	"davidgf.net (libopencm3 based)", // iManufacturer
	"Bootloader Upgrade Tool [" VERSION "]", // iProduct
	serial_no,
};

// Some custom sutff int the EP0 for rebooting and poking
void reboot_info_dfu() {
	// Reboot into DFU!
	reboot_into_bootloader();
	scb_reset_system();
}

void reboot_info_upd() {
	// Regular reboot!
	reboot_into_updater();
	scb_reset_system();
}

static int process_upgrade(const uint8_t *buf, unsigned len, uint16_t refcheck) {
	if (len != BOOTLOADER_FW_SIZE)
		return 0;

	// wValue must hold a valid xor-checksum
	uint16_t checksum = 0xf153;
	uint16_t *p16 = (uint16_t*)(buf);
	for (unsigned i = 0; i < BOOTLOADER_FW_SIZE / sizeof(uint16_t); i++)
		checksum ^= p16[i];

	if (checksum != refcheck)
		return 0;

	// Proceed with a mass-erase so that we can flash the first pages too!
	flash_unlock();
	flash_erase_all_pages();
	for (unsigned i = 0; i < BOOTLOADER_FW_SIZE / sizeof(uint16_t); i++)
		flash_program_half_word(BOOTLOADER_BASE_ADDR + i*2, p16[i]);
	flash_lock();

	return 1;
}

static enum usbd_request_return_codes usr_control_request_out(
	usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	switch (req->bRequest) {
	case CMD_REBOOT:
		*len = 0;
		if (req->wValue)
			*complete = reboot_info_dfu;
		else
			*complete = reboot_info_upd;
		return USBD_REQ_HANDLED;
	case CMD_UPGRADE:
		// Take the received payload, which must be exactly 4KiB, and flash it
		if (process_upgrade(*buf, *len, req->wValue))
			return USBD_REQ_HANDLED;
		return USBD_REQ_NOTSUPP;
	case CMD_ACCESS_OPTB:
		if (*len != 16)
			return USBD_REQ_NOTSUPP;
		flash_unlock();
		flash_unlock_option_bytes();
		flash_erase_option_bytes();
		for (unsigned i = 0; i < 16; i += 2)
			flash_program_option_bytes(0x1FFFF800U + i, (*buf)[i] | ((*buf)[i+1] << 8));
		return USBD_REQ_HANDLED;
	default:
		return USBD_REQ_NOTSUPP;
	};
}

static enum usbd_request_return_codes usr_control_request_in(
	usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	switch (req->bRequest) {
	case CMD_ACCESS_OPTB:
		*buf = (uint8_t*)OPTION_BYTES_ADDR;
		*len = 16;
		return USBD_REQ_HANDLED;
	case CMD_ACCESS_FLASH_WRPR:
		*buf = (uint8_t*)&FLASH_WRPR;
		*len = 4;
		return USBD_REQ_HANDLED;
	default:
		return USBD_REQ_NOTSUPP;
	};
}


static void usr_set_config(usbd_device *dev, uint16_t wValue) {
	(void)wValue;
	(void)dev;

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_OUT,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
				usr_control_request_out);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE | USB_REQ_TYPE_IN,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT | USB_REQ_TYPE_DIRECTION,
				usr_control_request_in);
}

int main(void) {
	rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
	get_dev_unique_id(serial_no);

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	for (unsigned i = 0; i < 800000; i++)
		__asm__ volatile("nop");

	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3,
		usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usr_set_config);

	while (1) {
		usbd_poll(usbd_dev);
		iwdg_reset();
	}
}


