/*
 * Copyright (C) 2018 David Guillen Fandos <david@davidgf.net>
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "flash_config.h"
#include <string.h>
#include "usb.h"
#include "reboot.h"
#include "flash.h"
#include "watchdog.h"

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

// Payload/app comes inmediately after Bootloader
#define APP_ADDRESS (FLASH_BASE_ADDR + (FLASH_BOOTLDR_SIZE_KB)*1024)

// USB control data buffer
uint8_t usbd_control_buffer[DFU_TRANSFER_SIZE];

// DFU state
static enum dfu_state usbdfu_state = STATE_DFU_IDLE;
static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;

// Serial number to expose via USB
static char serial_no[25];

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
const char * const _usb_strings[5] = {
	"davidgf.net (libopencm3 based)", // iManufacturer
	"DFU bootloader [" VERSION "]", // iProduct
	serial_no, // iSerialNumber
	// Interface desc string
	/* This string is used by ST Microelectronics' DfuSe utility. */
	/* Change check_do_erase() accordingly */
	"@Internal Flash /0x08000000/"
	  STR(FLASH_BOOTLDR_SIZE_KB) "*001Ka,"
	  STR(FLASH_BOOTLDR_PAYLOAD_SIZE_KB) "*001Kg",
	// Config desc string
	"Bootloader config: "
	#ifdef ENABLE_WATCHDOG
	"WtDg[" STR(ENABLE_WATCHDOG) "s] "
	#endif
	#ifdef ENABLE_SAFEWRITE
	"SafeWr "
	#endif
	#ifdef ENABLE_PROTECTIONS
	"RDO/DBG "
	#endif
	#ifdef ENABLE_CHECKSUM
	"FW-CRC "
	#endif
};

static const char hcharset[16] = "0123456789abcdef";
static void get_dev_unique_id(char *s) {
	volatile uint8_t *unique_id = (volatile uint8_t *)0x1FFFF7E8;
	/* Fetch serial number from chip's unique ID */
	for (int i = 0; i < 24; i += 2) {
		s[i]   = hcharset[(*unique_id >> 4) & 0xF];
		s[i+1] = hcharset[*unique_id++ & 0xF];
	}
}

static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout) {
	switch (usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = 100;
		return DFU_STATUS_OK;
	case STATE_DFU_MANIFEST_SYNC:
		// Device will reset when read is complete.
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	case STATE_DFU_ERROR:
		return STATE_DFU_ERROR;
	default:
		return DFU_STATUS_OK;
	}
}

static void _full_system_reset() {
	// Reset and wait for it!
	volatile uint32_t *_scb_aircr = (uint32_t*)0xE000ED0CU;
	*_scb_aircr = 0x05FA0000 | 0x4;
	while(1);
	__builtin_unreachable();
}

// GPIO/RCC stuff

#define RCC_APB2ENR  (*(volatile uint32_t*)0x40021018U)

#define rcc_gpio_enable(gpion) \
	RCC_APB2ENR |= (1 << (gpion + 2));


static void usbdfu_getstatus_complete(struct usb_setup_data *req) {
	(void)req;

	// Protect the flash by only writing to the valid flash area
	const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
	const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);

	switch (usbdfu_state) {
	case STATE_DFU_DNBUSY:
		_flash_unlock(0);
		if (prog.blocknum == 0) {
			switch (prog.buf[0]) {
			case CMD_ERASE: {
				#ifdef ENABLE_SAFEWRITE
				check_do_erase();
				#endif

				// Clear this page here.
				uint32_t baseaddr = *(uint32_t *)(prog.buf + 1);
				if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
					if (!_flash_page_is_erased(baseaddr))
						_flash_erase_page(baseaddr);
				}
				} break;
			case CMD_SETADDR:
				// Assuming little endian here.
				prog.addr = *(uint32_t *)(prog.buf + 1);
				break;
			}
		} else {
			#ifdef ENABLE_SAFEWRITE
			check_do_erase();
			#endif

			// From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((prog.blocknum - 2) * DFU_TRANSFER_SIZE);

			if (baseaddr >= start_addr && baseaddr + prog.len <= end_addr) {
				// Program buffer in one go after erasing.
				if (!_flash_page_is_erased(baseaddr))
					_flash_erase_page(baseaddr);
				_flash_program_buffer(baseaddr, (uint16_t*)prog.buf, prog.len);
			}
		}
		_flash_lock();

		/* Jump straight to dfuDNLOAD-IDLE, skipping dfuDNLOAD-SYNC. */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;
	case STATE_DFU_MANIFEST:
		return;  // Reset placed in main loop.
	default:
		return;
	}
}

enum usbd_request_return_codes
usbdfu_control_request(struct usb_setup_data *req,
		uint16_t *len, void (**complete)(struct usb_setup_data *req)) {
	switch (req->bRequest) {
	case DFU_DNLOAD:
		if ((len == NULL) || (*len == 0)) {
			// wLength = 0 means leave DFU
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return USBD_REQ_HANDLED;
		} else {
			/* Copy download data for use on GET_STATUS. */
			prog.blocknum = req->wValue;
			// Beware overflows!
			prog.len = *len;
			if (prog.len > sizeof(prog.buf))
				prog.len = sizeof(prog.buf);
			memcpy(prog.buf, usbd_control_buffer, prog.len);
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return USBD_REQ_HANDLED;
		}
	case DFU_CLRSTATUS:
		// Just clears errors.
		if (usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		// Abort just returns to IDLE state.
		usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_DETACH:
		usbdfu_state = STATE_DFU_MANIFEST;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		// Send data back to host by reading the image.
		usbdfu_state = STATE_DFU_UPLOAD_IDLE;
		if (!req->wValue) {
			// Send back supported commands.
			usbd_control_buffer[0] = 0x00;
			usbd_control_buffer[1] = CMD_SETADDR;
			usbd_control_buffer[2] = CMD_ERASE;
			*len = 3;
			return USBD_REQ_HANDLED;
		} else {
			// Send back data if only if we enabled that.
			#ifndef ENABLE_DFU_UPLOAD
			usbdfu_state = STATE_DFU_ERROR;
			*len = 0;
			#else
			// From formula Address_Pointer + ((wBlockNum - 2)*wTransferSize)
			uint32_t baseaddr = prog.addr + ((req->wValue - 2) * DFU_TRANSFER_SIZE);
			const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
			const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);
			if (baseaddr >= start_addr && baseaddr + DFU_TRANSFER_SIZE <= end_addr) {
				memcpy(usbd_control_buffer, (void*)baseaddr, DFU_TRANSFER_SIZE);
				*len = DFU_TRANSFER_SIZE;
			} else {
				usbdfu_state = STATE_DFU_ERROR;
				*len = 0;
			}
			#endif
		}
		return USBD_REQ_HANDLED;
	case DFU_GETSTATUS: {
		// Perfom the action and register complete callback.
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */
		usbd_control_buffer[0] = usbdfu_getstatus(&bwPollTimeout);
		usbd_control_buffer[1] = bwPollTimeout & 0xFF;
		usbd_control_buffer[2] = (bwPollTimeout >> 8) & 0xFF;
		usbd_control_buffer[3] = (bwPollTimeout >> 16) & 0xFF;
		usbd_control_buffer[4] = usbdfu_state;
		usbd_control_buffer[5] = 0; /* iString not used here */
		*len = 6;
		*complete = usbdfu_getstatus_complete;
		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		// Return state with no state transision.
		usbd_control_buffer[0] = usbdfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NEXT_CALLBACK;
}

#define GPIOA 0
#define GPIOB 1
#define GPIOC 2
#define GPIOD 3
#define GPIOE 4
#define GPIOF 5

#define GPIO_CRL(x)  *((volatile uint32_t*)(x*0x400 +  0 + 0x40010800U))
#define GPIO_CRH(x)  *((volatile uint32_t*)(x*0x400 +  4 + 0x40010800U))
#define GPIO_IDR(x)  *((volatile uint32_t*)(x*0x400 +  8 + 0x40010800U))
#define GPIO_BSRR(x) *((volatile uint32_t*)(x*0x400 + 16 + 0x40010800U))

inline static void gpio_set_mode(uint32_t gpiodev, uint16_t gpion, uint8_t mode) {
	if (gpion < 8)
		GPIO_CRL(gpiodev) = (GPIO_CRL(gpiodev) & ~(0xf << ((gpion)<<2))) | (mode << ((gpion)<<2));
	else
		GPIO_CRH(gpiodev) = (GPIO_CRL(gpiodev) & ~(0xf << ((gpion-8)<<2))) | (mode << ((gpion-8)<<2));
}

#define gpio_set_output(a,b)    gpio_set_mode(a,b,0x2)
#define gpio_set_input(a,b)     gpio_set_mode(a,b,0x0)
#define gpio_set_input_pp(a,b)  gpio_set_mode(a,b,0x8)

#define gpio_clear(gpiodev, gpion) \
	GPIO_BSRR(gpiodev) = (1 << (16 + gpion))
#define gpio_set(gpiodev, gpion) \
	GPIO_BSRR(gpiodev) = (1 << (gpion))

#define gpio_read(gpiodev, gpion) \
	(GPIO_IDR(gpiodev) & (1 << (gpion)))

#ifdef ENABLE_GPIO_DFU_BOOT
int force_dfu_gpio() {
	rcc_gpio_enable(GPIO_DFU_BOOT_PORT);
	gpio_set_input_pp(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
	gpio_clear(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
	for (unsigned int i = 0; i < 512; i++)
		__asm__("nop");
	uint16_t val = gpio_read(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
	gpio_set_input(GPIO_DFU_BOOT_PORT, GPIO_DFU_BOOT_PIN);
	return val != 0;
}
#else
#define force_dfu_gpio()  (0)
#endif

#define FLASH_ACR_LATENCY         7
#define FLASH_ACR_LATENCY_2WS  0x02
#define FLASH_ACR (*(volatile uint32_t*)0x40022000U)

#define RCC_CFGR_HPRE_SYSCLK_NODIV      0x0
#define RCC_CFGR_PPRE1_HCLK_DIV2        0x4
#define RCC_CFGR_PPRE2_HCLK_NODIV       0x0
#define RCC_CFGR_ADCPRE_PCLK2_DIV8      0x3
#define RCC_CFGR_PLLMUL_PLL_CLK_MUL9    0x7
#define RCC_CFGR_PLLSRC_HSE_CLK         0x1
#define RCC_CFGR_PLLXTPRE_HSE_CLK       0x0
#define RCC_CFGR_SW_SYSCLKSEL_PLLCLK    0x2
#define RCC_CFGR_SW_SHIFT                 0
#define RCC_CFGR_SW (3 << RCC_CFGR_SW_SHIFT)

#define RCC_CR_HSEON    (1 << 16)
#define RCC_CR_HSERDY   (1 << 17)
#define RCC_CR_PLLON    (1 << 24)
#define RCC_CR_PLLRDY   (1 << 25)
#define RCC_CR       (*(volatile uint32_t*)0x40021000U)
#define RCC_CFGR     (*(volatile uint32_t*)0x40021004U)

static void clock_setup_in_hse_8mhz_out_72mhz() {
	// No need to use HSI or HSE while setting up the PLL, just use the RC osc.

	/* Enable external high-speed oscillator 8MHz. */
	RCC_CR |= RCC_CR_HSEON;
	while (!(RCC_CR & RCC_CR_HSERDY));

	/*
	 * Set prescalers for AHB, ADC, ABP1, ABP2.
	 * Do this before touching the PLL (TODO: why?).
	 */
	uint32_t reg32 = RCC_CFGR & 0xFFC0000F;
	reg32 |= (RCC_CFGR_HPRE_SYSCLK_NODIV << 4) | (RCC_CFGR_PPRE1_HCLK_DIV2 << 8) |
	         (RCC_CFGR_PPRE2_HCLK_NODIV << 11) | (RCC_CFGR_ADCPRE_PCLK2_DIV8 << 14) |
	         (RCC_CFGR_PLLMUL_PLL_CLK_MUL9 << 18) | (RCC_CFGR_PLLSRC_HSE_CLK << 16) |
	         (RCC_CFGR_PLLXTPRE_HSE_CLK << 17);
	RCC_CFGR = reg32;

	// 0WS from 0-24MHz
	// 1WS from 24-48MHz
	// 2WS from 48-72MHz
	FLASH_ACR = (FLASH_ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2WS;
	
	/* Enable PLL oscillator and wait for it to stabilize. */
    RCC_CR |= RCC_CR_PLLON;
	while (!(RCC_CR & RCC_CR_PLLRDY));

	// Select PLL as SYSCLK source.
    RCC_CFGR = (RCC_CFGR & ~RCC_CFGR_SW) | (RCC_CFGR_SW_SYSCLKSEL_PLLCLK << RCC_CFGR_SW_SHIFT);
}

int main(void) {
	/* Boot the application if it seems valid and we haven't been
	 * asked to reboot into DFU mode. This should make the CPU to 
	 * boot into DFU if the user app has been erased. */

	#ifdef ENABLE_PROTECTIONS
	// Check for RDP protection, and in case it's not enabled, do it!
	volatile uint32_t *_flash_obr = (uint32_t*)0x4002201CU;
	if (!((*_flash_obr) & 0x2)) {
		// Read protection NOT enabled ->

		// Unlock option bytes
		_flash_unlock(1);

		// Delete them all
		_flash_erase_option_bytes();

		// Now write a pair of bytes that are complentary [RDP, nRDP]
		_flash_program_option_bytes(0x1FFFF800U, 0x33CC);

		// Now reset, for RDP to take effect. We should not re-enter this path
		_full_system_reset();
	}

	// Disable JTAG and SWD to prevent debugging/readout
	volatile uint32_t *_AFIO_MAPR = (uint32_t*)0x40010004U;
	*_AFIO_MAPR = (*_AFIO_MAPR & ~(0x7 << 24)) | (0x4 << 24);
	#endif

	const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
	const uint32_t * const base_addr = (uint32_t*)start_addr;

	#ifdef ENABLE_CHECKSUM
	uint32_t imagesize = base_addr[0x20 / 4];
	#else
	uint32_t imagesize = 0;
	#endif

	int go_dfu = rebooted_into_dfu() ||
	#ifdef ENABLE_WATCHDOG
	             reset_due_to_watchdog() ||
	#endif
	             imagesize > FLASH_BOOTLDR_PAYLOAD_SIZE_KB*1024/4 ||
	             force_dfu_gpio();
	             
	if (!go_dfu && 
	   (*(volatile uint32_t *)APP_ADDRESS & 0x2FFE0000) == 0x20000000) {

		// Do some simple XOR checking
		uint32_t xorv = 0;
		for (unsigned i = 0; i < imagesize; i++)
			xorv ^= base_addr[i];

		if (xorv == 0) {  // Matches!
			// Clear flags
			clear_reboot_flags();
			#ifdef ENABLE_WATCHDOG
			// Enable the watchdog
			enable_iwdg(4096 * ENABLE_WATCHDOG / 26);
			#endif
			// Set vector table base address.
			volatile uint32_t *_csb_vtor = (uint32_t*)0xE000ED08U;
			*_csb_vtor = APP_ADDRESS & 0xFFFF;
			// Initialise master stack pointer.
			__asm__ volatile("msr msp, %0"::"g"
					 (*(volatile uint32_t *)APP_ADDRESS));
			// Jump to application.
			(*(void (**)())(APP_ADDRESS + 4))();
		}
	}

	clock_setup_in_hse_8mhz_out_72mhz();

	/* Disable USB peripheral as it overrides GPIO settings */
	*USB_CNTR_REG = USB_CNTR_PWDN;
	/*
	 * Vile hack to reenumerate, physically _drag_ d+ low.
	 * (need at least 2.5us to trigger usb disconnect)
	 */
	rcc_gpio_enable(GPIOA);
	gpio_set_output(GPIOA, 12);
	gpio_clear(GPIOA, 12);
	for (unsigned int i = 0; i < 100000; i++)
		__asm__("nop");

	get_dev_unique_id(serial_no);
	usb_init();

	while (1) {
		// Poll based approach
		do_usb_poll();
		if (usbdfu_state == STATE_DFU_MANIFEST) {
			// USB device must detach, we just reset...
			_full_system_reset();
		}
	}
}

// Implement this here to save space, quite minimalistic :D
void *memcpy(void * dst, const void * src, size_t count) {
	uint8_t * dstb = (uint8_t*)dst;
	uint8_t * srcb = (uint8_t*)src;
	while (count--)
		*dstb++ = *srcb++;
	return dst;
}


