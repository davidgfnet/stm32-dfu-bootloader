/*
 * Author: David Guillen Fandos (2023) <david@davidgf.net>
 * Upgrades bootloader using the upgrader firmware.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#define CMD_ACCESS_OPTB        0x01
#define CMD_ACCESS_FLASH_WRPR  0x02
#define CMD_REBOOT             0xff
#define CMD_GO_UPGRADE         0x55
#define VENDOR_ID            0xdead
#define PRODUCT_ID           0x10ad
#define IFACE_NUMBER            0x0
#define TIMEOUT_MS             5000
#define BOOTLOADER_FW_SIZE (4*1024)

#define RDPRT_UNPROTECTED   0xA5
#define RDPRT_UNPROTECTED_N 0x5A

static const int CTRL_REQ_TYPE_IN  = LIBUSB_ENDPOINT_IN  | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE;
static const int CTRL_REQ_TYPE_OUT = LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_INTERFACE;

typedef struct {
	uint8_t RDP, nRDP;
	uint8_t USER, nUSER;
	uint8_t Data0, nData0, Data1, nData1;
	uint8_t WRP0, nWRP0, WRP1, nWRP1;
	uint8_t WRP2, nWRP2, WRP3, nWRP3;
} t_optbytes;

const t_optbytes def_opts = {
	.RDP = 0xa5, .nRDP = 0x5a, .USER = 0xff, .nUSER = 0x00,   // Level 0
	.Data0 = 0xff, .nData0 = 0x00, .Data1 = 0xff, .nData1 = 0x00,

	// No protections!
	.WRP0 = 0xff, .nWRP0 = 0x00, .WRP1 = 0xff, .nWRP1 = 0x00,
	.WRP2 = 0xff, .nWRP2 = 0x00, .WRP3 = 0xff, .nWRP3 = 0x00,
};

void fatal_error(const char * errmsg, int code) {
	fprintf(stderr, "ERROR! %d %s\n", code, errmsg);
	exit(1);
}

int main(int argc, char ** argv) {
	if (argc < 2 || !strcmp(argv[1], "-h")) {
		fprintf(stderr, "Usage: %s (-h|-r|fw.bin)\n", argv[0]);
		fprintf(stderr, "  -h       Prints this help message\n");
		fprintf(stderr, "  -r       Reboots (regular reset)\n");
		fprintf(stderr, "  -b       Reboots to bootloader DFU mode\n");
		fprintf(stderr, "  -i       Prints some info\n");
		fprintf(stderr, "  -u [-f]  Unprotect device (switch to RDP level 0)\n");
		fprintf(stderr, "  fw.bin   Uploads and flashes a new bootloader using the specified file\n");
		return 1;
	}

	int result = libusb_init(NULL);
	if (result < 0)
		fatal_error("libusb_init failed!", result);

	struct libusb_device_handle *devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);

	if (!devh)
		fatal_error("libusb_open_device_with_vid_pid failed to find a matching device!", 0);

	result = libusb_detach_kernel_driver(devh, IFACE_NUMBER);
	result = libusb_claim_interface(devh, IFACE_NUMBER);
	if (result < 0)
		fatal_error("libusb_claim_interface failed!", result);

	if (!strcmp(argv[1], "-r")) {
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_OUT, CMD_REBOOT, 0, IFACE_NUMBER, 0, 0, TIMEOUT_MS) < 0)
			fatal_error("Reboot command failed!", 0);
		printf("Device rebooted!\n");
	}
	else if (!strcmp(argv[1], "-b")) {
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_OUT, CMD_REBOOT, 1, IFACE_NUMBER, 0, 0, TIMEOUT_MS) < 0)
			fatal_error("Reboot into DFU command failed!", 0);
		printf("Device rebooted into DFU mode!\n");
	}
	else if (!strcmp(argv[1], "-i")) {
		uint8_t fwrp[4];
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_ACCESS_FLASH_WRPR, 0,
		                            IFACE_NUMBER, &fwrp[0], sizeof(fwrp), TIMEOUT_MS) < 0)
			fatal_error("Cannot read flash protection bits (FLASH_WRPR)!", 0);

		union {
			t_optbytes opts;
			uint8_t buf[16];
		} buf;
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_ACCESS_OPTB, 0,
		                            IFACE_NUMBER, &buf.buf[0], sizeof(buf), TIMEOUT_MS) < 0)
			fatal_error("Cannot read option bytes!", 0);

		if (buf.opts.RDP != RDPRT_UNPROTECTED || buf.opts.nRDP != RDPRT_UNPROTECTED_N)
			printf("Flash readout is enabled (sys mem is read-protected)\n");

		if (fwrp[0] & 1)
			printf("Bootloader flash is *not* write-protected, can flash a new bootloader!\n");
		else
			printf("Bootloader flash is write-protected, must unlock it before a new bootloader can be flashed\n");

		if (fwrp[0] != buf.opts.WRP0)
			printf("Mismatch between WRP0 and FLASH_WRPR, you might need to reset the device\n");

		printf("Raw FLASH_WRPR: %02x%02x%02x%02x\n", fwrp[3], fwrp[2], fwrp[1], fwrp[0]);
		printf("Raw option bytes:\n");
		const char *names[] = { "RDP/Usr", "Data0/1", "WRP 0/1", "WRP 2/3" };
		for (unsigned i = 0; i < 16; i += 4) {
			uint16_t a = buf.buf[i+0] | (buf.buf[i+1] << 8), b = buf.buf[i+2] | (buf.buf[i+3] << 8);
			printf(" > %s: %04x %04x\n", names[i/4], a, b);
		}
	}
	else if (!strcmp(argv[1], "-u")) {
		t_optbytes copts;
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_ACCESS_OPTB, 0, IFACE_NUMBER,
		                            (unsigned char*)&copts, sizeof(copts), TIMEOUT_MS) < 0)
			fatal_error("Cannot read option bytes!", 0);

		if (copts.RDP != RDPRT_UNPROTECTED || copts.nRDP != RDPRT_UNPROTECTED_N) {
			printf("System memory readout protection is enabled! It is not possible to update the bootloader\n");
			printf("without going through a mass-erase and reset, you will need an SWD programmer!\n");
			if (argc < 3 || strcmp(argv[2], "-f")) {
				printf("If you want to proceed and wipe the device, use force (-f)\n");
				return 1;
			}
		}
		else if ((copts.WRP0 & 1) == 0) {
			printf("First 4KB are protected, let's unprotect them!\n");
			copts.WRP0 |= 1;
		} else {
			if (argc < 3 || strcmp(argv[2], "-f")) {
				printf("No protection detected, no action will be performed.\n");
				printf("You can use '-f' to force Option Bytes reset to their default values anyway.\n");
				return 1;
			}
			memcpy(&copts, &def_opts, sizeof(copts));
		}

		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_OUT, CMD_ACCESS_OPTB, 0, IFACE_NUMBER,
		                            (unsigned char*)&copts, sizeof(copts), TIMEOUT_MS) < 0)
			fatal_error("Cannot write option bytes!", 0);

		printf("Updated Option bytes, device should be unlocked now!\n");
		printf("If you want to flash a new bootloader, perform an updater reboot using '-r'\n");
	}
	else {
		// Read option bytes and check if we can actually write the bootloader
		t_optbytes copts;
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_ACCESS_OPTB, 0, IFACE_NUMBER,
		                            (unsigned char*)&copts, sizeof(copts), TIMEOUT_MS) < 0)
			fatal_error("Cannot read option bytes!", 0);

		uint8_t fwrp[4];
		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_IN, CMD_ACCESS_FLASH_WRPR, 0,
		                            IFACE_NUMBER, &fwrp[0], sizeof(fwrp), TIMEOUT_MS) < 0)
			fatal_error("Cannot read flash protection bits (FLASH_WRPR)!", 0);

		// Check if they deviate from the default
		if (copts.RDP != RDPRT_UNPROTECTED || copts.nRDP != RDPRT_UNPROTECTED_N)
			fatal_error("Device seems locked, cannot update. Check '-u' option.", 0);

		if (!(fwrp[0] & 1))
			fatal_error("Bootloader flash is write-protected, the update cannot proceed. Check '-u' option.\n", 0);

		unsigned char buffer[4*1024] = {0};
		FILE *fd = fopen(argv[1], "rb");
		int plen = fread(buffer, 1, sizeof(buffer), fd);
		fclose(fd);

		uint16_t checksum = 0xf153;
		uint16_t *p16 = (uint16_t*)(buffer);
		for (unsigned i = 0; i < BOOTLOADER_FW_SIZE / sizeof(uint16_t); i++)
			checksum ^= p16[i];

		printf("Flashing new bootloader: %d bytes, checksum %04x\n", plen, checksum);

		if (libusb_control_transfer(devh, CTRL_REQ_TYPE_OUT, CMD_GO_UPGRADE, checksum,
		                            IFACE_NUMBER, buffer, sizeof(buffer), TIMEOUT_MS) < 0)
			fatal_error("Firmware flash failed!", 0);
		printf("Device bootloader updated, it should reset itself\n");
	}

	libusb_release_interface(devh, 0);
	libusb_close(devh);
	libusb_exit(NULL);
	return 0;
}

