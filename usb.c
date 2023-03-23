
// Stuff copied from libopencm3 and simplified/reworked
// to make it simpler and fit 4KB

#include <string.h>

#include "usb.h"

// Defined in main
extern uint8_t usbd_control_buffer[1024];
extern const char * const _usb_strings[5];
extern enum usbd_request_return_codes
usbdfu_control_request(struct usb_setup_data *req,
		uint16_t *len, void (**complete)(struct usb_setup_data *req));

// Simple builtin fns
size_t strlen(const char *s) {
	size_t ret = 0;
	while (*s++)
		ret++;
	return ret;
}

const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = USB_VID,
	.idProduct = USB_PID,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor iface;
	struct usb_dfu_descriptor dfu_function;
} config_desc = {
	.config = {
		.bLength = USB_DT_CONFIGURATION_SIZE,
		.bDescriptorType = USB_DT_CONFIGURATION,
		.wTotalLength = sizeof(config_desc),
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 5,
		.bmAttributes = 0xC0,
		.bMaxPower = 0x32,
	},
	.iface = {
		.bLength = USB_DT_INTERFACE_SIZE,
		.bDescriptorType = USB_DT_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 0,
		.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
		.bInterfaceSubClass = 1,
		.bInterfaceProtocol = 2,
		.iInterface = 4,
	},
	.dfu_function = {
		.bLength = sizeof(struct usb_dfu_descriptor),
		.bDescriptorType = DFU_FUNCTIONAL,
		.bmAttributes =
			#ifdef ENABLE_DFU_UPLOAD
			USB_DFU_CAN_UPLOAD |
			#endif
			USB_DFU_CAN_DOWNLOAD |
			USB_DFU_WILL_DETACH,
		.wDetachTimeout = 255,
		.wTransferSize = DFU_TRANSFER_SIZE,
		.bcdDFUVersion = 0x011A,
	},
};

// USB FSM state
enum {
	IDLE, STALLED,
	DATA_IN, LAST_DATA_IN, STATUS_IN,
	DATA_OUT, LAST_DATA_OUT, STATUS_OUT,
} usb_fsm_state = IDLE;
uint16_t datasize = 0;
uint16_t dataoff = 0;
uint16_t usb_pm_top = 0;
uint8_t  usb_needs_zlp = 0;
struct usb_setup_data usb_req;
uint8_t usb_force_nak[8] = {0};
void (*usb_complete_cb)(struct usb_setup_data *req) = 0;

#define RCC_APB1ENR  (*(volatile uint32_t*)0x4002101CU)
#define RCC_USB   23

#define rcc_periph_enable(pn) RCC_APB1ENR |= (1 << (pn));

void usb_init() {
	rcc_periph_enable(RCC_USB);
	SET_REG(USB_CNTR_REG, 0);
	SET_REG(USB_BTABLE_REG, 0);
	SET_REG(USB_ISTR_REG, 0);

	/* Enable RESET, SUSPEND, RESUME and CTR interrupts. */
	SET_REG(USB_CNTR_REG, USB_CNTR_RESETM | USB_CNTR_CTRM |
		USB_CNTR_SUSPM | USB_CNTR_WKUPM);
}

#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#define USBD_PM_TOP 0x40

static void st_usbfs_copy_to_pm(volatile void *vPM, const void *buf, uint16_t len) {
	const uint16_t *lbuf = buf;
	volatile uint32_t *PM = vPM;
	for (len = (len + 1) >> 1; len; len--)
		*PM++ = *lbuf++;
}

static void st_usbfs_copy_from_pm(void *buf, const volatile void *vPM, uint16_t len) {
	uint16_t *lbuf = buf;
	const volatile uint16_t *PM = vPM;
	uint8_t odd = len & 1;

	for (len >>= 1; len; PM += 2, lbuf++, len--)
		*lbuf = *PM;

	if (odd)
		*(uint8_t *) lbuf = *(uint8_t *) PM;
}

static uint16_t _usbd_ep_write_packet(uint8_t addr, const void *buf, uint16_t len) {
	addr &= 0x7F;

	if ((*USB_EP_REG(addr) & USB_EP_TX_STAT) == USB_EP_TX_STAT_VALID)
		return 0;

	st_usbfs_copy_to_pm(USB_GET_EP_TX_BUFF(addr), buf, len);
	USB_SET_EP_TX_COUNT(addr, len);
	USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_VALID);

	return len;
}

static uint16_t _usbd_ep_read_packet(uint8_t addr, void *buf, uint16_t len) {
	if ((*USB_EP_REG(addr) & USB_EP_RX_STAT) == USB_EP_RX_STAT_VALID)
		return 0;

	len = MIN(USB_GET_EP_RX_COUNT(addr) & 0x3ff, len);
	st_usbfs_copy_from_pm(buf, USB_GET_EP_RX_BUFF(addr), len);
	USB_CLR_EP_RX_CTR(addr);

	if (!usb_force_nak[addr]) {
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
	}
	return len;
}

static void _usbd_ep_nak_set(uint8_t addr, uint8_t nak) {
	// It does not make sense to force NAK on IN endpoints.
	if (addr & 0x80)
		return;

	usb_force_nak[addr] = nak;
	if (nak)
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_NAK);
	else
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
}

void _ep_stall_set(uint8_t addr, uint8_t stall) {
	if (addr == 0)
		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL : USB_EP_TX_STAT_NAK);

	if (addr & 0x80) {
		addr &= 0x7F;

		USB_SET_EP_TX_STAT(addr, stall ? USB_EP_TX_STAT_STALL : USB_EP_TX_STAT_NAK);

		// Reset to DATA0 if clearing stall condition.
		if (!stall)
			USB_CLR_EP_TX_DTOG(addr);
	} else {
		// Reset to DATA0 if clearing stall condition.
		if (!stall)
			USB_CLR_EP_RX_DTOG(addr);

		USB_SET_EP_RX_STAT(addr, stall ? USB_EP_RX_STAT_STALL : USB_EP_RX_STAT_VALID);
	}
}

uint8_t _ep_stall_get(uint8_t addr) {
	if (addr & 0x80) {
		if ((*USB_EP_REG(addr & 0x7F) & USB_EP_TX_STAT) == USB_EP_TX_STAT_STALL)
			return 1;
	} else {
		if ((*USB_EP_REG(addr) & USB_EP_RX_STAT) == USB_EP_RX_STAT_STALL)
			return 1;
	}
	return 0;
}

static inline void _stall_transaction() {
	_ep_stall_set(0, 1);
	usb_fsm_state = IDLE;
}


// Sends or keeps sending data to host
static void usb_control_send_chunk() {
	if (dev_desc.bMaxPacketSize0 < datasize) {
		/* Data stage, normal transmission */
		_usbd_ep_write_packet(0, &usbd_control_buffer[dataoff], dev_desc.bMaxPacketSize0);
		usb_fsm_state = DATA_IN;
		dataoff += dev_desc.bMaxPacketSize0;
		datasize -= dev_desc.bMaxPacketSize0;
	} else {
		/* Data stage, end of transmission */
		_usbd_ep_write_packet(0, &usbd_control_buffer[dataoff], datasize);

		usb_fsm_state = usb_needs_zlp ? DATA_IN : LAST_DATA_IN;
		usb_needs_zlp = 0;
		datasize = 0;
	}
}

// Receives data from host
static int usb_control_recv_chunk() {
	uint16_t packetsize = MIN(dev_desc.bMaxPacketSize0, usb_req.wLength - datasize);
	uint16_t size = _usbd_ep_read_packet(0, &usbd_control_buffer[datasize], packetsize);

	if (size != packetsize) {
		_stall_transaction();
		return -1;
	}

	datasize += size;
	return packetsize;
}

static enum usbd_request_return_codes usb_standard_get_descriptor() {
	int array_idx, descr_idx, descr_type;
	struct usb_string_descriptor *sd = (struct usb_string_descriptor *)usbd_control_buffer;

	descr_idx = usb_req.wValue & 0xFF;
	descr_type = usb_req.wValue >> 8;

	switch (descr_type) {
	case USB_DT_DEVICE:
		memcpy(usbd_control_buffer, &dev_desc, sizeof(dev_desc));
		datasize = sizeof(dev_desc);
		return USBD_REQ_HANDLED;
	case USB_DT_CONFIGURATION:
		memcpy(usbd_control_buffer, &config_desc, sizeof(config_desc));
		datasize = sizeof(config_desc);
		return USBD_REQ_HANDLED;
	case USB_DT_STRING:
		sd->bDescriptorType = USB_DT_STRING;
		if (descr_idx == 0) {
			/* Send sane Language ID descriptor... */
			sd->wData[0] = USB_LANGID_ENGLISH_US;
			datasize = sd->bLength = sizeof(sd->bLength) + sizeof(sd->bDescriptorType) + sizeof(sd->wData[0]);
		#ifdef WINUSB_SUPPORT
		} else if (descr_idx == 0xEE) {
			const char winusbstr[] = {'M','S','F','T','1','0','0','A','\0'};
			for (int i = 0; i < sizeof(winusbstr); i++)
				sd->wData[i] = winusbstr[i];
			datasize = sd->bLength = sizeof(sd->bLength) + sizeof(sd->bDescriptorType) + sizeof(winusbstr)*2;
		#endif
		} else {
			array_idx = descr_idx - 1;

			/* Check that string index is in range. */
			if (array_idx >= sizeof(_usb_strings) / sizeof(_usb_strings[0]))
				return USBD_REQ_NOTSUPP;

			/* Strings with Language ID different from
			 * USB_LANGID_ENGLISH_US are not supported */
			if (usb_req.wIndex != USB_LANGID_ENGLISH_US)
				return USBD_REQ_NOTSUPP;

			/* This string is returned as UTF16, hence the
			 * multiplication
			 */
			unsigned numchars = strlen(_usb_strings[array_idx]);
			datasize = sd->bLength = numchars * 2 +
			          sizeof(sd->bLength) + sizeof(sd->bDescriptorType);

			for (int i = 0; i < numchars; i++)
				sd->wData[i] = _usb_strings[array_idx][i];
		}
		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

enum usbd_request_return_codes _usbd_standard_request_device() {
	switch (usb_req.bRequest) {
	case USB_REQ_SET_ADDRESS:
		/* The actual address is only latched at the STATUS IN stage. */
		if ((usb_req.bmRequestType != 0) || (usb_req.wValue >= 128))
			return USBD_REQ_NOTSUPP;

		// Do not set addr here, wait for status IN
		// SET_REG(USB_DADDR_REG, (usb_req.wValue & USB_DADDR_ADDR) | USB_DADDR_EF);

		return USBD_REQ_HANDLED;
	case USB_REQ_SET_CONFIGURATION:
		// Reset all endpoints
		if (usb_req.wValue == config_desc.config.bConfigurationValue) {
			for (int i = 1; i < 8; i++) {
				USB_SET_EP_TX_STAT(i, USB_EP_TX_STAT_DISABLED);
				USB_SET_EP_RX_STAT(i, USB_EP_RX_STAT_DISABLED);
			}
			usb_pm_top = USBD_PM_TOP + (2 * dev_desc.bMaxPacketSize0);
			return USBD_REQ_HANDLED;
		}
		return USBD_REQ_NOTSUPP;
	case USB_REQ_GET_CONFIGURATION:
		usbd_control_buffer[0] = 1;  // FIXME?
		datasize = 1;
		return USBD_REQ_HANDLED;
	case USB_REQ_GET_DESCRIPTOR:
		return usb_standard_get_descriptor();
	case USB_REQ_GET_STATUS:
		// GET_STATUS always responds with zero reply.
		datasize = 2;
		usbd_control_buffer[0] = 0;
		usbd_control_buffer[1] = 0;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

enum usbd_request_return_codes _usbd_standard_request_interface() {
	switch (usb_req.bRequest) {
	case USB_REQ_GET_INTERFACE:
		// command = usb_standard_get_interface;
		usbd_control_buffer[0] = 1;
		datasize = 1;
		return USBD_REQ_HANDLED;
	case USB_REQ_SET_INTERFACE:
		datasize = 0;
		return USBD_REQ_HANDLED;
	case USB_REQ_GET_STATUS:
		datasize = 2;
		usbd_control_buffer[0] = 0;
		usbd_control_buffer[1] = 0;
		break;
	}
	return USBD_REQ_NOTSUPP;
}

enum usbd_request_return_codes _usbd_standard_request_endpoint() {
	switch (usb_req.bRequest) {
	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		if (usb_req.wValue == USB_FEAT_ENDPOINT_HALT)
			_ep_stall_set(usb_req.wIndex, usb_req.bRequest == USB_REQ_SET_FEATURE);
		else
			return USBD_REQ_NOTSUPP;
		return USBD_REQ_HANDLED;
	case USB_REQ_GET_STATUS:
		usbd_control_buffer[0] = _ep_stall_get(usb_req.wIndex) ? 1 : 0;
		usbd_control_buffer[1] = 0;
		datasize = 2;
		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

enum usbd_request_return_codes _usbd_standard_request() {
	if ((usb_req.bmRequestType & USB_REQ_TYPE_TYPE) != USB_REQ_TYPE_STANDARD)
		return USBD_REQ_NOTSUPP;

	switch (usb_req.bmRequestType & USB_REQ_TYPE_RECIPIENT) {
	case USB_REQ_TYPE_DEVICE:
		return _usbd_standard_request_device();
	case USB_REQ_TYPE_INTERFACE:
		return _usbd_standard_request_interface();
	case USB_REQ_TYPE_ENDPOINT:
		return _usbd_standard_request_endpoint();
	}
	return USBD_REQ_NOTSUPP;
}

static enum usbd_request_return_codes usb_control_request_dispatch() {
	// Filter out
	const uint8_t type = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
	const uint8_t mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;
	if ((usb_req.bmRequestType & mask) == type) {
		datasize = usb_req.wLength;
		int result = usbdfu_control_request(&usb_req, &datasize, &usb_complete_cb);
		if (result == USBD_REQ_HANDLED || result == USBD_REQ_NOTSUPP)
			return result;
	}

	#ifdef WINUSB_SUPPORT
	const uint8_t wtype = USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_DEVICE;
	const uint8_t wmask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;
	if ((usb_req.bmRequestType & wmask) == wtype && usb_req.bRequest == 0x41 /* A */) {
		// From https://github.com/pbatard/libwdi/wiki/WCID-Devices
		const uint8_t winusb_desc[] = {
			0x28, 0x00, 0x00, 0x00,       // Descriptor length (32bit word) (40 bytes)
			0x00, 0x01,                   // bcdVersion (1.0)
			0x04, 0x00,                   // wIndex = 0x0004 (Compat ID descriptor Index)
			0x01,                         // Num of sections (1)
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Reserved (7bytes)
			0x00,                         // interface num (0)
			0x01,                         // Reserved
			0x57, 0x49, 0x4E, 0x55, 0x53, 0x42, 0x00, 0x00, // compatibleID[8]    "WINUSB\0\0"
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // subCompatibleID[6] ""
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00  //Reserved
		};

		memcpy(usbd_control_buffer, winusb_desc, sizeof(winusb_desc));
		datasize = sizeof(winusb_desc);
		return USBD_REQ_HANDLED;
	}
	#endif

	/* Try standard request if not already handled. */
	return _usbd_standard_request();
}

static uint8_t _needs_zlp(uint16_t len, uint16_t wLength, uint8_t ep_size) {
	if (len < wLength) {
		if (len && (len % ep_size == 0)) {
			return 1;
		}
	}
	return 0;
}

static void _usb_control_setup_read() {
	unsigned maxdataout = usb_req.wLength;

	dataoff = 0; // Restart transmission counter
	if (usb_control_request_dispatch()) {
		if (datasize > maxdataout)  // Truncate output
			datasize = maxdataout;

		if (maxdataout) {
			usb_needs_zlp = _needs_zlp(datasize, maxdataout, dev_desc.bMaxPacketSize0);
			/* Go to data out stage if handled. */
			usb_control_send_chunk();
		} else {
			/* Go to status stage if handled. */
			_usbd_ep_write_packet(0, 0, 0);
			usb_fsm_state = STATUS_IN;
		}
	}
	else
		_stall_transaction();  // Stall endpoint on failure.
}

static void _usb_control_setup_write() {
	// Stall EP if we have too much data?
	if (usb_req.wLength > sizeof(usbd_control_buffer)) {
		_stall_transaction();
		return;
	}

	/* Buffer into which to write received data. */
	datasize = 0;
	/* Wait for DATA OUT stage. */
	if (usb_req.wLength > dev_desc.bMaxPacketSize0)
		usb_fsm_state = DATA_OUT;
	else
		usb_fsm_state = LAST_DATA_OUT;

	_usbd_ep_nak_set(0, 0);
}

static void _usbd_control_setup() {
	usb_complete_cb = 0;
	_usbd_ep_nak_set(0, 1);

	if (_usbd_ep_read_packet(0, &usb_req, sizeof(usb_req)) != sizeof(usb_req)) {
		_stall_transaction();
		return;
	}

	if ((usb_req.wLength == 0) || (usb_req.bmRequestType & 0x80))
		_usb_control_setup_read();
	else
		_usb_control_setup_write();
}

static void _usbd_control_out() {
	switch (usb_fsm_state) {
	case DATA_OUT:
		if (usb_control_recv_chunk() < 0)
			break;

		// Check for last packet
		if ((usb_req.wLength - datasize) <= dev_desc.bMaxPacketSize0)
			usb_fsm_state = LAST_DATA_OUT;
		break;
	case LAST_DATA_OUT:
		if (usb_control_recv_chunk() < 0)
			break;
		/*
		 * We have now received the full data payload.
		 * Invoke callback to process.
		 */
		if (usb_control_request_dispatch()) {
			/* Go to status stage on success. */
			_usbd_ep_write_packet(0, 0, 0);
			usb_fsm_state = STATUS_IN;
		} else
			_stall_transaction();
		break;
	case STATUS_OUT:
		_usbd_ep_read_packet(0, 0, 0);
		usb_fsm_state = IDLE;
		if (usb_complete_cb)
			usb_complete_cb(&usb_req);

		usb_complete_cb = 0;
		break;
	default:
		_stall_transaction();
	}
}

static void _usbd_control_in() {
	switch (usb_fsm_state) {
	case DATA_IN:
		usb_control_send_chunk();
		break;
	case LAST_DATA_IN:
		usb_fsm_state = STATUS_OUT;
		_usbd_ep_nak_set(0, 0);
		break;
	case STATUS_IN:
		if (usb_complete_cb)
			usb_complete_cb(&usb_req);

		/* Exception: Handle SET ADDRESS function here... */
		if ((usb_req.bmRequestType == 0) && (usb_req.bRequest == USB_REQ_SET_ADDRESS)) {
			/* Set device address and enable. */
			SET_REG(USB_DADDR_REG, (usb_req.wValue & USB_DADDR_ADDR) | USB_DADDR_EF);
		}
		usb_fsm_state = IDLE;
		break;
	default:
		_stall_transaction();
	}
}

void _set_ep_rx_bufsize(uint8_t ep, uint32_t size) {
	if (size > 62) {
		if (size & 0x1f) {
			size -= 32;
		}
		USB_SET_EP_RX_COUNT(ep, (size << 5) | 0x8000);
	} else {
		if (size & 1) {
			size++;
		}
		USB_SET_EP_RX_COUNT(ep, size << 10);
	}
}

void _usbd_ep_setup(uint8_t addr, uint8_t type, uint16_t max_size) {
	/* Translate USB standard type codes to STM32. */
	const uint16_t typelookup[] = {
		[USB_ENDPOINT_ATTR_CONTROL] = USB_EP_TYPE_CONTROL,
		[USB_ENDPOINT_ATTR_ISOCHRONOUS] = USB_EP_TYPE_ISO,
		[USB_ENDPOINT_ATTR_BULK] = USB_EP_TYPE_BULK,
		[USB_ENDPOINT_ATTR_INTERRUPT] = USB_EP_TYPE_INTERRUPT,
	};
	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	/* Assign address. */
	USB_SET_EP_ADDR(addr, addr);
	USB_SET_EP_TYPE(addr, typelookup[type]);

	if (dir || (addr == 0)) {
		USB_SET_EP_TX_ADDR(addr, usb_pm_top);
		USB_CLR_EP_TX_DTOG(addr);
		USB_SET_EP_TX_STAT(addr, USB_EP_TX_STAT_NAK);
		usb_pm_top += max_size;
	}

	if (!dir) {
		USB_SET_EP_RX_ADDR(addr, usb_pm_top);
		_set_ep_rx_bufsize(addr, max_size);
		USB_CLR_EP_RX_DTOG(addr);
		USB_SET_EP_RX_STAT(addr, USB_EP_RX_STAT_VALID);
		usb_pm_top += max_size;
	}
}

void do_usb_poll() {
	uint16_t istr = *USB_ISTR_REG;

	if (istr & USB_ISTR_RESET) {
		USB_CLR_ISTR_RESET();
		usb_pm_top = USBD_PM_TOP;

		_usbd_ep_setup(0, USB_ENDPOINT_ATTR_CONTROL, dev_desc.bMaxPacketSize0);
		// Set driver addr to zero
		SET_REG(USB_DADDR_REG, 0 | USB_DADDR_EF);

		return;
	}

	if (istr & USB_ISTR_CTR) {
		uint8_t ep = istr & USB_ISTR_EP_ID;
		if (istr & USB_ISTR_DIR) {
			if (*USB_EP_REG(ep) & USB_EP_SETUP)
				_usbd_control_setup();
			else
				_usbd_control_out();
		} else {
			USB_CLR_EP_TX_CTR(ep);
			_usbd_control_in();
		}
	}

	if (istr & USB_ISTR_SUSP)
		USB_CLR_ISTR_SUSP();

	if (istr & USB_ISTR_WKUP)
		USB_CLR_ISTR_WKUP();

	if (istr & USB_ISTR_SOF)
		USB_CLR_ISTR_SOF();

	*USB_CNTR_REG &= ~USB_CNTR_SOFM;
}


