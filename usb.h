
#include <stdint.h>
#include <stdlib.h>

// Basic defs
#define MMIO32(addr)       (*(volatile uint32_t *)(addr))
#define PERIPH_BASE        (0x40000000U)
#define PERIPH_BASE_APB1   (PERIPH_BASE + 0x00000)
#define USB_DEV_FS_BASE    (PERIPH_BASE_APB1 + 0x5c00)
#define USB_PMA_BASE       (PERIPH_BASE_APB1 + 0x6000)

// DFU definitions

enum dfu_req {
	DFU_DETACH,
	DFU_DNLOAD,
	DFU_UPLOAD,
	DFU_GETSTATUS,
	DFU_CLRSTATUS,
	DFU_GETSTATE,
	DFU_ABORT,
};

enum dfu_status {
	DFU_STATUS_OK,
	DFU_STATUS_ERR_TARGET,
	DFU_STATUS_ERR_FILE,
	DFU_STATUS_ERR_WRITE,
	DFU_STATUS_ERR_ERASE,
	DFU_STATUS_ERR_CHECK_ERASED,
	DFU_STATUS_ERR_PROG,
	DFU_STATUS_ERR_VERIFY,
	DFU_STATUS_ERR_ADDRESS,
	DFU_STATUS_ERR_NOTDONE,
	DFU_STATUS_ERR_FIRMWARE,
	DFU_STATUS_ERR_VENDOR,
	DFU_STATUS_ERR_USBR,
	DFU_STATUS_ERR_POR,
	DFU_STATUS_ERR_UNKNOWN,
	DFU_STATUS_ERR_STALLEDPKT,
};

enum dfu_state {
	STATE_APP_IDLE,
	STATE_APP_DETACH,
	STATE_DFU_IDLE,
	STATE_DFU_DNLOAD_SYNC,
	STATE_DFU_DNBUSY,
	STATE_DFU_DNLOAD_IDLE,
	STATE_DFU_MANIFEST_SYNC,
	STATE_DFU_MANIFEST,
	STATE_DFU_MANIFEST_WAIT_RESET,
	STATE_DFU_UPLOAD_IDLE,
	STATE_DFU_ERROR,
};

#define DFU_FUNCTIONAL			0x21
struct usb_dfu_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bmAttributes;
#define USB_DFU_CAN_DOWNLOAD		0x01
#define USB_DFU_CAN_UPLOAD		0x02
#define USB_DFU_MANIFEST_TOLERANT	0x04
#define USB_DFU_WILL_DETACH		0x08

	uint16_t wDetachTimeout;
	uint16_t wTransferSize;
	uint16_t bcdDFUVersion;
} __attribute__((packed));


/* USB Standard Device Descriptor - Table 9-8 */
struct usb_device_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __attribute__((packed));
#define USB_DT_DEVICE_SIZE sizeof(struct usb_device_descriptor)

/* USB Standard Configuration Descriptor - Table 9-10 */
struct usb_config_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __attribute__((packed));
#define USB_DT_CONFIGURATION_SIZE       9

struct usb_interface_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __attribute__((packed));
#define USB_DT_INTERFACE_SIZE           9

/* USB Standard Endpoint Descriptor - Table 9-13 */
struct usb_endpoint_descriptor {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __attribute__((packed));
#define USB_DT_ENDPOINT_SIZE        7

enum usbd_request_return_codes {
	USBD_REQ_NOTSUPP	= 0,
	USBD_REQ_HANDLED	= 1,
	USBD_REQ_NEXT_CALLBACK	= 2,
};

/* Table 9-15 specifies String Descriptor Zero.
 * Table 9-16 specified UNICODE String Descriptor.
 */
struct usb_string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t wData[];
} __attribute__((packed));


/* USB Descriptor Types - Table 9-5 */
#define USB_DT_DEVICE				1
#define USB_DT_CONFIGURATION			2
#define USB_DT_STRING				3
#define USB_DT_INTERFACE			4
#define USB_DT_ENDPOINT				5
#define USB_DT_DEVICE_QUALIFIER			6
#define USB_DT_OTHER_SPEED_CONFIGURATION	7
#define USB_DT_INTERFACE_POWER			8


/* USB Setup Data structure - Table 9-2 */
struct usb_setup_data {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
} __attribute__((packed));

/* Class Definition */
#define USB_CLASS_VENDOR			0xFF

/* bmRequestType bit definitions */
/* bit 7 : direction */
#define USB_REQ_TYPE_DIRECTION			0x80
#define USB_REQ_TYPE_IN				0x80
/* bits 6..5 : type */
#define USB_REQ_TYPE_TYPE			0x60
#define USB_REQ_TYPE_STANDARD			0x00
#define USB_REQ_TYPE_CLASS			0x20
#define USB_REQ_TYPE_VENDOR			0x40
/* bits 4..0 : recipient */
#define USB_REQ_TYPE_RECIPIENT			0x1F
#define USB_REQ_TYPE_DEVICE			0x00
#define USB_REQ_TYPE_INTERFACE			0x01
#define USB_REQ_TYPE_ENDPOINT			0x02
#define USB_REQ_TYPE_OTHER			0x03

/* USB Standard Request Codes - Table 9-4 */
#define USB_REQ_GET_STATUS			0
#define USB_REQ_CLEAR_FEATURE			1
/* Reserved for future use: 2 */
#define USB_REQ_SET_FEATURE			3
/* Reserved for future use: 3 */
#define USB_REQ_SET_ADDRESS			5
#define USB_REQ_GET_DESCRIPTOR			6
#define USB_REQ_SET_DESCRIPTOR			7
#define USB_REQ_GET_CONFIGURATION		8
#define USB_REQ_SET_CONFIGURATION		9
#define USB_REQ_GET_INTERFACE			10
#define USB_REQ_SET_INTERFACE			11
#define USB_REQ_SET_SYNCH_FRAME			12

/* USB Descriptor Types - Table 9-5 */
#define USB_DT_DEVICE				1
#define USB_DT_CONFIGURATION			2
#define USB_DT_STRING				3
#define USB_DT_INTERFACE			4
#define USB_DT_ENDPOINT				5
#define USB_DT_DEVICE_QUALIFIER			6
#define USB_DT_OTHER_SPEED_CONFIGURATION	7
#define USB_DT_INTERFACE_POWER			8
/* From ECNs */
#define USB_DT_OTG				9
#define USB_DT_DEBUG				10
#define USB_DT_INTERFACE_ASSOCIATION		11

/* USB Standard Feature Selectors - Table 9-6 */
#define USB_FEAT_ENDPOINT_HALT			0
#define USB_FEAT_DEVICE_REMOTE_WAKEUP		1
#define USB_FEAT_TEST_MODE			2

/* Information Returned by a GetStatus() Request to a Device - Figure 9-4 */
#define USB_DEV_STATUS_SELF_POWERED		0x01
#define USB_DEV_STATUS_REMOTE_WAKEUP		0x02

/* USB Endpoint Descriptor bmAttributes bit definitions - Table 9-13 */
/* bits 1..0 : transfer type */
#define USB_ENDPOINT_ATTR_CONTROL		0x00
#define USB_ENDPOINT_ATTR_ISOCHRONOUS		0x01
#define USB_ENDPOINT_ATTR_BULK			0x02
#define USB_ENDPOINT_ATTR_INTERRUPT		0x03
#define USB_ENDPOINT_ATTR_TYPE		0x03
/* bits 3..2 : Sync type (only if ISOCHRONOUS) */
#define USB_ENDPOINT_ATTR_NOSYNC		0x00
#define USB_ENDPOINT_ATTR_ASYNC			0x04
#define USB_ENDPOINT_ATTR_ADAPTIVE		0x08
#define USB_ENDPOINT_ATTR_SYNC			0x0C
#define USB_ENDPOINT_ATTR_SYNCTYPE		0x0C
/* bits 5..4 : usage type (only if ISOCHRONOUS) */
#define USB_ENDPOINT_ATTR_DATA			0x00
#define USB_ENDPOINT_ATTR_FEEDBACK		0x10
#define USB_ENDPOINT_ATTR_IMPLICIT_FEEDBACK_DATA 0x20
#define USB_ENDPOINT_ATTR_USAGETYPE		0x30

enum usb_language_id {
	USB_LANGID_ENGLISH_US = 0x409,
};

/* Get register content. */
#define GET_REG(REG)		((uint16_t) *(REG))

/* Set register content. */
#define SET_REG(REG, VAL)	(*(REG) = (uint16_t)(VAL))

/* Clear register bit. */
#define CLR_REG_BIT(REG, BIT)	SET_REG((REG), (~(BIT)))

/* Clear register bit masking out some bits that must not be touched. */
#define CLR_REG_BIT_MSK_AND_SET(REG, MSK, BIT, EXTRA_BITS) \
	SET_REG((REG), (GET_REG((REG)) & (MSK) & (~(BIT))) | (EXTRA_BITS))

#define CLR_REG_BIT_MSK(REG, MSK, BIT) \
		CLR_REG_BIT_MSK_AND_SET((REG), (MSK), (BIT), 0)

/* Get masked out bit value. */
#define GET_REG_BIT(REG, BIT)	(GET_REG(REG) & (BIT))

#define USB_DADDR_EF		(1 << 7)
#define USB_DADDR_ADDR		0x007F



/* --- USB general registers ----------------------------------------------- */

/* USB Control register */
#define USB_CNTR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x40))
/* USB Interrupt status register */
#define USB_ISTR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x44))
/* USB Frame number register */
#define USB_FNR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x48))
/* USB Device address register */
#define USB_DADDR_REG		(&MMIO32(USB_DEV_FS_BASE + 0x4C))
/* USB Buffer table address register */
#define USB_BTABLE_REG		(&MMIO32(USB_DEV_FS_BASE + 0x50))

/* USB EP register */
#define USB_EP_REG(EP)		(&MMIO32(USB_DEV_FS_BASE) + (EP))


/* --- USB control register masks / bits ----------------------------------- */

/* Interrupt mask bits, set to 1 to enable interrupt generation */
#define USB_CNTR_CTRM		0x8000
#define USB_CNTR_PMAOVRM	0x4000
#define USB_CNTR_ERRM		0x2000
#define USB_CNTR_WKUPM		0x1000
#define USB_CNTR_SUSPM		0x0800
#define USB_CNTR_RESETM		0x0400
#define USB_CNTR_SOFM		0x0200
#define USB_CNTR_ESOFM		0x0100

/* Request/Force bits */
#define USB_CNTR_RESUME		0x0010 /* Resume request */
#define USB_CNTR_FSUSP		0x0008 /* Force suspend */
#define USB_CNTR_LP_MODE	0x0004 /* Low-power mode */
#define USB_CNTR_PWDN		0x0002 /* Power down */
#define USB_CNTR_FRES		0x0001 /* Force reset */

/* --- USB interrupt status register masks / bits -------------------------- */

#define USB_ISTR_CTR		0x8000 /* Correct Transfer */
#define USB_ISTR_PMAOVR		0x4000 /* Packet Memory Area Over / Underrun */
#define USB_ISTR_ERR		0x2000 /* Error */
#define USB_ISTR_WKUP		0x1000 /* Wake up */
#define USB_ISTR_SUSP		0x0800 /* Suspend mode request */
#define USB_ISTR_RESET		0x0400 /* USB RESET request */
#define USB_ISTR_SOF		0x0200 /* Start Of Frame */
#define USB_ISTR_ESOF		0x0100 /* Expected Start Of Frame */
#define USB_ISTR_DIR		0x0010 /* Direction of transaction */
#define USB_ISTR_EP_ID		0x000F /* Endpoint Identifier */

/* --- USB interrupt status register manipulators -------------------------- */

/* Note: CTR is read only! */
#define USB_CLR_ISTR_PMAOVR()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_PMAOVR)
#define USB_CLR_ISTR_ERR()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_ERR)
#define USB_CLR_ISTR_WKUP()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_WKUP)
#define USB_CLR_ISTR_SUSP()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_SUSP)
#define USB_CLR_ISTR_RESET()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_RESET)
#define USB_CLR_ISTR_SOF()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_SOF)
#define USB_CLR_ISTR_ESOF()	CLR_REG_BIT(USB_ISTR_REG, USB_ISTR_ESOF)



/* --- USB endpoint register masks / bits ---------------------------------- */

/* Masks and toggle bits */
#define USB_EP_RX_CTR		0x8000 /* Correct transfer RX */
#define USB_EP_RX_DTOG		0x4000 /* Data toggle RX */
#define USB_EP_RX_STAT		0x3000 /* Endpoint status for RX */

#define USB_EP_SETUP		0x0800 /* Setup transaction completed */
#define USB_EP_TYPE		0x0600 /* Endpoint type */
#define USB_EP_KIND		0x0100 /* Endpoint kind.
				* When set and type=bulk    -> double buffer
				* When set and type=control -> status out
				*/

#define USB_EP_TX_CTR		0x0080 /* Correct transfer TX */
#define USB_EP_TX_DTOG		0x0040 /* Data toggle TX */
#define USB_EP_TX_STAT		0x0030 /* Endpoint status for TX */

#define USB_EP_ADDR		0x000F /* Endpoint Address */

/* Masking all toggle bits */
#define USB_EP_NTOGGLE_MSK	(USB_EP_RX_CTR | \
				 USB_EP_SETUP | \
				 USB_EP_TYPE | \
				 USB_EP_KIND | \
				 USB_EP_TX_CTR | \
				 USB_EP_ADDR)

/* All non toggle bits plus EP_RX toggle bits */
#define USB_EP_RX_STAT_TOG_MSK	(USB_EP_RX_STAT | USB_EP_NTOGGLE_MSK)
/* All non toggle bits plus EP_TX toggle bits */
#define USB_EP_TX_STAT_TOG_MSK	(USB_EP_TX_STAT | USB_EP_NTOGGLE_MSK)

/* Endpoint status bits for USB_EP_RX_STAT bit field */
#define USB_EP_RX_STAT_DISABLED	0x0000
#define USB_EP_RX_STAT_STALL	0x1000
#define USB_EP_RX_STAT_NAK	0x2000
#define USB_EP_RX_STAT_VALID	0x3000

/* Endpoint status bits for USB_EP_TX_STAT bit field */
#define USB_EP_TX_STAT_DISABLED	0x0000
#define USB_EP_TX_STAT_STALL	0x0010
#define USB_EP_TX_STAT_NAK	0x0020
#define USB_EP_TX_STAT_VALID	0x0030

/* Endpoint type bits for USB_EP_TYPE bit field */
#define USB_EP_TYPE_BULK	0x0000
#define USB_EP_TYPE_CONTROL	0x0200
#define USB_EP_TYPE_ISO		0x0400
#define USB_EP_TYPE_INTERRUPT	0x0600

/* --- USB BTABLE Registers ------------------------------------------------ */

#define USB_EP_TX_ADDR(EP) \
	((uint32_t *)(USB_PMA_BASE + (USB_GET_BTABLE + EP * 8 + 0) * 2))

#define USB_EP_TX_COUNT(EP) \
	((uint32_t *)(USB_PMA_BASE + (USB_GET_BTABLE + EP * 8 + 2) * 2))

#define USB_EP_RX_ADDR(EP) \
	((uint32_t *)(USB_PMA_BASE + (USB_GET_BTABLE + EP * 8 + 4) * 2))

#define USB_EP_RX_COUNT(EP) \
	((uint32_t *)(USB_PMA_BASE + (USB_GET_BTABLE + EP * 8 + 6) * 2))

/* --- USB BTABLE manipulators --------------------------------------------- */

#define USB_GET_EP_TX_BUFF(EP) \
	(USB_PMA_BASE + (uint8_t *)(USB_GET_EP_TX_ADDR(EP) * 2))

#define USB_GET_EP_RX_BUFF(EP) \
	(USB_PMA_BASE + (uint8_t *)(USB_GET_EP_RX_ADDR(EP) * 2))

/* --- USB BTABLE registers ------------------------------------------------ */

#define USB_GET_BTABLE		GET_REG(USB_BTABLE_REG)

/* --- USB BTABLE manipulators --------------------------------------------- */

#define USB_GET_EP_TX_ADDR(EP)		GET_REG(USB_EP_TX_ADDR(EP))
#define USB_GET_EP_TX_COUNT(EP)		GET_REG(USB_EP_TX_COUNT(EP))
#define USB_GET_EP_RX_ADDR(EP)		GET_REG(USB_EP_RX_ADDR(EP))
#define USB_GET_EP_RX_COUNT(EP)		GET_REG(USB_EP_RX_COUNT(EP))
#define USB_SET_EP_TX_ADDR(EP, ADDR)	SET_REG(USB_EP_TX_ADDR(EP), ADDR)
#define USB_SET_EP_TX_COUNT(EP, COUNT)	SET_REG(USB_EP_TX_COUNT(EP), COUNT)
#define USB_SET_EP_RX_ADDR(EP, ADDR)	SET_REG(USB_EP_RX_ADDR(EP), ADDR)
#define USB_SET_EP_RX_COUNT(EP, COUNT)	SET_REG(USB_EP_RX_COUNT(EP), COUNT)

#define USB_SET_EP_TYPE(EP, TYPE) \
	SET_REG(USB_EP_REG(EP), \
		(GET_REG(USB_EP_REG(EP)) & \
		(USB_EP_NTOGGLE_MSK & \
		(~USB_EP_TYPE))) | TYPE)

#define USB_SET_EP_ADDR(EP, ADDR) \
	SET_REG(USB_EP_REG(EP), \
		((GET_REG(USB_EP_REG(EP)) & \
		(USB_EP_NTOGGLE_MSK & \
		(~USB_EP_ADDR))) | ADDR))


/*
 * Set/reset a bit in a masked window by using toggle mechanism.
 *
 * This means that we look at the bits in the bit window designated by
 * the mask. If the bit in the masked window is not matching the
 * bit mask BIT then we write 1 and if the bit in the masked window is
 * matching the bit mask BIT we write 0.
 *
 * TODO: We may need a faster implementation of that one?
 */
#define TOG_SET_REG_BIT_MSK_AND_SET(REG, MSK, BIT, EXTRA_BITS)		\
do {									\
	register uint16_t toggle_mask = GET_REG(REG) & (MSK);		\
	toggle_mask ^= BIT;						\
	SET_REG((REG), toggle_mask | (EXTRA_BITS));			\
} while (0)

#define TOG_SET_REG_BIT_MSK(REG, MSK, BIT) \
	TOG_SET_REG_BIT_MSK_AND_SET((REG), (MSK), (BIT), 0)


/*
 * Set USB endpoint tx/rx status.
 *
 * USB status field is changed using an awkward toggle mechanism, that
 * is why we use some helper macros for that.
 */
#define USB_SET_EP_RX_STAT(EP, STAT) \
	TOG_SET_REG_BIT_MSK_AND_SET(USB_EP_REG(EP), \
		USB_EP_RX_STAT_TOG_MSK, STAT, USB_EP_RX_CTR | USB_EP_TX_CTR)

#define USB_SET_EP_TX_STAT(EP, STAT) \
	TOG_SET_REG_BIT_MSK_AND_SET(USB_EP_REG(EP), \
		USB_EP_TX_STAT_TOG_MSK, STAT, USB_EP_RX_CTR | USB_EP_TX_CTR)

/* Macros for clearing DTOG bits */
#define USB_CLR_EP_TX_DTOG(EP) \
	SET_REG(USB_EP_REG(EP), \
		GET_REG(USB_EP_REG(EP)) & \
		(USB_EP_NTOGGLE_MSK | USB_EP_TX_DTOG))

#define USB_CLR_EP_RX_DTOG(EP) \
	SET_REG(USB_EP_REG(EP), \
		GET_REG(USB_EP_REG(EP)) & \
		(USB_EP_NTOGGLE_MSK | USB_EP_RX_DTOG))



/*
 * Macros for clearing and setting USB endpoint register bits that do
 * not use the toggle mechanism.
 *
 * Because the register contains some bits that use the toggle
 * mechanism we need a helper macro here. Otherwise the code gets really messy.
 */
#define USB_CLR_EP_NTOGGLE_BIT_AND_SET(EP, BIT, EXTRA_BITS) \
	CLR_REG_BIT_MSK_AND_SET(USB_EP_REG(EP), \
		USB_EP_NTOGGLE_MSK, BIT, EXTRA_BITS)

#define USB_CLR_EP_RX_CTR(EP) \
	USB_CLR_EP_NTOGGLE_BIT_AND_SET(EP, USB_EP_RX_CTR, USB_EP_TX_CTR)

#define USB_CLR_EP_TX_CTR(EP) \
	USB_CLR_EP_NTOGGLE_BIT_AND_SET(EP, USB_EP_TX_CTR, USB_EP_RX_CTR)


#define USB_CLR_EP_RX_CTR(EP) \
	USB_CLR_EP_NTOGGLE_BIT_AND_SET(EP, USB_EP_RX_CTR, USB_EP_TX_CTR)

#define USB_CLR_EP_TX_CTR(EP) \
	USB_CLR_EP_NTOGGLE_BIT_AND_SET(EP, USB_EP_TX_CTR, USB_EP_RX_CTR)


// Exported API
#define DFU_TRANSFER_SIZE 1024
void usb_init();
void do_usb_poll();



