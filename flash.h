
// Flashing routines //

#define FLASH_CR_LOCK  (1 << 7)
#define FLASH_CR_STRT  (1 << 6)
#define FLASH_CR_OPTER (1 << 5)
#define FLASH_CR_OPTPG (1 << 4)
#define FLASH_CR_PER   (1 << 1)
#define FLASH_CR_PG   (1 << 0)
#define FLASH_SR_BSY   (1 << 0)
#define FLASH_KEYR    (*(volatile uint32_t*)0x40022004U)
#define FLASH_OPTKEYR (*(volatile uint32_t*)0x40022008U)
#define FLASH_SR      (*(volatile uint32_t*)0x4002200CU)
#define FLASH_CR      (*(volatile uint32_t*)0x40022010U)
#define FLASH_AR      (*(volatile uint32_t*)0x40022014U)

static void _flash_unlock(int opt) {
	// Clear the unlock state.
	FLASH_CR |= FLASH_CR_LOCK;

	// Authorize the FPEC access.
	FLASH_KEYR = 0x45670123U;
	FLASH_KEYR = 0xcdef89abU;

	if (opt) {
		// F1 uses same keys for flash and option
		FLASH_OPTKEYR = 0x45670123U;
		FLASH_OPTKEYR = 0xcdef89abU;
	}
}

static void _flash_lock() {
	FLASH_CR |= FLASH_CR_LOCK;
}

#define _flash_wait_for_last_operation() \
	while (FLASH_SR & FLASH_SR_BSY);

static void _flash_erase_page(uint32_t page_address) {
	_flash_wait_for_last_operation();

	FLASH_CR |= FLASH_CR_PER;
	FLASH_AR = page_address;
	FLASH_CR |= FLASH_CR_STRT;

	_flash_wait_for_last_operation();

	FLASH_CR &= ~FLASH_CR_PER;
}

static int _flash_page_is_erased(uint32_t addr) {
	volatile uint32_t *_ptr32 = (uint32_t*)addr;
	for (unsigned i = 0; i < 1024/sizeof(uint32_t); i++)
		if (_ptr32[i] != 0xffffffffU)
			return 0;
	return 1;
}

static void _flash_program_buffer(uint32_t address, uint16_t *data, unsigned len) {
	_flash_wait_for_last_operation();

	// Enable programming
	FLASH_CR |= FLASH_CR_PG;

	volatile uint16_t *addr_ptr = (uint16_t*)address;
	for (unsigned i = 0; i < len/2; i++) {
		addr_ptr[i] = data[i];
		_flash_wait_for_last_operation();
	}

	// Disable programming
	FLASH_CR &= ~FLASH_CR_PG;
}

#ifdef ENABLE_PROTECTIONS
static void _flash_erase_option_bytes() {
	_flash_wait_for_last_operation();

	FLASH_CR |= FLASH_CR_OPTER;
	FLASH_CR |= FLASH_CR_STRT;

	_flash_wait_for_last_operation();

	FLASH_CR &= ~FLASH_CR_OPTER;
}

static void _flash_program_option_bytes(uint32_t address, uint16_t data) {
	_flash_wait_for_last_operation();

	FLASH_CR |= FLASH_CR_OPTPG;  // Enable option byte programming.
	volatile uint16_t *addr_ptr = (uint16_t*)address;
	*addr_ptr = data;
	_flash_wait_for_last_operation();
	FLASH_CR &= ~FLASH_CR_OPTPG;  // Disable option byte programming.
}
#endif

#ifdef ENABLE_SAFEWRITE
static void check_do_erase() {
	// For protection reasons, we do not allow reading the flash using DFU
	// and also we make sure to wipe the entire flash on an ERASE/WRITE command
	// just to guarantee that nobody is able to extract the data by flashing a
	// stub and executing it.

	static int erased = 0;
	if (erased) return;

	/* Change usb_strings accordingly */
	const uint32_t start_addr = 0x08000000 + (FLASH_BOOTLDR_SIZE_KB*1024);
	const uint32_t end_addr   = 0x08000000 + (        FLASH_SIZE_KB*1024);
	for (uint32_t addr = start_addr; addr < end_addr; addr += 1024)
		if (!_flash_page_is_erased(addr))
			_flash_erase_page(addr);

	erased = 1;
}
#endif


