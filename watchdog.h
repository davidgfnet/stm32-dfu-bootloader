
// Independent watchdog interface

#define IWDG_PVU 1
#define IWDG_RVU 2

#ifdef ENABLE_WATCHDOG

#define IWDG_KR   *((volatile uint32_t*)0x40003000U)
#define IWDG_PR   *((volatile uint32_t*)0x40003004U)
#define IWDG_RLR  *((volatile uint32_t*)0x40003008U)
#define IWDG_SR   *((volatile uint32_t*)0x4000300CU)

#define IWDG_RSTF (1 << 29)
#define LSI_RDY   (1 <<  1)
#define LSI_ON    (1 <<  0)
#define RCC_CSR   (*(volatile uint32_t*)0x40021024U)

// Enables the watchdog using a period of 1/(40kHz / 256 / 4095) = 26.2s
static void enable_iwdg(uint16_t rldval) {
	// First start LSI oscillator
	RCC_CSR |= LSI_ON;
	while (RCC_CSR & LSI_RDY);

	while (IWDG_SR & IWDG_PVU);
	IWDG_KR  = 0x5555;  // Unlock PR/RLR
	IWDG_PR  = 7;       // 256 prescaler

	while (IWDG_SR & IWDG_RVU);
	IWDG_KR  = 0x5555;  // Unlock PR/RLR
	IWDG_RLR = rldval;  // 4095 reload value

	// Starts the watchdog
	IWDG_KR  = 0xcccc;
}

static int reset_due_to_watchdog() {
	return (RCC_CSR & IWDG_RSTF);
}

#endif

