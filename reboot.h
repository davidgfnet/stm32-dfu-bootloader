
#ifndef __REBOOT__HH__
#define __REBOOT__HH__

// Points to the bottom of the stack, we should have 8 bytes free there
extern uint32_t _stack;

// Reboots the system into the bootloader, making sure
// it enters in DFU mode.
static inline void reboot_into_bootloader() {
	uint64_t * ptr = (uint64_t*)&_stack;
	*ptr = 0xDEADBEEFCC00FFEEULL;
}

// Reboots into user app (non-DFU) but doesn't perform any security locks
static inline void reboot_into_updater() {
	uint64_t * ptr = (uint64_t*)&_stack;
	*ptr = 0xDEADBEEF600DF00DULL;
}

// Clears reboot information so we reboot in "normal" mode
static inline void clear_reboot_flags() {
	uint64_t * ptr = (uint64_t*)&_stack;
	*ptr = 0;
}

// Returns whether we were rebooted into DFU mode
static inline int rebooted_into_dfu() {
	uint64_t * ptr = (uint64_t*)&_stack;
	return (*ptr == 0xDEADBEEFCC00FFEEULL);
}

// Returns whether we were rebooted into an updater app
static inline int rebooted_into_updater() {
	uint64_t * ptr = (uint64_t*)&_stack;
	return (*ptr == 0xDEADBEEF600DF00DULL);
}

#endif


