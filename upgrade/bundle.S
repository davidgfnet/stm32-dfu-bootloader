
.global vector_table
.global reset_handler
.global _start

#define BASE_RAM  0x20000000
#define VTOR_RELO 0xE000ED08
#define MAX_PAYLOAD_SIZE (10*1024)

# Vectors for the ROM user app

.text
.section .vectors
vector_table:
.long 0x20000000           // Stack pointer must point to RAM
.long _start               // Reset vector
.long _dummyhandler        // NMI
.long _dummyhandler        // hard-fault
.long _dummyhandler        // mem-fault
.long _dummyhandler        // bus-fault
.long _dummyhandler        // usage-fault
.long 0x00000000           // reserved
.long 0x00000000           // reserved
.long 0x00000000           // reserved
.long 0x00000000           // reserved
.long _dummyhandler        // svc
.long _dummyhandler        // debug
.long 0x00000000           // reserved
.long _dummyhandler        // pend sv
.long _dummyhandler        // systick


.text
.thumb

# This simple code copies the payload (at the end of the file) into RAM
# According to stm32f103-ram.ld the payload starts at the begining of the RAM
# and takes less than 10KB, followed by data/bss sections and the stack.
# It relocates the VTOR and sets up the stack before jumping to execute the
# (now in RAM) payload.

.thumb_func
_start:
reset_handler:
	# Copy the payload into RAM. Always copies 10KB and potentially unused data
	adr r0, _payload
	ldr r1, =MAX_PAYLOAD_SIZE
	ldr r2, =BASE_RAM
	1:
		ldmia r0!, {r3, r4, r5, r6}
		stmia r2!, {r3, r4, r5, r6}
		sub r1, #16
		bgt 1b

	# Setup VTOR table (at base ram)
	ldr r2, =BASE_RAM
	ldr r0, =VTOR_RELO
	str r2, [r0]

	# Load stack and start executing the RAM image
	ldr r1, [r2, #0]
	msr MSP, r1
	ldr r1, [r2, #4]
	bx r1
	nop

.thumb_func
_dummyhandler:
	b _dummyhandler
	nop

.pool

.align 4
_payload:
	.incbin "payload.bin"
_payload_end:

