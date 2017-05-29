#include <linux/linkage.h>

.arch_extension sec

ENTRY(set_secondary_entry)
	stmfd   sp!, {r1-r11, lr}
	mov r1, r0
	ldr r0, =1
	mrc p15, 0, r3, c1, c0, 0
	mov r4, r3
	and r3, #0xbfffffff
	mcr p15, 0, r3, c1, c0, 0
	smc #0
	mcr p15, 0, r4, c1, c0, 0
	ldmfd   sp!, {r1-r11, pc}
ENDPROC(set_secondary_entry)
