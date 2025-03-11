/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * armboot - Startup Code for STM32MP1xx of HiSparkTrace
 */
#define ASM_NL		 ;
#define SYMBOL_NAME_STR(X)	#X
#define SYMBOL_NAME(X)		X
#ifdef __STDC__
#define SYMBOL_NAME_LABEL(X)	X##:
#else
#define SYMBOL_NAME_LABEL(X)	X:
#endif
#define ALIGN			.align 2

#define LENTRY(name) \
	ALIGN ASM_NL \
	SYMBOL_NAME_LABEL(name)

#define ENTRY(name) \
	.globl SYMBOL_NAME(name) ASM_NL \
	LENTRY(name)

#define END(name) \
	.size name, .-name

#define ENDPROC(name) \
	.type name STT_FUNC ASM_NL \
	END(name)

/*************************************************************************
 *
 * Startup Code (reset vector)
 *
 * Do important init only if we don't start from memory!
 * Setup memory and board specific bits prior to relocation.
 * Relocate armboot to ram. Setup stack.
 *
 *************************************************************************/
    .section   .text.entry

	.globl	_start
	.globl	save_boot_params_ret
	.extern InterruptEntry
	.type   save_boot_params_ret,%function
	.global raise

_start:
    /* 创建中断向量表 */
    ldr pc, =Reset_Handler      /* 复位中断 */
    ldr pc, =Undefined_Handler  /* 未定义指令中断 */
    ldr pc, =SVC_Handler        /* Supervisor中断 */
    ldr pc, =PrefAbort_Handler  /* 预取终止中断 */
    ldr pc, =DataAbort_Handler  /* 数据终止中断 */
    ldr pc, =NotUsed_Handler    /* 未使用中断 */
    ldr pc, =IRQ_Handler        /* IRQ中断 */
    ldr pc, =FIQ_Handler        /* FIQ中断 */

Reset_Handler:
	/* Allow the board to save important registers */
	b	save_boot_params

/* 未定义指令中断函数 */
Undefined_Handler:
    b .

/* Supervisor中断服务函数 */
SVC_Handler:
    b .

/* 预取终止中断服务函数 */
PrefAbort_Handler:
    b .

/* 数据终止中断服务函数 */
DataAbort_Handler:
    b .

/* 未使用的中断服务函数 */
NotUsed_Handler:
    b .

/* FIQ中断服务函数 */
FIQ_Handler:
    b .

/* IRQ中断服务函数 */
IRQ_Handler:
    push {lr}
	push {r0-r3, r12}

	mrs  r0, spsr
	push {r0}

	mov r1, #0xA0000000
	add r1, r1, #0x20000
	add r1, r1, #0x2000  // 见RM0442_STM32MP153参考手册 P154, GICC(CIC CPU Interface)
	ldr r0, [r1, #0xC]    // 读CICC_IAR,获取中断ID
	push {r0, r1}
	cps #0x13            //进入SVC模式,允许中断嵌套
	push {lr}            //保存SVC模式的lr
	ldr r2, =InterruptEntry  // C中断程序入口，函数入参为中断id，保存在r0中
	cpsie i
	blx r2
	cpsid i
	pop {lr}
	cps #0x12
	pop {r0, r1}
	str r0, [r1, #0x10]

	pop {r0}
	msr spsr_cxsf, r0

	pop {r0-r3, r12}
	pop {lr}
	subs pc, lr, #4  //设置PC指针

save_boot_params_ret:
	/*
	 * disable interrupts (FIQ and IRQ), also set the cpu to SVC32 mode,
	 * except if in HYP mode already
	 */
	mrs	r0, cpsr
	and	r1, r0, #0x1f		@ mask mode bits
	teq	r1, #0x1a		    @ test for HYP mode
	bicne	r0, r0, #0x1f	@ clear all mode bits
	orrne	r0, r0, #0x13	@ set SVC mode
	orr	r0, r0, #0xc0		@ disable FIQ and IRQ
	msr	cpsr,r0

/*
 * Setup vector:
 */
	/* Set V=0 in CP15 SCTLR register - for VBAR to point to vector */
	mrc	p15, 0, r0, c1, c0, 0	@ Read CP15 SCTLR Register
	bic	r0, #(1<<13)		@ V = 0
	mcr	p15, 0, r0, c1, c0, 0	@ Write CP15 SCTLR Register

	/* Set vector address in CP15 VBAR register */
	ldr	r0, =_start
	mcr	p15, 0, r0, c12, c0, 0	@Set VBAR

	/* the mask ROM code should have PLL and others stable */
	bl	cpu_init_cp15

	bl	_main

raise:
	b .


/*************************************************************************
 *
 * void save_boot_params(u32 r0, u32 r1, u32 r2, u32 r3)
 *	__attribute__((weak));
 *
 * Stack pointer is not yet initialized at this moment
 * Don't save anything to stack even if compiled with -O0
 *
 *************************************************************************/
ENTRY(save_boot_params)
	b	save_boot_params_ret		@ back to my caller
ENDPROC(save_boot_params)

/*************************************************************************
 *
 * cpu_init_cp15
 *
 * Setup CP15 registers (cache, MMU, TLBs). The I-cache is turned on unless
 * CONFIG_SYS_ICACHE_OFF is defined.
 *
 *************************************************************************/
ENTRY(cpu_init_cp15)
	/*
	 * Invalidate L1 I/D
	 */
	mov	r0, #0			@ set up for MCR
	mcr	p15, 0, r0, c8, c7, 0	@ invalidate TLBs
	mcr	p15, 0, r0, c7, c5, 0	@ invalidate icache
	mcr	p15, 0, r0, c7, c5, 6	@ invalidate BP array
	mcr     p15, 0, r0, c7, c10, 4	@ DSB
	mcr     p15, 0, r0, c7, c5, 4	@ ISB

	/*
	 * disable MMU stuff and caches
	 */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #0x00002000	@ clear bits 13 (--V-)
	bic	r0, r0, #0x00000007	@ clear bits 2:0 (-CAM)
	orr	r0, r0, #0x00000002	@ set bit 1 (--A-) Align
	orr	r0, r0, #0x00000800	@ set bit 11 (Z---) BTB
	orr	r0, r0, #0x00001000	@ set bit 12 (I) I-cache
	mcr	p15, 0, r0, c1, c0, 0

	mov	r5, lr			@ Store my Caller
	mrc	p15, 0, r1, c0, c0, 0	@ r1 has Read Main ID Register (MIDR)
	mov	r3, r1, lsr #20		@ get variant field
	and	r3, r3, #0xf		@ r3 has CPU variant
	and	r4, r1, #0xf		@ r4 has CPU revision
	mov	r2, r3, lsl #4		@ shift variant field for combined value
	orr	r2, r4, r2		@ r2 has combined CPU variant + revision

	ldr	r0, =(STACK_TOP)
	bic	r0, r0, #7	/* 8-byte alignment for ABI compliance */
	mov	sp, r0
	mov	pc, r5			@ back to my caller
ENDPROC(cpu_init_cp15)

ENTRY(data_copy)
	ldr r4, =(__data_load)
	ldr r5, =(__data_start)
	ldr r6, =(__data_end)
copy_loop:
	ldr r7, [r4]
	str r7, [r5]
	add r4, r4, #4
	add r5, r5, #4
	cmp r5, r6
	blt copy_loop
	mov	pc, lr
ENDPROC(data_copy)

ENTRY(bss_clear)
	ldr r4, =(__bss_start)
	ldr r5, =(__bss_end)
	mov r6, #0
clear_loop:
	str r6, [r4]
	add r4, r4, #4
	cmp r4, r5
	blt clear_loop
	mov	pc, lr
ENDPROC(bss_clear)

ENTRY(_main)
    bl data_copy
	bl bss_clear
	bl main

ENDPROC(_main)
