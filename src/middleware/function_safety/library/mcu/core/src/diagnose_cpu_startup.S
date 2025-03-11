# File Name  : diagnose_cpu_startup.S
# Description    : Full RISC-V CPU test at start-up
# Input          : None.
# Output         : None.
# Return         : SUCCESS (=0)
# WARNING        : all registers content destroyed when exiting this function

# Externally declare fail-safe routines to prevent unrecoverable failures


.section .text
.align 2

.global CPU_DiagnoseStartup
.type CPU_DiagnoseStartup,@function

#ifdef __riscv64
#define LREG ld
#define SREG sd
#define REGBYTES 8
#else
#define LREG lw
#define SREG sw
#define REGBYTES 4
#endif

#define TOTAL_INT_SIZE_ON_STACK  (30 * REGBYTES)

.macro push_reg
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)
    stmia {ra, t0-t2, a0-a7, t3-t6}, (sp)
    addi  sp, sp, -(TOTAL_INT_SIZE_ON_STACK)
.endm

.macro pop_reg
    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK
    ldmia {ra, t0-t2, a0-a7, t3-t6}, (sp)
    addi  sp, sp, TOTAL_INT_SIZE_ON_STACK
.endm

CPU_DiagnoseStartup:
    push_reg

    # Function test of a0, a1
    li a0, 0xAAAAAAAA
    li a1, 0xAAAAAAAA
    bne a0, a1, FailReturnLabel
    li a0, 0x55555555
    li a1, 0x55555555
    bne a0, a1, FailReturnLabel

    # Register a2
    li a2, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a2, FailReturnLabel
    li a2, 0x55555555
    li a0, 0x55555555
    bne a0, a2, FailReturnLabel

    # Register a3
    li a3, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a3, FailReturnLabel
    li a3, 0x55555555
    li a0, 0x55555555
    bne a0, a3, FailReturnLabel

    # Register a4
    li a4, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a4, FailReturnLabel
    li a4, 0x55555555
    li a0, 0x55555555
    bne a0, a4, FailReturnLabel

    # Register a5
    li a5, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a5, FailReturnLabel
    li a5, 0x55555555
    li a0, 0x55555555
    bne a0, a5, FailReturnLabel

    # Register a6
    li a6, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a6, FailReturnLabel
    li a6, 0x55555555
    li a0, 0x55555555
    bne a0, a6, FailReturnLabel

    # Register a7
    li a7, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, a7, FailReturnLabel
    li a7, 0x55555555
    li a0, 0x55555555
    bne a0, a7, FailReturnLabel

    # Register t0
    li t0, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t0, FailReturnLabel
    li t0, 0x55555555
    li a0, 0x55555555
    bne a0, t0, FailReturnLabel

    # Register t1
    li t1, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t1, FailReturnLabel
    li t1, 0x55555555
    li a0, 0x55555555
    bne a0, t1, FailReturnLabel

    # Register t2
    li t2, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t2, FailReturnLabel
    li t2, 0x55555555
    li a0, 0x55555555
    bne a0, t2, FailReturnLabel

    # Register t3
    li t3, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t3, FailReturnLabel
    li t3, 0x55555555
    li a0, 0x55555555
    bne a0, t3, FailReturnLabel

    # Register t4
    li t4, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t4, FailReturnLabel
    li t4, 0x55555555
    li a0, 0x55555555
    bne a0, t4, FailReturnLabel

    # Register t5
    li t5, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t5, FailReturnLabel
    li t5, 0x55555555
    li a0, 0x55555555
    bne a0, t5, FailReturnLabel

    # Register t6
    li t6, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, t6, FailReturnLabel
    li t6, 0x55555555
    li a0, 0x55555555
    bne a0, t6, FailReturnLabel

    # Ramp pattern verification
    li a0, 0x00000000
    li a1, 0x00000001
    li a2, 0x00000002
    li a3, 0x00000003
    li a4, 0x00000004
    li a5, 0x00000005
    li a6, 0x00000006
    li a7, 0x00000007
    li t0, 0x00000008
    li t1, 0x00000009
    li t2, 0x0000000A
    li t3, 0x0000000B
    li t4, 0x0000000C
    li t5, 0x0000000D
    li t6, 0x0000000E

    xori a0, a0, 0x00000000
    bnez a0, FailReturnLabel
    xori a1, a1, 0x00000001
    bnez a1, FailReturnLabel
    xori a2, a2, 0x00000002
    bnez a2, FailReturnLabel
    xori a3, a3, 0x00000003
    bnez a3, FailReturnLabel
    xori a4, a4, 0x00000004
    bnez a4, FailReturnLabel
    xori a5, a5, 0x00000005
    bnez a5, FailReturnLabel
    xori a6, a6, 0x00000006
    bnez a6, FailReturnLabel
    xori a7, a7, 0x00000007
    bnez a7, FailReturnLabel
    xori t0, t0, 0x00000008
    bnez t0, FailReturnLabel
    xori t1, t1, 0x00000009
    bnez t1, FailReturnLabel
    xori t2, t2, 0x0000000A
    bnez t2, FailReturnLabel
    xori t3, t3, 0x0000000B
    bnez t3, FailReturnLabel
    xori t4, t4, 0x0000000C
    bnez t4, FailReturnLabel
    xori t5, t5, 0x0000000D
    bnez t5, FailReturnLabel
    xori t6, t6, 0x0000000E
    bnez t6, FailReturnLabel

    # Process return address register (ra)
    mv a1, ra # The return address must be saved
    li ra, 0xAAAAAAAA
    li a0, 0xAAAAAAAA
    bne a0, ra, FailReturnLabel
    li ra, 0x55555555
    li a0, 0x55555555
    bne a0, ra, FailReturnLabel
    mv ra, a1

    # Process thread pointer (tp)
    mv a0, tp # Save process stack value.
    li a1, 0xAAAAAAA8
    mv a1, tp
    mv a2, tp
    bne a1, a2, FailReturnLabel
    li a1, 0x55555554
    mv a1, tp
    mv a2, tp
    bne a1, a2, FailReturnLabel
    mv tp, a0 # Restore process stack value

    # Process global pointer (gp).
    mv a0, gp # Save process stack value.
    li a1, 0xAAAAAAA8
    mv a1, gp
    mv a2, gp
    bne a1, a2, FailReturnLabel
    li a1, 0x55555554
    mv a1, gp
    mv a2, gp
    bne a1, a2, FailReturnLabel
    mv gp, a0 # Restore process stack value

SuccessReturnLabel:
    pop_reg
    # CPU test success
    li a0, 0x0
    ret

FailReturnLabel:
    pop_reg
    # CPU test fail
    li a0, 0x1
    ret