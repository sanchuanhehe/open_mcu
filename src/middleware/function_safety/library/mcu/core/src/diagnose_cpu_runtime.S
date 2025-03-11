# File Name  : diagnose_cpu_runtime.S
# Description    : Full RISC-V CPU test during run-time
# Input          : None.
# Output         : None.
# Return         : SUCCESS (=0) if test is ok

.section .text
.align 2

.global CPU_DiagnoseRuntime
.type CPU_DiagnoseRuntime,@function

#ifdef __riscv64
#define LREG ld
#define SREG sd
#define REGBYTES 8
#else
#define LREG lw
#define SREG sw
#define REGBYTES 4
#endif

#define TOTAL_INT_SIZE_ON_STACK  (20 * REGBYTES)

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

CPU_DiagnoseRuntime:
    push_reg

    # Register a0, a1 (holds value returned by the function)
    li a0, 0xAAAAAAAA
    li a1, 0xAAAAAAAA
    bne a0, a1, FailReturnLabel
    li a0, 0x55555555
    li a1, 0x55555555
    bne a0, a1, FailReturnLabel

    # Register a2
    li a0, 0xAAAAAAAA
    li a2, 0xAAAAAAAA
    bne a0, a2, FailReturnLabel
    li a0, 0x55555555
    li a2, 0x55555555
    bne a0, a2, FailReturnLabel

    # Register a3
    li a0, 0xAAAAAAAA
    li a3, 0xAAAAAAAA
    bne a0, a3, FailReturnLabel
    li a0, 0x55555555
    li a3, 0x55555555
    bne a0, a3, FailReturnLabel

    # Register a4
    li a0, 0xAAAAAAAA
    li a4, 0xAAAAAAAA
    bne a0, a4, FailReturnLabel
    li a0, 0x55555555
    li a4, 0x55555555
    bne a0, a4, FailReturnLabel

    # Register a5
    li a0, 0xAAAAAAAA
    li a5, 0xAAAAAAAA
    bne a0, a5, FailReturnLabel
    li a0, 0x55555555
    li a5, 0x55555555
    bne a0, a5, FailReturnLabel

    # Register a6
    li a0, 0xAAAAAAAA
    li a6, 0xAAAAAAAA
    bne a0, a6, FailReturnLabel
    li a0, 0x55555555
    li a6, 0x55555555
    bne a0, a6, FailReturnLabel

    # Register a7
    li a0, 0xAAAAAAAA
    li a7, 0xAAAAAAAA
    bne a0, a7, FailReturnLabel
    li a0, 0x55555555
    li a7, 0x55555555
    bne a0, a7, FailReturnLabel

    # Ramp pattern verification
    li a0, 0x00000000
    li a1, 0x00000001
    li a2, 0x00000002
    li a3, 0x00000003
    li a4, 0x00000004
    li a5, 0x00000005
    li a6, 0x00000006
    li a7, 0x00000007
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