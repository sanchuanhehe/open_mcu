# File Name  : diagnose_ram_march.S
# Description    : Full RISC-V CPU test full ram at start-up or 
#                  test ram for classB parameters during run-time
# Input          : None.
# Output         : None.
# Return         : SUCCESS (=0) if test is ok

.section .text
.align 2

.global RAM_DiagnoseFullRange
.type RAM_DiagnoseFullRange,@function
.global RAM_DiagnoseStepRange
.type RAM_DiagnoseStepRange,@function

# tables with offsets of physical order of address in RAM
.extern g_standRamOrder
.extern g_scrambleRamOrder

# RAM Test Control
# #define USE_MARCHX_TEST

#ifdef __riscv64
    #define LREG ld
    #define SREG sd
    #define REGBYTES 8
#else
    #define LREG lw
    #define SREG sw
    #define REGBYTES 4
#endif

/* ------------------------------------RAM_DiagnoseFullRange--------------------------------- */
# Function Name  : RAM_DiagnoseFullRange
# Description    : Full RAM MarchC test for start-up
# Input          : a0 .. RAM begin (first address to check), 
#                  a1 .. RAM end (last address to check)
#                  a2 .. Background pattern
# Return         : SUCCESS (=1)
# WARNING        : All the RAM area including stack is destroyed during this test
RAM_DiagnoseFullRange:
    not a3, a2
#ifdef DIAGNOSE_RAM_SCRAMBLER
    la a4, g_scrambleRamOrder # setup pointer to physical order of the addresses (a4)
#else
    la a4, g_standRamOrder
#endif

/* Save ram address register value */
    mv t0, a0
    mv t1, a1
    add t2, a4, 4

# *** Step 1 ***
# Write background pattern with addresses increasing
__FULL1_LOOP:
    SREG a2, (a0)
    addi a0, a0, 4
    blt a0, a1, __FULL1_LOOP

# *** Step 2 ***
# Verify background and write inverted background with addresses increasing
    mv a0, t0
    mv a4, t2
    add t3, t2, 16
__FULL2_LOOP:
    __CHECK2_LOOP:
        LREG t4, (a4)
        addi a4, a4, 4
        add t5, a0, t4
        LREG t6, (t5)
        bne t6, a2, __FULL_ERR
        SREG a3, (t5)
        blt a4, t3, __CHECK2_LOOP
        # restore register
        addi a0, a0, 16
        mv a4, t2
    blt a0, a1, __FULL2_LOOP

# *** Step 3 ***
# Verify inverted background and write background with addresses increasing  
    mv a0, t0
    mv a4, t2
    add t3, t2, 16
__FULL3_LOOP:
    __CHECK3_LOOP:
        LREG t4, (a4)
        addi a4, a4, 4
        add t5, a0, t4
        LREG t6, (t5)
        bne t6, a3, __FULL_ERR
        SREG a2, (t5)
        blt a4, t3, __CHECK3_LOOP
        # restore register
        addi a0, a0, 16
        mv a4, t2
    blt a0, a1, __FULL3_LOOP

# *** Step 4 ***
# Verify background and write inverted background with addresses decreasing
    mv a0, t0
    add a1, t1, -16 # Include ram end address
    mv a4, t2
    add t3, t2, 16
__FULL4_LOOP:
    __CHECK4_LOOP:
        LREG t4, (a4)
        addi a4, a4, 4
        add t5, a1, t4
        LREG t6, (t5)
        bne t6, a2, __FULL_ERR
        SREG a3, (t5)
        blt a4, t3, __CHECK4_LOOP
        # restore register
        addi a1, a1, -16
        mv a4, t2
    bge a1, a0, __FULL4_LOOP

# *** Step 5 ***
# Verify inverted background and write background with addresses decreasing
    mv a0, t0
    add a1, t1, -16 # Include ram end address
    mv a4, t2
    add t3, t2, 16
__FULL5_LOOP:
    __CHECK5_LOOP:
        LREG t4, (a4)
        addi a4, a4, 4
        add t5, a1, t4
        LREG t6, (t5)
        bne t6, a3, __FULL_ERR
        SREG a2, (t5)
        blt a4, t3, __CHECK5_LOOP
        # restore register
        addi a1, a1, -16
        mv a4, t2
    bge a1, a0, __FULL5_LOOP

# *** Step 6 ***
# Verify background with addresses increasing
    mv a0, t0
    mv a1, t1
__FULL6_LOOP:
    LREG t3, (a0)
    addi a0, a0, 4
    bne t3, a2, __FULL_ERR
    blt a0, a1, __FULL6_LOOP

__FULL_MARCH_RET:
    li  a0, 0       # Correct return
    jal   __FULL_RET

__FULL_ERR:
    li  a0, 1       # error result

__FULL_RET:
    ret           # return to the caller



/* ------------------------------------RAM_DiagnoseStepRange--------------------------------- */
# Function Name  : RAM_DiagnoseStepRange
# Description    : Transparent RAM MarchC-/March X test for run time
# Input          : a0 .. RAM begin (first address to test), 
#                  a1 .. Buffer begin (First address of backup buffer)
#                  a2 .. Background pattern
# Return         : SUCCESS (=1)
# WARNING -  The RAM area under test is out of original content during this test!
#            Neighbour addresses (first-1 or -2 and last+1) are tested, too.
# Compilation parameters : ARTISAN - changes order of the sequence of tested
#                                   addresses to respect their physical order
#                 USE_MARCHX_TEST - Skip step 3 and 4 of March C- to make the test
#                                   shorter and faster overall
RAM_DiagnoseStepRange:
#ifdef ARTISAN
    la a4, g_scrambleRamOrder # setup pointer to physical order of the addresses (a4)
#else
    la a4, g_standRamOrder
#endif

    not a3, a2 # setup inverted background pattern (a3)

    # Save address register value
    mv t0, a1 # save buffer start address
    mv t1, a4 # save offset start address

    # Runtime RAM buffer test
    beq a0, a1, __BUFF_TEST # a0 == a1 is buffer test

# ***************** test of the RAM slice *********************
    mv a1, t0
    addi t2, t0, 20 # save 6 word
  __SAVE_LOOP:
    LREG t3, (a4) # load data offset
    addi a4, a4, 4
    add  t4, a0, t3
    LREG t5, (t4)
    SREG t5, (a1)
    addi a1, a1, 4
    ble  a1, t2, __SAVE_LOOP

# *** Step 1 ***
# Write background pattern with addresses increasing
  mv a4, t1 # restore offset start address
  addi t2, t1, 20
__STEP1_LOOP:
  LREG t3, (a4) # load data offset
  addi a4, a4, 4
  add t4, a0, t3
  SREG a2, (t4)  # store background pattern
  ble a4, t2, __STEP1_LOOP

# *** Step 2 ***
# Verify background and write inverted background with addresses increasing
  mv a4, t1
  addi t2, t1, 20
__STEP2_LOOP:
  LREG t3, (a4)
  addi a4, a4, 4
  add t4, a0, t3
  LREG t5, (t4)
  bne t5, a2, __STEP_ERR # verify background pattern
  SREG a3, (t4) # store inverted background pattern
  ble a4, t2, __STEP2_LOOP

#ifdef USE_MARCHX_TEST
#else
# *** Step 3 *** (not used at March-X test)
# Verify inverted background and write background with addresses increasing  
  mv a4, t1
  addi t2, t1, 20
__STEP3_LOOP:
  LREG t3, (a4)
  addi a4, a4, 4
  add t4, a0, t3
  LREG t5, (t4)
  bne t5, a3, __STEP_ERR
  SREG a2, (t4)
  ble a4, t2, __STEP3_LOOP

# *** Step 4 *** (not used at March-X test)
# Verify background and write inverted background with addresses decreasing  
  add a4, t1, 20
  mv t2, t1
__STEP4_LOOP:
  LREG t3, (a4)
  addi a4, a4, -4
  add t4, a0, t3
  LREG t5, (t4) 
  bne  t5, a2, __STEP_ERR # verify background pattern
  SREG a3, (t4) # store inverted background pattrern
  bge  a4, t2, __STEP4_LOOP
#endif

# *** Step 5 ***
# Verify inverted background and write background with addresses decreasing 
  add a4, t1, 20
  mv t2, t1
__STEP5_LOOP:
  LREG t3, (a4)
  addi a4, a4, -4
  add t4, a0, t3
  LREG t5, (t4)
  bne  t5, a3, __STEP_ERR
  SREG a2, (t4)
  bge  a4, t2, __STEP5_LOOP

# *** Step 6 ***
# Verify background with addresses increasing
  mv a4, t1
  add t2, t1, 20
__STEP6_LOOP:
  LREG t3, (a4)
  add a4, a4, 4
  add t4, a0, t3
  LREG t5, (t4)
  bne  t5, a2, __STEP_ERR # verify background pattern
  ble  a4, t2, __STEP6_LOOP

/* --------------------------- Recovery ram data ----------------------- */
  mv a4, t1
  mv a1, t0
  add t2, t0, 20
__RESTORE_LOOP:
  LREG t3, (a4) # load data offset
  add a4, a4, 4
  add t4, a0, t3
  LREG t5, (a1)
  addi a1, a1, 4
  SREG t5, (t4) # load data from RAM buffer
  ble a1, t2, __RESTORE_LOOP
  j __MARCH_RET

/* --------------------------- BUFFER Test  ----------------------- */
# ************** test of the buffer itself ********************
__BUFF_TEST:
# *** Step 1 *** 
# Write background pattern with addresses increasing
  add a4, t1, 4
  add t2, t1, 32
__BUFF1_LOOP:
  LREG t3, (a4) # load data offset  The test starts from 0 and the array index is 1.
  addi a4, a4, 4
  add t4, a1, t3
  SREG a2, (t4) # store background pattern
  ble a4, t2, __BUFF1_LOOP

# *** Step 2 ***
# Verify background and write inverted background with addresses increasing
  add a4, t1, 4
  add t2, t1, 32
__BUFF2_LOOP:
  LREG t3, (a4)
  addi a4, a4, 4
  add t4, a1, t3
  LREG t5, (t4)
  bne t5, a2, __STEP_ERR
  SREG a3, (t4)
  ble a4, t2, __BUFF2_LOOP

#ifdef USE_MARCHX_TEST
#else
# *** Step 3 *** (not used at March-X test)
# Verify inverted background and write background with addresses increasing
  add a4, t1, 4
  add t2, t1, 32
__BUFF3_LOOP:
  LREG t3, (a4)
  addi a4, a4, 4
  add t4, a1, t3
  LREG t5, (t4)
  bne t5, a3, __STEP_ERR
  SREG a2, (t4)
  ble a4, t2, __BUFF3_LOOP

# *** Step 4 *** (not used at March-X test)
# Verify background and write inverted background with addresses decreasing
  add a4, t1, 32
  add t2, t1, 4
__BUFF4_LOOP:
  LREG t3, (a4)
  add a4, a4, -4
  add t4, a1, t3
  LREG t5, (t4)
  bne t5, a2, __STEP_ERR
  SREG a3, (t4)
  bge a4, t2, __BUFF4_LOOP
#endif

# *** Step 5 ***
# Verify inverted background and write background with addresses decreasing 
  add a4, t1, 32
  add t2, t1, 4
__BUFF5_LOOP:
  LREG t3, (a4)
  add a4, a4, -4
  add t4, a1, t3
  LREG t5, (t4)
  bne t5, a3, __STEP_ERR
  SREG a2, (t4)
  bge a4, t2, __BUFF5_LOOP

# *** Step 6 ***
# Verify background with addresses increasing
  add a4, t1, 4
  add t2, t1, 32
__BUFF6_LOOP:
  LREG t3, (a4)
  addi a4, a4, 4
  add t4, a1, t3
  LREG t5, (t4)
  bne t5, a2, __STEP_ERR
  ble a4, t2, __BUFF6_LOOP

__MARCH_RET:
  li  a0, 0       # Correct return
  j   __STEP_RET

__STEP_ERR:
  li  a0, 1 
  ret
#  li  a0, 1       # error result

__STEP_RET:
  ret           # return to the caller