@ DO NOT EDIT
.syntax unified
.text
.global ASM_Main
.thumb_func

@ DO NOT EDIT
vectors:
    .word 0x20002000
    .word ASM_Main + 1

@ DO NOT EDIT label ASM_Main
ASM_Main:

    @ Setup GPIO clocks and modes
    LDR R0, RCC_BASE         @ Enable clock for GPIOA and B
    LDR R1, [R0, #0x14]
    LDR R2, AHBENR_GPIOAB
    ORRS R1, R1, R2
    STR R1, [R0, #0x14]

    LDR R0, GPIOA_BASE       @ Enable pull-up resistors for pushbuttons
    MOVS R1, #0b01010101     @ sets the pull-up resistors for pins [0, 2, 4, 6]
    STR R1, [R0, #0x0C]      @ stores the value
    LDR R1, GPIOB_BASE       @ Set pins connected to LEDs to outputs
    LDR R2, MODER_OUTPUT
    STR R2, [R1, #0]

    MOVS R2, #0              @ Initialize R2 to 0 (LEDs should start at 0)

main_loop:
    @ Read GPIOA to check the state of the switches
    LDR R0, GPIOA_BASE
    LDR R3, [R0, #0x10]      @ R3 now holds the input state of switches

    @ Check if SW2 (bit 2) is pressed
    MOVS R4, #0x04           @ Load immediate value 0x04 into R4
    ANDS R4, R3              @ AND R4 with R3
    CMP R4, #0               @ Compare result to 0
    BNE check_sw3            @ If SW2 is not pressed, check SW3

    @ If SW2 is pressed, set R2 to 0xAA
    MOVS R2, #0xAA           @ Set R2 to the pattern 0xAA
    B delay_check             @ Skip to delay check

check_sw3:
    @ Check if SW3 (bit 3) is pressed
    MOVS R4, #0x08           @ Load immediate value 0x08 into R4
    ANDS R4, R3
    CMP R4, #0
    BNE check_sw0            @ If SW3 is not pressed, check SW0

    @ If SW3 is pressed, just skip the incrementing and continue to next loop
    B delay_check             @ Skip to delay check

check_sw0:
    @ Check if SW0 (bit 0) is pressed
    MOVS R4, #0x01
    ANDS R4, R3
    CMP R4, #0
    BEQ inc_by_1             @ If SW0 is not pressed, increment by 1

    @ If SW0 is pressed, increment by 2
    ADDS R2, R2, #2           @ Increment R2 by 2
    B check_sw1              @ Move to check SW1

inc_by_1:
    ADDS R2, R2, #1           @ Increment R2 by 1

inc_by_2:
	ADDS R2, R2, #2			   @ Increment R2 by 2

check_sw1:
    @ Check if SW1 (bit 1) is pressed
    MOVS R4, #0x02
    ANDS R4, R3
    CMP R4, #0
    BNE long_delay            @ If SW1 is not pressed, branch to long delay

    @ If SW1 is pressed, change to short delay
    LDR R0, SHORT_DELAY_CNT   @ Use short delay for SW1 pressed
    B delay_loop

long_delay:
    @ Implement a delay for 0.7 seconds
    LDR R0, LONG_DELAY_CNT

delay_loop:
    SUBS R0, R0, #1
    BNE delay_loop

delay_check:
    STR R2, [R1, #0x14]      @ Write the LED value to the output
    B main_loop              @ Repeat the main loop

@ LITERALS; DO NOT EDIT
.align
RCC_BASE:             .word 0x40021000
AHBENR_GPIOAB:       .word 0b1100000000000000000
GPIOA_BASE:          .word 0x48000000
GPIOB_BASE:          .word 0x48000400
MODER_OUTPUT:        .word 0x5555

@ TODO: Add your own values for these delays
LONG_DELAY_CNT:      .word 0x1E8480    @ Adjust value for approximately 0.7 seconds
SHORT_DELAY_CNT:     .word 0x0F4240    @ Adjust value for approximately 0.3 seconds
