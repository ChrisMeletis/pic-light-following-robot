
        PROCESSOR 16F876A
        INCLUDE <P16F876A.INC>

        __CONFIG _XT_OSC & _WDT_OFF & _PWRTE_OFF & _CP_OFF & _LVP_OFF & _BODEN_OFF

ANVAL   EQU 0x20        ;Save analog values register
CNT0    EQU 0x21        ;COUNTER IN BANK 00
CNT1    EQU 0xA1        ;COUNTER IN BANK 01


;---------------------------------------------
;	HC-SR04 Configuration and constants
;---------------------------------------------
	#define	HCSR04_TRIG_TRIS			TRISC		; Trigger Port and Pin
	#define HCSR04_TRIG_PORT			PORTC
	#define HCSR04_TRIG_PIN				0

	#define	HCSR04_ECHO_TRIS			TRISC		; Echo Port and Pin
	#define HCSR04_ECHO_PORT			PORTC		; (Must be CCP1 input pin)
	#define HCSR04_ECHO_PIN				2

	#define HCSR04_STATUS_Error			0			; Definitions of bit positions
	#define	HCSR04_STATUS_OutOfRange	1			; for HCSR_STATUS register
	#define HCSR04_STATUS_RisingEdge	2


;---------------------------------------------
;Chassis constants
;---------------------------------------------
	#define chassis_TRIS				TRISB
	#define chassis_PORT				PORTB
	#define chassis_left_motor_en		0
	#define chassis_right_motor_en		3
	#define LM_in						1
	#define LM_gnd						2
	#define RM_in						4
	#define RM_gnd						5

;---------------------------------------------
;Memory Variables
;---------------------------------------------
	cblock	0x22

        ;Double Analog Read variables
        DR_left
        DR_right

        ;Delay variables
        delay1
        delay2
        delay3

		;HC-SR04 Memory Fields
		HCSR04_WIDTH_L				; Pulse width registers
		HCSR04_WIDTH_H

	endc

        ORG 0x00
INIT:
        BSF STATUS,5   ;SELECT BANK 01

        ;PORTA IS SET TO INPUT MODE
        MOVLW B'11000111'
        MOVWF TRISA

        ;PORTB IS SET TO OUTPUT MODE
        MOVLW B'00000000'
        MOVWF TRISB

        ;PORTC IS SET TO OUTPUT MODE
        movlw b'00000100'
        movwf TRISC

        BCF STATUS,5 ;SELECT BANK 00


;--------------------------------
;Main programm
;--------------------------------

MAIN:
;-----Read analog inputs-----
;-----Read first analog input-------
        CALL ADC_INIT_0 ;Initialize ADC module, read photoresistor, store its value

        ;OUTPUT ANVAL TO DRLeft
        MOVF ANVAL,W
        MOVWF DR_left

;-----Read second analog input-----
        CALL ADC_INIT_1

        MOVF ANVAL, W
        MOVWF DR_right
;-----Continue to motor driving-----
        call Compare
;-----Seek for obstacles--------
        call Ultrasonic_Range_Finder
        goto MAIN


;----------------------------------------------------------------------------
;Double Analog Read
;----------------------------------------------------------------------------
;This part of teh programm reads the two photoresistors values and then
;store them into 2 data_registers for later use.
;
;   Analog values are stored in a 10bit result and so it is crusial
;   to change that result into 8bit (loosing the 2 LSBs)
;----------------------------------------------------------------------------


;-----Analog to Digital Convert INIT PROC-----
ADC_INIT_0:
        BSF STATUS,5 ;SELECT BANK 01

        ;RIGHT JUSTIFIED RESULT, +VREF=VDD, -VREF=VSS, AN0-AN7=ANALOG
        MOVLW B'10000000'
        MOVWF ADCON1

        BCF STATUS,5   ;SELECT BANK 00

        ;FOSC/8(bit7-6) use 5MHz conversion,USE AN0(bit5-3), ADON=1(bit0)
        MOVLW B'01000001';;;;;;;;;;;;;
        MOVWF ADCON0

;-----ADC READ PROC-----
ADC_READ_0:
        BSF ADCON0,2   ;START CONVERTION PROCESS (WE SET THE GO BIT)
WAIT:
        BTFSC ADCON0,2
        GOTO WAIT       ;WAIT FOR CONVERTION TO FINISH (WAIT FOR GO BIT TO CLEAR)

        ;;;WE SAVE RESULT INTO AN 8BIT REGISTER TO USE FOR OUTPUT (WE DROP THE TWO LESS SIGNIFICANT BITS);;;

        ;FIRST WE PROCESS THE HIGH BYTE OF RESULT
        MOVLW 0x06
        MOVWF CNT0
AGAIN_00:
        BCF STATUS,C   ;WE WANT SHIFT (NOT ROTATE), SO WE CLEAR CARRY
        RLF ADRESH,1   ;SHIFT LEFT 6 BITS A/D RESULT HIGH BYTE
        DECFSZ CNT0
        GOTO AGAIN_00

        MOVF ADRESH,W  ;MOVE ADRESH TO W
        MOVWF ANVAL     ;MOVE W TO ANVAL


        ;NOW WE PROCESS THE LOW BYTE OF RESULT
        BSF STATUS,5    ;SELECT BANK 01
        MOVLW 0x02
        MOVWF CNT1
AGAIN_01:
        BCF STATUS,C    ;WE WANT SHIFT (NOT ROTATE), SO WE CLEAR CARRY
        RRF ADRESL,1    ;SHIFT RIGHT 2 BITS A/D RESULT LOW BYTE
        DECFSZ CNT1
        GOTO AGAIN_01

        MOVF ADRESL,W   ;MOVE ADRESL TO W
        BCF STATUS,5   ;SELECT BANK 00
        ADDWF ANVAL,1   ;ADD W TO ANVAL


        RETURN

;---SECOND ANALOG INPUT READ---
;;;;ADC INIT PROC;;;;
ADC_INIT_1:
        BSF STATUS,5   ;SELECT BANK 01

        ;RIGHT JUSTIFIED RESULT, +VREF=VDD, -VREF=VSS, AN0-AN7=ANALOG
        MOVLW B'10000000'
        MOVWF ADCON1

        BCF STATUS,5   ;SELECT BANK 00

        ;FOSC/8(bit7-6),USE AN1(bit5-3), ADON=1(bit0)
        MOVLW B'01001001';;;;;;;;;;;;;
        MOVWF ADCON0

;;;;ADC READ PROC;;;;
ADC_READ_1:
        BSF ADCON0,2   ;START CONVERTION PROCESS (WE SET THE GO BIT)
WAIT1:
        BTFSC ADCON0,2
        GOTO WAIT1     ;WAIT FOR CONVERTION TO FINISH (WAIT FOR GO BIT TO CLEAR)

        ;;;WE SAVE RESULT INTO AN 8BIT REGISTER TO USE FOR OUTPUT (WE DROP THE TWO LESS SIGNIFICANT BITS);;;

        ;FIRST WE PROCESS THE HIGH BYTE OF RESULT
        MOVLW 0x06
        MOVWF CNT0
AGAIN_10:
        BCF STATUS,C   ;WE WANT SHIFT (NOT ROTATE), SO WE CLEAR CARRY
        RLF ADRESH,1   ;SHIFT LEFT 6 BITS A/D RESULT HIGH BYTE
        DECFSZ CNT0
        GOTO AGAIN_10

        MOVF ADRESH,W  ;MOVE ADRESH TO W
        MOVWF ANVAL     ;MOVE W TO ANVAL


        ;NOW WE PROCESS THE LOW BYTE OF RESULT
        BSF STATUS,5   ;SELECT BANK 01
        MOVLW 0x02
        MOVWF CNT1
AGAIN_11:
        BCF STATUS,C   ;WE WANT SHIFT (NOT ROTATE), SO WE CLEAR CARRY
        RRF ADRESL,1   ;SHIFT RIGHT 2 BITS A/D RESULT LOW BYTE
        DECFSZ CNT1
        GOTO AGAIN_11

        MOVF ADRESL,W  ;MOVE ADRESL TO W
        BCF STATUS,5   ;SELECT BANK 00
        ADDWF ANVAL,1  ;ADD W TO ANVAL
        RETURN

;-------------------------------------------------------------
;Compare
;-------------------------------------------------------------
;This Part of the Programm determines weather the car will go
;front, left or right.
;
;   Because commands such as compare are missing in pic
;   microcontrollers another technique must be used for
;   comparing 2 values, here we use the substraction technique.
;-------------------------------------------------------------
Compare:
    clrf PORTB  ;We disable all motors to be sure no collisions occure

    movfw DR_left       ;Load 1st value into W register
    subwf DR_right,w    ;Then subtract it from the 2nd value and store the result in W
    btfsc STATUS,Z      ;If equal Z flag is set
    goto Equal_Forward  ;Then go Forward
    btfsc STATUS,C      ;Else if Right>Left then the carry flag is set
    goto LG_Left        ;IF C=0 skip this instruction
    goto RG_Right
Ret_Comp_End:
    RETURN

Equal_Forward:
    bsf chassis_PORT,chassis_right_motor_en ;Rigth Motor Enable
    bsf chassis_PORT,RM_in                  ;Right motor front
    bcf chassis_PORT,RM_gnd
    bsf chassis_PORT,chassis_left_motor_en  ;Left Motor Enable
    bsf chassis_PORT,LM_in                  ;Left motor front
    bcf chassis_PORT,LM_gnd
    goto Ret_Comp_End                       ;Return to main programm
RG_Right:
    bsf chassis_PORT,chassis_right_motor_en ;Rigth Motor Enable
    bsf chassis_PORT,RM_in                  ;Right motor front
    bcf chassis_PORT,RM_gnd
    bsf chassis_PORT,chassis_left_motor_en  ;Left Motor Enable
    bcf chassis_PORT,LM_in                  ;Left motor Halt
    bcf chassis_PORT,LM_gnd
    goto Ret_Comp_End                       ;Return to main programm
LG_Left:
    bsf chassis_PORT,chassis_right_motor_en ;Rigth Motor Enable
    bcf chassis_PORT,RM_in                  ;Right motor Halt
    bcf chassis_PORT,RM_gnd
    bsf chassis_PORT,chassis_left_motor_en  ;Left Motor Enable
    bsf chassis_PORT,LM_in                  ;Left motor front
    bcf chassis_PORT,LM_gnd
    goto Ret_Comp_End                       ;Return to main programm

;----------------------------------------------------------------------------
;Ultrasonic Range Finder
;----------------------------------------------------------------------------
;The following programm detects whether an object is about 20centimeters from
;the car and if distance<20cm then it stops, reverses ant changes direction.
;
;   In order to measure the distance a 10uSecond pulse called "Trigger"
;   is provided to the Ultrasonic Sensor, then it emmits a 40kHz burst
;   And waits for its Echo thus giving back an "Echo" pulse with delay period
;   proportional to the time taken for the Echo to arrive.
;----------------------------------------------------------------------------
Ultrasonic_Range_Finder:
HCSR04_TriggerSensor:;10us delay
	bsf		HCSR04_TRIG_PORT,HCSR04_TRIG_PIN			; Set the pin high
    call Delay_10u                                      ;Wait 10uS
	bcf		HCSR04_TRIG_PORT,HCSR04_TRIG_PIN			; Set the pin low

HCSR04_CaptureEchoTransition:
    bcf		T1CON, TMR1ON		;Stop the timer
	clrf	TMR1H				;Clear timer
    clrf	TMR1L
    movlw	(0 << T1CKPS0)		;Timer Setup (1:1 prescale)
Wait_High:
    btfss HCSR04_ECHO_TRIS,HCSR04_ECHO_PIN
    goto Wait_High              ;Wait for Echo to start
    bsf	T1CON, TMR1ON			;Echo is set -> Start timer
Wait_Low:
    btfsc HCSR04_ECHO_TRIS,HCSR04_ECHO_PIN
    goto Wait_Low               ;Wait for Echo to stop while counting
    bcf	T1CON, TMR1ON			;Ehco is clear -> Stop the timer
;-------------------------------------------------------
;When distanceis about 20 timer high register is 3 to 4 (binary)
;and so if distance< ~20cm then call Stop, Compare works as before.
;-------------------------------------------------------
Compute_Distance:               ;Chech to see if the distance is <20cm
    movf TMR1H,w
    movwf HCSR04_WIDTH_H
    movlw b'00000101'
    subwf HCSR04_WIDTH_H,w
    btfsc STATUS,C
    goto S_Ret
STOP:
    clrf PORTB                              ;Stop all motors and wait
    call DELAY
    call DELAY
    call DELAY

    bsf chassis_PORT,chassis_right_motor_en ;Rigth Motor Enable
    bcf chassis_PORT,RM_in                  ;Right motor back
    bsf chassis_PORT,RM_gnd
    bsf chassis_PORT,chassis_left_motor_en  ;Left Motor Enable
    bcf chassis_PORT,LM_in                  ;Left motor back
    bsf chassis_PORT,LM_gnd

    call DELAY
    call DELAY
    call DELAY

    movfw DR_left
    subwf DR_right,w
    btfss STATUS,C
    goto S_Left
S_Right:bsf chassis_PORT,chassis_right_motor_en ;Rigth Motor Enable
    bcf chassis_PORT,RM_in                  ;Right motor back
    bsf chassis_PORT,RM_gnd
    bcf chassis_PORT,chassis_left_motor_en  ;Left Motor Disable
    bcf chassis_PORT,LM_in
    bcf chassis_PORT,LM_gnd
    goto S_Ret
S_Left:bcf chassis_PORT,chassis_right_motor_en ;Rigth Motor Disable
    bcf chassis_PORT,RM_in
    bcf chassis_PORT,RM_gnd
    bsf chassis_PORT,chassis_left_motor_en  ;Left Motor Enable
    bcf chassis_PORT,LM_in                  ;Left motor back
    bsf chassis_PORT,LM_gnd
S_Ret:
    call DELAY
    call DELAY
    call DELAY
    Return

;-----------------------------------------------
;Following part of the programm are delays used
;Delays are actualy autorepeating subroutines, we
;set value into a temporary register and decrese
;it with each repeat. When a register is empty (0)
;the routine is broken and programm continues
;-----------------------------------------------
DELAY:
    movlw 0x50
    movwf delay1
    movwf delay2
    movwf delay3
loop:
    ;decfsz delay1,f
    ;goto $+4
    decfsz delay2,f
    goto $+2
    decfsz delay3,f
    goto loop
    return
;----------------------------------------------
;Because of the speed it takes 10uS to pass no
;registers are needed here.
;>Call takes 4 cycles to be done.
;>Each goto $+1 goes to next line and takes 2 cycles.
;>Each cycle is 1uS
;----------------------------------------------
Delay_10u:
    goto	$+1
	goto	$+1
	goto	$+1
    return

        END
;------*Other comments*---------------
;*> When a motor is enabled creates sparks
;   and may interfere with the microcontroler
;   or peripherial units outside of it, this
;   time it interferes whith Echo through the
;   sensors' microphone maybe. Thats why we
;   use "Echo" pin only when needed.
;-------------------------------------



