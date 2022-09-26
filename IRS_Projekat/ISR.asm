;-------------------------------------------------------------------------------
; Interrupt service routines for timers A0, B0 and PORT1 (SW3 & SW4)
;
;-------------------------------------------------------------------------------
            .cdecls C,LIST,"msp430.h"       ; Include device header file
;-------------------------------------------------------------------------------
			.ref	WriteLed				; reference the extern function
			.ref	disp1					; reference the global variables
			.ref	disp2
			.ref	current_digit
			.ref	button
;-------------------------------------------------------------------------------
			.text
; Timer B0 routine, used for multiplexing 7seg display

TB0CCR0ISR	push.w	R12						; save R12 value on stack
			cmp.b	#0x01, &current_digit   ; check current digit
			jz 		display2
display1:	bis.b	#BIT4, &P6OUT			; turn off disp2
			mov.b	&disp1, R12				; move disp1 nubmer to R12
			bic.b	#BIT0, &P7OUT 			; turn on disp1
			jmp 	display
display2:	bis.b	#BIT0, &P7OUT 			; turn off disp1
			mov.b	&disp2, R12				; move disp2 nubmer to R12
			bic.b	#BIT4, &P6OUT 			; turn on disp2
display:	call	#WriteLed				; call function WriteLed with argument in R12
			xor.b	#0x01, &current_digit	; change selected display
			pop.w	R12						; return R12 original value
			reti							; back to the main loop
;-------------------------------------------------------------------------------
; PORT1 (SW3 & SW4) routine, starts debouncing timer

PORT1ISR	bis.w	#MC__UP, &TA0CTL		; start timer A0
			bic.b	#BIT4, &P1IFG			; clear Interrupts
			bic.b	#BIT5, &P1IFG
			reti
;-------------------------------------------------------------------------------
; Timer A0 routine, used for debouncing of SW3 & SW4
TA0CCR0ISR	bit.b	#BIT4, &P1IN			; check which button is pressed
			jnz		sw2						; if its not SW1, test SW2
			mov.b	#0x01, &button			; save button pressed value
			jmp 	exit
sw2:		bit.b	#BIT5, &P1IN			; check if sw2 is still pressed
			jnz		exit					; if not, exit
			mov.b	#0x02, &button			; save button pressed value
exit:		bic.w	#MC__UP, &TA0CTL		; disable timer
			reti

;-------------------------------------------------------------------------------
; Interrupt Vectors
;-------------------------------------------------------------------------------
            .sect   ".int59"                ; MSP430 TIMER0_B0 Interrupt Vector
            .short  TB0CCR0ISR

            .sect	".int47"				; MSP430 PORT1 Interrupt Vector
            .short	PORT1ISR

            .sect	".int53"				; MSP430 TIMER0_A0 Interrupt Vector
            .short	TA0CCR0ISR

