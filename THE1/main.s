PROCESSOR 18F8722

#include <xc.inc>

; CONFIGURATION (DO NOT EDIT)
; CONFIG1H
CONFIG OSC = HSPLL      ; Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
CONFIG FCMEN = OFF      ; Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
CONFIG IESO = OFF       ; Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)
; CONFIG2L
CONFIG PWRT = OFF       ; Power-up Timer Enable bit (PWRT disabled)
CONFIG BOREN = OFF      ; Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
; CONFIG2H
CONFIG WDT = OFF        ; Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
; CONFIG3H
CONFIG LPT1OSC = OFF    ; Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
CONFIG MCLRE = ON       ; MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
; CONFIG4L
CONFIG LVP = OFF        ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
CONFIG XINST = OFF      ; Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
CONFIG DEBUG = OFF      ; Disable In-Circuit Debugger


GLOBAL var1
GLOBAL var2
GLOBAL var3
GLOBAL result
GLOBAL counter
GLOBAL counter1
GLOBAL blink_var
GLOBAL re0_press
GLOBAL re0_release
GLOBAL re1_release
GLOBAL re1_press
GLOBAL portc_var
GLOBAL portb_var


; Define space for the variables in RAM
PSECT udata_acs
var1:
    DS 1 ; Allocate 1 byte for var1
var2:
    DS 1
var3:
    DS 1
portc_var:
    DS 1
portb_var:
    DS 1
counter:
    DS 1
counter1:
    DS 1
blink_var:
    DS 1
re1_press:
    DS 1
re1_release:
    DS 1
re0_press:
    DS 1
re0_release:
    DS 1
temp_result:
    DS 1   
result: 
    DS 1


PSECT resetVec,class=CODE,reloc=2
resetVec:
    goto       main

PSECT CODE
main:
    clrf var1	; var1 = 0		
    clrf result ; result = 0
    clrf var2
    clrf var3
    clrf blink_var
    clrf counter
    clrf counter1
    clrf re0_press
    clrf re0_release
    clrf re1_release
    clrf re1_press
    clrf portc_var
    clrf portb_var
    ; PORTB
    ; LATB
    ; TRISB determines whether the port is input/output
    ; set output ports
    clrf TRISB
    clrf TRISC
    clrf TRISD
    setf TRISE ; PORTE is input
    
    setf PORTB
    setf LATC ; light up all pins in PORTC
    setf LATD
    
    call busy_wait
    
    clrf PORTB
    clrf LATC ; light up all pins in PORTC
    clrf LATD
main_loop:
    ; Round robin
    call check_buttons
    call update_display
    goto main_loop

r0_release:
    btfss re0_press, 0
    return
    btfsc PORTE,0
    return
    btg re0_release, 0
    clrf re0_press
    return
r1_release:
    btfss re1_press, 0
    return
    btfsc PORTE,1
    return
    btg re1_release, 0
    clrf re1_press
    return   
check_buttons:
    btfsc PORTE, 0
    setf re0_press
    call r0_release
    btfsc PORTE, 1
    setf re1_press
    call r1_release
    return
handle_portc:
    rrcf LATC, 1, 0
    bsf LATC, 7
    return
handle_portb:
    rlcf LATB, 1, 0
    bsf LATB, 0
    return
update_portc:
    movff LATC, portc_var
    movlw 11111111B
    subwf portc_var
    bz clear_portc
    call handle_portc
    return
update_portb:
    movff LATB, portb_var
    movlw 11111111B
    subwf portb_var
    bz clear_portb
    call handle_portb
    return
clear_portb:
    clrf LATB
    return
clear_portc:
    clrf LATC
    return
update_display:
    incfsz counter
    return
    counter_overflowed:
	incfsz counter1
	return
	counter1_overflowed:
	    btg LATD, 0
	    btfsc re0_release, 0
	    call update_portc
	    btfsc re1_release, 0
	    call update_portb
	    btfss re0_release, 0
	    clrf LATC
	    btfss re1_release, 0
	    clrf LATB
	    movlw 180
	    movwf counter1	
    return
    
busy_wait:
    setf var1
    loop:
	setf var2
	inner_loop:
	    movlw 4
	    movwf var3
	    innner_loop:
		decf var3
		bnz innner_loop
	    decf var2
	    bnz inner_loop
	decf var1
	bnz loop
    return 
    
end resetVec