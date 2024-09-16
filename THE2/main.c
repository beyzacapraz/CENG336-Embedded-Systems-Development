// ============================ //
// Do not edit this part!!!!    //
// ============================ //
// 0x300001 - CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator,
                                // PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit
                                // (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit
                                // (Oscillator Switchover mode disabled)
// 0x300002 - CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out
                                // Reset disabled in hardware and software)
// 0x300003 - CONFIG1H
#pragma config WDT = OFF        // Watchdog Timer Enable bit
                                // (WDT disabled (control is placed on the SWDTEN bit))
// 0x300004 - CONFIG3L
// 0x300005 - CONFIG3H
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit
                                // (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled;
                                // RE3 input pin disabled)
// 0x300006 - CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply
                                // ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit
                                // (Instruction set extension and Indexed
                                // Addressing mode disabled (Legacy mode))

#pragma config DEBUG = OFF      // Disable In-Circuit Debugger

#define KHZ 1000UL
#define MHZ (KHZ * KHZ)
#define _XTAL_FREQ (40UL * MHZ)

// ============================ //
//             End              //
// ============================ //


#include <xc.h>
#include <stdint.h>

#define T_PRELOAD_LOW 0x6A
#define T_PRELOAD_HIGH 0x67

#define zero  0x3F
#define one 0x06
#define two 0x5B
#define three 0x4F
#define four 0x66
#define five 0x6D
#define six   0x7D
#define seven 0x07
#define eight 0x7F
#define nine  0x6F

int num_arr[10] = {zero, one, two, three, four, five, six, seven, eight, nine};

uint8_t prevB;
int count8 = 0;

int obj_type; // 0 dot, 1 square, 2 L
int hor_cor;
int ver_cor;
int obj_visible;
int rotation_state;
int lit_pixels = 0;
uint8_t prevC, prevD, prevE, prevF;
int a0pressed=0, a1pressed=0, a2pressed=0, a3pressed=0;


typedef enum {
    IDLE,
    PRESSING,
    PRESSED,
    RELEASED
} ButtonState;

ButtonState a1state = IDLE;
ButtonState a0state = IDLE; 
ButtonState a3state = IDLE;
ButtonState a2state = IDLE;
ButtonState rb6state = IDLE;
ButtonState rb7state = IDLE;

int press_counter[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize all elements to 0
int rb_threshold = 5;
int threshold = 100;
int release_counter[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize all elements to 0
int rb7pressed=0, rb6pressed=0;

int submit_flag = 0;
int rotate_flag = 0;


void check_reset(){
    if(lit_pixels == 32){
        __delay_ms(30);
        lit_pixels = 0;
        obj_type = 0;
        hor_cor = 0; ver_cor = 0;
        obj_visible = 1;
        rotation_state = 0;
        count8 = 0;
        prevC = 0; prevD = 0; prevE = 0; prevF = 0; 
    }
}

void light_leds(){
    LATC=prevC; LATD=prevD; LATE=prevE; LATF=prevF;
    if(obj_visible){
        if(obj_type == 0){
            if (hor_cor == 0) {
                LATC |= (1 << ver_cor);
            } 
            else if (hor_cor == 1) {
                LATD |= (1 << ver_cor);
            } 
            else if (hor_cor == 2) {
                LATE |= (1 << ver_cor);
            } 
            else if (hor_cor == 3) {
                LATF |= (1 << ver_cor);
            }
        }
        else if(obj_type == 1){
            switch(hor_cor){
                case 0:
                    LATC |= (1 << ver_cor);
                    LATD |= (1 << ver_cor);
                    LATC |= (1 << (ver_cor+1));
                    LATD |= (1 << (ver_cor+1));
                    break;
                case 1:
                    LATD |= (1 << ver_cor);
                    LATE |= (1 << ver_cor);
                    LATD |= (1 << (ver_cor+1));
                    LATE |= (1 << (ver_cor+1));
                    break;
                case 2:
                    LATE |= (1 << ver_cor);
                    LATF |= (1 << ver_cor);
                    LATE |= (1 << (ver_cor+1));
                    LATF |= (1 << (ver_cor+1));
                    break;
            }
        }
        else if(obj_type == 2){         
            if(hor_cor == 0){
                if(rotation_state == 0){
                    LATC |= (1 << ver_cor);
                    LATD |= (1 << ver_cor);
                    LATD |= (1 << (ver_cor+1));
                }
                else if(rotation_state == 1){
                    LATC |= (1 << (ver_cor+1));
                    LATD |= (1 << ver_cor);
                    LATD |= (1 << (ver_cor+1));
                }
                else if(rotation_state == 2){
                    LATC |= (1 << ver_cor);
                    LATD |= (1 << (ver_cor+1));
                    LATC |= (1 << (ver_cor+1));
                }
                else if(rotation_state == 3){
                    LATC |= (1 << ver_cor);
                    LATD |= (1 << ver_cor);
                    LATC |= (1 << (ver_cor+1));
                }
                
            }
            else if (hor_cor == 1) {
                if (rotation_state == 0) {
                    LATD |= (1 << ver_cor);
                    LATE |= (1 << ver_cor);
                    LATE |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 1) {
                    LATD |= (1 << (ver_cor + 1));
                    LATE |= (1 << ver_cor);
                    LATE |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 2) {
                    LATD |= (1 << ver_cor);
                    LATE |= (1 << (ver_cor + 1));
                    LATD |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 3) {
                    LATD |= (1 << ver_cor);
                    LATE |= (1 << ver_cor);
                    LATD |= (1 << (ver_cor + 1));
                }
            }
            else if (hor_cor == 2) {
                if (rotation_state == 0) {
                    LATE |= (1 << ver_cor);
                    LATF |= (1 << ver_cor);
                    LATF |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 1) {
                    LATE |= (1 << (ver_cor + 1));
                    LATF |= (1 << ver_cor);
                    LATF |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 2) {
                    LATE |= (1 << ver_cor);
                    LATF |= (1 << (ver_cor + 1));
                    LATE |= (1 << (ver_cor + 1));
                }
                else if (rotation_state == 3) {
                    LATE |= (1 << ver_cor);
                    LATF |= (1 << ver_cor);
                    LATE |= (1 << (ver_cor + 1));
                }
            }

        }
    }
    else{
        if (obj_type == 0) {
            if (hor_cor == 0) {
                LATC &= ~(1 << ver_cor);
            } 
            else if (hor_cor == 1) {
                LATD &= ~(1 << ver_cor);
            } 
            else if (hor_cor == 2) {
                LATE &= ~(1 << ver_cor);
            } 
            else if (hor_cor == 3) {
                LATF &= ~(1 << ver_cor);
            }
        }
        else if (obj_type == 1) {
            if (hor_cor == 0) {
                LATC &= ~(1 << ver_cor);
                LATD &= ~(1 << ver_cor);
                LATC &= ~(1 << (ver_cor + 1));
                LATD &= ~(1 << (ver_cor + 1));
            } 
            else if (hor_cor == 1) {
                LATD &= ~(1 << ver_cor);
                LATE &= ~(1 << ver_cor);
                LATD &= ~(1 << (ver_cor + 1));
                LATE &= ~(1 << (ver_cor + 1));
            } 
            else if (hor_cor == 2) {
                LATE &= ~(1 << ver_cor);
                LATF &= ~(1 << ver_cor);
                LATE &= ~(1 << (ver_cor + 1));
                LATF &= ~(1 << (ver_cor + 1));
            }
        }
        else if (obj_type == 2) {
            if (hor_cor == 0) {
                if (rotation_state == 0) {
                    LATC &= ~(1 << ver_cor);
                    LATD &= ~(1 << ver_cor);
                    LATD &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 1) {
                    LATC &= ~(1 << (ver_cor + 1));
                    LATD &= ~(1 << ver_cor);
                    LATD &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 2) {
                    LATC &= ~(1 << ver_cor);
                    LATD &= ~(1 << (ver_cor + 1));
                    LATC &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 3) {
                    LATC &= ~(1 << ver_cor);
                    LATD &= ~(1 << ver_cor);
                    LATC &= ~(1 << (ver_cor + 1));
                }
            } 
            else if (hor_cor == 1) {
                if (rotation_state == 0) {
                    LATD &= ~(1 << ver_cor);
                    LATE &= ~(1 << ver_cor);
                    LATE &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 1) {
                    LATD &= ~(1 << (ver_cor + 1));
                    LATE &= ~(1 << ver_cor);
                    LATE &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 2) {
                    LATD &= ~(1 << ver_cor);
                    LATE &= ~(1 << (ver_cor + 1));
                    LATD &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 3) {
                    LATD &= ~(1 << ver_cor);
                    LATE &= ~(1 << ver_cor);
                    LATD &= ~(1 << (ver_cor + 1));
                }
            } 
            else if (hor_cor == 2) {
                if (rotation_state == 0) {
                    LATE &= ~(1 << ver_cor);
                    LATF &= ~(1 << ver_cor);
                    LATF &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 1) {
                    LATE &= ~(1 << (ver_cor + 1));
                    LATF &= ~(1 << ver_cor);
                    LATF &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 2) {
                    LATE &= ~(1 << ver_cor);
                    LATF &= ~(1 << (ver_cor + 1));
                    LATE &= ~(1 << (ver_cor + 1));
                } 
                else if (rotation_state == 3) {
                    LATE &= ~(1 << ver_cor);
                    LATF &= ~(1 << ver_cor);
                    LATE &= ~(1 << (ver_cor + 1));
                }
            }
        }
    }
    
}

void light_7seg(){
    int digit2 = lit_pixels/10;
    int digit1 = lit_pixels%10;
    
    LATH = 0b00000011;
    LATJ = zero;
    __delay_ms(0.1);
    LATH = 0b00000100;
    LATJ = num_arr[digit2];
    __delay_ms(0.1);
    LATH = 0b00001000;
    LATJ = num_arr[digit1];
    __delay_ms(0.1);
}

void move_object() {
    if (a1state == IDLE && (PORTG & (1<<4))) { 
        a1state = PRESSING;
    } 
    else if (a1state == PRESSING) {
        press_counter[1]++;
        if (press_counter[1] > threshold) {
            a1state = PRESSED;
            press_counter[1] = 0; // Reset counter
        }
    } 
    else if (a1state == PRESSED && !(PORTG & (1<<4))) {
        a1state = RELEASED;
    } 
    else if (a1state == RELEASED) {
        release_counter[1]++;
        if (release_counter[1] > threshold) {
            release_counter[1] = 0; // Reset counter
            if(ver_cor > 0){
                ver_cor--;
            }
            a1state = IDLE;
        }
    }
    
    if (a3state == IDLE && (PORTG & (1<<3))) { 
        a3state = PRESSING;
    } 
    else if (a3state == PRESSING) {
        press_counter[3]++;
        if (press_counter[3] > threshold) {
            a3state = PRESSED;
            press_counter[3] = 0; // Reset counter
        }
    } 
    else if (a3state == PRESSED && !(PORTG & (1<<3))) {
        a3state = RELEASED;
    } 
    else if (a3state == RELEASED) {
        release_counter[3]++;
        if (release_counter[3] > threshold) {
            release_counter[3] = 0; // Reset counter
            if(hor_cor > 0){
                hor_cor--;
            }
            a3state = IDLE;
        }
    }
    
    if (a0state == IDLE && (PORTG & (1))) { 
        a0state = PRESSING;
    } 
    else if (a0state == PRESSING) {
        press_counter[0]++;
        if (press_counter[0] > threshold) {
            a0state = PRESSED;
            press_counter[0] = 0; // Reset counter
        }
    } 
    else if (a0state == PRESSED && !(PORTG & (1))) {
        a0state = RELEASED;
    } 
    else if (a0state == RELEASED) {
        release_counter[0]++;
        if (release_counter[0] > threshold) {
            release_counter[0] = 0; // Reset counter
            if (obj_type == 0 && hor_cor < 3) {
                hor_cor++;
            } 
            else if (hor_cor < 2) {
                hor_cor++;
            }
            a0state = IDLE;
        }
    }
    
    
    
    if (a2state == IDLE && (PORTG & (1<<2))) { 
        a2state = PRESSING;
    } 
    else if (a2state == PRESSING) {
        press_counter[2]++;
        if (press_counter[2] > threshold) {
            a2state = PRESSED;
            press_counter[2] = 0; // Reset counter
        }
    } 
    else if (a2state == PRESSED && !(PORTG & (1<<2))) {
        a2state = RELEASED;
    } 
    else if (a2state == RELEASED) {
        release_counter[2]++;
        if (release_counter[2] > threshold) {
            release_counter[2] = 0; // Reset counter
            if(obj_type==0 && ver_cor<7){
                ver_cor++;
            }
            else if(ver_cor<6){
                ver_cor++;
            }
            a2state = IDLE;
        }
    }
    
}

void Init()
{
    // A
    ADCON1 = 0x0F;
    LATA = 0x00; PORTA = 0x00; TRISA = 0x00;
    LATG = 0x00; PORTG = 0x00; TRISG = 0x1F;

    // B
    LATB = 0x00; PORTB = 0x00; TRISB = 0xC0;
    // C
    LATC = 0x00; PORTC = 0x00; TRISC = 0x00;
    // D
    LATD = 0x00; PORTD = 0x00; TRISD = 0x00;
    // E
    LATE = 0x00; PORTE = 0x00; TRISE = 0x00;
    // F
    LATF = 0x00; PORTF = 0x00; TRISF = 0x00;
    prevC=0; prevD=0; prevE=0; prevF=0;
    
    LATH = 0x00;
    LATJ = 0x3F; PORTJ = 0x3F; TRISJ = 0x00;
    LATH = 0x0F; PORTH = 0x0F; TRISH = 0x00;
    //PORTH = 0x00;
}

void InitializeTimerAndInterrupts()
{
    // Enable pre-scalar
    // Full pre-scale
    // we also need to do in-code scaling
    T0CON = 0x00;
    T0CONbits.TMR0ON = 1;
    T0CONbits.T0PS2 = 1;
    T0CONbits.T0PS1 = 0;
    T0CONbits.T0PS0 = 1;
    // Pre-load the value
    TMR0H = T_PRELOAD_HIGH;
    TMR0L = T_PRELOAD_LOW;

    RCONbits.IPEN = 0; // priorities off
    INTCON = 0x00;
    INTCONbits.TMR0IE = 1;
    INTCONbits.RBIE = 1;
    //INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
}

// ============================ //
//   INTERRUPT SERVICE ROUTINE  //
// ============================ //

void rb_count_func(){
    if (rb6state == PRESSING) {
        press_counter[6]++;
        if (press_counter[6] > rb_threshold) {
            rb6state = PRESSED;
            press_counter[6] = 0; // Reset counter
        }
    }
    else if (rb6state == RELEASED) {
        release_counter[6]++;
        if (release_counter[6] > rb_threshold) {
            release_counter[6] = 0;
            rotation_state = (rotation_state+1)%4;
            rb6state = IDLE;
        }
    }
    
    
    if (rb7state == PRESSING) {
        press_counter[7]++;
        if (press_counter[7] > rb_threshold) {
            rb7state = PRESSED;
            press_counter[7] = 0; // Reset counter
        }
    }
    else if (rb7state == RELEASED) {
        release_counter[7]++;
        if (release_counter[7] > rb_threshold) {
            release_counter[7] = 0;
            if(obj_type == 0){
                if(hor_cor == 0){
                    if(!(prevC & (1<<ver_cor))){
                        prevC |= (1<<ver_cor);
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels++;
                    }
                }
                else if(hor_cor == 1){
                    if(!(prevD & (1<<ver_cor))){
                        prevD |= (1<<ver_cor);
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels++;
                    }
                }
                else if(hor_cor == 2){
                    if(!(prevE & (1<<ver_cor))){
                        prevE |= (1<<ver_cor);
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels++;
                    }
                }
                else if(hor_cor == 3){
                    if(!(prevF & (1<<ver_cor))){
                        prevF |= (1<<ver_cor);
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels++;
                    }
                }
            }
            else if(obj_type == 1){
                if(hor_cor == 0){
                    if(!(prevC & (1<<ver_cor)) && !(prevC & (1<<(ver_cor+1))) && !(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1)))){
                        prevC |= (1<<ver_cor);
                        prevC |= (1<<(ver_cor+1));
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels += 4;
                    }
                }
                else if(hor_cor == 1){
                    if(!(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1))) && !(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1)))){
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels += 4;
                    }
                }
                else if(hor_cor == 2){
                    if(!(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1))) && !(prevF & (1<<ver_cor)) && !(prevF & (1<<(ver_cor+1)))){
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        prevF |= (1<<ver_cor);
                        prevF |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        lit_pixels += 4;
                    }
                }
            }
            else if(obj_type == 2){
                if(hor_cor == 0){
                    if(rotation_state == 0 && !(prevC & (1<<ver_cor)) && !(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1)))){
                        prevC |= (1<<ver_cor);
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 1 && !(prevC & (1<<(ver_cor+1))) && !(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1)))){
                        prevC |= (1<<(ver_cor+1));
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 2 && !(prevC & (1<<ver_cor)) && !(prevC & (1<<(ver_cor+1))) && !(prevD & (1<<(ver_cor+1)))){
                        prevD |= (1<<(ver_cor+1));
                        prevC |= (1<<ver_cor);
                        prevC |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 3 && !(prevC & (1<<ver_cor)) && !(prevC & (1<<(ver_cor+1))) && !(prevD & (1<<ver_cor))){
                        prevD |= (1<<ver_cor);
                        prevC |= (1<<ver_cor);
                        prevC |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                }
                else if(hor_cor == 1){
                    if(rotation_state == 0 && !(prevD & (1<<ver_cor)) && !(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1)))){
                        prevD |= (1<<ver_cor);
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 1 && !(prevD & (1<<(ver_cor+1))) && !(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1)))){
                        prevD |= (1<<(ver_cor+1));
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 2 && !(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1))) && !(prevE & (1<<(ver_cor+1)))){
                        prevE |= (1<<(ver_cor+1));
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 3 && !(prevD & (1<<ver_cor)) && !(prevD & (1<<(ver_cor+1))) && !(prevE & (1<<ver_cor))){
                        prevE |= (1<<ver_cor);
                        prevD |= (1<<ver_cor);
                        prevD |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                }
                else if(hor_cor == 2){
                    if(rotation_state == 0 && !(prevE & (1<<ver_cor)) && !(prevF & (1<<ver_cor)) && !(prevF & (1<<(ver_cor+1)))){
                        prevE |= (1<<ver_cor);
                        prevF |= (1<<ver_cor);
                        prevF |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 1 && !(prevE & (1<<(ver_cor+1))) && !(prevF & (1<<ver_cor)) && !(prevF & (1<<(ver_cor+1)))){
                        prevE |= (1<<(ver_cor+1));   
                        prevF |= (1<<ver_cor);
                        prevF |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 2 && !(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1))) && !(prevF & (1<<(ver_cor+1)))){
                        prevF |= (1<<(ver_cor+1));
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                    else if(rotation_state == 3 && !(prevE & (1<<ver_cor)) && !(prevE & (1<<(ver_cor+1))) && !(prevF & (1<<ver_cor))){
                        prevF |= (1<<ver_cor);
                        prevE |= (1<<ver_cor);
                        prevE |= (1<<(ver_cor+1));
                        ver_cor = 0;
                        hor_cor = 0;
                        obj_type = (obj_type+1)%3;
                        rotation_state = 0;
                        lit_pixels += 3;
                    }
                }
            }
            rb7state = IDLE;
        }
    }
    
}

__interrupt(high_priority)
void HandleInterrupt()
{
    
    if(INTCONbits.RBIF)
    {
        INTCONbits.RBIF = 0;
        if (rb6state == IDLE && (PORTB & (1<<6))) { // 3. elemana bak   && ((PORTB & (1<<5)) ^ (prevB & (1<<5)))
            rb6state = PRESSING;
        }
        else if (rb6state == PRESSED && !(PORTB & (1<<6))) {
            rb6state = RELEASED;
        }
        
        
        
        
        if (rb7state == IDLE && (PORTB & (1<<7))) { // 3. elemana bak   && ((PORTB & (1<<5)) ^ (prevB & (1<<5)))
            rb7state = PRESSING;
        }
        else if (rb7state == PRESSED && !(PORTB & (1<<7))) {
            rb7state = RELEASED;
        }
        
        prevB = PORTB;
        
        
        // Then clear the bit
    }
    
    // Timer overflowed (250 ms)
    if(INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        obj_visible = !obj_visible;
        count8++;
        if(count8 == 8){
            count8=0;
            // go down
            if(obj_type==0 && ver_cor<7){
                ver_cor++;
            }
            else if(ver_cor<6){
                ver_cor++;
            }
        }
        // Pre-load the value
        TMR0H = T_PRELOAD_HIGH;
        TMR0L = T_PRELOAD_LOW;

    }
}

__interrupt(low_priority)
void HandleInterrupt2()
{

}
// ============================ //
//            MAIN              //
// ============================ //
void main()
{
    Init();
    InitializeTimerAndInterrupts();

    prevB = PORTB;
    __delay_ms(750);
    INTCONbits.GIE = 1;
    obj_type = 0;
    ver_cor = 0;
    hor_cor = 0;
    rotation_state = 0;
    obj_visible = 1;
    //light_7seg();
    // Main Loop
    while(1)
    {
        //...
        
        light_leds();
        light_7seg();
        move_object();
        check_reset();
        rb_count_func();
    }
}