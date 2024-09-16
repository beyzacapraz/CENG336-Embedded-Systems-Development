/* Group 20
 * Beyza �APRAZ
 * H�sna Aysu KAYA
 * Hasan K�reli
 */


#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pic18f8722.h>
#include "pragmas.h"


typedef enum {DIST, ALT, PRS} output_st_t;
output_st_t output_st = DIST; // State for which output should be given

void output_task();


typedef enum {IDLEE, clear, A, B, C, D} led_cmd_t;
led_cmd_t led_cmd = IDLEE; // State for led state


// Button states and other variables for schmitt trigger implementation for RB4-7 interrupts
typedef enum {
    IDLE,
    PRESSING,
    PRESSED,
    RELEASED
} ButtonState; 

ButtonState rb4state = IDLE;
ButtonState rb5state = IDLE; 
ButtonState rb6state = IDLE;
ButtonState rb7state = IDLE;
int press_counter[8] = {0, 0, 0, 0, 0, 0, 0, 0}; // Initialize all elements to 0
int rb_threshold = 5; 
int release_counter[8] = {0, 0, 0, 0,0, 0, 0, 0}; // Initialize all elements to 0
uint8_t prevB; 
int released[8] = {0, 0, 0, 0,0, 0, 0, 0};

int started = 0; // Variable to check if GOO command arrived
int dist; // Current distance
int speed = 0; // Current speed
int end = 0;

int alt_period = 0; // Period given by ALT command
int alt_value; // Converted altitude value
int prs_value = 0; // value for $PRSXX# command
int counter_limit = 0; // Period of ALT/100
int counter = 0; // counter for printing ALT
int man_num; // Number of $MANXX# command 
int man_flag = 0; // To check if $MAN01# has arrived 
 


inline void disable_rxtx( void ) { PIE1bits.RC1IE = 0;PIE1bits.TX1IE = 0;}
inline void enable_rxtx( void )  { PIE1bits.RC1IE = 1;PIE1bits.TX1IE = 1;}


// Here are the necessary initializations
void init_ports() {
    LATB = 0x00; PORTB = 0x00; TRISB = 0xF0; // Set RB4-7 as input
    LATD = 0x00; PORTD = 0x00; TRISD = 0x00; // Output
    LATH = 0x00; PORTH = 0x00; TRISH = 0x10; // LATbits.LATH4 is set as input
    LATA = 0x00; PORTA = 0x00; TRISA = 0x00; // Output
    LATC = 0x00; PORTC = 0x00; TRISC = 0x00; // Output
}

#define SPBRG_VAL (21) // Selected from table 20-3 according to SYNC = 0, BRGH = 0, BRG16 = 1 and 15200 bps 
void init_serial() {
    TRISCbits.TRISC7 = 1; // RX pin as input
    TRISCbits.TRISC6 = 0; // TX pin as output
    PIE1bits.TX1IE = 1; // Enable transmit interrupt
    PIE1bits.RC1IE = 1; // Enable receive interrupt
    TXSTA1 = 0x20; // CSRC:0, 8-bit transmission,  transmit enable, no parity
    RCSTA1 = 0x90; // 8-bit receiver, enable receiver, serial port enabled  
    BAUDCON1bits.BRG16 = 1;
    SPBRGH1 = (SPBRG_VAL >> 8) & 0xff; //EUSART1 Baud Rate Generator Register High Byte
    SPBRG1 = SPBRG_VAL & 0xff; // EUSART1 Baud Rate Generator Register Low Byte
}


void init_timer_and_int() {
    enable_rxtx();
    RCONbits.IPEN = 1; // Enables all unmasked interrupts
    T0CON = 0x85; // 16-bit timer, prescaler 1:64
    TMR0H = 0xC2; 
    TMR0L = 0xFF; // Load Timer0 for 100 ms
    INTCONbits.TMR0IE = 0; // Disable Timer0 interrupt in the beginning
    INTCONbits.TMR0IF = 0; // Clear Timer0 interrupt flag
    INTCONbits.PEIE = 1;
    INTCON2bits.RBIP = 0; // RB Port Change Interrupt Priority bit

}

void init_adc() {
    ADCON0 = 0x31; // Select channel 12 (AN12) and turn on ADC
    ADCON1 = 0x0F; // Configure all pins as digital
    ADCON2 = 0xA2; // Right align result, 8 Tad, Fosc/32
    ADRESH = 0x00;
    ADRESL = 0x00;
    PIR1bits.ADIF = 0; // Clear ADC interrupt flag
    PIE1bits.ADIE = 1; // Enable ADC interrupt
}


/* **** Ring-buffers for incoming and outgoing data **** */
// These buffer functions are modularized to handle both the input and
// output buffers with an input argument.
typedef enum {INBUF = 0, OUTBUF = 1} buf_t;

#define BUFSIZE 128       /* Static buffer size. Maximum amount of data */
uint8_t inbuf[BUFSIZE];   /* Preallocated buffer for incoming data */
uint8_t outbuf[BUFSIZE];  /* Preallocated buffer for outgoing data  */
uint8_t head[2] = {0, 0}; /* head for pushing, tail for popping */
uint8_t tail[2] = {0, 0};

/* Check if a buffer had data or not */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_isempty( buf_t buf ) { return (head[buf] == tail[buf])?1:0; }
/* Place new data in buffer */
#pragma interrupt_level 2 // Prevents duplication of function
void buf_push( uint8_t v, buf_t buf) {
    if (buf == INBUF) inbuf[head[buf]] = v;
    else outbuf[head[buf]] = v;
    head[buf]++;
    if (head[buf] == BUFSIZE) head[buf] = 0;
    if (head[buf] == tail[buf]) { 
    }
}
/* Retrieve data from buffer */
#pragma interrupt_level 2 // Prevents duplication of function
uint8_t buf_pop( buf_t buf ) {
    uint8_t v;
    if (buf_isempty(buf)) { 
        return 0; 
    } else {
        if (buf == INBUF){
            v = inbuf[tail[buf]];
        }
        
        else v = outbuf[tail[buf]];
        tail[buf]++;
        if (tail[buf] == BUFSIZE) tail[buf] = 0;
        return v;
    }
}


/* ISR
In the high priority interrupt we checked receive, transmit, timer and ADC flags.
 * In receive part we send the received command to the buf_push function and store
 * it in the input buffer
 * 
 * In transmit part we checked whether transmission is finished or not, if finished
 * we disabled transmit register. If not finished we used buf_pop to store the command
 * we will send into the TXREG1
 * 
 * In timer part we checked if #ALTXXXX$ command's period is not zero. If not zero, and
 * counter is equal to counter_limit, we started the conversion and set the alt_value
 * according to the converted result value we got from potentiometer. We set output_st
 * as ALT and called output_task for transmitting altitude response
 * We also checked started value and if GOO command came, we set output_st as DST and
 * called output_task function for transmitting distance response
 * 
 * IN ADC part we cleared the ADIF flag
 * 
 * 
 */
void __interrupt(high_priority) highPriorityISR(void) {
    if (PIR1bits.RC1IF){
        PIR1bits.RC1IF = 0;
        buf_push(RCREG1, INBUF);
    }
    if (PIR1bits.TX1IF){
        
        PIR1bits.TX1IF = 0;
        // If all bytes are transmitted, turn off transmission
        if (buf_isempty(OUTBUF)){
            
            while(TXSTA1bits.TRMT == 0);
            TXSTA1bits.TXEN = 0; 
        }
        else{ 
            TXREG1 = buf_pop(OUTBUF);
        }
    } 
    if (INTCONbits.TMR0IF == 1) {
        // Handle Timer0 interrupt for periodic tasks
        TMR0H = 0xC2;
        TMR0L = 0xFF;
        INTCONbits.TMR0IF = 0;
        
        if (end) return;
        
        if(alt_period) counter++;
        
        if(dist > 0) dist -= speed;
        else dist = 0;
        
        for (int i = 4; i < 8; i++){
            if (released[i] == 1){
                output_st = PRS;
                prs_value = i;
                break;
            }
        }
        
        if (prs_value) output_task();
        
        else if(counter_limit && counter == counter_limit){
            GODONE = 1; //???
            while(GODONE);
            unsigned int result = (ADRESH << 8) + ADRESL; // Get the result;
            if(768 <= result && result <= 1023) alt_value = 12000;
            else if(512 <= result && result < 768) alt_value = 11000;
            else if(256 <= result && result < 512) alt_value = 10000;
            else alt_value = 9000;
            output_st = ALT;
            counter = 0;
            output_task();  
        }  
        else if(started){
            output_st = DIST;
            output_task();

        }
        
    } 
    
    if (PIR1bits.ADIF == 1){
        PIR1bits.ADIF = 0;
    }
}

/*
 * This part is schmitt trigger part for manual RB pressing action.
 * If we release the button, we set output_st as PRS and prs_value according
 * to the pressed button number, RB4 -> prs_value = 4. After that we called 
 * output_task function for transmitting press response
 */
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
            released[6] = 1;
            rb6state = IDLE;
        }
    }
    
    if (rb4state == PRESSING) {
        press_counter[4]++;
        if (press_counter[4] > rb_threshold) {
            rb4state = PRESSED;
            press_counter[4] = 0; // Reset counter
        }
    }
    else if (rb4state == RELEASED) {
        release_counter[4]++;
        if (release_counter[4] > rb_threshold) {
            release_counter[4] = 0;
            released[4] = 1;
            rb4state = IDLE;
        }
    }
    
    if (rb5state == PRESSING) {
        press_counter[5]++;
        if (press_counter[5] > rb_threshold) {
            rb5state = PRESSED;
            press_counter[5] = 0; // Reset counter
        }
    }
    else if (rb5state == RELEASED) {
        release_counter[5]++;
        if (release_counter[5] > rb_threshold) {
            release_counter[5] = 0;
            released[5] = 1;
            rb5state = IDLE;
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
            released[7] = 1;
            rb7state = IDLE;
        }
    }
    
}

void __interrupt(low_priority) lowPriorityISR(void) {
    if(INTCONbits.RBIF) { // Handle RB interrupt
        INTCONbits.RBIF = 0; 
        // Schmitt trigger implementation same as we did in THE2
        if (led_cmd == B && rb6state == IDLE && !PORTBbits.RB6) { 
            rb6state = PRESSING;
        }
        else if (rb6state == PRESSED && PORTBbits.RB6) {
            rb6state = RELEASED;
        }
        
        if (led_cmd == A && rb7state == IDLE && !PORTBbits.RB7) { 
            rb7state = PRESSING;
        }
        else if (rb7state == PRESSED && PORTBbits.RB7) {
            rb7state = RELEASED;
        }
        
        if (led_cmd == C && rb5state == IDLE && !PORTBbits.RB5) { 
            rb5state = PRESSING;
        }
        else if (rb5state == PRESSED && PORTBbits.RB5) {
            rb5state = RELEASED;
        }
        
        if (led_cmd == D && rb4state == IDLE && !PORTBbits.RB4) {
            rb4state = PRESSING;
        }
        else if (rb4state == PRESSED && PORTBbits.RB4) {
            rb4state = RELEASED;
        }
        prevB = PORTB; // Read value so it does not call again
    }
}


/* **** Packet task **** */
// Used Uluc Hoca's code for this part 
#define PKT_MAX_SIZE 128 // Maximum packet size. Syntax errors can be large!
#define PKT_HEADER '$'  // Marker for start-of-packet
#define PKT_END '#'   // Marker for end-of-packet

// State machine states for packet reception. 
typedef enum {PKT_WAIT_HDR, PKT_GET_BODY, PKT_WAIT_ACK} pkt_state_t;
pkt_state_t pkt_state = PKT_WAIT_HDR;
uint8_t pkt_body[PKT_MAX_SIZE]; // Packet data
uint8_t pkt_bodysize;           // Size of the current packet
// Set to 1 when packet has been received. Must be set to 0 once packet is processed
uint8_t pkt_valid = 0;
uint8_t pkt_id = 0; // Incremented for every valid packet reception

/* The packet task is responsible from monitoring the input buffer, identify
 * the start marker 0x00, and retrieve all subsequent bytes until the end marker
 * 0xff is encountered. This packet will then be processed by the calc_task()
 * to parse and execute the arithmetic expression. */
void packet_task() {
    disable_rxtx();
    // Wait until new bytes arrive
    if (!buf_isempty(INBUF)) {
        uint8_t v;
        switch(pkt_state) {
        case PKT_WAIT_HDR:
            v = buf_pop(INBUF);
            if (v == PKT_HEADER) {
                // Packet header is encountered, retrieve the rest of the packet
                pkt_state = PKT_GET_BODY;
                pkt_bodysize = 0;
            }
            break;
        case PKT_GET_BODY:
            v = buf_pop(INBUF);
            if (v == PKT_END) {
                // End of packet is encountered, signal calc_task())
                pkt_state = PKT_WAIT_ACK;
                pkt_valid = 1;
            } else if (v == PKT_HEADER) {
                // Unexpected packet start. Abort current packet and restart
                //error_packet();
                pkt_bodysize = 0;
            } else 
                pkt_body[pkt_bodysize++] = v;
            break;
        case PKT_WAIT_ACK:
            if (pkt_valid == 0) {
                // Packet processing seems to be finished, continue monitoring
                pkt_state = PKT_WAIT_HDR;
                pkt_id++;
            }
            break;
        }
    }
    enable_rxtx();
}



/* ***** Serial output task **** */


/* Output a string to the outgoing buffer */
void output_str( char *str ) {
    uint8_t ind = 0;    
    while (str[ind] != 0) {
        disable_rxtx();
        buf_push(str[ind++], OUTBUF);
        enable_rxtx();
    }
}



/* Output task function */
// This function will set the correct output string to transmit
void output_task() {
    switch (output_st) { 
    case DIST: {
        char str[10] = "$DST";  
        char dist_chr[5];       
        sprintf(dist_chr, "%04x", dist);
        strcat(str, dist_chr);
        strcat(str, "#");
        output_str(str);

        break;
    }
    case ALT:{
            char str[12] = "$ALT";  
            char alt_char[5];      
            sprintf(alt_char, "%04x", alt_value);
            strcat(str, alt_char);
            strcat(str, "#");
            output_str(str);
            break;
        }
    case PRS:{
            char str[10] = "$PRS";  
            char prs_char[3];
            sprintf(prs_char, "%02x", prs_value);
            strcat(str, prs_char);
            strcat(str, "#");
            output_str(str);
            prs_value = 0;
            for (int i = 4; i < 8; i++){
                released[i] = 0;
            }
            break;
        }
    
    }

    disable_rxtx();
    // Check if there is any buffered output or ongoing transmission
    if (!buf_isempty(OUTBUF)&& TXSTA1bits.TXEN == 0) { 
        // If transmission is already ongoing, do nothing, 
        // the ISR will send the next char. Otherwise, send the 
        // first char and enable transmission

        TXSTA1bits.TXEN = 1;
        TXREG1 = buf_pop(OUTBUF);
    }
    enable_rxtx();
}



/* Calculator task function */
void calc_task() {
    if (pkt_valid == 0) return; // Wait until there is a valid packet to process
    
    
    char cmd[4];
    char man_led[6];
    strncpy(cmd, pkt_body, 3);
    strncpy(man_led, pkt_body, 5);
    cmd[3] = NULL;
    man_led[5] = NULL;
    // Handle different interrupt cases if there is no case that matches pkt_body set it as invalid and take next input
    if(strcmp(cmd, "GOO") == 0 && pkt_bodysize == 7){ 
        started = 1;
        end = 0;
        sscanf(pkt_body+3, "%04x", &dist); // Get initial distance
        INTCONbits.TMR0IE = 1; // Enable Timer0 interrupt after go command
        //INTCONbits.TMR0IF = 1;
    }
    else if(started && strcmp(cmd, "SPD") == 0 && pkt_bodysize == 7){
        sscanf(pkt_body+3, "%04x", &speed); // Get speed
    }
    else if(started && strcmp(cmd, "ALT") == 0 && pkt_bodysize == 7){
        sscanf(pkt_body+3, "%04x", &alt_period); // Get Altitude period
        if(alt_period == 0){ // means dont print ALT 
            counter_limit = 0;
            counter = 0;
        }
        else{
            counter_limit = alt_period / 100; 
        }
       
    }
    else if(started && strcmp(cmd, "MAN") == 0){
        sscanf(pkt_body+3, "%02x", &man_num); // Get manual command's number
        if(man_num == 1){ // enter manual mode
            man_flag = 1;
            INTCONbits.RBIE = 1; // enable rb interrupts

        }
        else if(man_num == 0){ // exit manual mode
            man_flag = 0;
            INTCONbits.RBIE = 0; // disable rb interrupts
            led_cmd = IDLEE; // go back to IDLEE state for leds
        }
    }
    if(man_flag == 1){ // check for manual mode and set corresponding led state
        if(strcmp(man_led, "LED00") == 0 && pkt_bodysize == 5){
            led_cmd = clear;
        }
        else if(strcmp(man_led, "LED01") == 0 && pkt_bodysize == 5){
            led_cmd = D;
        }
        else if(strcmp(man_led, "LED02") == 0 && pkt_bodysize == 5){
            led_cmd = C;
        }
        else if(strcmp(man_led, "LED03") == 0 && pkt_bodysize == 5){
            led_cmd = B;
        }
        else if(strcmp(man_led, "LED04") == 0 && pkt_bodysize == 5){
            led_cmd = A;
        }   
    
    }
    else if(started && strcmp(cmd, "END") == 0 && pkt_bodysize == 3){ // Stop receive and transmit 
        end = 1;
        disable_rxtx();
    }
    
   
    pkt_valid = 0; // Packet is processed if valid so get next packet
    
    return;
}

/*
IDLEE state means no led task is yet given
clear means $LED00# is given so we clear all
D, C, B and A light the corresponding leds
*/
void control_leds(){
    switch(led_cmd){
        case IDLEE:
            break;
        case clear:
            LATDbits.LATD0 = 0;
            LATCbits.LATC0 = 0;
            LATBbits.LATB0 = 0;
            LATAbits.LATA0 = 0;
            break;
        case D:
            LATDbits.LATD0 = 1;
            break;
        case C:
            LATCbits.LATC0 = 1;
            break;
        case B:
            LATBbits.LATB0 = 1;
            break;
        case A:
            LATAbits.LATA0 = 1;
            break;   
    }
    return;
}


void main(void) {
    // initialization
    init_ports();
    init_adc();
    init_serial();
    init_timer_and_int();
    INTCONbits.GIE = 1; // Enable interrupts
    // Round Robin
    while(1){
        if (!end) {
            packet_task(); // Task for getting input in a format that is useful for us
            calc_task(); // Process the packets we receive
            //output_task(); We call this in timer and rb interrupt since that is where we need to give output
            control_leds(); // Control leds based on the current state
            rb_count_func(); // Schmitt trigger implementation for rb interrupt
        }
    }
    return;
}
