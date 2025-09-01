// Program Name: ADAS Level 2 powered by TI's MSP430FR5969
// Author: Fayyaz Nisar Shaikh 
// Date: 31 Aug 2025 
// Author: Fayyaz Nisar Shaikh
// Email: fayyaz.shaikh7862005@gmail.com
// LinkedIn: https://www.linkedin.com/in/fayyaz-shaikh-7646312a3/



#include <msp430.h>

#define TRIGGER BIT3   // P4.3
#define ECHO    BIT4   // P2.4
#define LED     BIT0   // P1.0
#define BUZZER  BIT5   // P1.5

#define LINE_RIGHT   BIT4   // P3.4
#define LINE_LEFT    BIT3   // P1.3

#define COLL_BACK_R  BIT6   // P2.6
#define COLL_BACK_L  BIT2   // P1.2

#define IN1 BIT0   // P3.0  
#define IN2 BIT6   // P1.6  
#define IN3 BIT7   // P1.7  
#define IN4 BIT4   // P1.4  

#define TIMEOUT 30000
#define DIST_MIN 2
#define DIST_MAX 400
// ultrasonic settings 


void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);
void delay_us(unsigned int us);
unsigned int pulseIn(void);
void int_to_str(unsigned int num, char *str);
void delay_cycles_var(unsigned int cycles);

unsigned char line_right(void);
unsigned char line_left(void);
unsigned char collision_back_right(void);
unsigned char collision_back_left(void);

void buzzer_on(void);
void buzzer_off(void);

void motor_force_direction_A(unsigned char cmd);
void motor_force_direction_B(unsigned char cmd);
void motor_stop_all(void);

unsigned char motor_pwm_both_blocking(unsigned char dirA, unsigned char dirB, unsigned int duty, unsigned int duration_ms);

void uart_init(void) {
    P2SEL0 &= ~(BIT0 | BIT1);
    P2SEL1 |= BIT0 | BIT1;      
    UCA0CTLW0 = UCSWRST;        
    UCA0CTLW0 |= UCSSEL__SMCLK; 
    UCA0BR0 = 8;                
    UCA0BR1 = 0;
    UCA0MCTLW = 0xD600;
    UCA0CTLW0 &= ~UCSWRST;      
}
void uart_putc(char c){
    while (!(UCA0IFG & UCTXIFG));
    UCA0TXBUF = c;
}
void uart_puts(const char *str){
    while(*str) uart_putc(*str++);
}
void delay_us(unsigned int us){
    while(us--) __delay_cycles(1);
}

// pulse on time detection
unsigned int pulseIn() {
    unsigned int timeout = TIMEOUT;
    while((P2IN & ECHO) == 0 && timeout--) __no_operation();
    if(timeout == 0) return 0;
    unsigned int start = TA0R;
    timeout = TIMEOUT;
    while((P2IN & ECHO) != 0 && timeout--) __no_operation();
    unsigned int stop = TA0R;
    if(timeout == 0) return 0;
    if(stop >= start) return stop - start;
    else return 0xFFFF - start + stop + 1;
}
void int_to_str(unsigned int num, char *str){
    int i=0;
    if(num==0){ str[i++]='0'; str[i]='\0'; return; }
    char temp[10]; int j=0;
    while(num>0){ temp[j++] = (num%10)+'0'; num/=10; }
    int k;
    for(k=j-1;k>=0;k--) str[i++] = temp[k];
    str[i]='\0';
}

// custom delay function which accepts variables as delay value
void delay_cycles_var(unsigned int cycles) { while (cycles--) __no_operation(); }

unsigned char line_right()          { return (P3IN & LINE_RIGHT) ? 1 : 0; }
unsigned char line_left()           { return (P1IN & LINE_LEFT)  ? 1 : 0; }
unsigned char collision_back_right(){ return (P2IN & COLL_BACK_R)? 1 : 0; }
unsigned char collision_back_left() { return (P1IN & COLL_BACK_L)? 1 : 0; }

void buzzer_on(void)  { P1OUT |= BUZZER; }
void buzzer_off(void) { P1OUT &= ~BUZZER; }

// Motor A: IN1 = P3.0, IN2 = P1.6
void motor_force_direction_A(unsigned char cmd) {
    switch(cmd) {
        case 0: P3OUT &= ~IN1; P1OUT &= ~IN2; break;  // stop
        case 1: P3OUT &= ~IN1; P1OUT |= IN2;  break;  // forward
        case 2: P3OUT |= IN1;  P1OUT &= ~IN2; break;  // backward
    }
}
// Motor B: IN3 = P1.7, IN4 = P1.4
void motor_force_direction_B(unsigned char cmd) {
    switch(cmd) {
        case 0: P1OUT &= ~(IN3 | IN4); break;        // stop
        case 1: P1OUT &= ~IN3; P1OUT |= IN4; break;  // forward
        case 2: P1OUT |= IN3;  P1OUT &= ~IN4; break; // backward
    }
}
void motor_stop_all(void) {
    P3OUT &= ~IN1;
    P1OUT &= ~(IN2 | IN3 | IN4);
}

unsigned char motor_pwm_both_blocking(unsigned char dirA, unsigned char dirB, unsigned int duty, unsigned int duration_ms) {
    if(duty > 100) duty = 100;
    unsigned int high_time = duty * 10;     // 1% -> 10 cycles
    unsigned int low_time  = 1000 - high_time;
    unsigned int cycles = duration_ms;
    unsigned int i;

    for(i = 0; i < cycles; i++) {
        unsigned char rawBL = collision_back_left();
        unsigned char rawBR = collision_back_right();
        if(rawBL == 0 || rawBR == 0) {
            motor_stop_all();
            buzzer_on();
            return 1;
        }

        motor_force_direction_A(dirA);
        motor_force_direction_B(dirB);

        delay_cycles_var(high_time);

        motor_stop_all();
        delay_cycles_var(low_time);
    }
    return 0;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // Stop watchdog

    
    CSCTL0_H = CSKEY_H;        
    CSCTL1 = DCOFSEL_0;        
    CSCTL2 = SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVS__1 | DIVM__1;
    CSCTL0_H = 0;

    PM5CTL0 &= ~LOCKLPM5;      // unlock GPIOs

    P1DIR |= LED | BUZZER; P1OUT &= ~(LED | BUZZER);
    P4DIR |= TRIGGER; P4OUT &= ~TRIGGER;
    P2DIR &= ~ECHO; P2REN &= ~ECHO;

    P3DIR |= IN1;
    P1DIR |= IN2 | IN3 | IN4;
    motor_stop_all();

    P3DIR &= ~LINE_RIGHT; P3REN |= LINE_RIGHT; P3OUT |= LINE_RIGHT;
    P1DIR &= ~(LINE_LEFT | COLL_BACK_L); P1REN |= (LINE_LEFT | COLL_BACK_L); P1OUT |= (LINE_LEFT | COLL_BACK_L);
    P2DIR &= ~COLL_BACK_R; P2REN |= COLL_BACK_R; P2OUT |= COLL_BACK_R;

    TA0CTL = TASSEL_2 | MC_2 | TACLR;

    uart_init();
    uart_puts("ADAS integrated: Starting...\r\n");

    while(1) {
    // ultrasonic measure
    P4OUT &= ~TRIGGER; delay_us(2);
    P4OUT |= TRIGGER;  delay_us(10);
    P4OUT &= ~TRIGGER;

    unsigned int pulse_width = pulseIn();
    unsigned int distance_cm = (pulse_width ? pulse_width/58 : 0);

    unsigned char lr = line_right();
    unsigned char ll = line_left();
    unsigned char br = collision_back_right();
    unsigned char bl = collision_back_left();


    char buf[16];
    uart_puts("Distance:");
    int_to_str(distance_cm, buf); uart_puts(buf); uart_puts("cm | ");
    uart_puts("L:"); int_to_str(ll, buf); uart_puts(buf);
    uart_puts(" R:"); int_to_str(lr, buf); uart_puts(buf);
    uart_puts(" | BackL:"); int_to_str(bl, buf); uart_puts(buf);
    uart_puts(" BackR:"); int_to_str(br, buf); uart_puts(buf);
    uart_puts(" | ");

    // anti collision logic for safe parking, safe driving
    if(bl == 0 || br == 0) {
        uart_puts("ACTION: EMERGENCY - COLLISION_BACK\r\n");
        motor_stop_all();
        buzzer_on();
        while(collision_back_left()==0 || collision_back_right()==0);
        buzzer_off();
        continue;
    }
    else if(distance_cm != 0 && distance_cm <= 15) {
        uart_puts("ACTION: ULTRA <15cm -> STOP\r\n");
        motor_stop_all();
        buzzer_on();
        while(1) {
            P4OUT &= ~TRIGGER; delay_us(2);
            P4OUT |= TRIGGER;  delay_us(10);
            P4OUT &= ~TRIGGER;
            unsigned int p = pulseIn();
            unsigned int d = (p ? p/58 : 0);
            if(d > 20) break;
        }
        buzzer_off();
        continue;
    }
    // adaptive cruise control 
    else if(distance_cm != 0 && distance_cm < 30) {
        uart_puts("ACTION: SLOWDOWN\r\n");
        motor_pwm_both_blocking(1,1,80,100);
    }
    // full speed when no danger 
else {
    if(lr==1 && ll==1) {
        uart_puts("ACTION: LINE -> FORWARD\r\n");
        motor_force_direction_A(1); motor_force_direction_B(1);
    }
    else if(lr==1 && ll==0) {   
        uart_puts("ACTION: TURN LEFT\r\n");
        motor_pwm_both_blocking(2,1,60,50);
    }
    else if(lr==0 && ll==1) {  
        uart_puts("ACTION: TURN RIGHT\r\n");
        motor_pwm_both_blocking(1,2,60,50);
    }
    else {
        uart_puts("ACTION: LINE LOST -> STOP\r\n");
        motor_stop_all();
    }

}


}

}
