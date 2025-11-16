#include "xc.h"
#include "tools.h"
#include <math.h>
#include <stdbool.h>

void tmr_setup_period(int8_t timer, int ms) {
    switch (timer) {
        case 1:
            TMR1 = 0;
            PR1 = (ms / 1000.0) * 281250;
            T1CONbits.TCKPS = 0b11;
            T1CONbits.TON = 1;
            break;
        case 2:
            TMR2 = 0;
            PR2 = (ms / 1000.0) * 281250;
            T2CONbits.TCKPS = 0b11;
            T2CONbits.TON = 1;
            break;
    }
}
int tmr_wait_period(int8_t timer) {
    int isNotMissed;
    switch (timer) {
        case 1:
            isNotMissed = IFS0bits.T1IF;
            while (!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
            break;
        case 2:
            isNotMissed = IFS0bits.T2IF;
            while (!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
            break;
    }
    return isNotMissed;
}

void tmr_wait_ms(int8_t timer, int ms) {
    unsigned long long ticks = (ms / 1000.0) * 281250; 
    switch (timer) {
        case 1:
            T1CONbits.TON = 0;
            T1CONbits.TCKPS = 0b11;
            TMR1 = 0;
            while (ticks > 65535) {
                ticks -= 65535;
                PR1 = 65535;
                T1CONbits.TON = 1;
                tmr_wait_period(TIMER1);
            }
            T1CONbits.TON = 0;
            PR1 = (unsigned int) ticks;
            T1CONbits.TON = 1;
            tmr_wait_period(TIMER1);
            break;
        case 2:
            T2CONbits.TON = 0;
            T2CONbits.TCKPS = 0b11;
            TMR2 = 0;
            while (ticks > 65535) {
                ticks -= 65535;
                PR2 = 65535;
                T2CONbits.TON = 1;
                tmr_wait_period(TIMER2);
            }
            T2CONbits.TON = 0;
            PR2 = (unsigned int) ticks;
            T2CONbits.TON = 1;
            tmr_wait_period(TIMER2);
            break;
    }
}

// initializing the SPI
void init_SPI1() {
    //configuring the SPI
    SPI1CON1bits.MSTEN = 1; // master mode
    SPI1CON1bits.MODE16 = 0; // 8?bit mode
    SPI1CON1bits.PPRE = 0; // 1:1 primary prescaler
    SPI1CON1bits.SPRE = 5; // 5:1 secondary prescaler
    SPI1CON1bits.CKP = 1; // setting the idle bit as 1
    SPI1STATbits.SPIEN = 1; // enable SPI

    //setting up pins
    TRISAbits.TRISA1 = 1; // RA1?RPI17 MISO
    TRISFbits.TRISF12 = 0; //RF12?RP108 SCK
    TRISFbits.TRISF13 = 0; // RF13?RP109 MOSI

    //reamapping the pins
    RPINR20bits.SDI1R = 0b0010001; // MISO (SDI1) ? RPI17
    RPOR12bits.RP109R = 0b000101; // MOSI (SDO1) ? RF13
    RPOR11bits.RP108R = 0b000110; // SCK1;
    
    //setting up chip select pins and making sure theyre not selected
    TRISBbits.TRISB3 = 0;
    TRISBbits.TRISB4 = 0;
    TRISDbits.TRISD6 = 0;

    LATBbits.LATB3 = 1;
    LATBbits.LATB4 = 1;
    LATDbits.LATD6 = 1;
}

// initializing the UART
void init_UART1(bool en_tx, bool en_rx) {  
    // remap input pin U1RX to RD11
    TRISDbits.TRISD11 = 1;
    RPINR18bits.U1RXR = 75;
    
    // remap output pin U1TX to RD0
    TRISDbits.TRISD0 = 0;
    RPOR0bits.RP64R = 1;

    //configuring the UART
    U1BRG = 468; // 72M / (16 * 9600) - 1
    U1MODEbits.UARTEN = 1; // enable UART
    U1STAbits.UTXEN = 1; // enabling transmission
    
    if(en_tx){
        //enabling the TX interrupt, it is triggered when UART is available for printing
        U1STAbits.UTXISEL0 = 0;
        U1STAbits.UTXISEL1 = 0;
        IEC0bits.U1TXIE = 1;
    }
    
    if(en_rx){
        //enabling the RX interrupt, it is triggered when UART is available for receiving
        U1STAbits.URXISEL0 = 0;
        U1STAbits.URXISEL1 = 0;
        IEC0bits.U1RXIE = 1;
    }
}

// a function to receive one char from SPI
static unsigned char recieve_SPI1() {
    //waits until the reading buffer is available
    while (SPI1STATbits.SPIRBF == 0);
    //reads then returns the read data
    unsigned char data = SPI1BUF;
    return data;
}

// a function to transmit one char through SPI
static unsigned char transmit_SPI1(unsigned char msg) {
    //waits until the writing buffer is available
    while (SPI1STATbits.SPITBF == 1);
    //sends the data
    SPI1BUF = msg;
    //read the resulting in case were reading, or just to avoid overrun
    unsigned char data = recieve_SPI1();
    return data;
}

// a function that writes specific data ona specific address
void write_SPI1(unsigned char addr, unsigned char msg) {
    // enabling the chip select
    CS = 0;
    //transmit the address then the message
    transmit_SPI1(addr);
    transmit_SPI1(msg);
    // disabling the chip select again
    CS = 1;
}

// a function that writes specific data ona specific address
unsigned char read_SPI1(unsigned char addr) {
    // enabling the chip select
    CS = 0;
    //transmit the address then recieve the message
    transmit_SPI1(addr | 0x80); // force msb to 1
    unsigned char reced = transmit_SPI1(0x00);
    // disabling the chip select again
    CS = 1;
    //returning the data
    return reced;
}

// setting up the magnetometer
void setup_mag() {
    //sleep mode
    write_SPI1(MAGSLEEPMODE, 0x01);

    //sleep for 2 ms
    tmr_wait_ms(TIMER2, 2);

    //active mode
    write_SPI1(MAGOPMODE, 0x00);
}

// a function that joins two characters
int16_t join_msb_lsb(unsigned char lsb, unsigned char msb) {
    return (msb << 8) | lsb;
}

// a function that gets the x axis value
int16_t mag_get_x() {
    // read the lsb and the msb from the sensor
    int16_t lsb = read_SPI1(MAGXLSB);
    int16_t msb = read_SPI1(MAGXMSB);

    lsb = (lsb & 0b11111000); // masking
    int16_t x = join_msb_lsb(lsb, msb); // merging the lsb and the msb
    x /= 8; // deleting the first unnecessary first 0s
    
    //fixing signs and returning the data
    return x;
}

// a function that gets the y axis value
int16_t mag_get_y() {
        // read the lsb and the msb from the sensor
    int16_t lsb = read_SPI1(MAGYLSB);
    int16_t msb = read_SPI1(MAGYMSB);
    lsb = (lsb & 0b11111000); // masking
    int16_t y = join_msb_lsb(lsb, msb); // merging the lsb and the msb
    y /= 8;  // deleting the first unnecessary first 0s

    //fixing signs and returning the data
    return y;
}

// a function that gets the z axis value
int16_t mag_get_z() {
    int16_t lsb = read_SPI1(MAGZLSB);
    int16_t msb = read_SPI1(MAGZMSB);
    lsb = (lsb & 0b11111110); // masking
    int16_t z = join_msb_lsb(lsb, msb); // merging the lsb and the msb
    z /= 2; // deleting the first unnecessary first 0s

    //fixing signs and returning the data
    return z;
}

// Initialize the circular buffer
void initCircularBuffer(CircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

// Check if the circular buffer is empty
int isEmpty(CircularBuffer *cb) {
    return (cb->count == 0);
}

// Check if the circular buffer is full
int isFull(CircularBuffer *cb) {
    return (cb->count == BUFFER_SIZE);
}

// Enqueue an element into the circular buffer
void enqueue(CircularBuffer *cb, char value) {
    if (isFull(cb)) {
        return;
    }
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % BUFFER_SIZE;
    cb->count++;
}

// Dequeue an element from the circular buffer
int dequeue(CircularBuffer *cb, char *value) {
    if (isEmpty(cb)) {
        return 0; // Assuming 0 represents an error condition
    }
    *value = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % BUFFER_SIZE;
    cb->count--;
    
    return 1; // Assuming 1 represents a successful condition
}

// Initialize the circular buffer
void CMDinitCircularBuffer(CMDCircularBuffer *cb) {
    cb->head = 0;
    cb->tail = 0;
    cb->count = 0;
}

// Check if the circular buffer is empty
int CMDisEmpty(CMDCircularBuffer *cb) {
    return (cb->count == 0);
}

// Check if the circular buffer is full
int CMDisFull(CMDCircularBuffer *cb) {
    return (cb->count == MAX_CMD);
}

// Enqueue an element into the circular buffer
void CMDenqueue(CMDCircularBuffer *cb, parser_state value) {
    if (isFull(cb)) {
        return;
    }
    cb->buffer[cb->head] = value;
    cb->head = (cb->head + 1) % MAX_CMD;
    cb->count++;
}

// Dequeue an element from the circular buffer
int CMDdequeue(CMDCircularBuffer *cb, parser_state *value) {
    if (CMDisEmpty(cb)) {
        return 0; // Assuming 0 represents an error condition
    }
    *value = cb->buffer[cb->tail];
    cb->tail = (cb->tail + 1) % MAX_CMD;
    cb->count--;
    
    return 1; 
}

void init_adc(){
    ANSELA = ANSELB = ANSELC = ANSELD = ANSELE = ANSELG = 0x0000;
    
    //automatic starting automatic ending
    AD1CON3bits.ADCS = 8; // choosing the size of the TAD
    AD1CON1bits.ASAM = 1; // automatic sampling
    AD1CON1bits.SSRC = 7; // automatic conversion
    AD1CON3bits.SAMC = 16; // sampling size in TAD
    
    AD1CON2bits.CHPS = 0; // selecting channel 0
    AD1CON2bits.CSCNA = 1; // scan mode  
    AD1CON2bits.SMPI = 1; //number of conversions we scan
    
    AD1CSSLbits.CSS14 = 1; // activating RB14 for the IR sensor
    AD1CSSLbits.CSS11 = 1; // activating RB11 for the battery
    
    ANSELBbits.ANSB14 = 1; // setting the RB14 bit to be an analog pin
    ANSELBbits.ANSB11 = 1; // setting the RB11 bit to be an analog pin
    
    TRISBbits.TRISB11 = 1; // setting the RB11 as input
    TRISBbits.TRISB14 = 1; // setting the RB14 as input
    
    AD1CON1bits.ADON = 1; // starting the ADC
    
    // enabling the IR sensor
    TRISBbits.TRISB9 = 0; 
    IRENABLE = 1;

}
void set_up_PWM_wheels(){
    OC1CON1 = OC1CON2 = OC2CON1 = OC2CON2 = OC3CON1 = OC3CON2 = OC4CON1 = OC4CON2 = 0;

    // select input clock for the OC1 module
    OC1CON1bits.OCTSEL = 7; 
    OC2CON1bits.OCTSEL = 7; 
    OC3CON1bits.OCTSEL = 7;
    OC4CON1bits.OCTSEL = 7;

    // setting the OCMs as Edge-Aligned
    OC1CON1bits.OCM = 0b110;
    OC2CON1bits.OCM = 0b110;
    OC3CON1bits.OCM = 0b110;
    OC4CON1bits.OCM = 0b110;

    // no sync needed
    OC1CON2bits.SYNCSEL = 0x1F;
    OC2CON2bits.SYNCSEL = 0x1F;
    OC3CON2bits.SYNCSEL = 0x1F;
    OC4CON2bits.SYNCSEL = 0x1F;

    // Remapping pins
    RPOR0bits.RP65R = 0b010000; // RD1 is RP65
    RPOR1bits.RP66R = 0b010001; // RD2 is RP66
    RPOR1bits.RP67R = 0b010010; // RD3 is RP67
    RPOR2bits.RP68R = 0b010011; // RD4 is RP68
    
    // Tpwm/Tcy = 7200 with Tpwm = 1/10kHz  number of TMR pulse is equal to 7200 to match the frequency of 10kHz
    OC1RS = OC2RS = OC3RS = OC4RS = PWMFREQUENCY; 
}   

void stop_moving(){
    LEFTF  = 0;
    LEFTB  = 0;
    RIGHTF  = 0;
    RIGHTB  = 0;
}

int parse_byte(parser_state* ps, char byte) {
    switch (ps->state) {
        case STATE_DOLLAR:
            if (byte == '$') {
                ps->state = STATE_TYPE;
                ps->index_type = 0;
            }
            break;
        case STATE_TYPE:
            if (byte == ',') {
                ps->state = STATE_PAYLOAD;
                ps->msg_type[ps->index_type] = '\0';
                ps->index_payload = 0; // initialize properly the index
            } else if (ps->index_type == 6) { // error! 
                ps->state = STATE_DOLLAR;
                ps->index_type = 0;
            } else if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_type[ps->index_type] = '\0';
                ps->msg_payload[0] = '\0'; // no payload
                return NEW_MESSAGE;
            } else {
                ps->msg_type[ps->index_type] = byte; // ok!
                ps->index_type++; // increment for the next time;
            }
            break;
        case STATE_PAYLOAD:
            if (byte == '*') {
                ps->state = STATE_DOLLAR; // get ready for a new message
                ps->msg_payload[ps->index_payload] = '\0';
                return NEW_MESSAGE;
            } else if (ps->index_payload == 100) { // error
                ps->state = STATE_DOLLAR;
                ps->index_payload = 0;
            } else {
                ps->msg_payload[ps->index_payload] = byte; // ok!
                ps->index_payload++; // increment for the next time;
            }
            break;
    }
    return NO_MESSAGE;
}

int extract_integer(const char* str) {
    int i = 0, number = 0, sign = 1;

    if (str[i] == '-') {
        sign = -1;
        i++;
    }
    else if (str[i] == '+') {
        sign = 1;
        i++;
    }
    while (str[i] != ',' && str[i] != '\0') {
        number *= 10; // multiply the current number by 10;
        number += str[i] - '0'; // converting character to decimal number
        i++;
    }
    return sign*number;
}		

int next_value(const char* msg, int i) {
    while (msg[i] != ',' && msg[i] != '\0') { i++; }
    if (msg[i] == ',')
        i++;
    return i;
}

void get_distance_and_battery(double* distance, double* battery) {
    AD1CON1bits.DONE = 0;
    while (!AD1CON1bits.DONE);

    int ADCIR = ADC1BUF1;
    int ADCbattery = ADC1BUF0;

    double x = ADCIR * 3.3 / 1023.0;     
    *distance =(2.34 - 4.74 * x + 4.06 * pow(x, 2) - 1.6 * pow(x, 3) + 0.24 * pow(x, 4))*100;

    double y = ADCbattery * 3.3 / 1023.0;     
    *battery = y * 3;
}

void enqueue_buffer(CircularBuffer *cb, char* m) {
    // We queue all the characters of the string to be printed to the UART
    int c = 0;
    // We disable the TX interrupt flag to make sure we won't have an interrupt while updating the buffer.
    IEC0bits.U1TXIE = 0;
    do {
        enqueue(cb, m[c]);
        c++;
    } while (m[c] != '\0');
    
    // We re-enable the interrupt when we are out of critical area
    IEC0bits.U1TXIE = 1;
}

void init_LED(){
    TRISBbits.TRISB8 = 0; // left side lights
    TRISFbits.TRISF1 = 0; // right side lights
    TRISFbits.TRISF0 = 0; // breaks lights
    TRISGbits.TRISG1 = 0; // low intensity lights
    TRISAbits.TRISA7 = 0; // beam lights
    TRISAbits.TRISA0 = 0; // beam lights
    turnoff_lights(); 
}

void turnoff_lights(){
    LIGHTLEFT = 0;
    LIGHTRIGHT = 0;
    LED = 0;
}

// Direct PWM control based on left/right speeds (used in main state logic)
void apply_motor_pwm(int left_pwm, int right_pwm) {
    if (left_pwm > 100) left_pwm = 100;
    if (right_pwm > 100) right_pwm = 100;
    if (left_pwm < -100) left_pwm = -100;
    if (right_pwm < -100) right_pwm = -100;
    
    // Left wheels
    if (left_pwm >= 0) {
        LEFTF = PWMFREQUENCY * left_pwm / 100;
        LEFTB = 0;
    } else {
        LEFTF = 0;
        LEFTB = PWMFREQUENCY * (-left_pwm) / 100;
    }

    // Right wheels
    if (right_pwm >= 0) {
        RIGHTF = PWMFREQUENCY * right_pwm / 100;
        RIGHTB = 0;
    } else {
        RIGHTF = 0;
        RIGHTB = PWMFREQUENCY * (-right_pwm) / 100;
    }
}





