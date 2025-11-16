/*
 * Mokrani Lisa 7229301
 * Aguenarous Mohamed 6752572
 */

#include "xc.h"
#include "tools.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

// === Global Variables ===
parser_state pstate;
int speed = 0, yawrate = 0;
volatile int state = STATE_WAIT;
volatile unsigned int counter1ms = 0;
unsigned int emergencyCounter = 0;
volatile bool printingFlag = false;

CircularBuffer cb;        // TX buffer
CMDCircularBuffer cbCMD;  // Command RX buffer

// an interrupt triggered when the UART is ready to print
void __attribute__((__interrupt__, __auto_psv__)) _U1TXInterrupt() {
    //resetting the interrupt flag
    IFS0bits.U1TXIF = 0;
    //printing the queued data
    char ch;
    if (dequeue(&cb, &ch)) {
        //printing the next character on the circular buffer if any are available
        U1TXREG = ch;
    }
}

// an interrupt triggered when the UART is ready to print
void __attribute__((__interrupt__, __auto_psv__)) _U1RXInterrupt() {
    //resetting the interrupt flag
    IFS0bits.U1RXIF = 0;
    //reading the character
    char rec = U1RXREG;
    
    //parsing the received data until the command is complete
    if (parse_byte(&pstate, rec)) {
        if (CMDisFull(&cbCMD)) {
            //sending the acknowledgment that the FIFO is full, thus, the command was ignored
            enqueue_buffer(&cb, "$MACK,0*");  
        } else {
            // // enqueuing the command
            CMDenqueue(&cbCMD, pstate);
            //sending acknowledgment that the command was received and enqueued
            enqueue_buffer(&cb, "$MACK,1*");
        }
        printingFlag = true;
        IFS0bits.U1TXIF = 1;
    }
}

//  Using timer to avoid debouncing and changing the state when the button is clicked
void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt() {
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 0;
    T1CONbits.TON = 0;

    if (PORTEbits.RE8 == 1 && state != STATE_EMERGENCY) {
        state = (state == STATE_WAIT) ? STATE_MOVING : STATE_WAIT;
    }
}

// the interrupt for the button
void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() {
    IFS1bits.INT1IF = 0;
    IEC0bits.T1IE = 1;
    tmr_setup_period(TIMER1, 50); // debounce period
}

//  Main 
int main(void) {
    //initializing
    init_adc();
    initCircularBuffer(&cb);
    CMDinitCircularBuffer(&cbCMD);
    init_UART1(true, true); //initiating UART to print results and enabling RX and TX interrupts
    
    set_up_PWM_wheels();
    init_LED();
    init_SPI1();
    setup_mag();
    
    // initializing parser state
    pstate.state = STATE_DOLLAR;

    // setting up the button that changes the state with the interrupt 1
    TRISEbits.TRISE8 = 1;
    RPINR0bits.INT1R = 88;
    INTCON2bits.GIE = 1;
    IFS1bits.INT1IF = 0;
    IEC1bits.INT1IE = 1;

    tmr_setup_period(TIMER2, 2); // 500Hz loop
    double battery, distance;
    
    // infinite loop
    while (1) {
        get_distance_and_battery(&distance, &battery);
        char msg[BUFFER_SIZE];

        // printing the distance and the acceleration
        if (counter1ms % 50 == 0) {
            sprintf(msg, "$MDIST,%d*", (int)distance); // Format the output
            enqueue_buffer(&cb, msg);
          
            sprintf(msg, "$MACC,%d,%d,%d*", mag_get_x(), mag_get_y(), mag_get_z()); // Format the output
            enqueue_buffer(&cb, msg);
            printingFlag = true;
        }
        
        // printing the battery
        if (counter1ms % 500 == 0) {
            sprintf(msg, "$MBATT,%.2f*", battery); // Format the output
            enqueue_buffer(&cb, msg);
            LED = !LED;
            printingFlag = true;
        }
        
        // forcing the first print
        if (!isEmpty(&cb) && printingFlag) {
            IFS0bits.U1TXIF = 1;
            printingFlag = false;
        }

        // Command Processor
        parser_state cmd;
        if (CMDdequeue(&cbCMD, &cmd)) {
            if (strcmp(cmd.msg_type, "PCREF") == 0) {
                int i = 0;
                speed = extract_integer(&cmd.msg_payload[i]);
                i = next_value(cmd.msg_payload, i);
                yawrate = extract_integer(&cmd.msg_payload[i]);

                // Clamp to [-100, 100]
                if (speed > 100) speed = 100;
                if (speed < -100) speed = -100;
                if (yawrate > 100) yawrate = 100;
                if (yawrate < -100) yawrate = -100;

            } else if (strcmp(cmd.msg_type, "PCSTP") == 0) {
                if (state != STATE_EMERGENCY) state = STATE_WAIT;
            } else if (strcmp(cmd.msg_type, "PCSTT") == 0) {
                if (state != STATE_EMERGENCY) state = STATE_MOVING;
            }
        }

        //handling our different states
        switch (state) {
            case STATE_WAIT:
                stop_moving();
                break;

            case STATE_MOVING:
                // check for obstacles
                if (distance < 20) {
                    state = STATE_EMERGENCY;
                    stop_moving();
                    enqueue_buffer(&cb, "$MEMRG,1*");
                    printingFlag = true;
                    IFS0bits.U1TXIF = 1;
                    emergencyCounter = 0;
                } else {
                    int left = speed - yawrate;
                    int right = speed + yawrate;
                    int left_pwm = (PWMFREQUENCY * abs(left)) / 100;
                    int right_pwm = (PWMFREQUENCY * abs(right)) / 100;

                    apply_motor_pwm((left >= 0) ? left_pwm : -left_pwm,
                                    (right >= 0) ? right_pwm : -right_pwm);
                }
                break;

            case STATE_EMERGENCY:
                stop_moving();
                // toggling the lights on a 1Hz rate
                if (counter1ms % 500 == 0) {
                    LIGHTLEFT = !LIGHTLEFT;
                    LIGHTRIGHT = !LIGHTRIGHT;
                }
                if (distance >= 20) emergencyCounter++;
                else emergencyCounter = 0;

                if (emergencyCounter >= 2500) {
                    state = STATE_WAIT;
                    enqueue_buffer(&cb, "$MEMRG,0*");
                    LIGHTLEFT = 0;
                    LIGHTRIGHT = 0;
                    printingFlag = true;
                    IFS0bits.U1TXIF = 1;
                }
                break;
        }
        // incrementing the counter to count different times as needed 
        counter1ms++;
        
        tmr_wait_period(TIMER2);
    }
    return 0;
}





