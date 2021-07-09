#include "SysTickInts.h"
#include "PLL.h"
#include "tm4c123gh6pm.h"

void PortF_Init(void);
void PortA_Init(void);
void PortB_Init(void);
void PortD_Init(void);
void Buttons_Init(void);
void Motor_Init(void);

void disable_interrupts(void);
void enable_interrupts(void);
void wait_for_interrupts(void);

void SysTick_Handler(void);
void GPIO_PortF_Handler(void);
void GPIO_PortB_Handler(void);
void GPIO_PortA_Handler(void);

void MSDelay(unsigned int time);
void SevenSegmentTest(void);

/*
 *  Current Ports Config. (April 8, 2020)
 * Port F (all are inputs)
 *  PF0 = start, PF1 = program 3, PF2 = program 2, PF3 = program 1, PF4 = cancel
 * Port B
 *  PB0-3 = ABCD for seven segment (output), PB4 = buzzer (output), PB5 = door (input), PB6 = reset
 * Port D
 *  PD1 = motor direction (output), PD2 = IR speed feedback (input)
 * Port A
 *  PA5 = motor control (output with PWM)
 *
 * Need to confirm
 *  Motor direction default = CCW
 *  Motor direction 0 = CCW ??
 *  Motor direction 1 = CW ??
 *
 * March 16, 2020
 * 1. need to figure out motor control (PD0), try to implement PWM to change the speed
 * 2. need to figure out level-sensitive button for door (PB5), properly configure ports
 * 3. clicking start or cancel button will change the direction of the motor
 * 4. when program 1,2,3 is 1, and if pressed start -> buzzer will be on and display 9
 * 5. LEDs seems to work but not reliably (sometimes the leds stay ON once unpressed, sometimes it doesn't)
 *
 * March 23, 2020
 * 1. in order for the button (interrupts) to works properly motor control switch (on-board) must be in CTC mode... for some reason?
 * 2. in order to control the motor with the board, the motor control switch (on-board) should be in Port mode... (issue with point 1)
 * 3. still need to figure out the door button
 * 4. motor direction control is unreliable (sometimes delayed, sometimes doesn't change)
 *
 * April 8, 2020
 * 1. motor control and direction is complete
 * 2. display shows the current program selection by checking in the main (before, we checked in the accept interrupt)
 * 3. there problem will starting a motor to a low speed (need to push to start the motor sometimes)
 *
 * About I/O port config
 * http://users.ece.utexas.edu/~valvano/Volume1/E-Book/C6_MicrocontrollerPorts.htm
 * http://shukra.cedt.iisc.ernet.in/edwiki/EmSys:Programming_the_GPIO_in_TM4C123
 *
 * About interrupts
 * https://users.ece.utexas.edu/~valvano/Volume1/E-Book/C12_Interrupts.htm
 * https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-gpio/digital-input-with-interrupt
 *
 */

// control 7-seg directly
#define n0 0x3F; // 0011 1111
#define n1 0x06; // 0000 0110
#define n2 0x5B; // 0101 1011
#define n3 0x4F; // 0100 1111
#define n4 0x66; // 0110 0110
#define n5 0x6D; // 0110 1101
#define n6 0x7D; // 0111 1101
#define n7 0x07; // 0000 0111
#define n8 0xFF; // 1111 1111
#define n9 0x67; // 0110 0111

// control 7-seg with BCD
#define b0 0x00; // 0000 0000
#define b1 0x01; // 0000 0001
#define b2 0x02; // 0000 0010
#define b3 0x03; // 0000 0011
#define b4 0x04; // 0000 0100
#define b5 0x05; // 0000 0101
#define b6 0x06; // 0000 0110
#define b7 0x07; // 0000 0111
#define b8 0x08; // 0000 1000
#define b9 0x09; // 0000 1001
#define buzzerON 0x10; // 0001 0000
#define buzzerOFF 0x00

volatile unsigned int time, program1, program2, program3, motorDirection;
volatile unsigned int dummy;
unsigned long H,L;


void main(void)
{
    PortF_Init();
    PortB_Init();
    PortD_Init();
    //PortA_Init();
    Buttons_Init();
    PLL_Init();           // bus clock at 80 MHz
    Motor_Init();         // output from PA5, SysTick interrupts
    time = 1000;
    program1,program2,program3,dummy = 0;
    //SevenSegmentTest();   //run all number in loop
    GPIO_PORTB_DATA_R = b0;
    GPIO_PORTB_DATA_R = 0x00; //PB4 = buzzer
    GPIO_PORTD_DATA_R = 0x02; //PD0 and PD1 = 0
    enable_interrupts();
    while(1){
        wait_for_interrupts();
        if (program1 == 0 && program2 == 0 && program3 == 0){ //1. white wash
            GPIO_PORTB_DATA_R = b1; // display 1
        }else if(program1 == 0 && program2 == 0 && program3 == 1){
            GPIO_PORTB_DATA_R = b2; // display 2
        }else if(program1 == 0 && program2 == 1 && program3 == 0){
            GPIO_PORTB_DATA_R = b3; // display 3
        }else if(program1 == 0 && program2 == 1 && program3 == 1){
            GPIO_PORTB_DATA_R = b4; // display 4
        }else if(program1 == 1 && program2 == 0 && program3 == 0){
            GPIO_PORTB_DATA_R = b5; // display 5
        }else if(program1 == 1 && program2 == 0 && program3 == 1){
            GPIO_PORTB_DATA_R = b6; // display 6
        //}else if(program1 == 1 && program2 == 1 && program3 == 1){ // idle mode
        //    GPIO_PORTB_DATA_R = b0; // display 0
        }
    }
}

void GPIO_PortB_Handler(void){
    // this part doesn't work
    if (GPIO_PORTB_RIS_R & 0x20){ //once door button is pressed
        GPIO_PORTB_ICR_R = 0x20;
        disable_interrupts();
        dummy = 10;
        GPIO_PORTB_DATA_R = buzzerON;
        //GPIO_PORTB_DATA_R = !GPIO_PORTB_DATA_R;
        enable_interrupts();
    }
}

void GPIO_PortA_Handler(void){
    // not used
}

void GPIO_PortF_Handler(void){
    if (GPIO_PORTF_RIS_R & 0x01){ //accept
        GPIO_PORTF_ICR_R = 0x01;
        disable_interrupts();
        GPIO_PORTB_DATA_R = b9;
        if (program1 == 0 && program2 == 0 && program3 == 0){ //1. white wash
            GPIO_PORTB_DATA_R = b1; // display 1
        }
        if (program1 == 0 && program2 == 0 && program3 == 1){ //2. colour cycle
            int timer = 100000;
            GPIO_PORTB_DATA_R = b2; // display 2
            MSDelay(5000*2);
            GPIO_PORTB_DATA_R = buzzerON;
            MSDelay(5000+2000);
            GPIO_PORTB_DATA_R = b1; //empty
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b2; //fill
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b3; //heat
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b4; //wash
            L = 56000; H = 24000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b5; //rinse
            L = 64000; H = 16000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b6; //spin
            L = 72000; H = 8000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b7; //dry
            GPIO_PORTD_DATA_R = 0x00;
            L = 72000; H = 8000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = 0x18; //complete
            MSDelay(5000+2000);
            L = 8000; H = 8000;
            GPIO_PORTB_DATA_R = b0;
            program1 = 0;
            program2 = 0;
            program3 = 0;
        }
        if (program1 == 0 && program2 == 1 && program3 == 0){ //3. mixed wash
            GPIO_PORTB_DATA_R = b3; // display 3
        }
        if (program1 == 0 && program2 == 1 && program3 == 1){ //4. economy wash
            int timer = 100000;
            GPIO_PORTB_DATA_R = b4; // display 2
            MSDelay(5000*2);
            GPIO_PORTB_DATA_R = buzzerON;
            MSDelay(5000+2000);
            GPIO_PORTB_DATA_R = b1; //empty
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b2; //fill
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b3; //heat
            MSDelay(20000);
            GPIO_PORTB_DATA_R = b4; //wash
            GPIO_PORTD_DATA_R = 0x00;
            L = 72000; H = 8000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b5; //rinse
            L = 56000, H = 24000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b6; //spin
            L = 72000; H = 8000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = b7; //dry
            GPIO_PORTD_DATA_R = 0x02;
            L = 72000; H = 8000;
            MSDelay(timer);
            GPIO_PORTB_DATA_R = 0x18; //complete
            MSDelay(5000+2000);
            L = 8000; H = 8000;
            GPIO_PORTB_DATA_R = b0;
            program1 = 0;
            program2 = 0;
            program3 = 0;
        }
        if (program1 == 1 && program2 == 0 && program3 == 0){ //5. program 1
            GPIO_PORTB_DATA_R = b5; // display 5
        }
        if (program1 == 1 && program2 == 0 && program3 == 1){ //6. program 2
            GPIO_PORTB_DATA_R = b6; // display 6
        }
        if (program1 == 1 && program2 == 1 && program3 == 1){ //9. test buzzer
            GPIO_PORTB_DATA_R = 0x19; // display 9 and buzzer ON
        }
        //if(L>8000) L = L-8000;    // slow down
        enable_interrupts();
    }
    if (GPIO_PORTF_RIS_R & 0x02){ //program 3
        GPIO_PORTF_ICR_R = 0x02;
        disable_interrupts();
        //GPIO_PORTB_DATA_R = b3;
        program3 = !program3;
        enable_interrupts();
    }
    if (GPIO_PORTF_RIS_R & 0x04){ //program 2
        GPIO_PORTF_ICR_R = 0x04;
        disable_interrupts();
        //GPIO_PORTB_DATA_R = b2;
        program2 = !program2;
        enable_interrupts();
    }
    if (GPIO_PORTF_RIS_R & 0x08){ //program 1
        GPIO_PORTF_ICR_R = 0x08;
        disable_interrupts();
        //GPIO_PORTB_DATA_R = b1;
        program1 = !program1;
        enable_interrupts();
    }
    if (GPIO_PORTF_RIS_R & 0x10){ //cancel
        GPIO_PORTF_ICR_R = 0x10;
        disable_interrupts();
        program1 = 0;
        program2 = 0;
        program3 = 0;
        GPIO_PORTB_DATA_R = b0;
        //GPIO_PORTD_DATA_R = 0x02;
        //if(L<72000) L = L+8000;   // speed up
        //enable_interrupts();
    }
    //H = 80000-L; // constant period of 1ms, variable duty cycle
}

void SysTick_Handler(void){
  if(GPIO_PORTA_DATA_R&0x20){   // toggle PA5
    GPIO_PORTA_DATA_R &= ~0x20; // make PA5 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{
    GPIO_PORTA_DATA_R |= 0x20;  // make PA5 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
}

void Motor_Init(void){
    SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
    H = L = 8000;                 // 10%
    GPIO_PORTA_AMSEL_R &= ~0x20;      // disable analog functionality on PA5
    GPIO_PORTA_PCTL_R &= ~0x00F00000; // configure PA5 as GPIO
    GPIO_PORTA_DIR_R |= 0x20;     // make PA5 out
    GPIO_PORTA_DR8R_R |= 0x20;    // enable 8 mA drive on PA5
    GPIO_PORTA_AFSEL_R &= ~0x20;  // disable alt funct on PA5
    GPIO_PORTA_DEN_R |= 0x20;     // enable digital I/O on PA5
    GPIO_PORTA_DATA_R &= ~0x20;   // make PA5 low
    NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
    NVIC_ST_RELOAD_R = L-1;       // reload value for 500us
    NVIC_ST_CURRENT_R = 0;        // any write to current clears it
    NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
    NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void Buttons_Init(void) {
    //Port F interrupt init
    GPIO_PORTF_IS_R &= ~0x1F;          // (d) All PortF are edge-sensitive
    GPIO_PORTF_IBE_R &= ~0x1F;         //     All PortF are not both edges
    GPIO_PORTF_IEV_R &= ~0x1F;         //     All PortF falling edge event
    GPIO_PORTF_ICR_R = 0x1F;           // (e) clear flags
    GPIO_PORTF_IM_R |= 0x1F;           // (f) arm interrupt on All PortF
    /*
    //Port F interrupt init
    GPIO_PORTB_IS_R &= 0xFF;          // (d) All PortB are level-sensitive
    GPIO_PORTB_IBE_R &= ~0xFF;         //    All PortB is not both edges
    GPIO_PORTB_IEV_R &= ~0xFF;         //    All PortB falling edge event, low-level trigger
    GPIO_PORTB_ICR_R = 0xFF;           // (e) clear flags
    GPIO_PORTB_IM_R |= 0xFF;           // (f) arm interrupt on All PortB
    */
    NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
    NVIC_EN0_R = 0x40000000;
}

void PortA_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000001;           // activate clock for PortA
    while ((SYSCTL_PRGPIO_R & 0x00000001) == 0){}; // wait until PortA is ready
    GPIO_PORTA_LOCK_R = 0x4C4F434B;         // unlock GPIO PortA
    GPIO_PORTA_CR_R = 0xFF;                 // allow changes to PF4-0
    GPIO_PORTA_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTA_PCTL_R = 0x00000000;         // use PF4-0 as GPIO
    GPIO_PORTA_DIR_R = 0x00;                // PF4-0 are inputs
    GPIO_PORTA_AFSEL_R = 0x00;              // disable alt function on PF
    GPIO_PORTA_PUR_R = 0x00; //orig. FF     // enable pull-up on PF4-0
    GPIO_PORTA_DEN_R = 0xFF;                // enable digital I/O on PF4-0
}

void PortF_Init(void) {
    // there are only 5 PF, PF0,PF1,PF2,PF3,PF4
    SYSCTL_RCGC2_R |= 0x00000020;           // activate clock for PortF
    while ((SYSCTL_PRGPIO_R & 0x00000020) == 0){}; // wait until PortF is ready
    GPIO_PORTF_LOCK_R = 0x4C4F434B;         // unlock GPIO PortF
    GPIO_PORTF_CR_R = 0x1F;                 // allow changes to PF4-0
    GPIO_PORTF_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTF_PCTL_R = 0x00000000;         // use PF4-0 as GPIO
    GPIO_PORTF_DIR_R = 0x00;                // PF4-0 are inputs
    GPIO_PORTF_AFSEL_R = 0x00;              // disable alt function on PF
    GPIO_PORTF_PUR_R = 0x1F;                // enable pull-up on PF4-0
    GPIO_PORTF_DEN_R = 0x1F;                // enable digital I/O on PF4-0
}

void PortB_Init(void){
    SYSCTL_RCGC2_R |= 0x00000002;     // 1) B clock
    while ((SYSCTL_PRGPIO_R & 0x0000002) == 0){}; // wait until PortB is ready
    GPIO_PORTB_CR_R = 0x7F;           // allow changes to PB6-0
    GPIO_PORTB_AMSEL_R = 0x00;        // 3) disable analog function
    GPIO_PORTB_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
    GPIO_PORTB_DIR_R = 0x1F;          // 5) PB5,PB7 input, PB0,1,2,3,4,6 output
    GPIO_PORTB_AFSEL_R = 0x00;        // 6) no alternate function
    GPIO_PORTB_PUR_R = 0x00;          // enable pullup resistors on PF4,PF0???
    GPIO_PORTB_DEN_R = 0x7F;          // 7) enable digital pins PB6-PF0
}

void PortD_Init(void) {
    SYSCTL_RCGC2_R |= 0x00000008;           // activate clock for PortD
    while ((SYSCTL_PRGPIO_R & 0x00000008) == 0){}; // wait until PortD is ready
    GPIO_PORTD_CR_R = 0xFF;                 // allow changes to PD0,1
    GPIO_PORTD_AMSEL_R = 0x00;              // disable analog on PortF
    GPIO_PORTD_PCTL_R = 0x00000000;         // use PD4-0 as GPIO
    GPIO_PORTD_DIR_R = 0x03;                // PD0 and PD1 out
    GPIO_PORTD_AFSEL_R = 0x00;              // disable alt function on PD
    GPIO_PORTD_PUR_R = 0x00;                // enable pull-up on PD
    GPIO_PORTD_DEN_R = 0xFF;                // enable digital I/O on PD4-0
}

/* Disable interrupts by setting the I bit in the PRIMASK system register */
void disable_interrupts(void){
    __asm("    CPSID  I\n"      // __asm is for using assembly code
          "    BX     LR");
}

/* Enable interrupts by clearing the I bit in the PRIMASK system register */
void enable_interrupts(void){
    __asm("    CPSIE  I\n"
          "    BX     LR");
}

/* Enter low-power mode while waiting for interrupts */
void wait_for_interrupts(void){
    __asm("    WFI\n"
          "    BX     LR");
}
void MSDelay(unsigned int time) {
    unsigned int i;
    unsigned int j;
    for(i=0;i<time;i++){
        for(j=0;j<331;j++);
    }
}

void SevenSegmentTest(void){ //cycle through all the number to see if the connection is okay
    GPIO_PORTB_DATA_R = b0;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b1;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b2;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b3;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b4;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b5;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b6;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b7;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b8;
    MSDelay(time);
    GPIO_PORTB_DATA_R = b9;
    MSDelay(time);
}
