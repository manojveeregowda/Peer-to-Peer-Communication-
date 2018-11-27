//--------------------------------------------------------------------------------------------
//Project code.
//Name:   Manoj Mysore Veere Gowda.
//Course: Embedded system[5314], Fall 2017.
//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------
//Including Libraries.
//--------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include "tm4c123gh6pm.h"
//--------------------------------------------------------------------------------------------
//Defining variables and constants.
//--------------------------------------------------------------------------------------------
#define MAX_CHARS      90
#define MAX_COUNT      25
#define REFMAX_CHARS   90
#define Max_Msgs       255
#define GREEN_LED      PWM1_3_CMPB_R
#define BLUE_LED       PWM1_3_CMPA_R
#define RED_LED        PWM1_2_CMPB_R
#define ENABLE         (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define GREEN_ON_BOARD (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))
#define RED_ON_BOARD   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))

char* strg;
char type[100];
char string[50];
char on, off, set, cs;
char value[MAX_CHARS+1];
char str[MAX_CHARS+1];
char struart[MAX_CHARS+1];
char strref[REFMAX_CHARS+1];

bool ackreq[MAX_COUNT] = {0};
bool random=0;

uint8_t count=0;
uint8_t da;
uint8_t rx_phase;
uint8_t retry_count=0;
uint8_t receive_phase, oldphase;
uint8_t source_add=7;
uint8_t broadcast_add=255;
uint8_t index=0;
uint8_t position[100];
uint8_t i,a=0,v=0,ACK = 0,p;
uint8_t look = 1;
uint8_t field = 0,o;
uint8_t rx_data[9];
uint8_t n[5]={0,1,2,3,4};
uint8_t m[15]={1,0,4,2,3,0,2,3,0,2,1,2,3,0,3};
uint8_t rx_data[9];
uint8_t receive_data[50];
uint8_t destinationADDR[MAX_COUNT];
uint8_t command[MAX_COUNT];
uint8_t channel1[MAX_COUNT];
uint8_t size1[MAX_COUNT];
uint8_t data[MAX_COUNT][MAX_COUNT];
uint8_t checksum[MAX_COUNT];
uint8_t sequenceID;
uint8_t sqID[MAX_COUNT];
uint8_t valid[MAX_COUNT];
uint8_t retrans_count[MAX_COUNT] ={0};
uint8_t current_index,in_progress,current_phase;
uint8_t  w=0,s=0,l,p=0;
uint8_t receive_ckecksum,u,curr_index,y;
uint8_t command_set;
uint8_t size, variable1,z1=0,z2=0;
uint8_t addr, chan, val[MAX_COUNT], num, c, d, naddr,x;

uint16_t f, t0=100, t=500;
uint16_t Max_retries=3000;
uint16_t x1;
uint16_t returns_timeout=3000;
uint16_t h=0,x1,a1,b1;
uint16_t g=0,r1,t1,r2,t2,r3,t3,r4,t4,a1,a2,b1,b2,x2,c1,c2,d1,d2,a3,a4,x3,g1,c3,z3,p1;
uint16_t retrans_timeout[MAX_COUNT] = {0};
uint16_t z,k;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Initialize Hardware
//-----------------------------------------------------------------------------

void initHw()
{
    ///EEPROM initialization.
    SYSCTL_RCGCEEPROM_R |= 0x01;
    __asm(" NOP");
    __asm(" NOP");
    __asm(" NOP");
    EEPROM_EESIZE_R |=0X00010000;
    EEPROM_EEBLOCK_R|=0X00;
    EEPROM_EEOFFSET_R|=0X05;

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (nssot needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC;

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0E;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0E; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x0E;  // enable LEDs and pushbuttons
    GPIO_PORTF_AFSEL_R = 0x0E;
    GPIO_PORTF_PCTL_R = GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7 ;


    // Configure LED and pushbutton pins on board Port A
    GPIO_PORTA_DIR_R = 0xC0;  // bits 6 and 7 are outputs, other pins are inputs
    GPIO_PORTA_DR2R_R = 0xC0; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTA_DEN_R = 0xC0;  // enable LEDs


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Configure UART1 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         // turn-on UART1, leave other uarts in same status
    GPIO_PORTC_DIR_R |= 0x60;
    GPIO_PORTC_DEN_R |= 0x70;                           // default, added for clarity
    GPIO_PORTC_AFSEL_R |= 0x30;                         // default, added for clarity
    GPIO_PORTC_PCTL_R |= 0x00220000;


   // Configure UART1 to 38400 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
   UART1_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
   UART1_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
   UART1_IBRD_R = 65;                               // r = 40 MHz / (Nx384.0kHz), set floor(r)=21, where N=16
   UART1_FBRD_R = 7;                               // round(fract(r)*64)=45
   UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_EPS | UART_LCRH_SPS | UART_LCRH_PEN; // configure for 8N1 w/ 16-level FIFO
   UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

   // Configure Timer 1 as the time base
   SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       // turn-on timer
   TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
   TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
   TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
   TIMER1_TAILR_R = 0x9C40;                        // set load value to 40e3 for 1 Hz interrupt rate
   TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
   NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
   TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

   //PWM Hardware Initialization
   SYSCTL_RCGC0_R |= SYSCTL_RCGC0_PWM0;
   SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
   __asm(" NOP");
   __asm(" NOP");
   __asm(" NOP");
   SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
   SYSCTL_SRPWM_R = 0;
   PWM1_2_CTL_R = 0;
   PWM1_3_CTL_R = 0;
   PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;
   PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;
   PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ZERO | PWM_1_GENB_ACTLOAD_ONE;

   PWM1_2_LOAD_R = 1024;
   PWM1_3_LOAD_R = 1024;
   PWM1_INVERT_R = PWM_INVERT_PWM5INV | PWM_INVERT_PWM6INV | PWM_INVERT_PWM7INV;

   PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
   PWM1_3_CMPB_R = 0;                               // green off
   PWM1_3_CMPA_R = 0;                               // blue off

   PWM1_2_CTL_R = PWM_1_CTL_ENABLE;
   PWM1_3_CTL_R = PWM_1_CTL_ENABLE;
   PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;

}
//------------------------------------------------------------------------------
//Writing functions.
//------------------------------------------------------------------------------
//Send packet function.
 send_packet(uint8_t address,uint8_t command_set,uint8_t channel,uint8_t size,uint8_t value[MAX_COUNT])
{
    for(v=0;v<=MAX_COUNT;v++)
    {
      if(valid[v] == false)
         break;
    }
    destinationADDR[v]=address;
    sqID[v]=sequenceID ++;
    command[v]=command_set;
    channel1[v]=channel;
    size1[v]=size;
    if(command_set==0x70)
        goto p;
    if (ACK==1)
    {
      command[v]|=0x80;
      ackreq[v] = true;
    }
    else if(ACK==0)
    {
    p:  command[v]=command_set;
        ackreq[v] = false;
    }
    for(x=0;x<size1[v];x++)
    {
      data[v][x]=value[x];
    }
    checksum[v]=0;
    checksum[v]=(source_add+destinationADDR[v]+sqID[v]+command[v]+channel1[v]+size1[v]);
    for(i=0;i<size1[v];i++)
    {
      checksum[v]=checksum[v]+data[v][i];
    }
    checksum[v]=~checksum[v];
    valid[v]=1;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
// Transmit data interrupt function
void TX_DATA()
{
        ENABLE=1;
        switch (current_phase)
        {
            case 0:
            {
                if((UART_FR_BUSY & UART1_FR_R) == 0)
                {
                    RED_ON_BOARD=1;
                    UART1_LCRH_R &= ~UART_LCRH_EPS;
                    UART1_DR_R=destinationADDR[current_index];
                    current_phase=current_phase+1;
                    break;
                }
             }
            case 1:
            {
                if((UART_FR_BUSY & UART1_FR_R) == 0)
                {
                    UART1_LCRH_R |= UART_LCRH_EPS;
                    if (UART_FR_TXFF & UART1_FR_R)
                        break;
                    UART1_DR_R = source_add;
                    current_phase=current_phase+1;
                }
                    break;
            }
            case 2:
            {
                 if (UART_FR_TXFF & UART1_FR_R)
                    break;
                    UART1_DR_R=sqID[current_index];
                    current_phase=current_phase+1;
            }
            case 3:
            {
                if (UART_FR_TXFF & UART1_FR_R)
                    break;
                    UART1_DR_R=command[current_index];
                    current_phase=current_phase+1;
            }
            case 4:
            {
                if (UART_FR_TXFF & UART1_FR_R)
                    break;
                    UART1_DR_R=channel1[current_index];
                    current_phase=current_phase+1;
            }
            case 5:
            {
                if (UART_FR_TXFF & UART1_FR_R)
                    break;
                    UART1_DR_R=size1[current_index];
                    current_phase=current_phase+1;
            }
            case 6:
            {
                if (UART_FR_TXFF & UART1_FR_R)
                    break;
                    i=0;
                for(i=0;i<size1[current_index];i++)
                {
                    UART1_DR_R=data[current_index][i];
                }
                current_phase=current_phase+1;
            }
            case 7:
            {
                if (UART_FR_TXFF & UART1_FR_R)
                    break;
                RED_ON_BOARD=0;
                UART1_DR_R=checksum[current_index];
                current_phase=current_phase+1;
                break;
            }
         }
            if(current_phase==8)
            {
                if((UART_FR_BUSY & UART1_FR_R) == 0)
                {
                    ENABLE=0;
                    in_progress=0;
                    current_phase=0;
                    sprintf(string,"Queuing message %u \r\n ",sqID[current_index]);
                    putsUart0(string);
                    if(!ackreq[current_index])
                        valid[current_index]=0;
                    else
                    {
                        retrans_count[current_index]++;
                        sprintf(string,"Transmitting message: %u , Attempt %u \r\n ",sqID[current_index],retrans_count[current_index]);
                        putsUart0(string);
                    if(retrans_count[current_index]>4)
                    {
                         valid[current_index] = 0;
                         retrans_count[current_index] = 0;
                         retrans_timeout[current_index] = 0;
                         RED_ON_BOARD=1;
                    }
                    else
                    {
                    if (random==0)
                    {
                          retrans_timeout[current_index]= t0 + pow(2,n[f]) * t;
                          f++;
                          f=f%5;
                    }
                    if (random==1)
                    {
                          retrans_timeout[current_index]= t0 + pow(2,m[f]) * t;
                          f++;
                          f=f%15;
                     }
                 }
              }
              z=500;
            }
       }
}
//-----------------------------------------------------------------------------
//TIMER1 interrupt
//-----------------------------------------------------------------------------
// Frequency counter service publishing latest frequency measurements every second
void Timer1Isr()
{
    if (!in_progress)
    {
        for(p=0;p<=MAX_COUNT;p++)
        {
            if((valid[p]==1) && (retrans_timeout[p] == 0))
            {
            in_progress=1;
            current_index=p;
            current_phase=0;
            }
        }
    }
    if (in_progress)
    {
        if(s==1 && receive_phase==0)
        {
            TX_DATA();
        }
        else if (s==0)
        {
            TX_DATA();
        }
    }
    for(p=0;p<MAX_COUNT;p++)
    {
        if(retrans_timeout[p]>0)
            retrans_timeout[p]--;
    }

    //PULSE program.
    if(z1==1)
    {
        if(r1>0)
        {
          r1--;
        }
        if(r1==0)
        RED_LED=0;
    }

   //SQUARE program.
    if(z2==1)
    {
       if(r4>0)
       {
        if(r2>0)
        {
             BLUE_LED=c1;
             r2--;
        }
        if(r3>0 && r2==0)
        {
             BLUE_LED=c2;
             r3--;
        }
       }
      if (r2==0 && r3==0)
      {
        r4--;
        r2=d1;
        r3=d2;
      }
      if(r4==0)
      BLUE_LED=0;
    }

    //sawtooth program.
    if(z3==1)
    {
        if(r4>0)
        {
           GREEN_LED=p1;
           if(d1>0)
           d1--;
           //g1=c3; //define
           if(d1==0)
           {
             p1=p1+c3;
             //GREEN_LED=c3;
             d1=r2;
           }
           if(p1>c2)
           {
               GREEN_LED=c1;
               r4--;
               p1=c3;
           }
        }
        if(r4==0)
        {
            GREEN_LED=0;
            z3=0;
        }
    }


    //Deadlock Program.
    if (oldphase==receive_phase)
    {
        variable1++;
    if (variable1==100)
    {
        variable1=0;
    }
    oldphase=receive_phase ;
    }
    if (oldphase==current_phase)
     {
          variable1++;
      if (variable1==100)
        {
          variable1=0;
        }
        oldphase=current_phase ;
     }
    //Receive data function.
     if(current_phase==0)
    {
    UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN | UART_LCRH_PEN | UART_LCRH_SPS| UART_LCRH_EPS;
    while(!(UART1_FR_R & UART_FR_RXFE))
    {

        h= UART1_DR_R;
        if(h & 0x200)
        {
            receive_phase=0;
            receive_data[receive_phase]=(h & 0xFF);
            if (receive_data[0]==source_add||receive_data[0]==0xFF)
                receive_phase++;
        }
        else
        {
            if(receive_phase!=0)
            {
                receive_data[receive_phase]=h;
                receive_phase++;
            }
        }
        if(receive_phase==(receive_data[5]+7))
        {
            receive_phase=0;
            k=500;
            GREEN_ON_BOARD=1;
            if(receive_data[0]==source_add || receive_data[0]==0xff)
            {
            u=0;
            //checksum calculation.
            for(l=0;l<6+receive_data[5];l++)
            {
                u+=receive_data[l];
            }
                receive_ckecksum=~u;
             //SET command receive
             if(receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[4]==1 )
             {
                 if (receive_data[6]==0)
                     RED_LED=0;
                 else
                     RED_LED=255;
             }
             //RGB command receive
             if(receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[4]==2)
             {
                RED_LED=receive_data[6];
                BLUE_LED=receive_data[7];
                GREEN_LED=receive_data[8];
             }
             //POLL command sending
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x78)
             {
                 addr= receive_data[1];
                 chan= receive_data[4];
                 val[0]= source_add;
                 send_packet(addr,0x79,chan,1,&val[0]);

             }
             //EEPROM set address.
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x7A)
             {
                EEPROM_EEOFFSET_R=0;
                EEPROM_EERDWR_R=receive_data[6];
                EEPROM_EEOFFSET_R=0;
                source_add=EEPROM_EERDWR_R;
                sprintf(string,"Your set address is changed from 7 to %u \r\n",receive_data[6]);
                putsUart0(string);
             }
             //POLL command receiving
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x79)
             {
                sprintf(string,"Poll response received from : %u \r\n",receive_data[1]);
                putsUart0(string);
             }
             //RESET command receive
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x7F)
             {
                NVIC_APINT_R= NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;

             }
             //SEND DTA command send.
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x20)
              {
                  addr= receive_data[1];
                  chan= 1;
                  val[0]= RED_LED;
                  send_packet(addr,0x21,chan,1,&val[0]);
              }

             //RECEIVE DATA command receive.
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x21)
              {
                 sprintf(string,"Data in the channel : %u \r\n",receive_data[6]);
                 putsUart0(string);
              }

             //PULSE command receive.
             if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x02 && receive_data[4]==3)
             {
                 t1=receive_data[7]*256;
                 r1=t1|receive_data[8];
                 r1=10*r1;
                 z1=1;
                 RED_LED=receive_data[6];
             }
             //SQUARE command receive.
              if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x03 && receive_data[4]==4)
              {
                  t2=receive_data[8]*256;
                  r2=t2|receive_data[9];
                  r2=10*r2;
                  d1=r2;
                  t3=receive_data[10]*256;
                  r3=t3|receive_data[11];
                  r3=10*r3;
                  d2=r3;
                  t4=receive_data[12]*256;
                  r4=t4|receive_data[13];
                  r4=r4;
                  z2=1;
                  c1=receive_data[6];
                  c2=receive_data[7];
              }
              //sawtooth command receive.
              if (receive_data[6+receive_data[5]]==receive_ckecksum && receive_data[3]==0x04 && receive_data[4]==5)
              {
                  c1=receive_data[6];
                  p1=c1;
                  c2=receive_data[7];
                  c3=receive_data[8];
                  t2=receive_data[9]*256;
                  r2=t2|receive_data[10];
                  d1=r2;
                  t4=receive_data[11]*256;
                  r4=t4|receive_data[12];
                  r4=r4;
                  z3=1;
              }

             else if(receive_data[7]!=receive_ckecksum)
              {
                 GREEN_ON_BOARD=1;
                 p=1;
              }
    }
       //ACK command receiving
       if(receive_data[3]==0x70)
       {
           for(o=0;o<MAX_CHARS;o++)
           {
               if (valid[o])
                   curr_index=o;
               break;
           }
           if((receive_data[6]==sqID[curr_index]))
           {
               valid[curr_index]=false;

           }
           putsUart0("received ack\r\n");

       }
       //ACK command sending
       if((receive_data[3] & 0x80)==0x80)
       {
           send_packet(receive_data[1],0x70,0x00,1,&receive_data[2]);
           putsUart0("sending ack\r\n");
       }


        }

    }
    }

    if(k>0 && p!=1)
    {
       k--;
    }
    if(k==0 && p!=1)
    {
       GREEN_ON_BOARD=0;

    }
    //Clear interrupt.
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);
    return UART0_DR_R & 0xFF;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);
    UART1_DR_R = c;
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart1(str[i]);
}
//Is_command function which is used in sending side.
int iscommand(char strp[],int num)
{

    if ( (strcmp(strp, &str[position[0]])==0) && field>=num+1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }

}
//Function to get string from is_command.
char* getstring(uint8_t field)
{
if (type[field]=='a')
    {
    return &str[position[field]];
    }
else
{
    return 0;
}
}

//Function to get number from is_command.
int getnumber(uint8_t field)
{
    if (type[field]=='n')
    {
    uint8_t c;
    c = atoi(&str[position[field]]);
    return c;
    }
    else
    {
        return 0;
    }
}
//Function to get 16 bit number
int getnumber1(uint16_t field)
{
    if (type[field]=='n')
    {
    uint16_t c;
    c = atoi(&str[position[field]]);
    return c;
    }
    else
    {
        return 0;
    }
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    initHw();
    EEPROM_EEOFFSET_R=0;
    if (EEPROM_EERDWR_R != 0XFFFFFFFF)
    {
        EEPROM_EEOFFSET_R=0;
        source_add=EEPROM_EERDWR_R;
    }
    GREEN_LED=255;
    waitMicrosecond(500000);
    GREEN_LED=0;
    putsUart0("READY\r\n");
    waitMicrosecond(900000);
    putsUart0("Enter the Commands\r\n");
    while(1)
    {
    for(i=0; i<MAX_CHARS; i++)
    {
        str[i]=0;
    }

    for(count=0;count<(MAX_CHARS);count++)
    {

        str[count]=getcUart0();
        struart[count]=str[count];


        if(str[count]==8)
        {
            if(count>0)
               count=count-2;
            if(count==0)
               count=0;
        }
        else if(str[count]==32)
        {
            count=count++;
        }
        else if(str[count]<32 && str[count]!=8 && str[count]!=13)
        {
            putsUart0("Enter a valid character\r\n");
        }
        else if(str[count]==13)
        {
            str[count]=0x00;
            putsUart0(str);
            putsUart0("\r\n");
            break;
        }
        if(count>85)
         {
           putsUart0("\r\n");
           break;
         }
    }
    index=0;
    a=0;
    look = 1;
    field = 0;
    for(i=0;i<85;i++)
    {
        if(!((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||(str[i]>=97 && str[i]<=122)))
        {
            look=1;
        }
        if(((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||(str[i]>=97 && str[i]<=122)) && look==1)
        {
            position[index] = i;

            if((str[i]>=65 && str[i]<=90)||(str[i]>=97 && str[i]<=122))
            {

                type[index] = 'a';
                index++;

            }
            if((str[i]>=48 && str[i]<=57))
            {
                type[index] = 'n';
                index++;
            }
            look = 0;
            field++;

        }

    }

    for(i=0;i<85;i++)
    {
        if(((str[i]>=48 && str[i]<=57)||(str[i]>=65 && str[i]<=90)||(str[i]>=97 && str[i]<=122)))
        {
            strref[a]=tolower(str[i]);
            a++;
        }
        else
        {
            str[i] = '\0';
            strref[a]=str[i];
            a++;
        }
        str[i]=tolower(str[i]);
    }

    if (iscommand("set",3))
    {

        strg=getstring(0);
        addr=getnumber(1);
        chan=getnumber(2);
        val[0]=getnumber(3);
        if(type[1]=='n' && type[2]=='n' && type[3]=='n')
        {
            send_packet(addr,0x00,chan,1,&val[0]);

        }
    }

    if (iscommand("ack",1))
    {
        strg=getstring(1);
        if(strcmp(strg,"on")==0)
            ACK=1;
        if(strcmp(strg,"off")==0)
            ACK=0;
    }

    if (iscommand("cs",1))
    {
        strg=getstring(1);
        if(strcmp(strg,"on")==0)
            s=1;
        if(strcmp(strg,"off")==0)
            s=0;
    }

    if (iscommand("rgb",5))
    {
        strg=getstring(0);
        addr=getnumber(1);
        chan=getnumber(2);
        val[0]=getnumber(3);
        val[1]=getnumber(4);
        val[2]=getnumber(5);
        if(type[1]=='n' && type[2]=='n' && type[3]=='n' && type[4]=='n' && type[5]=='n')
            send_packet(addr,0x48,chan,3,&val[0]);

    }

    if (iscommand("poll",0))
    {
        strg=getstring(0);
        addr=broadcast_add;
        chan=0;
        val[0]=0;
        send_packet(addr,0x78,chan,0,&val[0]);

    }
    if (iscommand("reset",1))
        {
            strg=getstring(0);
            addr=getnumber(1);
            chan=0;
            val[0]=0;
            send_packet(addr,0x7F,chan,0,&val[0]);

        }
    if (iscommand("random",1))
        {
            strg=getstring(1);
            if(strcmp(strg,"on")==0)
                random=1;
            if(strcmp(strg,"off")==0)
                random=0;

        }
    if (iscommand("get",2))
    {
        strg=getstring(0);
        addr=getnumber(1);
        chan=getnumber(2);
        send_packet(addr,0x20,chan,0,0);
    }
   if (iscommand("pulse",4))
    {
        strg=getstring(0);
        addr=getnumber(1);
        chan=getnumber(2);
        val[0]=getnumber(3);
        x1=getnumber1(4);
        a1=x1/256;
        b1=0xFF&x1;
        val[1]=a1;
        val[2]=b1;
        send_packet(addr,0x02,chan,3,&val[0]);
    }
   if (iscommand("square",7))
   {
       strg=getstring(0);
       addr=getnumber(1);
       chan=getnumber(2);
       val[0]=getnumber(3);
       val[1]=getnumber(4);
       x1=getnumber1(5);
       x2=getnumber1(6);
       a1=x1/256;
       a2=0xFF&x1;
       b1=x2/256;
       b2=0xFF&x2;
       val[2]=a1;
       val[3]=a2;
       val[4]=b1;
       val[5]=b2;
       x3=getnumber1(7);
       a3=x3/256;
       a4=0xFF&x3;
       val[6]=a3;
       val[7]=a4;
       send_packet(addr,0x03,chan,8,&val[0]);
   }
   if (iscommand("sa",2))
   {
       strg=getstring(0);
       addr=getnumber(1);
       val[0]= getnumber(2);
       send_packet(addr,0x7A,chan,1,&val[0]);
   }
   if (iscommand("sawtooth",7))
   {
       strg=getstring(0);
       addr=getnumber(1);
       chan=getnumber(2);
       val[0]=getnumber(3);
       val[1]=getnumber(4);
       val[2]=getnumber(5);
       x1=getnumber1(6);
       x2=getnumber1(7);
       a1=x1/256;
       a2=0xFF&x1;
       b1=x2/256;
       b2=0xFF&x2;
       val[3]=a1;
       val[4]=a2;
       val[5]=b1;
       val[6]=b2;
       send_packet(addr,0x04,chan,7,&val[0]);
   }
  }
}
