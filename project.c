#include "device_registers.h"
#include "clocks_and_modes.h"
#include "ADC.h"
int lpit0_ch0_flag_counter = 0; /*< LPIT0 timeout counter */
unsigned int FND_DATA[10]={0x7E, 0x0C,0xB6,0x9E,0xCC,0xDA,0xFA,0x4E,0xFE,0xCE};
unsigned int FND_SEL[4]={0x0100,0x0200,0x0400,0x0800};
unsigned int j=0; /*FND select pin index */
unsigned int num=0,num0=0,num1=0,num2=0,num3=0;
/*num is Counting value, num0 is '1', num2 is '10', num2 is '100', num3 is '1000'*/
unsigned int mode=0; /* External_PIN:SW External input Assignment */
unsigned int direction=0;
unsigned int Dtime = 0; /* Delay Time Setting Variable*/
unsigned int Delaytime = 0;

void PORT_init (void)
{

    PCC->PCCn[PCC_PORTB_INDEX]|=PCC_PCCn_CGC_MASK;
    PCC->PCCn[PCC_PORTC_INDEX]|=PCC_PCCn_CGC_MASK;  //PORTC clock enable
    PCC->PCCn[PCC_PORTD_INDEX]|=PCC_PCCn_CGC_MASK;  //PORTD clock enable
    PCC->PCCn[PCC_PORTE_INDEX]|=PCC_PCCn_CGC_MASK;  //PORTE clock enable

    // DC motor
    PORTB->PCR[0]|=PORT_PCR_MUX(1); 
    PORTB->PCR[1]|=PORT_PCR_MUX(1);
    PTB->PDDR |= 1<<0|1<<1;


  //PortC,D Data Direction Set
    PTC->PDDR &= ~(1<<11); /* Port C11 Port Input set, value '0'*/
    PTC->PDDR &= ~(1<<12); /* Port C12 Port Input set, value '0'*/
    PTC->PDDR &= ~(1<<13); /* Port C12 Port Input set, value '0'*/
    PTC->PDDR &= ~(1<<16); /* Port C12 Port Input set, value '0'*/
    PTC->PDDR &= ~(1<<17); /* Port C12 Port Input set, value '0'*/
    PTD->PDDR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<6|1<<7 /* Port D0~7:  FND Data Direction = output */
        |1<<8|1<<9|1<<10|1<<11; /*FND Select Direction */

    // PORTC_11 pin GPIO and Falling-edge Set break
    PORTC->PCR[11] |= PORT_PCR_MUX(1); // Port C11 mux = GPIO
    PORTC->PCR[11] |=(1000<<16); // Port C11 IRQC : interrupt on Falling-edge

    // PORTC_12 pin GPIO and Falling-edge Set left gg
    PORTC->PCR[12] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTC->PCR[12] |=(1000<<16); // Port C12 IRQC : interrupt on Falling-edge

    // PORTC_13 pin GPIO and Falling-edge Set right gg
    PORTC->PCR[13] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTC->PCR[13] |=(1000<<16); // Port C12 IRQC : interrupt on Falling-edge

    // PORTC_16 pin GPIO and Falling-edge Set  bisang
    PORTC->PCR[16] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTC->PCR[16] |=(1000<<16); // Port C12 IRQC : interrupt on Falling-edge

    // PORTC_17 pin GPIO and Falling-edge Set R
    PORTC->PCR[17] |= PORT_PCR_MUX(1); // Port C12 mux = GPIO
    PORTC->PCR[17] |=(10<<16); // Port C12 IRQC : interrupt on Falling-edge

    //PORTD FND Control Pin PCR Set, PTD1~PTD7:FND_DATA Control , PTD8~PTD11:FND_Select Control
    PORTD->PCR[1]  = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO  */
    PORTD->PCR[2]  = PORT_PCR_MUX(1); /* Port D2: MUX = GPIO  */
    PORTD->PCR[3]  = PORT_PCR_MUX(1); /* Port D3: MUX = GPIO  */
    PORTD->PCR[4]  = PORT_PCR_MUX(1); /* Port D4: MUX = GPIO  */
    PORTD->PCR[5]  = PORT_PCR_MUX(1); /* Port D5: MUX = GPIO  */
    PORTD->PCR[6]  = PORT_PCR_MUX(1); /* Port D6: MUX = GPIO  */
    PORTD->PCR[7] = PORT_PCR_MUX(1); /* Port D7: MUX = GPIO */

    PORTD->PCR[8] = PORT_PCR_MUX(1); /* Port D8: MUX = GPIO */
    PORTD->PCR[9] = PORT_PCR_MUX(1); /* Port D9: MUX = GPIO */
    PORTD->PCR[10] = PORT_PCR_MUX(1); /* Port D10: MUX = GPIO */
    PORTD->PCR[11] = PORT_PCR_MUX(1); /* Port D11: MUX = GPIO */

    //LED
    PTE->PDDR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
    PORTE->PCR[1]  = PORT_PCR_MUX(1); /* Port D1: MUX = GPIO  */
    PORTE->PCR[2]  = PORT_PCR_MUX(1); /* Port D2: MUX = GPIO  */
    PORTE->PCR[3]  = PORT_PCR_MUX(1); /* Port D3: MUX = GPIO  */
    PORTE->PCR[4]  = PORT_PCR_MUX(1); /* Port D4: MUX = GPIO  */
    PORTE->PCR[5]  = PORT_PCR_MUX(1); /* Port D5: MUX = GPIO  */
    PORTE->PCR[7]  = PORT_PCR_MUX(1); /* Port D6: MUX = GPIO  */
    PORTE->PCR[8] = PORT_PCR_MUX(1); /* Port D7: MUX = GPIO */
    PORTE->PCR[9] = PORT_PCR_MUX(1); /* Port D7: MUX = GPIO */

    PTD->PDDR |= 1<<15|1<<16;
    //PORTD->PCR[0] = PORT_PCR_MUX(1);
    PORTD->PCR[15] = PORT_PCR_MUX(1);
    PORTD->PCR[16] = PORT_PCR_MUX(1);

}


void WDOG_disable (void)
{
    WDOG->CNT=0xD928C520;     /* Unlock watchdog */
    WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value */
    WDOG->CS = 0x00002100;    /* Disable watchdog */
}

void LPIT0_init (uint32_t delay)
{
   uint32_t timeout;

  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs       */

  LPIT0->MCR |= LPIT_MCR_M_CEN_MASK;  
                                    
  timeout=delay* 40;
  LPIT0->TMR[0].TVAL = timeout;     
  LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_T_EN_MASK;
}

void delay_us (volatile int us){
   LPIT0_init(us);           /* Initialize PIT0 for 1 second timeout  */
   while (0 == (LPIT0->MSR & LPIT_MSR_TIF0_MASK)) {} /* Wait for LPIT0 CH0 Flag */
               lpit0_ch0_flag_counter++;         /* Increment LPIT0 timeout counter */
               LPIT0->MSR |= LPIT_MSR_TIF0_MASK; /* Clear LPIT0 timer flag 0 */
}

void NVIC_init_IRQs(void){
    S32_NVIC->ICPR[1] |= 1<<(61%32); // Clear any pending IRQ61
    S32_NVIC->ISER[1] |= 1<<(61%32); // Enable IRQ61
    S32_NVIC->IP[61] =0xB; //Priority 11 of 15
}

void Seg_out_stop(void){
    Delaytime = 1000;
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = 0b11011010;
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 100??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = 0b00001110;
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 10??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = 0b01111111;
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 1??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = 0b11100110;
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j=0;
}


void Seg_out(int number){
  Delaytime = 1000;

    num3=(number/1000)%10;
    num2=(number/100)%10;
    num1=(number/10)%10;
    num0= number%10;


    // 1000??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = FND_DATA[num3];
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 100??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = FND_DATA[num2];
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 10??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = FND_DATA[num1];
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j++;

    // 1??? ??
    PTD->PSOR = FND_SEL[j];
    PTD->PCOR =0x7f;
    PTD->PSOR = FND_DATA[num0];
    delay_us(Delaytime);
    PTD->PCOR = 0xfff;
    j=0;
}




void PORTC_IRQHandler(void){

    PORTC->PCR[11] &= ~(0x01000000); 
    PORTC->PCR[12] &= ~(0x01000000); 
    PORTC->PCR[13] &= ~(0x01000000); 
    PORTC->PCR[16] &= ~(0x01000000); 
    PORTC->PCR[17] &= ~(0x01000000); 

    if((PORTC->ISFR & (1<<11)) != 0){
        PTB->PCOR |= 1<<0;
        PTB->PCOR |= 1<<1;
        PTD->PCOR |= 1<<15;
        Seg_out_stop();
    }
    else if((PORTC->ISFR & (1<<12)) != 0){

    // left 
    for(int i = 0; i< 1; i++){
        PTE->PCOR |= 1<<9;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<8;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<7;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<5;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<4;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<3;
        PTE->PSOR |= 1<<1|1<<2|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<2;
        PTE->PSOR |= 1<<1|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        PTE->PCOR |= 1<<1;
        PTE->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<12)) == 0) break;
        delay_us(150000);
        }
    }

    // right point
    else if((PORTC->ISFR & (1<<13)) != 0){
    for(int i = 0; i< 1; i++){
        PTE->PCOR |= 1<<1;
        PTE->PSOR |= 1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<2;
        PTE->PSOR |= 1<<1|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<3;
        PTE->PSOR |= 1<<1|1<<2|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<4;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<5;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<7;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<8;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<9;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        PTE->PCOR |= 1<<9;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8;
        if ((PORTC->ISFR & (1<<13)) == 0) break;
        if (mode != 3) break;
        delay_us(150000);
        }
    }

    else if((PORTC->ISFR & (1<<16)) != 0){
    for(int i = 0; i< 3; i++){
        PTE->PCOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<16)) == 0) break;
        delay_us(500000);
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
        if ((PORTC->ISFR & (1<<16)) == 0) break;
        delay_us(500000);
        }
    }
    else if((PORTC->ISFR & (1<<17)) != 0){
        if (direction == 0){
        direction = 1;
        }
        else{
        direction = 0;
        }
    }

    PORTC->PCR[11] |= 0x01000000; // Port Control Register ISF bit '1' set
    PORTC->PCR[12] |= 0x01000000; // Port Control Register ISF bit '1' set
    PORTC->PCR[13] |= 0x01000000; // Port Control Register ISF bit '1' set
    PORTC->PCR[16] |= 0x01000000; // Port Control Register ISF bit '1' set
    PORTC->PCR[17] |= 0x01000000; // Port Control Register ISF bit '1' set

}


int main(void)
{
  uint32_t adcResultInMv=0; /*< ADC0 Result in miliVolts */

    WDOG_disable();        /* Disable WDOG */
    SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
    SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
    NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash*/
    PORT_init();     /* Init  port clocks and gpio outputs */
    ADC_init();            /* Init ADC resolution 12 bit */
    NVIC_init_IRQs(); /*Interrupt Pending, Endable, Priority Set*/


    for(;;)
    {
        // ADC
        convertAdcChan(13);                   
        while(adc_complete()==0){}            
        adcResultInMv = read_adc_chx();       

        if (adcResultInMv > 2500) {
            if (direction == 0){
                PTB->PSOR |= 1<<0;
                PTB->PCOR |= 1<<1;
            }
            else{
                PTB->PCOR |= 1<<0;
                PTB->PSOR |= 1<<1;
            }
        }

        else {
            PTB->PCOR |= 1<<0;
            PTB->PCOR |= 1<<0;
        }

        // LED off
        PTD->PSOR |= 1<<0|1<<15|1<<16;
        PTE->PSOR |= 1<<1|1<<2|1<<3|1<<4|1<<5|1<<7|1<<8|1<<9;
    }
}
