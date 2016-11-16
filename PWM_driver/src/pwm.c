/* File: pwm.c
   Description : PWM driver for LPC1769
   Author : Bhupendra Naphade
*/
#include <lpc17xx.h>
#include <stdio.h>
void delay_ms(unsigned int ms)
{
    unsigned int i,j;
    for(i=0;i<ms;i++)
        for(j=0;j<50000;j++);
}


#define SBIT_CNTEN     0
#define SBIT_PWMEN     2
#define SBIT_PWMMR0R   1
#define SBIT_LEN0      0
#define SBIT_PWMENA1   9



#define PWM_1          0 //P2_0 (0-1 Bits of PINSEL4)

int PWMclock;
int PWMrate;

void pwm_conrol(int frequency, int dutycycle)
{
    //frequency = 50;

        PWMclock = SystemCoreClock / 4;
        PWMrate = PWMclock / frequency;
      //  printf("%d :%d",SystemCoreClock,PWMrate);
    /* Cofigure pins(P2_0)  for PWM mode. */
    LPC_PINCON->PINSEL4 = (1<<PWM_1);

    /* Enable Counters,PWM module */
    LPC_PWM1->TCR = (1<<SBIT_CNTEN) | (1<<SBIT_PWMEN);

    LPC_PWM1->PR  =  0x0;               /* No Prescalar */
    LPC_PWM1->MCR = (1<<SBIT_PWMMR0R);  /*Reset on PWMMR0, reset TC if it matches MR0 */

    LPC_PWM1->MR0 = PWMrate;//100;                /* set PWM cycle(Ton+Toff)=100) */
    LPC_PWM1->MR1 = (dutycycle*PWMrate)/100;                /* Set 50% Duty Cycle for pwm1 */
    /* Trigger the latch Enable Bits to load the new Match Values */
    LPC_PWM1->LER = (1<<SBIT_LEN0);

    /* Enable the PWM output pins for PWM_1-PWM_4(P2_0 - P2_3) */
    LPC_PWM1->PCR = (1<<SBIT_PWMENA1);

}
int main(void)
{
    SystemInit();
    int frequency, duty;
    printf("Enter the frequency for PWM\n");
    scanf("%d", &frequency);
    printf("Enter the duty cycle for PWM\n");
    scanf("%d", &duty);
    while(1)
    {
    	pwm_conrol(frequency, duty);
    	//duty = duty + 2;
    	if (duty>100)
    	{
    		duty =0;
    	}
    }


}
