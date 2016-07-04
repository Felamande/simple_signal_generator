#include <msp430.h>

#define ADC10_IN_PORT          (BIT4)
#define P1_IN_PORTS           ~(BIT0 + BIT1 + BIT2 + ADC10_IN_PORT) //0:switch wave, 1:add freq, 2:sub freq, 4:adc10 in
#define P1_OUT_PORTS           (BIT3)      //3:DAC WR
#define P1_INTERRUPT           (BIT0 + BIT1 + BIT2)
#define P2_OUT_PORTS           (0xff) //DAC data in

#define TOTAL_SAMPLING_POINTS  200
#define MAX_FREQ_STEPS         100

#define ENABLE_WR_PORT         P1OUT &= ~BIT3 //WR->0
#define DISABLE_WR_PORT        P1OUT |= BIT3  //WR->1
#define write_dac(data)        P2OUT = data //write to DAC

#define CPU_FREQ               ((double)16000000) //CPU frequency set to 16M(CALBC_16MHZ)
#define delay_us(x)            __delay_cycles((long)(CPU_FREQ*(double)x/1000000.0))
#define delay_ms(x)            __delay_cycles((long)(CPU_FREQ*(double)x/1000.0))

#define uchar                  unsigned char
#define uint                   unsigned int

uint curr_signal_type;
int tccr0_now;
uint ccr0_idx;
uchar point_now;
int p1_push_key;
int duty_circle;

const int ccr0_table[MAX_FREQ_STEPS] = { 16000, 8000, 5333, 4000, 3200, 2666,
		2285, 2000, 1777, 1600, 1454, 1333, 1230, 1142, 1066, 1000, 941, 888,
		842, 800, 761, 727, 695, 666, 640, 615, 592, 571, 551, 533, 516, 500,
		484, 470, 457, 444, 432, 421, 410, 400, 390, 380, 372, 363, 355, 347,
		340, 333, 326, 320, 313, 307, 301, 296, 290, 285, 280, 275, 271, 266,
		262, 258, 253, 250, 246, 242, 238, 235, 231, 228, 225, 222, 219, 216,
		213, 210, 207, 205, 202, 200, 197, 195, 192, 190, 188, 186, 183, 181,
		179, 177, 175, 173, 172, 170, 168, 166, 164, 163, 161, 160 };

const unsigned char sin_data[TOTAL_SAMPLING_POINTS] = { 127, 131, 135, 139, 143,
		147, 151, 155, 159, 162, 166, 170, 174, 177, 181, 185, 188, 192, 195,
		198, 202, 205, 208, 211, 214, 217, 220, 222, 225, 227, 230, 232, 234,
		236, 238, 240, 242, 244, 245, 246, 248, 249, 250, 251, 252, 252, 253,
		253, 254, 254, 254, 254, 254, 253, 253, 252, 252, 251, 250, 249, 248,
		246, 245, 244, 242, 240, 238, 236, 234, 232, 230, 227, 225, 222, 220,
		217, 214, 211, 208, 205, 202, 198, 195, 192, 188, 185, 181, 177, 174,
		170, 166, 162, 159, 155, 151, 147, 143, 139, 135, 131, 127, 123, 119,
		115, 111, 107, 103, 99, 95, 92, 88, 84, 80, 77, 73, 69, 66, 62, 59, 56,
		52, 49, 46, 43, 40, 37, 34, 32, 29, 27, 24, 22, 20, 18, 16, 14, 12, 10,
		9, 8, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 8,
		9, 10, 12, 14, 16, 18, 20, 22, 24, 27, 29, 32, 34, 37, 40, 43, 46, 49,
		52, 56, 59, 62, 66, 69, 73, 77, 80, 84, 88, 92, 95, 99, 103, 107, 111,
		115, 119, 123 };

const uchar tria_data[TOTAL_SAMPLING_POINTS] = { 0, 2, 5, 7, 10, 12, 15, 17, 20,
		22, 25, 28, 30, 33, 35, 38, 40, 43, 45, 48, 51, 53, 56, 58, 61, 63, 66,
		68, 71, 73, 76, 79, 81, 84, 86, 89, 91, 94, 96, 99, 102, 104, 107, 109,
		112, 114, 117, 119, 122, 124, 127, 130, 132, 135, 137, 140, 142, 145,
		147, 150, 153, 155, 158, 160, 163, 165, 168, 170, 173, 175, 178, 181,
		183, 186, 188, 191, 193, 196, 198, 201, 204, 206, 209, 211, 214, 216,
		219, 221, 224, 226, 229, 232, 234, 237, 239, 242, 244, 247, 249, 252,
		252, 249, 247, 244, 242, 239, 237, 234, 232, 229, 226, 224, 221, 219,
		216, 214, 211, 209, 206, 204, 201, 198, 196, 193, 191, 188, 186, 183,
		181, 178, 175, 173, 170, 168, 165, 163, 160, 158, 155, 153, 150, 147,
		145, 142, 140, 137, 135, 132, 130, 127, 124, 122, 119, 117, 114, 112,
		109, 107, 104, 102, 99, 96, 94, 91, 89, 86, 84, 81, 79, 76, 73, 71, 68,
		66, 63, 61, 58, 56, 53, 51, 48, 45, 43, 40, 38, 35, 33, 30, 28, 25, 22,
		20, 17, 15, 12, 10, 7, 5, 2, 0 };

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_A0(void) {
//	P1OUT  |= (P1OUT^BIT6)&BIT6
	if (point_now >= TOTAL_SAMPLING_POINTS) {
		point_now = 0;
	}

	switch (curr_signal_type) {//genre
	case 0:
		//sin;
		write_dac(sin_data[point_now]);
		break;
	case 1:
		//triangle
		write_dac(tria_data[point_now]);
		break;
	case 2:
		//box
		if (point_now < duty_circle) {
			write_dac(0xff);
		} else {
			write_dac(0x00);
		}

		break;
	}

	ENABLE_WR_PORT;
	delay_us(1);
	DISABLE_WR_PORT;

	point_now++;
	TA0CCR0 += tccr0_now;
}

#pragma vector = PORT1_VECTOR
__interrupt void port1(void) {
	//消除抖动，延迟0.1s
	delay_ms(100);

	p1_push_key = P1IFG;

	if (p1_push_key & BIT0) {
		//调节波形
		curr_signal_type++;
		if (curr_signal_type >= 3) {
			curr_signal_type = 0;
		}
//		point_now = 0;
	} else if (p1_push_key & BIT1) {
		//调节频率增加
		ccr0_idx++;
		if (ccr0_idx >= MAX_FREQ_STEPS) {
			ccr0_idx = 0;
		}
		tccr0_now = ccr0_table[ccr0_idx];
	} else if (p1_push_key & BIT2) {
		//调节频率减小
		ccr0_idx--;
		if (ccr0_idx >= MAX_FREQ_STEPS) {
			ccr0_idx = 99;
		}

		tccr0_now = ccr0_table[ccr0_idx];

	}

	P1IFG &= ~P1_INTERRUPT; //清除标志位

}

void init_vars() {
	curr_signal_type = 0;
	point_now = 0;

	ccr0_idx = 0;
	tccr0_now = ccr0_table[ccr0_idx];

	duty_circle = 100;

}

void init_DCO() {
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;

	BCSCTL2 = SELM_1 + DIVM_0;
	// select DCO as the source of MCLK

	BCSCTL2 &= ~SELS;
	// select DCO as the source of SMCLK

}

void init_timer_A0(void) {
	TA0CTL |= TASSEL_2 + MC_2; //SMCLK source and Mode continous
	TA0CCR0 = tccr0_now;
	TA0CCTL0 |= CCIE; //interrupt enable
	_EINT();
}

void init_port_io(void) {
	P2DIR = P2_OUT_PORTS; //P2 11111111b all out
	P2REN = 0x00; //disable pull/down resistor
	P2SEL = 0x00; //io function is selected
	P2SEL2 = 0x00;

	P1DIR &= P1_IN_PORTS; //P1.0 P1.1 p1.2 p1.4 in
	P1DIR |= P1_OUT_PORTS; //P1.3 for DAC WR
	P1REN = 0x00;
	P1SEL = 0x00;
	P1SEL2 = 0x00;
}

void init_port_interrupt(void) {

	P1IES |= P1_INTERRUPT; //置1，下降沿触发
	P1IE |= P1_INTERRUPT;  //中断使能
	P1IFG &= ~P1_INTERRUPT; //清除标志位
}

void init_ADC10(void) {

	ADC10CTL1 |= INCH_4; //A4 channel for convertion, P1.4 in
	ADC10CTL1 |= SHS_0;
	ADC10CTL1 |= ADC10SSEL_3 + ADC10DIV_7;// SMCLK/8 2M
	ADC10CTL1 &= ~ADC10DF; // straght binary format
	ADC10CTL1 |= CONSEQ_2; // Single channel single convertion

	ADC10AE0 = ADC10_IN_PORT;

	ADC10CTL0 &= ~ADC10IE; //disable interrupt
	ADC10CTL0 |= SREF_1 + ADC10SHT_0 + REF2_5V + REFON; //V+ = 2.5V, V- = Vss = 0
	ADC10CTL0 |= MSC; //further sample and conversions are performed automatically
					  //as soon as the prior conversion is completed
	ADC10CTL0 &= ~REFOUT;			          // diasable refout to p1.3 p1.4
	ADC10CTL0 |= ADC10ON; //enable adc
	ADC10CTL0 |= ENC + ADC10SC;  //start repeated convertion
}



void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	init_vars();
	init_port_io();
	init_DCO();
	init_timer_A0();
	init_port_interrupt();
//	init_ADC10();

	_bis_SR_register(GIE);

	while (1) {
		_NOP();
		if(ADC10CTL1&ADC10BUSY)continue;
		int adc_data = ADC10MEM;
		duty_circle = (adc_data >> 3) + 50;

	}
}
