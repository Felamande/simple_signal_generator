#include <msp430.h>
#define SWITCH_SIG_TYPE        (BIT0)
#define ADD_FREQ               (BIT1)
#define SUB_FREQ               (BIT2)
#define ADC10_IN_PORT          (BIT4)
#define DAC_WR                 (BIT3)
#define P1_IN_PORTS           ~(SWITCH_SIG_TYPE + ADD_FREQ + SUB_FREQ + ADC10_IN_PORT)
#define P1_OUT_PORTS           DAC_WR      //3:DAC WR
#define P1_INTERRUPT           (SWITCH_SIG_TYPE + ADD_FREQ + SUB_FREQ)
#define P2_OUT_PORTS           (0xff) //DAC data in

#define TOTAL_SAMPLING_POINTS  200
#define MAX_FREQ_STEPS         100

#define ENABLE_WR_PORT         P1OUT &= ~DAC_WR //WR->0
#define DISABLE_WR_PORT        P1OUT |= DAC_WR  //WR->1
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
int push_key;
int duty_circle;

const int ccr0_table[MAX_FREQ_STEPS] = { 16000, 8000, 5333, 4000, 3200, 2666,
		2285, 2000, 1777, 1600, 1454, 1333, 1230, 1142, 1066, 1000, 941, 888,
		842, 800, 761, 727, 695, 666, 640, 615, 592, 571, 551, 533, 516, 500,
		484, 470, 457, 444, 432, 421, 410, 400, 390, 380, 372, 363, 355, 347,
		340, 333, 326, 320, 313, 307, 301, 296, 290, 285, 280, 275, 271, 266,
		262, 258, 253, 250, 246, 242, 238, 235, 231, 228, 225, 222, 219, 216,
		213, 210, 207, 205, 202, 200, 197, 195, 192, 190, 188, 186, 183, 181,
		179, 177, 175, 173, 172, 170, 168, 166, 164, 163, 161, 160 };

const uchar sin_data[TOTAL_SAMPLING_POINTS] = { 127, 131, 135, 139, 143, 147,
		151, 155, 159, 163, 167, 170, 174, 178, 182, 185, 189, 192, 196, 199,
		202, 205, 209, 212, 215, 218, 220, 223, 226, 228, 231, 233, 235, 237,
		239, 241, 243, 244, 246, 247, 249, 250, 251, 252, 252, 253, 254, 254,
		254, 254, 254, 254, 254, 254, 253, 253, 252, 251, 250, 249, 248, 247,
		245, 244, 242, 240, 238, 236, 234, 232, 229, 227, 224, 222, 219, 216,
		213, 210, 207, 204, 201, 197, 194, 190, 187, 183, 180, 176, 172, 168,
		165, 161, 157, 153, 149, 145, 141, 137, 133, 129, 125, 121, 117, 113,
		109, 105, 101, 97, 93, 89, 86, 82, 78, 74, 71, 67, 64, 60, 57, 53, 50,
		47, 44, 41, 38, 35, 32, 30, 27, 25, 22, 20, 18, 16, 14, 12, 10, 9, 7, 6,
		5, 4, 3, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 5, 7, 8, 10,
		11, 13, 15, 17, 19, 21, 23, 26, 28, 31, 34, 36, 39, 42, 45, 49, 52, 55,
		58, 62, 65, 69, 72, 76, 80, 84, 87, 91, 95, 99, 103, 107, 111, 115, 119,
		123, 127 };

const uchar tria_data[TOTAL_SAMPLING_POINTS] = { 0, 2, 4, 6, 8, 10, 12, 14, 16,
		18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50, 52,
		54, 56, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76, 78, 80, 82, 84, 86, 88,
		90, 92, 94, 96, 98, 100, 102, 104, 106, 108, 110, 112, 114, 116, 118,
		120, 122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 142, 144, 146,
		148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174,
		176, 178, 180, 182, 184, 186, 188, 190, 192, 194, 196, 198, 198, 196,
		194, 192, 190, 188, 186, 184, 182, 180, 178, 176, 174, 172, 170, 168,
		166, 164, 162, 160, 158, 156, 154, 152, 150, 148, 146, 144, 142, 140,
		138, 136, 134, 132, 130, 128, 126, 124, 122, 120, 118, 116, 114, 112,
		110, 108, 106, 104, 102, 100, 98, 96, 94, 92, 90, 88, 86, 84, 82, 80,
		78, 76, 74, 72, 70, 68, 66, 64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 44,
		42, 40, 38, 36, 34, 32, 30, 28, 26, 24, 22, 20, 18, 16, 14, 12, 10, 8,
		6, 4, 2, 0 };

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_A0(void) {

	if (point_now >= TOTAL_SAMPLING_POINTS) {
		point_now = 0;
	}

	switch (curr_signal_type) {
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

	push_key = P1IFG;

	if (push_key & SWITCH_SIG_TYPE) {
		//调节波形
		curr_signal_type++;
		if (curr_signal_type >= 3) {
			curr_signal_type = 0;
		}
//		point_now = 0;
	} else if (push_key & ADD_FREQ) {
		//调节频率增加
		ccr0_idx++;
		if (ccr0_idx >= MAX_FREQ_STEPS) {
			ccr0_idx = 0;
		}
		tccr0_now = ccr0_table[ccr0_idx];
	} else if (push_key & SUB_FREQ) {
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

	ccr0_idx = 50;
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
	P2REN = 0x00; //disable pull up/down resistor
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
	ADC10CTL1 |= SHS_0; //Sample-and-hold source select ADC10SC
	ADC10CTL1 |= ADC10SSEL_3; //SMCLK 16M
	ADC10CTL1 &= ~ADC10DF; // straght binary format
	ADC10CTL1 |= CONSEQ_0; // Single channel single convertion

	ADC10AE0 = ADC10_IN_PORT; //P1.4 in

	ADC10CTL0 &= ~ADC10IE; //disable interrupt
	ADC10CTL0 |= SREF_1 + ADC10SHT_0 + REF2_5V + REFON; //V+ = 2.5V, V- = Vss = 0
	ADC10CTL0 |= MSC; //further sample and conversions are performed automatically
					  //as soon as the prior conversion is completed
	ADC10CTL0 &= ~REFOUT;					  // diasable refout to p1.3 p1.4
	ADC10CTL0 |= ADC10ON; //enable adc
}

void test_dac(void) {
	TA0CCTL0 &= ~CCIE;
	uchar data = 0xff;
	while (1) {
		P2OUT = data;
		data--;
	}
}

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	init_vars();
	init_port_io();
	init_port_interrupt();
	init_DCO();
	//test_dac()
	init_timer_A0();
	init_ADC10();

	_bis_SR_register(GIE);

	while (1) {
		ADC10CTL0 &= ~ENC;          //关闭采样使能
		while (ADC10CTL1 & ADC10BUSY)
			; //检测是否忙
		ADC10CTL0 |= ENC + ADC10SC; //打开采样使能，开始转换
		while (ADC10CTL1 & ADC10BUSY)
			; //检测是否忙
		int adc_data = ADC10MEM;    //读取数据
		duty_circle = (adc_data >> 3) + 40; //占空比限制在 40(20%)~168(84%)之间
		//采集到的数据是0~1023
		//右移三位就是0~127
		//加40就是40~167
		//总点数是200点
		//占空比就是40/200=20% ~ 167/200=84% 之间
	}
}
