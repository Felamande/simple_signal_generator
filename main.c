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

#define TOTAL_SAMPLING_POINTS  64
#define MAX_FREQ_STEPS         400

#define ENABLE_WR_PORT         P1OUT &= ~DAC_WR //WR->0
#define DISABLE_WR_PORT        P1OUT |= DAC_WR  //WR->1
#define write_dac(data)        P2OUT = data //write to DAC

#define CPU_FREQ               ((double)16000000) //CPU frequency set to 16M(CALBC_16MHZ)
#define delay_us(x)            __delay_cycles((long)(CPU_FREQ*(double)x/1000000.0))
#define delay_ms(x)            __delay_cycles((long)(CPU_FREQ*(double)x/1000.0))

#define uchar                  unsigned char
#define uint                   unsigned int

uint   curr_signal_type;
int    tccr0_now;
uint   ccr0_idx;
uchar  point_now;
int    push_key;
int    duty_circle;

const long ccr0_table[MAX_FREQ_STEPS] = { 50000, 25000, 16666, 12500, 10000,
		8333, 7142, 6250, 5555, 5000, 4545, 4166, 3846, 3571, 3333, 3125, 2941,
		2777, 2631, 2500, 2380, 2272, 2173, 2083, 2000, 1923, 1851, 1785, 1724,
		1666, 1612, 1562, 1515, 1470, 1428, 1388, 1351, 1315, 1282, 1250, 1219,
		1190, 1162, 1136, 1111, 1086, 1063, 1041, 1020, 1000, 980, 961, 943,
		925, 909, 892, 877, 862, 847, 833, 819, 806, 793, 781, 769, 757, 746,
		735, 724, 714, 704, 694, 684, 675, 666, 657, 649, 641, 632, 625, 617,
		609, 602, 595, 588, 581, 574, 568, 561, 555, 549, 543, 537, 531, 526,
		520, 515, 510, 505, 500, 495, 490, 485, 480, 476, 471, 467, 462, 458,
		454, 450, 446, 442, 438, 434, 431, 427, 423, 420, 416, 413, 409, 406,
		403, 400, 396, 393, 390, 387, 384, 381, 378, 375, 373, 370, 367, 364,
		362, 359, 357, 354, 352, 349, 347, 344, 342, 340, 337, 335, 333, 331,
		328, 326, 324, 322, 320, 318, 316, 314, 312, 310, 308, 306, 304, 303,
		301, 299, 297, 295, 294, 292, 290, 289, 287, 285, 284, 282, 280, 279,
		277, 276, 274, 273, 271, 270, 268, 267, 265, 264, 263, 261, 260, 259,
		257, 256, 255, 253, 252, 251, 250, 248, 247, 246, 245, 243, 242, 241,
		240, 239, 238, 236, 235, 234, 233, 232, 231, 230, 229, 228, 227, 226,
		225, 224, 223, 222, 221, 220, 219, 218, 217, 216, 215, 214, 213, 212,
		211, 210, 210, 209, 208, 207, 206, 205, 204, 204, 203, 202, 201, 200,
		200, 199, 198, 197, 196, 196, 195, 194, 193, 193, 192, 191, 190, 190,
		189, 188, 187, 187, 186, 185, 185, 184, 183, 183, 182, 181, 181, 180,
		179, 179, 178, 177, 177, 176, 176, 175, 174, 174, 173, 173, 172, 171,
		171, 170, 170, 169, 168, 168, 167, 167, 166, 166, 165, 165, 164, 163,
		163, 162, 162, 161, 161, 160, 160, 159, 159, 158, 158, 157, 157, 156,
		156, 155, 155, 154, 154, 153, 153, 152, 152, 151, 151, 151, 150, 150,
		149, 149, 148, 148, 147, 147, 147, 146, 146, 145, 145, 144, 144, 144,
		143, 143, 142, 142, 142, 141, 141, 140, 140, 140, 139, 139, 138, 138,
		138, 137, 137, 136, 136, 136, 135, 135, 135, 134, 134, 134, 133, 133,
		132, 132, 132, 131, 131, 131, 130, 130, 130, 129, 129, 129, 128, 128,
		128, 127, 127, 127, 126, 126, 126, 125, 125, 125, 125 };

const uchar sin_data[TOTAL_SAMPLING_POINTS] = { 127, 140, 152, 165, 177, 188,
		199, 209, 218, 227, 234, 240, 246, 250, 253, 254, 254, 254, 251, 248,
		243, 237, 231, 223, 214, 204, 193, 182, 171, 158, 146, 133, 121, 108,
		96, 83, 72, 61, 50, 40, 31, 23, 17, 11, 6, 3, 0, 0, 0, 1, 4, 8, 14, 20,
		27, 36, 45, 55, 66, 77, 89, 102, 114, 127 };

const uchar tria_data[TOTAL_SAMPLING_POINTS] = { 0, 8, 16, 24, 32, 41, 49, 57,
		65, 74, 82, 90, 98, 106, 115, 123, 131, 139, 148, 156, 164, 172, 180,
		189, 197, 205, 213, 222, 230, 238, 246, 255, 255, 246, 238, 230, 222,
		213, 205, 197, 189, 180, 172, 164, 156, 148, 139, 131, 123, 115, 106,
		98, 90, 82, 74, 65, 57, 49, 41, 32, 24, 16, 8, 0 };

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
