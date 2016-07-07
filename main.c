#include <msp430.h>
#define SWITCH_SIG_TYPE                (BIT0)
#define ADD_FREQ                       (BIT1)
#define SUB_FREQ                       (BIT2)
#define ADC10_IN_PORT                  (BIT4)
#define DAC_WR                         (BIT3)
#define P1_IN_PORTS                   ~(SWITCH_SIG_TYPE + ADD_FREQ + SUB_FREQ + ADC10_IN_PORT)
#define P1_OUT_PORTS                    DAC_WR // 3:DAC WR
#define P1_INTERRUPT                   (SWITCH_SIG_TYPE + ADD_FREQ + SUB_FREQ)
#define P2_OUT_PORTS                   (0xff) // DAC data in

#define TOTAL_SAMPLING_POINTS           100
#define MAX_FREQ_STEPS                  400

#define ENABLE_WR_PORT                  P1OUT &= ~DAC_WR // WR->0
#define DISABLE_WR_PORT                 P1OUT |= DAC_WR // WR->1
#define write_dac(data)                 P2OUT = data    // write to DAC

#define CPU_FREQ    ((double)16000000) // CPU frequency set to 16M(CALBC_16MHZ)
#define delay_us(x) __delay_cycles((long)(CPU_FREQ * (double)x / 1000000.0))
#define delay_ms(x) __delay_cycles((long)(CPU_FREQ * (double)x / 1000.0))

#define uchar unsigned char
#define uint  unsigned int

uint   curr_signal_type;
int    tccr0_now;
uint   ccr0_idx;
uchar  point_now;
int    push_key;
int    duty_circle;

const long ccr0_table[MAX_FREQ_STEPS] = { 32000, 16000, 10666, 8000, 6400, 5333,
		4571, 4000, 3555, 3200, 2909, 2666, 2461, 2285, 2133, 2000, 1882, 1777,
		1684, 1600, 1523, 1454, 1391, 1333, 1280, 1230, 1185, 1142, 1103, 1066,
		1032, 1000, 969, 941, 914, 888, 864, 842, 820, 800, 780, 761, 744, 727,
		711, 695, 680, 666, 653, 640, 627, 615, 603, 592, 581, 571, 561, 551,
		542, 533, 524, 516, 507, 500, 492, 484, 477, 470, 463, 457, 450, 444,
		438, 432, 426, 421, 415, 410, 405, 400, 395, 390, 385, 380, 376, 372,
		367, 363, 359, 355, 351, 347, 344, 340, 336, 333, 329, 326, 323, 320,
		316, 313, 310, 307, 304, 301, 299, 296, 293, 290, 288, 285, 283, 280,
		278, 275, 273, 271, 268, 266, 264, 262, 260, 258, 256, 253, 251, 250,
		248, 246, 244, 242, 240, 238, 237, 235, 233, 231, 230, 228, 226, 225,
		223, 222, 220, 219, 217, 216, 214, 213, 211, 210, 209, 207, 206, 205,
		203, 202, 201, 200, 198, 197, 196, 195, 193, 192, 191, 190, 189, 188,
		187, 186, 184, 183, 182, 181, 180, 179, 178, 177, 176, 175, 174, 173,
		172, 172, 171, 170, 169, 168, 167, 166, 165, 164, 164, 163, 162, 161,
		160, 160, 159, 158, 157, 156, 156, 155, 154, 153, 153, 152, 151, 150,
		150, 149, 148, 148, 147, 146, 146, 145, 144, 144, 143, 142, 142, 141,
		140, 140, 139, 139, 138, 137, 137, 136, 136, 135, 135, 134, 133, 133,
		132, 132, 131, 131, 130, 130, 129, 129, 128, 128, 127, 126, 126, 125,
		125, 125, 124, 124, 123, 123, 122, 122, 121, 121, 120, 120, 119, 119,
		118, 118, 118, 117, 117, 116, 116, 115, 115, 115, 114, 114, 113, 113,
		113, 112, 112, 111, 111, 111, 110, 110, 109, 109, 109, 108, 108, 108,
		107, 107, 107, 106, 106, 105, 105, 105, 104, 104, 104, 103, 103, 103,
		102, 102, 102, 101, 101, 101, 100, 100, 100, 100, 99, 99, 99, 98, 98,
		98, 97, 97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 94, 93, 93, 93,
		93, 92, 92, 92, 91, 91, 91, 91, 90, 90, 90, 90, 89, 89, 89, 89, 88, 88,
		88, 88, 87, 87, 87, 87, 86, 86, 86, 86, 86, 85, 85, 85, 85, 84, 84, 84,
		84, 83, 83, 83, 83, 83, 82, 82, 82, 82, 82, 81, 81, 81, 81, 81, 80, 80,
		80, 78, 75 };

const uchar sin_data[TOTAL_SAMPLING_POINTS] = { 127, 135, 143, 151, 159, 167,
		174, 182, 189, 196, 203, 209, 215, 221, 226, 231, 235, 239, 243, 246,
		249, 251, 253, 254, 254, 254, 254, 253, 252, 250, 247, 245, 241, 237,
		233, 228, 223, 218, 212, 206, 199, 192, 185, 178, 171, 163, 155, 147,
		139, 131, 123, 115, 107, 99, 91, 83, 76, 69, 62, 55, 48, 42, 36, 31, 26,
		21, 17, 13, 9, 7, 4, 2, 1, 0, 0, 0, 0, 1, 3, 5, 8, 11, 15, 19, 23, 28,
		33, 39, 45, 51, 58, 65, 72, 80, 87, 95, 103, 111, 119, 127 };

const uchar tria_data[TOTAL_SAMPLING_POINTS] = { 0, 5, 10, 15, 20, 26, 31, 36,
		41, 46, 52, 57, 62, 67, 72, 78, 83, 88, 93, 98, 104, 109, 114, 119, 124,
		130, 135, 140, 145, 150, 156, 161, 166, 171, 176, 182, 187, 192, 197,
		202, 208, 213, 218, 223, 228, 234, 239, 244, 249, 255, 255, 249, 244,
		239, 234, 228, 223, 218, 213, 208, 202, 197, 192, 187, 182, 176, 171,
		166, 161, 156, 150, 145, 140, 135, 130, 124, 119, 114, 109, 104, 98, 93,
		88, 83, 78, 72, 67, 62, 57, 52, 46, 41, 36, 31, 26, 20, 15, 10, 5, 0 };

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_A0(void) {

	if (point_now >= TOTAL_SAMPLING_POINTS) {
		point_now = 0;
	}

	switch (curr_signal_type) {
	case 0:
		// sin;
		write_dac(sin_data[point_now]);
		break;
	case 1:
		// triangle
		write_dac(tria_data[point_now]);
		break;
	case 2:
		// box
		if (point_now < duty_circle) {
			write_dac(0xff);
		} else {
			write_dac(0x00);
		}

		break;
	}

	ENABLE_WR_PORT;
	__delay_cycles(4);
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
			ccr0_idx = MAX_FREQ_STEPS - 1;
		}

		tccr0_now = ccr0_table[ccr0_idx];
	}

	P1IFG &= ~P1_INTERRUPT; //清除标志位
}

void init_vars() {
	curr_signal_type = 0;
	point_now = 0;

	ccr0_idx = 398;
	tccr0_now = ccr0_table[ccr0_idx];

	duty_circle = TOTAL_SAMPLING_POINTS / 2;
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
	TA0CTL |= TASSEL_2 + MC_2; // SMCLK source and Mode continous
	TA0CCR0 = tccr0_now;
	TA0CCTL0 |= CCIE; // interrupt enable
	_EINT();
}

void init_port_io(void) {
	P2DIR = P2_OUT_PORTS; // P2 11111111b all out
	P2REN = 0x00;         // disable pull up/down resistor
	P2SEL = 0x00;         // io function is selected
	P2SEL2 = 0x00;

	P1DIR &= P1_IN_PORTS;  // P1.0 P1.1 p1.2 p1.4 in
	P1DIR |= P1_OUT_PORTS; // P1.3 for DAC WR
	P1REN = 0x00;
	P1SEL = 0x00;
	P1SEL2 = 0x00;
}

void init_port_interrupt(void) {

	P1IES |= P1_INTERRUPT;  //置1，下降沿触发
	P1IE |= P1_INTERRUPT;   //中断使能
	P1IFG &= ~P1_INTERRUPT; //清除标志位
}

void init_ADC10(void) {

	ADC10CTL1 |= INCH_4;      // A4 channel for convertion, P1.4 in
	ADC10CTL1 |= SHS_0;       // Sample-and-hold source select ADC10SC
	ADC10CTL1 |= ADC10SSEL_3; // SMCLK 16M
	ADC10CTL1 &= ~ADC10DF;    // straght binary format
	ADC10CTL1 |= CONSEQ_0;    // Single channel single convertion

	ADC10AE0 = ADC10_IN_PORT; // P1.4 in

	ADC10CTL0 &= ~ADC10IE;                              // disable interrupt
	ADC10CTL0 |= SREF_1 + ADC10SHT_0 + REF2_5V + REFON; // V+ = 2.5V, V- = Vss = 0
	ADC10CTL0 |= MSC; // further sample and conversions are performed
					  // automatically
	// as soon as the prior conversion is completed
	ADC10CTL0 &= ~REFOUT; // diasable refout to p1.3 p1.4
	ADC10CTL0 |= ADC10ON; // enable adc
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
	WDTCTL = WDTPW | WDTHOLD; // Stop watchdog timer

	init_vars();
	init_port_io();
	init_port_interrupt();
	init_DCO();
	init_timer_A0();
	init_ADC10();

	_bis_SR_register(GIE);

	while (1) {
		ADC10CTL0 &= ~ENC; //关闭采样使能
		while (ADC10CTL1 & ADC10BUSY);      //检测是否忙
		ADC10CTL0 |= ENC + ADC10SC;         //打开采样使能，开始转换
		while (ADC10CTL1 & ADC10BUSY);      //检测是否忙
		int adc_data = ADC10MEM;            //读取数据
		duty_circle = (adc_data >> 4) + 20; //占空比限制在 20(20%)~83(83%)之间
											//采集到的数据是0~1023
											//右移三位就是0~63
											//加20就是20~83
											//总点数是100点
		                                    //占空比就是20/100=20% ~ 83/100=83% 之间
	}
}
