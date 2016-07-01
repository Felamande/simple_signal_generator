#include <msp430.h>

#define P2_IN_PORTS           ~(BIT0 + BIT1 + BIT2)
#define P2_INTERRUPT           (BIT0 + BIT1 + BIT2)
#define P1_OUT_PORTS           (0xff)
#define TOTAL_SAMPLING_POINTS  200
#define MAX_FREQ_STEPS         100
#define ENABLE_WR_PORT         P2OUT |= BIT3
#define DISABLE_WR_PORT        P2OUT &= ~BIT3
//#define MAX_FREQ               500
//#define MIN_FREQ               5
#define FORWARD                1
#define BACKWARD              -1
#define uchar                  unsigned char
#define CPU_FREQ ((double)16000000) //CPU frequency set to 16M(CALBC_16MHZ)
#define delay_us(x) __delay_cycles((long)(CPU_FREQ*(double)x/1000000.0))
#define delay_ms(x) __delay_cycles((long)(CPU_FREQ*(double)x/1000.0))

//long signal_type_incr;
unsigned int curr_signal_type;
int tccr0_now;
unsigned int ccr0_idx;
unsigned char point_now;

int sin_sampling_points;
int triangle_rising_edge_sampling_points;
int triangle_falling_edge_sampling_points;
int box_highlevel_sampling_points;
int box_lowlevel_sampling_points;

const int ccr0_table[MAX_FREQ_STEPS] = {12500, 6250, 4166, 3125, 2500, 2083,
	1785, 1562, 1388, 1250, 1136, 1041, 961, 892, 833, 781, 735, 694, 657,
	625, 595, 568, 543, 520, 500, 480, 462, 446, 431, 416, 403, 390, 378,
	367, 357, 347, 337, 32\8, 320, 312, 304, 297, 290, 284, 277, 271, 265,
	260, 255, 250, 245, 240, 235, 231, 227, 223, 219, 215, 211, 208, 204,
	201, 198, 195, 192, 189, 186, 183, 181, 178, 176, 173, 171, 168, 166,
	164, 162, 160, 158, 156, 154, 152, 150, 148, 147, 145, 143, 142, 140,
	138, 137, 135, 134, 132, 131, 130, 128, 127, 126, 125};

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

#pragma vector = TIMER0_A0_VECTOR
__interrupt void timer_A0(void) {
	if (point_now >= TOTAL_SAMPLING_POINTS) {
		point_now = 0;
	}

	switch (curr_signal_type) {
	case 0:
		//sin;
		P1OUT = sin_data[point_now];
		break;
	case 1:
		//triangle
		if (point_now < triangle_rising_edge_sampling_points) {
			P1OUT = (uchar) (point_now);
		} else {
			P1OUT = (uchar) (TOTAL_SAMPLING_POINTS - point_now - 1);
		}
		break;
	case 2:
		//box
		if (point_now < box_highlevel_sampling_points) {
			P1OUT = (uchar) (1);
		} else {
			P1OUT = (uchar) (0);
		}

		break;
	}
	point_now++;
	TA0CCR0 += tccr0_now;
}

#pragma vector = PORT2_VECTOR
__interrupt void port2(void) {
	_DINT();
	//消除抖动，延迟0.1s
	delay_ms(100);
	if (P1OUT & BIT0) {
		//调节波形
//		signal_type_incr++;
		curr_signal_type++;
		if (curr_signal_type >= 3) {
			curr_signal_type = 0;
		}
		point_now = 0;
		return;
	} else if (P1OUT & BIT1) {
		//调节频率增加
		ccr0_idx++;
		if (ccr0_idx >= MAX_FREQ_STEPS) {
			ccr0_idx = 0;
		}
		tccr0_now = ccr0_table[ccr0_idx];

		TA0CCR0 = tccr0_now;

	} else if (P1OUT & BIT2) {
		//调节频率减小
		ccr0_idx--;
		if (ccr0_idx >= MAX_FREQ_STEPS) {
			ccr0_idx = 0;
		}

		tccr0_now = ccr0_table[ccr0_idx];

		TA0CCR0 = tccr0_now;

	}

	P2IFG &= P2_IN_PORTS; //清除标志位
	_EINT();

}

//void init_signal_generator(void) {
//	func_now = signal_func_map[0];
//	//初始产生正弦波
//}

void init_vars() {
	sin_sampling_points = TOTAL_SAMPLING_POINTS;

	triangle_rising_edge_sampling_points = TOTAL_SAMPLING_POINTS / 2;
	triangle_falling_edge_sampling_points = TOTAL_SAMPLING_POINTS
			- triangle_rising_edge_sampling_points;

	box_highlevel_sampling_points = TOTAL_SAMPLING_POINTS / 2;
	box_lowlevel_sampling_points = TOTAL_SAMPLING_POINTS
			- box_highlevel_sampling_points;

	curr_signal_type = 0;
	point_now = 0;

	ccr0_idx = 0;
	tccr0_now = 80; //ccr0_table[0];

}

void init_DCO() {
	BCSCTL1 = CALBC1_16MHZ;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL2 = SELM_1 + DIVM_0;
	// select DCO as the source of MCLK
	BCSCTL2 &= ~SELS;
	// select DCO as the source of  SMCLK

}

void init_timer_A0(void) {
	TA0CTL |= TASSEL_2 + MC_2; //SMCLK and Mode continous
	TA0CCR0 = tccr0_now;
	TA0CCTL0 |= CCIE; //interrupt enable
	_EINT();
}

//
void init_port_io(void) {
	P1DIR = P1_OUT_PORTS; //P1 11111111b all out
	P1REN = 0x00; //disable pull/down resistor
	P1SEL = 0x00; //io function is selected
	P1SEL2 = 0x00;

	P2DIR &= P2_IN_PORTS; //P2.0 P2.1 p2.2 in
	P2REN = 0x00;
	P2SEL = 0x00;
	P2SEL2 = 0x00;
	//TODO:设置DAC模式(单缓冲) P2REN的值
}

void init_port_interrupt(void) {

	P2IES |= P2_INTERRUPT; //置1，下降沿触发
	P2IE |= P2_INTERRUPT;  //中断使能
	P2IFG &= ~P2_INTERRUPT; //清除标志位
}

void test_port() {
	_DINT();
	init_port_io();
	P1OUT = 0x00;
	while (1) {
		P1OUT = 0xff;
//		P1OUT = 0x00;
//		delay_ms(1000);
//		P1OUT = 0xff;
//		delay_ms(1000);
	}
}

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	init_vars();
	init_port_io();
	init_DCO();
	init_timer_A0();
	init_port_interrupt();
//	test_port();

	P1OUT = 0x00;
	while (1) {
//		P1OUT = 0xf0;
//		LPM0;
//		__delay_cycles(500);
	}
//	return 0;
}
