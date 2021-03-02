#include "mcu_init.h"
#include "dataType.h"

#define dt3 0.05//위치 제어주기
#define dt2 0.005//속도 제어주기
#define dt1 0.0005//전류 제어주기
#define Kpc 0.827//전류 P
#define Kic 2.2117e+03//전류 I
#define Kac 4.030632809//1/3Kpc 전류 ANTI WINDUP
#define Kps 1.505//2.3004755//0.7297555861//속도 P
#define Kis 41.658//1.71944//7.22445528//속도 I
#define Kas 0.96445//1.75//속도 ANTIWINDUP
#define Kpp 2//8.391//위치 P
#define Kdp 0.1//2.51//위치 D
#define Kt 0.0683//역기전력 상수, 토크 상수

enum{
	CURRENT_CONTROL = 0x01,
	VELOCITY_CONTROL = 0x02,
	POSITION_CONTROL = 0x04
};

//엔코더에서 받은 현재 Shaft 위치
volatile int32_t g_Cnt, g_preCnt;
//SERIAL SIG
volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;
volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

//전류 제어
volatile double control_C = 0;//전류 제어기 출력(전압 지령)
volatile double g_current_des;//REP 전류
volatile double g_current_cur;//현재 전류
volatile double g_current_err;//전류 오차
volatile double g_current_esum;//전류 오차 누적
volatile double g_saturation_C = 24;//최대 전압 SATURATION
//속도 제어
volatile double control_S = 0;//속도 제어기 출력(전류 지령)
volatile double g_velocity_des;//REP 각속도
volatile double g_velocity_cur;//현재 각속도
volatile double g_velocity_err;//현재 각속도 오차
volatile double g_velocity_esum;//각속도 오차 누적
volatile double g_saturation_S = 1.46;//최대 전류 SATURATION

//위치 제어
volatile double control_P = 0;//위치 제어기 출력(각속도 지령)
volatile double g_position_des = 0;//REP 위치
volatile double g_position_cur = 0;//현재 위치
volatile double g_position_pre = 0;//이전 위치
volatile double g_position_err;//REP - 현재위치(오차)
volatile double g_position_perr;//이전 제어주기 위치 오차
volatile double g_saturation_P = 100*M_PI/180;//최대 각속도 SATURATION
volatile double g_velocity_cur;//현재 각속도
volatile double g_velocity_pre;//이전 제어주기 각속도


volatile double g_ADC;//adc(전류 측정) 저장
volatile int g_SendFlag = 0;//200번의 제어 주기를 실행할 때 마다 while문에서 pc로 데이터 전송하기 위한 flag

volatile unsigned char g_TimerCnt;
volatile unsigned char g_ControlMode = POSITION_CONTROL | VELOCITY_CONTROL | POSITION_CONTROL;


void SetDutyCW(double v){//-24~24V입력 음수는 ccw 양수는 cw 바이폴라 방식을 사용
	while(TCNT1==0);

	int ocr = v * (200. / 24.) + 200;//음수 ccw 양수 cw
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;
	
	OCR1A = OCR3B = ocr + 8;		//1 H dead time 미작성시 모터 드라이버 쇼트
	OCR1B = OCR3A = ocr - 8;		//1 L H->L L->H 가 되는데 시간이 필요함 이 때 순간 두 신호 모두 0이 되는순간이 발생하는데 이 때 24V와 GND가 쇼트됨
}

void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}



int getADC(){//acs712 전류센서 0A 2.5V를 기준으로 1A당 0.1V 변화
	ADMUX = (ADMUX & 0xf0);
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}

ISR(USART0_RX_vect){
	
	//while ( !(UCSR0A & (1<<RXC0)) );
	g_buf[g_BufWriteCnt++] = UDR0;
}

ISR(TIMER0_OVF_vect){
	
	TCNT0 = 256 - 125;//0.5ms 125^64/16000000
	
	//Read LS7366
	
	int32_t cnt;

	PORTC = 0x01;
	
	g_ADC = getADC();//전류 ADC 데이터 입력 받음
	
	g_preCnt = g_Cnt;
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);//0b00101000 0b11000000
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();      cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;//
	
	//if(cnt >= 331776) cnt = (int32_t)cnt- 331776;
	//else if(cnt <= -331776) cnt = (int32_t)cnt + 331776;
	g_Cnt = -cnt;//누적 pulse 값 반환
	g_position_cur = (double)(g_Cnt * M_PI / 165888.);//2piM/(PPR) M-Method[rad]
	//PPR = 1024 * 4 * 81(분해능 * 채배 * 기어비) rad각도로 변환
	
	PORTC = 0x03;
	//g_position_des = (double)M_PI/2;
	//if(g_position_des < 0) g_position_des +=2*M_PI;
	//else if(g_position_des >= 2*M_PI) g_position_des -=2*M_PI;//각도는 0~2pi 범위만 쓸 것임
	//if(g_position_cur < 0) g_position_cur +=2*M_PI;
	//else if(g_position_cur >= 2*M_PI) g_position_cur -=2*M_PI;//각도는 0~2pi 범위만 쓸 것임

	if((g_TimerCnt % 100) == 0){//위치 제어 50ms 0.5ms * 100
		g_TimerCnt = 0;
		
		g_position_perr = g_position_err;//이전 오차 저장
		g_position_err = g_position_des - g_position_cur;//현재 오차 계산
		control_P = Kpp * g_position_err + Kdp * (g_position_err - g_position_perr)/dt3;//미분이 아닌 차분 이용
		//현재 오차에서 이전 오차를 뺀 뒤 위치 제어 주기로 나누어 차분
		if(control_P > 2.2689) {//제어기 출력(속도 지령)이 최대 각속도 값보다 크면 최대 값으로 변경
			control_P = 2.2689;
		}
		else if(control_P < -2.2689) {
			control_P = -2.2689;
		}
	}
	
	g_velocity_des = control_P;//-10*M_PI/180;//위치 제어기의 속도 지령을 속도 제어기에 입력
	
	if((g_TimerCnt % 10) == 0){//속도 제어 5ms 0.5ms * 10
		
		if(g_velocity_des >= g_saturation_P) g_velocity_des = g_saturation_P;
		else if(g_velocity_des <= -g_saturation_P) g_velocity_des = -g_saturation_P;
		
		//if((g_position_cur < M_PI) & (g_position_pre > M_PI)& (g_velocity_pre>0) )   g_position_pre -= 2*M_PI;
		//CCW,CW회전시 각도 범위 구간 제한 0rad를 지나가는 구간에서 각속도 계산에 문제 발생
		//else if((g_position_cur > M_PI)&(g_position_pre < M_PI) & (g_velocity_pre<0)) g_position_pre += 2*M_PI;
		g_velocity_cur = (double)(g_position_cur - g_position_pre) / 0.005;//각속도 계산
		g_position_pre = g_position_cur;//이전 위치 값을 저장함
		g_velocity_pre = g_velocity_cur;
		
		g_velocity_err = (double)g_velocity_des - g_velocity_cur;//각속도 오차 계산
		control_S = (double)(Kps * g_velocity_err + Kis * g_velocity_esum *0.005);//제어기 출력(전류 지령)
		g_velocity_esum += g_velocity_err;//각속도 오차 누적
		
		if(control_S >= 2.08) {//제어기 출력이 전류 최대값보다 커질 경우 제어기 출력을 최대값으로 변경 & anti wind-up 실행
			g_velocity_esum -= (double)(control_S - 2.08) * Kas;
			control_S = 2.08;
		}
		else if(control_S <= -2.08) {
			g_velocity_esum -= (double)(control_S + 2.08) * Kas;
			control_S = -2.08;
		}
	}
	
	g_current_des = control_S;//control_S;
	if(g_current_des >= g_saturation_S) g_current_des = g_saturation_S;
	else if(g_current_des <= -g_saturation_S) g_current_des = -g_saturation_S;  
	//2.489
	g_current_cur = -( ((g_ADC / 1024. * 5.) - 2.49285) * 10.);//1A당 0.1V 증가 ->1V당 10A 증가 i = (adc*5/1024 - 2.5) * 10
	g_current_err = (double)g_current_des - g_current_cur;//desired current - current
	
	control_C = (double)Kpc * g_current_err + Kic * g_current_esum * dt1;//제어기 출력 (전압 지령)
	control_C += (double)g_velocity_cur * Kt;//역기전력 전향보상
	g_current_esum += (double)g_current_err;//오차 누적

	//Anti Wind up saturation 24V
	if(control_C >= 24){//제어기 출력이 전압 최대값보다 커질 경우 제어기 출력을 최대값으로 변경 & anti wind-up 실행
		g_current_esum -= (double)(control_C - 24) * Kac;
		control_C = 24;
	}
	else if(control_C <= -24){
		g_current_esum -= (double)(control_C + 24) * Kac;
		control_C = -24;
	}

	SetDutyCW(control_C);//전압 지령을 모터에 인가
	
	
	/////////////////////////////////////////
	g_TimerCnt++;
	g_SendFlag++;

}




int main(void){

	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;

	InitIO();
	
	//Uart
	InitUart0();
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	g_position_des=0;
	TCNT0 = 256 - 125;//0.5ms
	sei();

	unsigned char check = 0;
	
    while (1) {
		
		 for (;g_BufReadCnt !=g_BufWriteCnt;g_BufReadCnt++)
		 {
			 switch(g_PacketMode){
				 case 0:
				 if (g_buf[g_BufReadCnt] == 0xFF){
					 checkSize++;
					 if (checkSize == 4){
						 g_PacketMode = 1;
					 }
				 }
				 else
				 checkSize = 0;
				 break;
				 
				 case 1:
				 g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				 
				 if (checkSize == 8){
					 if (g_PacketBuffer.data.id == g_ID){
						 g_PacketMode = 2;
						 
					 }
					 else{
						 g_PacketMode = 0;
						 checkSize = 0;
					 }
				 }			 
				 break;
				 case 2:
				 g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				 check += g_buf[g_BufReadCnt];
				 
				 if (checkSize == g_PacketBuffer.data.size){
					 if (check == g_PacketBuffer.data.check){
						 switch(g_PacketBuffer.data.mode){
							 case 2:
								g_position_des = g_PacketBuffer.data.pos / 1000.;
								g_saturation_P = g_PacketBuffer.data.velo / 1000.;
								if(g_saturation_P < 0) g_saturation_P = -g_saturation_P;
								if(g_saturation_P > 2.2689) g_saturation_P = 2.2689;
								g_saturation_S = g_PacketBuffer.data.cur / 1000.;
								if(g_saturation_S < 0) g_saturation_S = -g_saturation_S;
								if(g_saturation_S >2.5) g_saturation_S = 2.5;
							 break;
						 }
					 }
					 check = 0;
					 g_PacketMode = 0;
					 checkSize = 0;
				 }
				 else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){
					 check = 0;
					 g_PacketMode = 0;
					 checkSize = 0;
				 }
			 }
		 }
		 
		 if (g_SendFlag > 19){//0번 timer OVERFLOW INTERRUP 20회 실행시(10ms) 현재 위치,각속도,전류 송신
			 g_SendFlag = 0;
			 
			 
			 packet.data.id = g_ID;
			 packet.data.size = sizeof(Packet_data_t);
			 packet.data.mode = 3;
			 packet.data.check = 0;
			 
			 packet.data.pos = g_position_cur * 1000.;
			 packet.data.velo = g_velocity_cur * 1000.;
			 packet.data.cur = g_current_cur * 1000.;
			 
			 for(int i = 8;i<sizeof(Packet_t);i++)	packet.data.check +=packet.buffer[i];
			 for(int i = 0;i<packet.data.size;i++)	TransUart0(packet.buffer[i]);
			 
		 }
	}
	
	return 0;
		
}

