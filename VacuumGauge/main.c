
#define F_CPU				16000000UL

//PB4-PB6
#define	CS_LOW(num)					PORTB&=~_BV(num)
#define	CS_HIGH(num)				PORTB|=_BV(num)

#define Sound_on()					PORTE|=_BV(3)
#define Sound_off()					PORTE&=~_BV(3)
#define led_Pa()					PORTB|=_BV(0); PORTE&=~_BV(6); PORTE&=~_BV(7)
#define led_mbar()					PORTE|=_BV(7); PORTE&=~_BV(6); PORTB&=~_BV(0)
#define led_mm_rt_st()				PORTE|=_BV(6); PORTE&=~_BV(7); PORTB&=~_BV(0) //����

#define ADC_mux_ch(ch_id)			PORTD|=_BV(6+ch_id); PORTD&=~_BV(7-ch_id)

#define UPDATE_STEP		150 //ms

#define FRAME_MAX_LEN		0x26

#include <stdbool.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <avr/eeprom.h>

#include "LCD.h"
#include "menu.h"
#include "sensors.h"
#include "curves.h"

//������ � eeprom
uint8_t EEMEM mem_rs_address = 1; //����� ���� RS485
pressure_unit EEMEM mem_currentUnit = Pa;
sensor_type EEMEM mem_currentType[2] = {analogSensor, analogSensor};
uint16_t EEMEM mem_bridge_offset[2] = {0, 0};

uint8_t rs_address = 1; //����� ���� RS485
pressure_unit currentUnit = Pa;
sensor_type currentType[2] = {analogSensor, analogSensor};
uint16_t bridge_offset[2] = {0, 0};
	
int boost_count = 1; //��������� ��� ��������� �������� ����� ��� ������� �� ������
volatile float adc_data[2] = {0, 0};
volatile bool isCalibration = false;
volatile uint8_t current_ch = 0; //������� �����


void unitLedOn(pressure_unit unit){
	switch(unit){
		case Pa: led_Pa(); break;
		case mbar: led_mbar(); break;
		case Torr: led_mm_rt_st(); break;
		default: break;
	}
}
void custom_delay(int ms){
	for (int i = 0; i < ms; i++){
		_delay_ms(1);
	}
}
void Global_Init(void){
	DDRA =	0b10000111;
	PORTA = 0b01111000;

	PORTB = 0b01110000;
	DDRB =	0b11110111;

	DDRC =	0b11111111;
	PORTC = 0b00000000;

	DDRD =	0b11101011;
	PORTD = 0b00000000;
	
	DDRF = 0b00000111;
	PORTF = 0b11111000;
	
	DDRE =	0b11111110;
	PORTE = 0b00000000;
	
	DDRG = 0b1111;
	PORTG = 0b0100;
	
	ADC_mux_ch(current_ch);
	
	//-----------��������� 8-������� ������� ��� ���������� ���� ������ UPDATE_STEP ��-----------//
	TCCR1B |= (1<<WGM12); //����� CTC
	TIMSK |= (1<<OCIE1A); //��������� ���������� �� ���������� � OCR1A
	TCCR1B |= (1<<CS12) | (1<<CS10); //������������ 1024
	OCR1A = (uint64_t)F_CPU/(1024*(1/(0.001*UPDATE_STEP))) - 1;  //���-�� ������ ��� ���������
	
	//-----------------------USART init------------------------------------------------
	// USART0
	UBRR0L = (((F_CPU / (9600 * 8UL))) - 1);			//������� ��� ��������� �������� 9600 bps @ 16 MHz
	//UBRR0L = 207;			
	UCSR0A = _BV(U2X0);
	UCSR0B = _BV(TXEN0)|_BV(RXEN0);
	UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);
	// USART1
	UBRR1L = 16; // 115200 bps @ 16 MHz
	UCSR1A = _BV(U2X1); //��������� �������� ��������
	UCSR1B = _BV(TXEN1)|_BV(RXEN1)|_BV(RXCIE1); //���������� �� ����� � �� �������� ����� USART
	UCSR1C = _BV(UCSZ11)|_BV(UCSZ10); //������ ����� 8 ��������
	
	//-----------------------------SPI init--------------------------------------------
	// SPI, 8 MHz clock, mode 0
	SPCR =_BV(SPE)|_BV(MSTR)|_BV(CPOL)|_BV(CPHA)|_BV(SPR0)|_BV(SPI2X);

	LCD_init();
}

/*******************************************************************************
 * USART send-receive
 *******************************************************************************/
void USART0_Send(uint8_t const* data){
	PORTE|= _BV(2); //WRITE_STATE
	while(*data){
		while(!(UCSR0A&_BV(UDRE0)));
		UCSR0A |= _BV(TXC0); //�������� ��� �������� �������� ����� (��� rs485)
		UDR0 = *data++;
	}
	while(!(UCSR0A&_BV(TXC0))); //���� ������� ������ ��������
	PORTE&=~_BV(2); //READ_STATE
}
void USART1_Send(uint8_t const* data){
	PORTD|= _BV(1); //WRITE_STATE
	while(*data){
		while(!(UCSR1A&_BV(UDRE1)));
		UCSR1A |= _BV(TXC1); //�������� ��� �������� �������� ����� (��� rs485)
		UDR1 = *data++;
	}
	while(!(UCSR1A&_BV(TXC1))); //���� ������� ������ ��������
	PORTD&=~_BV(1); //READ_STATE
}
void USART1_Send_Byte(uint8_t const byte){
	PORTD|= _BV(1); //WRITE_STATE
	
	while(!(UCSR1A&_BV(UDRE1)));
	UCSR1A |= _BV(TXC1); //�������� ��� �������� �������� ����� (��� rs485)
	UDR1 = byte;
	
	while(!(UCSR1A&_BV(TXC1))); //���� ������� ������ ��������
	PORTD&=~_BV(1); //READ_STATE
}
// �-� �������� ������
void sendResponse(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t length){
	uint8_t tx_data[FRAME_MAX_LEN];
	uint8_t checksum = 0;
	
	tx_data[0] = 0xAB;
	tx_data[1] = addr;
	tx_data[2] = cmd;
	tx_data[3] = length;
	checksum -= tx_data[1] + tx_data[2] + tx_data[3];
	
	for (int i = 0; i < length; i ++){
		tx_data[4+i] = data[i];
		checksum -= tx_data[4+i];
	}
	tx_data[4+length] = checksum;
	USART0_Send(tx_data);
}
/*******************************************************************************
 * SPI send-receive
 *******************************************************************************/
uint8_t spi(uint8_t byte)
{
	SPDR = byte;
	while (!(SPSR&_BV(SPIF)));
	return SPDR;
}
void write2DAC(int CS_id, int ch_id, uint16_t value){
	value = value > 4095? 4095: value;
	uint16_t pkg;
	pkg = ((0b0011 | (1 << ch_id*3)) << 12) | value;
	CS_LOW(CS_id);
	spi(pkg >> 8);
	spi(pkg);
	CS_HIGH(CS_id);
}
/*******************************************************************************
 * ������ ��������
 *******************************************************************************/
float adc_read(int channel_id){
	uint32_t tmp = 0;
	uint32_t ave = 0;

	for(uint8_t idx = 0; idx < 8; idx++) {
		CS_LOW(4);
		tmp = (uint32_t)spi(0x00) << 16;
		tmp |= (uint32_t)spi(0x00) << 8;
		tmp |= (uint32_t)spi(0x00);
		CS_HIGH(4);
		tmp = (tmp >> 2) & 0xFFFF;
		ave += tmp;
	}
	adc_data[channel_id] = (float)(ave >> 3);
	return adc_data[channel_id];
}
float analog_read(int sensor_id){

	float adc = adc_read(sensor_id);
	
	if (adc < ADC_scale[0]){
		return pressure_scale[0];
	}
	else{
		int pos = 0;
		float pressure = 0; //��
		for (int i = 0; i < sizeof(ADC_scale)/sizeof(*ADC_scale); i++){
			if (adc > ADC_scale[i]){
				pos = i;
			}
		}
		if (pos == (sizeof(ADC_scale)/sizeof(*ADC_scale) - 1)){
			pressure = 100000.0;
		}
		else{
			//������� �������� �� ������� ������������� 
			pressure = pressure_scale[pos] + (adc - (float)ADC_scale[pos])*((pressure_scale[pos + 1] - pressure_scale[pos])/((float)ADC_scale[pos + 1] - (float)ADC_scale[pos]));
			if (pressure > 100000.0){
				pressure = 100000.0;
			}
		}
		return pressure;
	}
}
float thyracont_read(int sensor_id){
	static uint8_t send_buf[16] = "001M^\r"; //3030314D5E0D
	USART0_Send(send_buf);
	custom_delay(3);
	volatile char read_buf[16];
	for (int i = 0; i < 16; i++)
		read_buf[i] = 0;
	volatile int counter = 25000;
	volatile char curr_symb = 0;
	volatile int pos = 0;
	while ((curr_symb != 13) && counter){
		// ����, ���� ���� �� ����� ������ � �� ������ ���������� ������
		while(!(UCSR0A & (1<<RXC0)) && counter){counter--;}
		if (counter){
			curr_symb = UDR0;
			read_buf[pos] = curr_symb;
			pos++;
			if (pos == 16) pos = 0;
		}
	}
	if (counter){
		char mantissa_s[5];
		for (int i = 0; i < 4; i++)
			mantissa_s[i] = read_buf[4+i];
		mantissa_s[4] = 0;
		int mantissa = 0;
		sscanf(mantissa_s, "%d", &mantissa);
		char power_s[3] = {read_buf[8], read_buf[9], 0};
		int power = 0;
		sscanf(power_s, "%d", &power);
		float pressure_pa = 100.0*((float)mantissa/1000.0 * pow(10, power - 20));
		char test_buf[30];
		char fl[15];
		dtostrf(pressure_pa, 1, 3, fl);
		sprintf(test_buf, "%s;%u\n", fl, (uint16_t)adc_data[0]);
		USART1_Send(test_buf);
		return pressure_pa;
	}
	else{
		return -1.0;
	}
}
float logarithmic_read(int sensor_id){
	return 1.2345;
}
void setSensorType(int sensor_id, sensor_type type){
	switch(type){
		case analogSensor: setCalcFunc(sensor_id, analog_read); break;
		case digitalSensor: setCalcFunc(sensor_id, thyracont_read); break;
		case sensorFormula: setCalcFunc(sensor_id, logarithmic_read); break;
		default: setCalcFunc(sensor_id, analog_read); break;
	}
}
/*******************************************************************************
 * ������ ���������� ��� ������ ������ � ��
 *******************************************************************************/
ISR(USART1_RX_vect){
	uint8_t byte = UDR1;
	USART1_Send_Byte(byte);
}
/*******************************************************************************
 * �������� ������ ��� ����������
 *******************************************************************************/
ISR(TIMER1_COMPA_vect){

	if (!isCalibration){
		sensorCalc(current_ch);
		setMenuText(0, current_ch == 0? sensor_str(0, currentUnit): NULL, current_ch == 0? NULL: sensor_str(1, currentUnit));
		uint16_t dac_value = sensor(current_ch, Pa) <= 0? 0: 636.896f*log10f(sensor(current_ch, Pa))+910.521f;
		write2DAC(6, current_ch, dac_value);
		current_ch = current_ch == 0? 1: 0;
		ADC_mux_ch(current_ch);
	}
	
	updateMenu(UPDATE_STEP);
}

/*******************************************************************************
 * T3/cont beep function
 *******************************************************************************/
void OnBeep(uint8_t t_ms){
	TCNT3 = 0;
	OCR3A = 16*t_ms;
	if((ETIFR & (1<<OCF3A))!=0) 
		ETIFR|=(1<<OCF3A);   //������� �� ������ ������ ���� ����������
	ETIMSK = _BV(OCIE3A);
	TCCR3B = _BV(WGM32)|_BV(CS32)|_BV(CS30);
	Sound_on();
}
/*******************************************************************************
 * Timer3 ISR (���������� �������) 
 *******************************************************************************/
ISR(TIMER3_COMPA_vect){	
	Sound_off();
	//�������� ���������� �� �������  
	ETIMSK &=~_BV(OCIE3A);
}

//������ ������
uint8_t keypad_read(void)
{
	uint8_t key_press = 0;
	static uint8_t key_rising = 0;
	static uint8_t press_count[2] = {0, 0}; //������� ������� ���������� ������
	
	if ((!(PINA&_BV(5)))&&(!(key_rising&_BV(2)) || (press_count[0] > 20))) { //������ ����
		PORTE |= (1<<PE4);
		moveMenu(Lower);
		key_press = 2; 
		key_rising|=_BV(2); 
	}
	else if(PINA&_BV(5)){
		key_rising&=~_BV(2);
		press_count[0] = 0;
	}
	if(!(PINA&_BV(5)) && (press_count[0] < 200)){
		press_count[0]++;
	}
	
	if ((!(PINA&_BV(6)))&&(!(key_rising&_BV(3)) || (press_count[1] > 20))) { //������ �����
		PORTE &= ~(1<<PE4);
		moveMenu(Up);
		key_press = 3; 
		key_rising|=_BV(3); 
	}
	else if(PINA&_BV(6)){ 
		key_rising&=~_BV(3);
		press_count[1] = 0;
	}
	if(!(PINA&_BV(6)) && (press_count[1] < 200)){
		press_count[1]++;
	}
	
	int count = press_count[0] > press_count[1] ? press_count[0]: press_count[1];
	if (count >= 200)
		boost_count = 10;
	else if (count > 150)
		boost_count = 5;
	else if (count > 100)
		boost_count = 3;
	else if (count > 50)
		boost_count = 2;
	else
		boost_count = 1;
	
	if ((!(PINA&_BV(3)))&&(!(key_rising&_BV(1)))) { //������ ����
		menuButtton();
		key_press = 4; 
		key_rising|=_BV(1); 
	}
	else if(PINA&_BV(3)) 
		key_rising&=~_BV(1);
	
	if ((!(PINA&_BV(4)))&&(!(key_rising&_BV(4)))) { //������ Ok
		menuOk();
		key_press = 1; 
		key_rising|=_BV(4); 
	}
	else if(PINA&_BV(4)) 
		key_rising&=~_BV(4);
		
	if (key_press && (press_count[0] < 10) && (press_count[1] < 10))
		OnBeep(150);
	return key_press;
}

/* ������� ���������� ������� � ��������� */
typedef enum {Vacuum, Atmosphere} pressure_type;
bool calibration(pressure_type type, int sensor_id){
	static char buf[16];
	switch(type){
		case Vacuum:
			break;
		case Atmosphere:{
			custom_delay(10);
			if (current_ch != sensor_id){
				ADC_mux_ch(sensor_id);
			}
			custom_delay(10);
			if (sensor(sensor_id, Pa) < 0){
				sprintf(buf, "������ �%d", sensor_id + 1);
				displayQueue(buf, "�� ���������", 3000);
				return false;
			}
			else if (sensor(sensor_id, Pa) < 35000){
				sprintf(buf, "�%d ����������", sensor_id + 1);
				displayQueue("� ������ ������/", buf, 3000);
				return false;
			}
			sprintf(buf, "���������� �%d...", sensor_id + 1);
			displayQueue("���������, ����", buf, 60000);
			uint16_t temp_bias = bridge_offset[sensor_id];
			size_t scale_size = sizeof(ADC_scale)/sizeof(*ADC_scale);
			int count = 4095;
			while(((adc_data[sensor_id] < 0.9999*(float)ADC_scale[scale_size-1]) || (adc_data[sensor_id] > 1.0001*(float)ADC_scale[scale_size-1])) && --count){
				if ((adc_data[sensor_id] < ADC_scale[scale_size-1]) && (temp_bias < 4095)){
					temp_bias++;
				}
				else if ((adc_data[sensor_id] > ADC_scale[scale_size-1]) && (temp_bias > 0)){
					temp_bias--;
				}
				write2DAC(5, sensor_id, temp_bias);
				custom_delay(5);
				adc_read(sensor_id);
				custom_delay(1);
			}
			ADC_mux_ch(current_ch);
			custom_delay(10);
			if (count){
				bridge_offset[sensor_id] = temp_bias;
				return true;
			}
			else{
				return false;
			}
			break;
		}
		default:
			break;
	}
	return false;
}

/************ ������� ��� ���� � ������� **************/
void atmosphCalibration(button but){ //���������� ���������
	static uint8_t curr_sensor = 0;
	static char buf[16];

	switch(but){
		case Up: case Lower:
			curr_sensor = curr_sensor == 0? 1: 0;
			sprintf(buf, "%d", curr_sensor + 1);
			setSubMenuText(2, 0, NULL, buf);
			break;
		case Ok:{
			isCalibration = true;
			sprintf(buf, "���������� �%d", curr_sensor+1);
			if (calibration(Atmosphere, curr_sensor)){
				displayQueue(buf, "������ �������!", 5000);
				eeprom_write_word(&mem_bridge_offset[curr_sensor], bridge_offset[curr_sensor]);
			}
			else{
				displayQueue(buf, "�� �������!", 5000);
			}
			isCalibration = false;
			break;
		}
		case Reset:
			curr_sensor = 0;
			sprintf(buf, "%d", curr_sensor + 1);
			setSubMenuText(2, 0, NULL, buf);
			break;
		default:
			break;
	}
}
void vacuumCalibration(button but){ //���������� �������
	static uint8_t curr_sensor = 0;
	char buf[16];
	switch(but){
		case Up: case Lower:
			curr_sensor = curr_sensor == 0? 1: 0;
			sprintf(buf, "%d", curr_sensor + 1);
			setSubMenuText(3, 0, NULL, buf);
			break;
		case Ok:
			calibration(Vacuum, curr_sensor);
			break;
		case Reset:
			curr_sensor = 0;
			sprintf(buf, "%d", curr_sensor + 1);
			setSubMenuText(3, 0, NULL, buf);
			break;
		default:
			break;
	}
}
void addressSelection(button but){ // ����� ������ RS485
	static uint8_t temp_address = 1;
	static uint8_t max = 32;
	static uint8_t min = 1;
	char buf[16];
	switch(but){
		case Up:
			if (temp_address == max) temp_address = min;
			else temp_address++;
			sprintf(buf, "%d", temp_address);
			setSubMenuText(4, 0, NULL, buf);
			break;
		case Lower:
			if (temp_address == min) temp_address = max;
			else temp_address--;
			sprintf(buf, "%d", temp_address);
			setSubMenuText(4, 0, NULL, buf);
			break;
		case Ok:
			rs_address = temp_address;
			eeprom_write_byte(&mem_rs_address, rs_address);
			break;
		case Reset:
			temp_address = rs_address;
			sprintf(buf, "%d", temp_address);
			setSubMenuText(4, 0, NULL, buf);
			break;
		default:
			break;
	}
}
void unitSelection(button but){ // ����� ������ ���������
	static char* unit_names[3] = {"��", "����", "����"};
	static uint8_t temp_unit = 0;
	switch(but){
		case Up:
			if (temp_unit == (sizeof(unit_names)/sizeof(*unit_names) - 1)) temp_unit = 0;
			else temp_unit++;
			setSubMenuText(1, 0, NULL, unit_names[temp_unit]);
			break;
		case Lower:
			if (temp_unit == 0) temp_unit = sizeof(unit_names)/sizeof(*unit_names) - 1;
			else temp_unit--;
			setSubMenuText(1, 0, NULL, unit_names[temp_unit]);
			break;
		case Ok:
			currentUnit = (pressure_unit)temp_unit;
			unitLedOn(currentUnit);
			eeprom_write_byte(&mem_currentUnit, currentUnit);
			break;
		case Reset:
			temp_unit = (uint8_t)currentUnit;
			setSubMenuText(1, 0, NULL, unit_names[temp_unit]);
			break;
		default:
			break;
	}
}
void hardReset(button but){ //����� �� ��������� ��������
	static char* confirm_text[2] = {"���", "��"};
	static bool confirm = false;
	switch(but){
		case Up: case Lower:
			confirm = !confirm;
			setSubMenuText(8, 0, NULL, confirm_text[confirm]);
			break;
		case Ok:{
			if (confirm){
				currentUnit = Pa;
				eeprom_write_byte(&mem_currentUnit, currentUnit);
				unitLedOn(currentUnit);
				
				for (uint8_t i = 0; i < 2; i++){
					bridge_offset[i] = 2047;
					eeprom_write_word(&mem_bridge_offset[i], bridge_offset[i]);
					write2DAC(5, i, bridge_offset[i]);
					currentType[i] = analogSensor;
					eeprom_write_byte(&mem_currentType[i], currentType[i]);
					setSensorType(i, currentType[i]);
				}
				
				rs_address = 1;
				eeprom_write_byte(&mem_rs_address, rs_address);
				
				displayQueue("�����", "��������!", 5000);
			}
			break;
		}
		case Reset:
			confirm = false;
			setSubMenuText(8, 0, NULL, confirm_text[confirm]);
			break;
		default:
			break;
	}
}
void choose_analog_0(button but){ //������� �1 ����������
	switch(but){
		case Ok:{
			currentType[0] = analogSensor;
			eeprom_write_byte(&mem_currentType[0], currentType[0]);
			setSensorType(0, currentType[0]);
			displayQueue("������ ������.", "������ �1!", 5000);
			break;
		}
		default:
		break;
	}
}
void choose_analog_1(button but){ //������� �2 ����������
	switch(but){
		case Ok:{
			currentType[1] = analogSensor;
			eeprom_write_byte(&mem_currentType[1], currentType[1]);
			setSensorType(1, currentType[1]);
			displayQueue("������ ������.", "������ �2!", 5000);
			break;
		}
		default:
		break;
	}
}
void choose_thyracont_0(button but){ //������� �1 ��������
	switch(but){
		case Ok:{
			if (currentType[1] != digitalSensor){
				currentType[0] = digitalSensor;
				eeprom_write_byte(&mem_currentType[0], currentType[0]);
				setSensorType(0, currentType[0]);
				displayQueue("������ ��������", "������ �1!", 5000);
			}
			else{
				displayQueue("������! ���", "������ �2!", 5000);
			}
			break;
		}
		default:
		break;
	}
}
void choose_thyracont_1(button but){ //������� �2 ��������
	switch(but){
		case Ok:{
			if (currentType[0] != digitalSensor){
				currentType[1] = digitalSensor;
				eeprom_write_byte(&mem_currentType[1], currentType[1]);
				setSensorType(1, currentType[1]);
				displayQueue("������ ��������", "������ �2!", 5000);
			}
			else{
				displayQueue("������! ���", "������ �1!", 5000);
			}
			break;
		}
		default:
		break;
	}
}
void choose_log_0(button but){ //������� �1 ���������������
	switch(but){
		case Ok:{
			currentType[0] = sensorFormula;
			eeprom_write_byte(&mem_currentType[0], currentType[0]);
			setSensorType(0, currentType[0]);
			break;
		}
		default:
		break;
	}
}
void choose_log_1(button but){ //������� �2 ���������������
	switch(but){
		case Ok:{
			currentType[1] = sensorFormula;
			eeprom_write_byte(&mem_currentType[1], currentType[1]);
			setSensorType(1, currentType[1]);
			break;
		}
		default:
		break;
	}
}

float map(float x, float in_min, float in_max, float out_min, float out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*** �������� ������ �� ������ ***/
void Data_Load(){
	currentUnit = eeprom_read_byte(&mem_currentUnit);
	unitLedOn(currentUnit);
	
	for (uint8_t i = 0; i < 2; i++){
		currentType[i] = eeprom_read_byte(&mem_currentType[i]);
		setSensorType(i, currentType[i]);
		bridge_offset[i] = eeprom_read_word(&mem_bridge_offset[i]);
		write2DAC(5, i, bridge_offset[i]);
	}
	
	rs_address = eeprom_read_byte(&mem_rs_address);
}

int main(void)
{
	Global_Init();
	sei();
	
	displayQueue("����������", "������� ��-10", 1500);
	displayQueue("��������", "���. 2.0", 1500);
	
	/*** ������ ���� ***/
	createMenusItem("P1 = ---", "P2 = ---");

	createMenusItem("�������", "���������");
	createSubItem(1, "����� ��. ���.:", "");
	setSubItemAsChangeable(1, 0);
	setSubItemAction(1, 0, unitSelection);

	createMenusItem("����������", "���������");
	createSubItem(2, "�������� ������:", "");
	setSubItemAsChangeable(2, 0);
	setSubItemAction(2, 0, atmosphCalibration);

	createMenusItem("����������", "�������");
	createSubItem(3, "�������� ������:", "");
	setSubItemAsChangeable(3, 0);
	setSubItemAction(3, 0, vacuumCalibration);

	createMenusItem("����� � ����", "RS485");
	createSubItem(4, "������� �����:", "");
	setSubItemAsChangeable(4, 0);
	setSubItemAction(4, 0, addressSelection);

	createMenusItem("����� ����", "������� �1");
	createSubItem(5, "����������", "������");
	setSubItemAction(5, 0, choose_analog_0);
	createSubItem(5, "�������� ������", "Teracont");
	setSubItemAction(5, 1, choose_thyracont_0);
	createSubItem(5, "������ ��������", "�� �������");
	setSubItemAction(5, 2, choose_log_0);
	createSub2Item(5, 2, "[a]*log( b )+ c", "");
	setSub2ItemAsChangeable(5, 2, 0);
	createSub2Item(5, 2, " a *log([b])+ c", "");
	setSub2ItemAsChangeable(5, 2, 1);
	createSub2Item(5, 2, " a *log( b )+[c]", "");
	setSub2ItemAsChangeable(5, 2, 2);
	
	createMenusItem("����� ����", "������� �2");
	createSubItem(6, "����������", "������");
	setSubItemAction(6, 0, choose_analog_1);
	createSubItem(6, "�������� ������", "Teracont");
	setSubItemAction(6, 1, choose_thyracont_1);
	createSubItem(6, "������ ��������", "�� �������");
	setSubItemAction(6, 2, choose_log_1);
	createSub2Item(6, 2, "[a]*log( b )+ c", "");
	setSub2ItemAsChangeable(6, 2, 0);
	createSub2Item(6, 2, " a *log([b])+ c", "");
	setSub2ItemAsChangeable(6, 2, 1);
	createSub2Item(6, 2, " a *log( b )+[c]", "");
	setSub2ItemAsChangeable(6, 2, 2);
	
	createMenusItem("����� ������", "���� �1 � �2");
	createSubItem(7, "������ ����", "�1 � �2 �� �1");
	createSub2Item(7, 0, "������� �1 [��]:", "");
	createSub2Item(7, 0, "������� �2 [��]:", "");
	setSub2ItemAsChangeable(7, 0, 0);
	setSub2ItemAsChangeable(7, 0, 1);
	createSubItem(7, "������ �1 �� �1,", "�2 �� �2");
	createSub2Item(7, 1, "������� �1 [��]:", "");
	createSub2Item(7, 1, "������� �2 [��]:", "");
	setSub2ItemAsChangeable(7, 1, 0);
	setSub2ItemAsChangeable(7, 1, 1);
	
	createMenusItem("���������", "���������");
	createSubItem(8, "��������� �����?", "");
	setSubItemAsChangeable(8, 0);
	setSubItemAction(8, 0, hardReset);

	/*** ������� ***/
	addSensors(2);
	setCalcFunc(0, analog_read);
	setCalcFunc(1, thyracont_read);
	
	/*** �������� ������ ***/
	Data_Load();
	
    while (true) 
    {
		keypad_read();
		_delay_ms(30); //������ �� �������� ���������
    }
	clearMenus();
	deleteSensors();
	return 1;
}

