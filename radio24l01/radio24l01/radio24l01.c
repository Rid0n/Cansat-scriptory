/*
 * radio24l01.c
 *
 * Created: 20.12.2018 18:15:18
 *  Author: goout
 */ 
#include <avr/io.h>
#define F_CPU 8000000
#include <util/delay.h>
#include <avr/interrupt.h>

#define BAUD 9600L
#define UBRRL_value (F_CPU/(BAUD*16))-1



#define CONFIG 0x00 //power&stuff
#define PWR_UP 1
#define CRCO 2
#define EN_CRC 3
#define MASK_RX_DR 6
#define MASK_MAX_RT 4
#define EN_AA 0x01 //confirmations
#define EN_RXADDR   0x02 //channels
#define SETUP_AW 0x03 // address length
#define AW 0
#define SETUP_AW_5BYTES_ADDRESS (3 << AW) 
#define RF_CH 0x05 //hz frequency
#define RF_SETUP 0x06 //  exchange settings 
#define PRIM_RX 0
#define R_RX_PL_WID         0x60

#define R_REGISTER  0x00 // + n Прочитать регистр n
#define W_REGISTER  0x20 // + n Записать регистр n
#define RX_ADDR_P1  0x0B
#define RX_PW_P0    0x11 // Размер данных при приёме по каналу 0: от 1 до 32. 0 - канал не используется.
#define RX_PW_P1    0x12
#define ERX_P1 1 // Включает канал 1 приёмника
#define ERX_P0 0 // Включает канал 0 приёмника

#define SPI_DDR DDRB

#define SPI_SS 2
#define SPI_MOSI 3
#define SPI_SCK 5
#define NOP	0xFF
#define STATUS 0x07
#define FIFO_STATUS 0x17
#define FLUSH_TX            0xE1 // Сбросить очередь передатчика
#define FLUSH_RX            0xE2
#define TX_EMPTY 4

#define RADIO_PORT PORTD
#define RADIO_DDR DDRD
#define RADIO_PIN PIND

#define RADIO_CSN 1
#define RADIO_CE 2
#define RADIO_IRQ 3
#define RX_DR   6
#define TX_DS   5
#define MAX_RT  4 
#define RX_P_NO 1 
#define TX_FULL_STATUS 0
#define R_RX_PAYLOAD        0x61 // Принять данные данные из верхнего слота очереди приёмника. 
///////////////////////////pressure.temp.accel

#include "USART.h"
#include "i2c.h"
#include "bmp280.h"

uint8_t adxlRead8(uint8_t address)
{
	uint8_t data;
	i2c_start_cond();
	i2c_send_byte(0xA6); // writing
	i2c_send_byte(address); // write register address
	i2c_stop_cond();

	i2c_start_cond();
	i2c_send_byte(0xA7); // reading
	data = i2c_get_byte(1); // Get LSB result
	i2c_stop_cond();
	return data;
}/*
void get_accel(){
	
	unsigned char xlsb;
	unsigned char xmsb;
	unsigned char ylsb;
	unsigned char ymsb;
	unsigned char zmsb;
	unsigned char zlsb;
	i2c_start_cond();
	i2c_send_byte(0xA6); // writing
	i2c_send_byte(0x32); // write register address
	i2c_stop_cond();
	
	i2c_start_cond();
	i2c_send_byte(0xA7); // reading
	xlsb = i2c_get_byte(0);
	xmsb = i2c_get_byte(0);
	ylsb = i2c_get_byte(0);
	ymsb = i2c_get_byte(0);
	zlsb = i2c_get_byte(0);
	zmsb = i2c_get_byte(1);
	i2c_stop_cond();
	dataX = ((xmsb*256) + xlsb)*31.2 ; //xAcceleration
	dataY = ((ymsb << 8) + ylsb)*31.2 ; //yAcceleration
	dataZ = ((zmsb << 8) + zlsb)*31.2-60 ; //zAcceleration
	
	
}*/
void writebyteAdxl(uint8_t address, uint8_t data)
{
	i2c_start_cond();
	i2c_send_byte(0xA6); // write 0xEE
	i2c_send_byte(address); // Register Address
	i2c_send_byte(data);   // Data
	i2c_stop_cond();
}




/*
void begin_adxl()
{
	if (adxlRead8(0x00) == 0xE5) {
		Uart_tr("ADXL Ready! \n" );
	}
	else {
		Uart_tr("ADXL Not very Ready! \n" );
	}
	writebyteAdxl(POWER_CTL, 0b00101000);
	writebyteAdxl(FIFO_CTL, 0b00000000);
	writebyteAdxl(DATA_FORMAT, 0b00000011);
}*/
/////////////////////////////end of that stuff
// Выбирает активное состояние (высокий уровень) на линии CE
inline void radio_assert_ce() {
	RADIO_PORT |= (1 << RADIO_CE); // Установка высокого уровня на линии CE
}

// Выбирает неактивное состояние (низкий уровень) на линии CE
inline void radio_deassert_ce() {
	RADIO_PORT &= ~(1 << RADIO_CE); // Установка низкого уровня на линии CE
}

// Выбирает активное состояние (низкий уровень) на линии CSN
inline static void csn_assert() {
	RADIO_PORT &= ~(1 << RADIO_CSN); // Установка низкого уровня на линии CSN
}

// Выбирает неактивное состояние (высокий уровень) на линии CSN
inline static void csn_deassert() {
	RADIO_PORT |= (1 << RADIO_CSN); // Установка высокого уровня на линии CSN
}
void spi_init() {
	SPI_DDR |= (1 << SPI_MOSI) | (1 <<  SPI_SCK) | (1 << SPI_SS);
	SPCR = (1 << SPE) | (1 << MSTR); // режим 0, мастер, частота 1/4 от частоты ЦП
}
// Инициализирует порты
void radio_init() {
	RADIO_DDR |= (1 << RADIO_CSN) | (1 << RADIO_CE); // Ножки CSN и CE на выход
	RADIO_DDR &= ~(1 < RADIO_IRQ); // IRQ - на вход
	csn_deassert();
	radio_deassert_ce();
	spi_init();
}


// Инициализация интерфейса

// Передаёт и принимает 1 байт по SPI, возвращает полученное значение
uint8_t spi_send_recv(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1 << SPIF)));
	return SPDR;
}
uint8_t radio_cmd(uint8_t cmd) {
	csn_assert();
	uint8_t status = spi_send_recv(cmd);
	csn_deassert();
	return status;
}
uint8_t radio_write_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
	csn_assert();
	uint8_t status = spi_send_recv(cmd);
	while (count--) {
		spi_send_recv(*(buf++));
	}
	csn_deassert();
	return status;
}

// Читает значение однобайтового регистра reg (от 0 до 31) и возвращает его
uint8_t radio_readreg(uint8_t reg) {
	csn_assert();
	spi_send_recv((reg & 31) | R_REGISTER);
	uint8_t answ = spi_send_recv(0xFF);
	csn_deassert();
	return answ;
}
void on_send_error() {
	// TODO здесь можно описать обработчик неудачной отправки
}

// Вызывается при получении нового пакета по каналу 1 от удалённой стороны.
// buf - буфер с данными, size - длина данных (от 1 до 32)
void on_packet(uint8_t * buf, uint8_t size) {
	
}

// Записывает значение однобайтового регистра reg (от 0 до 31), возвращает регистр статуса
uint8_t radio_writereg(uint8_t reg, uint8_t val) {
	csn_assert();
	uint8_t status = spi_send_recv((reg & 31) | W_REGISTER);
	spi_send_recv(val);
	csn_deassert();
	return status;
}
uint8_t radio_read_rx_payload_width() {
	csn_assert();
	spi_send_recv(R_RX_PL_WID);
	uint8_t answ = spi_send_recv(0xFF);
	csn_deassert();
	return answ;
}
uint8_t radio_writereg_buf(uint8_t reg, uint8_t * buf, uint8_t count) {
	return radio_write_buf((reg & 31) | W_REGISTER, buf, count);
}
uint8_t radio_read_buf(uint8_t cmd, uint8_t * buf, uint8_t count) {
	csn_assert();
	uint8_t status = spi_send_recv(cmd);
	while (count--) {
		*(buf++) = spi_send_recv(0xFF);
	}
	csn_deassert();
	return status;
}
ISR(INT0_vect)
{
	Uart_tr("ISR on!");
	radio_writereg(STATUS, 0b00101110);
	_delay_ms(10);
}
uint8_t radio_start() {
	uint8_t self_addr[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7}; // Собственный адрес
	uint8_t remote_addr[] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2}; // Адрес удалённой стороны
	uint8_t chan = 125; // Номер радио-канала (в диапазоне 0 - 125)

	radio_deassert_ce();
	for(uint8_t cnt = 100;;) {
		radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)); // Выключение питания
		if (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PRIM_RX)))
		break;
		// Если прочитано не то что записано, то значит либо радио-чип ещё инициализируется, либо не работает.
		if (!cnt--)
		return 0; // Если после 100 попыток не удалось записать что нужно, то выходим с ошибкой
		_delay_ms(1);
	} 

	radio_writereg(EN_AA, (0x00)); // выключение автоподтверждения 
	radio_writereg(EN_RXADDR, (1 << ERX_P0) | (1 << ERX_P1)); // включение каналов 0 и 1 - ??
	radio_writereg(SETUP_AW, SETUP_AW_5BYTES_ADDRESS); // выбор длины адреса 5 байт
	
	radio_writereg(RF_CH, chan); // Выбор частотного канала
	radio_writereg(RF_SETUP, 0b00001110); // выбор скорости 2 Мбит/с и мощности 0dBm
	

	radio_writereg_buf(RX_ADDR_P1, &self_addr[0], 5);
	
	radio_writereg(RX_PW_P0, 0);
	radio_writereg(RX_PW_P1, 32);
	
	

	radio_writereg(CONFIG, (1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP)  | (1 << MASK_RX_DR)| (1 << MASK_MAX_RT) | (1 << PRIM_RX)); // Включение питания , power-up, control sum, masking receive&transmit interruptions
	return (radio_readreg(CONFIG) == ((1 << EN_CRC) | (1 << CRCO) | (1 << PWR_UP) | (1 << PRIM_RX) | (1 << MASK_RX_DR)| (1 << MASK_MAX_RT))) ? 1 : 0;
}
uint8_t radio_is_interrupt() {
	return (RADIO_PIN & RADIO_IRQ) ? 0 : 1;
}
void check_radio() {
	if (!radio_is_interrupt()) // Если прерывания нет, то не задерживаемся
	return;
	uint8_t status = radio_cmd(NOP);
	radio_writereg(STATUS, status); // Просто запишем регистр обратно, тем самым сбросив биты прерываний
	
	if (status & ((1 << TX_DS) | (1 << MAX_RT))) { // Завершена передача успехом, или нет,
		if (status & (1 << MAX_RT)) { // Если достигнуто максимальное число попыток
			radio_cmd(FLUSH_TX); // Удалим последний пакет из очереди
			on_send_error(); // Вызовем обработчик
		}
		if (!(radio_readreg(FIFO_STATUS) & (1 << TX_EMPTY))) { // Если в очереди передатчика есть что передавать
			radio_assert_ce(); // Импульс на линии CE приведёт к началу передачи
			_delay_us(15); // Нужно минимум 10мкс, возьмём с запасом
			radio_deassert_ce();
			} else {
			uint8_t conf = radio_readreg(CONFIG);
			radio_writereg(CONFIG, conf | (1 << PRIM_RX)); // Устанавливаем бит PRIM_RX: приём
			radio_assert_ce(); // Высокий уровень на линии CE переводит радио-чип в режим приёма
		}
	}
	uint8_t protect = 4; // В очереди FIFO не должно быть более 3 пакетов. Если больше, значит что-то не так
	while (((status & (7 << RX_P_NO)) != (7 << RX_P_NO)) && protect--) { // Пока в очереди есть принятый пакет
		uint8_t l = radio_read_rx_payload_width(); // Узнаём длину пакета
		if (l > 32) { // Ошибка. Такой пакет нужно сбросить
			radio_cmd(FLUSH_RX);
			} else {
			uint8_t buf[32]; // буфер для принятого пакета
			radio_read_buf(R_RX_PAYLOAD, &buf[0], l); // начитывается пакет
			if ((status & (7 << RX_P_NO)) == (1 << RX_P_NO)) { // если datapipe 1
				on_packet(&buf[0], l); // вызываем обработчик полученного пакета
			}
		}
		status = radio_cmd(NOP);
	}
}
int main(void)
{
	EIMSK|=(1<<6);//разрешаем прерывание по  INT0
	EICRA|=(1<<ISC01)|(0<<ISC00); //прерывание по ниспадающему фронту сигнала на INT0
    while(1)
    {
        //TODO:: Please write your application code 
		radio_init();
		check_radio();
    }
}