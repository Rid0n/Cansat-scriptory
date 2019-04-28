#define SPI_MOSI 17
#define SPI_MISO 18
#define SPI_DDR DDRB
#define SPI_SCK 19
#define SPI_SS 16
void spi_init() {
	SPI_DDR |= (1UL << SPI_MOSI) | (1UL <<  SPI_SCK) | (1UL << SPI_SS);
	SPCR = (1UL << SPE) | (1UL << MSTR); // режим 0, мастер, частота 1/4 от частоты ЦП
}
uint8_t spi_send_recv(uint8_t data) {
	SPDR = data;
	while (!(SPSR & (1UL << SPIF)));
	return SPDR;
}