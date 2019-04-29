
#define BMP280_S32_t int32_t

uint32_t p_result;
uint32_t t_result;
uint8_t id;



uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
BMP280_S32_t t_fine;

int16_t Bmpintread(unsigned char adr)
{
	int8_t first;
	int8_t second;
	i2c_start_cond();
	i2c_send_byte(0b11101100);
	i2c_send_byte(adr);
	i2c_stop_cond();
	
	i2c_start_cond();
	i2c_send_byte(0b11101101);
	first = i2c_get_byte(0);
	second = i2c_get_byte(1);
	i2c_stop_cond();
	
	return ((second<<8)+first);
	
}

uint16_t Bmpuintread(unsigned char adr)
{
	uint8_t first;
	uint8_t second;
	i2c_start_cond();
	i2c_send_byte(0b11101100);
	i2c_send_byte(adr);
	i2c_stop_cond();
	
	i2c_start_cond();
	i2c_send_byte(0b11101101);
	first = i2c_get_byte(0);
	second = i2c_get_byte(1);
	i2c_stop_cond();
	
	return ((second<<8)+first);
	
}


uint32_t bmp_read(unsigned char reg) // reads temperature(0xA0) or pressure(0xF7)
{
	uint32_t p_msb;
	uint32_t p_lsb;
	uint32_t p_xlsb;
	uint32_t data;
	i2c_start_cond();
	i2c_send_byte(0b11101100);
	i2c_send_byte(reg);
	i2c_stop_cond();
	
	i2c_start_cond();
	i2c_send_byte(0b11101101);
	p_msb = i2c_get_byte(0);
	p_lsb = i2c_get_byte(0);
	p_xlsb = i2c_get_byte(1);
	i2c_stop_cond();
	
	data = ((p_msb<<16)+(p_lsb<<8)+ p_xlsb)>>4;
	return data;


}



double maketemp(BMP280_S32_t adc_T) // compensation temperature
{ 
	 double var1, var2, T; 
	 var1  = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2); 
	 var2  = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) * 
	  (((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3); 
	 t_fine = (BMP280_S32_t)(var1 + var2); 
	 T  = (var1 + var2) / 5120.0; 
	 return T;
}


double makepressure(BMP280_S32_t adc_P)  // compensation pressure ,do after maketemp
{ 
	double var1, var2, p; 
	var1 = ((double)t_fine/2.0) - 64000.0; 
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0; 
	var2 = var2 + var1 * ((double)dig_P5) * 2.0; 
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0); 
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0; 
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1); 
	if (var1 == 0.0) 
	{ 
		return 0; // avoid exception caused by division by zero 
	} 
	p = 1048576.0 - (double)adc_P; 
	p = (p - (var2 / 4096.0)) * 6250.0 / var1; 
	 var1 = ((double)dig_P9) * p * p / 2147483648.0; 
	 var2 = p * ((double)dig_P8) / 32768.0; 
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0; 
	return p; 
} 


void BMP280_init()
{
	i2c_start_cond();
	i2c_send_byte(0b11101100);
	i2c_send_byte(0xF0);
	i2c_stop_cond();
	
	i2c_start_cond();
	i2c_send_byte(0b11101101);
	id = i2c_get_byte(1);
	i2c_stop_cond();
	
	if (id==0b01011000)
	{
		Uart_tr("BMP OK \n");
		dig_T1 = Bmpuintread(0x88);
		dig_T2 = Bmpintread(0x8A);
		dig_T3 = Bmpintread(0x8C);
		
		dig_P1 = Bmpuintread(0x8E);
		dig_P2 = Bmpintread(0x90);
		dig_P3 = Bmpintread(0x92);
		dig_P1 = Bmpintread(0x94);
		dig_P2 = Bmpintread(0x96);
		dig_P3 = Bmpintread(0x98);
		dig_P1 = Bmpintread(0x9A);
		dig_P2 = Bmpintread(0x9C);
		dig_P3 = Bmpintread(0x9E);
	
		i2c_start_cond();
		i2c_send_byte(0b11101100);
		i2c_send_byte(0xF4);
		i2c_send_byte(0b11111111); // ctrl_meas: normal mode, resolution pressure, resolution temperature 
		i2c_stop_cond();
		
		
	}
	else
	{
		Uart_tr("BMP not OK \n");
	}
}

int32_t bmp_get_t()
{
	t_result = maketemp(bmp_read(0xA0));
	return t_result;
}

int32_t bmp_get_p()
{
	p_result = makepressure(bmp_read(0xF7));
	return p_result;
}