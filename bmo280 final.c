#include "main.h"
#define Ctrl_Address 0xF4
#define Temp_Address_LSB 0xFB
#define Temp_Address_MSB 0xFA
#define Press_Address_MSB 0xF7
#define Press_Address_LSB 0xF8
#define Bmo_280_Address 0b11101100
#define Status_Address 0xF3
#define Reset_Address 0xE0
#define Adjust_Temp_Address 0x88
#define Forced 0b00000010
#define Sleep  0
#define normal 0b00000011

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct bmo280
{
	uint16_t Ti[3];
	int16_t Pi[9];
	uint8_t adc_temp[3];
	uint8_t adc_press[3];
	uint8_t t_oversampling;
	uint8_t p_oversampling;
	uint32_t att_temp;
	uint32_t att_press;
	int32_t t_fine;
} Vbmo280 ;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */

uint8_t log2 (uint8_t value)
{
	uint8_t n = 0;
	if (value == 1) return 0;
	while(value != 1)
	{
		value = value/2 ;
		n++;
	}
	return n;
}

int16_t Conversion_Int_16(uint8_t *i)
{
	int16_t int_16;
	int_16 = ((int16_t)i[1]<<8) + ((int16_t)i[0]);
	return int_16;
}

int32_t Conversion_Int_32(uint8_t *i)
{
	int32_t int_32;
	int_32 = ((int32_t)i[0]<<12) + ((int32_t)i[1]<<4) + ((int32_t)i[2]>>4);
	return int_32;
}

int IsMeasuring()
{
	uint8_t data,address;
	address = Status_Address;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,&address,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,&data,1,100);
	data = data&0b00010000;
	if (data == 0b00010000) return 1;
	else return 0;
}

void Ini_Bmp280(Vbmo280 *Bmo_280)
{
		uint8_t data[24], address,i;
		if(IsMeasuring()) HAL_Delay(5);
		data[0] = Reset_Address;
		data[1] = 0xB6;
		HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,data,2,100);
		data[0] = Ctrl_Address;
		HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,data,1,100);
		HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,data+1,1,100);
		data[1] = data[1]&11111100;
		HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,data,2,100);
		address = Adjust_Temp_Address;
		HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,&address,1,100);
		HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,data,24,100);
		for (i=0 ; i<=2 ; i++) Bmo_280[0].Ti[i] = Conversion_Int_16(data+2*i);
		for (i=3 ; i<=11 ; i++) Bmo_280[0].Pi[i-3] = Conversion_Int_16(data+2*i);
}

void Mode_Bmo(Vbmo280 *Bmo_280, uint8_t mode)
{
	uint8_t aux[2];
	uint32_t dly;
	aux[0] = Ctrl_Address;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,aux,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,aux+1,1,100);
	aux[1] = aux[1]&(0b11111100 + mode);
	aux[1] = aux[1]|(0b00000000 + mode);
	aux[1] = aux[1]|(0b00000000 + ((log2(Bmo_280[0].t_oversampling)-1)<<6)+ ((log2(Bmo_280[0].p_oversampling)-1)<<4));
	aux[1] = aux[1]&(0b00001111 + ((log2(Bmo_280[0].t_oversampling)-1)<<6)+ ((log2(Bmo_280[0].p_oversampling)-1)<<4));
	aux[0]= Ctrl_Address;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,aux,2,100);
	if(Bmo_280[0].t_oversampling >= Bmo_280[0].p_oversampling ) dly = Bmo_280[0].t_oversampling;
	else dly = Bmo_280[0].p_oversampling;
	if (mode == Forced) HAL_Delay(dly*65);
}

void Adc_Att(Vbmo280 *Bmo_280)
{
	uint8_t address, data[6];
	Mode_Bmo(Bmo_280, Forced);
	address = Press_Address_MSB;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,&address,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,data ,6,100);
	Bmo_280[0].adc_press[0] = data[0];
	Bmo_280[0].adc_press[1] = data[1];
	Bmo_280[0].adc_press[2] = data[2];
	Bmo_280[0].adc_temp[0] = data[3];
	Bmo_280[0].adc_temp[1] = data[4];
	Bmo_280[0].adc_temp[2] = data[5];
}


void Temp_Att(Vbmo280 *Bmo_280)
{
	int32_t var1,var2, adc_T;
	adc_T = Conversion_Int_32(Bmo_280[0].adc_temp);
	var1 = ((((adc_T>>3)-((int32_t)Bmo_280[0].Ti[0]<<1))) * ((int32_t)Bmo_280[0].Ti[1])) >> 11;
	var2 = (((((adc_T>>4)-((int32_t)Bmo_280[0].Ti[0])) * ((adc_T>>4)-((int32_t)Bmo_280[0].Ti[0]))) >> 12) *((int32_t)Bmo_280[0].Ti[2])) >> 14;
	Bmo_280[0].t_fine = var1 + var2;
	Bmo_280[0].att_temp = ((Bmo_280[0].t_fine)* 5 + 128) >> 8;
}

void Press_Att(Vbmo280 *Bmo_280)
{
	int64_t var1, var2, p, adc_P;
	adc_P = Conversion_Int_32(Bmo_280[0].adc_press);
	var1 = ((int64_t)Bmo_280[0].t_fine)-128000;
	var2 = var1 * var1 * (int64_t)Bmo_280[0].Pi[5];
	var2 = var2 + ((var1*(int64_t)Bmo_280[0].Pi[4])<<17);
	var2 = var2 + (((int64_t)Bmo_280[0].Pi[3])<<35);
	var1 = ((var1 * var1 * (int64_t)Bmo_280[0].Pi[2])>>8) + ((var1 * (int64_t)Bmo_280[0].Pi[1])<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)(uint16_t)Bmo_280[0].Pi[0])>>33;
	if (var1 == 0)
	{
		p = 0; // avoid exception caused by division by zero
	}
	else
	{
		p = 1048576-adc_P;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)Bmo_280[0].Pi[8]) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)Bmo_280[0].Pi[7]) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)Bmo_280[0].Pi[6])<<4);
	}
	Bmo_280[0].att_press = (uint32_t)p/256;
}

void Status_Att(Vbmo280 *Bmo_280)
{
	Adc_Att(Bmo_280);
	Temp_Att(Bmo_280);
	Press_Att(Bmo_280);
}
