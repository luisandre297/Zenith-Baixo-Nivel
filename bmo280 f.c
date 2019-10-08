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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
typedef struct bmo280
{
	uint16_t Ti[3];
	int16_t Pi[9];
	uint8_t adc_temp[3];
	uint8_t adc_press[3];
	uint8_t t_oversampling;
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

void Forced_Mode_Bmo(Vbmo280 *Bmo_280)
{
	uint8_t aux[2];
	uint32_t dly;
	aux[0] = Ctrl_Address;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,aux,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,aux+1,1,100);
	aux[1] = aux[1]&0b11111110;
	aux[1] = aux[1]|0b00000010;
	switch (Bmo_280[0].t_oversampling)
	{
		case 1:
				aux[1] = aux[1]&0b11111111;
				aux[1] = aux[1]|0b00000000;
				dly = 150;
				break;

		case 2:
				aux[1] = aux[1]&0b11111111;
				aux[1] = aux[1]|0b00000000;
				dly = 150;
				break;

		case 4:
				aux[1] = aux[1]&0b11111111;
				aux[1] = aux[1]|0b00000000;
				dly = 150;
				break;
		case 8:
				aux[1] = aux[1]&0b11111111;
				aux[1] = aux[1]|0b00000000;
				dly = 150;
				break;
		case 16:
				aux[1] = aux[1]&0b00111111;
				aux[1] = aux[1]|0b00111100;
				dly = 1500;
				break;
		default:
				aux[1] = aux[1]&0b00111111;
				aux[1] = aux[1]|0b00111100;
				dly = 1500;
				break;
	}
	aux[0]= Ctrl_Address;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,aux,2,100);
	HAL_Delay(dly);
}

void Temp_Measuring(Vbmo280 *Bmo_280)
{
	uint8_t address;
	Forced_Mode_Bmo(Bmo_280);
	address = Temp_Address_MSB;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,&address,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,Bmo_280[0].adc_temp ,3,100);
}

void Press_Measuring(Vbmo280 *Bmo_280)
{
	uint8_t address;
	Forced_Mode_Bmo(Bmo_280);
	address = Press_Address_MSB;
	HAL_I2C_Master_Transmit(&hi2c1,Bmo_280_Address,&address,1,100);
	HAL_I2C_Master_Receive(&hi2c1,Bmo_280_Address,Bmo_280[0].adc_press,3,100);
}


void Temp_Att(Vbmo280 *Bmo_280)
{
	int32_t var1,var2, adc_T;
	Temp_Measuring(Bmo_280);
	adc_T = Conversion_Int_32(Bmo_280[0].adc_temp);
	var1 = ((((adc_T>>3)-((int32_t)Bmo_280[0].Ti[0]<<1))) * ((int32_t)Bmo_280[0].Ti[1])) >> 11;
	var2 = (((((adc_T>>4)-((int32_t)Bmo_280[0].Ti[0])) * ((adc_T>>4)-((int32_t)Bmo_280[0].Ti[0]))) >> 12) *((int32_t)Bmo_280[0].Ti[2])) >> 14;
	Bmo_280[0].t_fine = var1 + var2;
	Bmo_280[0].att_temp = ((Bmo_280[0].t_fine)* 5 + 128) >> 8;
}

void Press_Att(Vbmo280 *Bmo_280)
{
	int64_t var1, var2, p, adc_P;
	Temp_Att(Bmo_280);
	Press_Measuring(Bmo_280);
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
