#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef struct _V_Fita
{
	float perimetro;
	int	  n_div;
}V_Fita;

typedef struct _V_Encoder
{
	V_Fita Fita;
	float Att_v;
	float Att_Vn;
	int dt;
}V_Encoder ;
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9;

/* USER CODE BEGIN PV */
void Att_Vol(V_Encoder *Encoder)
{
	float n1,n2;
	n1 =  __HAL_TIM_GET_COUNTER(&htim2);
	HAL_Delay(Encoder[0].dt);
	n2 = __HAL_TIM_GET_COUNTER(&htim2);
	Encoder[0].Att_Vn =(n2-n1)/Encoder[0].dt;
	Encoder[0].Att_v = Encoder[0].Att_Vn*(Encoder[0].Fita.perimetro/Encoder[0].Fita.n_div);
}

