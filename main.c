#include "stm32f0xx.h"
#include "math.h"
#include <stdio.h> 
#include <stdlib.h>
#include <string.h>
#include <base64.h>
#include "cmsis_os.h"

#include "stdint.h"
#include "main.h"
#include "sim800.h"

struct GPGGA_Struct         					// Структура с данными GPS-ГЛОНАСС-модуля
{
	char _time [15];           					// Время
	char Latitude_int [10];     				// Широта целая часть
	char Latitude_float [12];   				// Широта десятичная часть
	char NS;                   					// Север-Юг
	char Longitude_int[10];     				// Долгота целая часть
	char Longitude_float[12];   				// Долгота десятичная часть
	char EW;                   					// Запад-Восток
	char ReceiverMode;         					// Режим работы приемника
	char SatelliteNum [10];     				// Количество спутников в решении
	char Altitude_int [12];     				// Высота целая часть
	char Altitude_float [10];   				// Высота десятичная часть
	char alt_full[20];
};
struct GPGGA_Struct GPSFixData; 				// С этой структурой и будем работать

struct Coords									// Структура координат
{
	double latitude;
	double longitude;
};
struct Coords coordSaveZone[SaveZonePoints_COUNT];
struct Coords coordRoute[RoutePoints_COUNT];


// Карта переменных
// ********************************************************************************************************************************************
const char char_map[10] =																		// Карта
{
	'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'
};


enum List_of_MODES_of_CONTROL				MODE_CONTROL = DISCONNECT_RC;									// Режим управления
u8											total_NUM_Points_SaveZone = 0;						// Общее количество точек зоны безопасности
u8											total_NUM_Points_Route = 0;							// Общее количество точек маршрута
u8											incData_from_SERVER1 = 0;							// Флаг полученной посылки с первого сервера
u8											incData_from_SERVER2 = 0;							// Флаг полученной посылки с NTRIP сервера


u16											s1_buf_cnt = 0;										// Счетчик буфера сервера

i64 										GPS_al  = 251156;									// Высота с GPS модуля после парсинга NMEA
double 										GPS_lat;											// Широта с GPS модуля после парсинга NMEA
double 										GPS_lon;											// Долгота с GPS модуля после парсинга NMEA




u8 											n_lat_float = 0; 									// Количество цифр после запятой в широте с GPS приемника(парсинг NMEA)
u8											n_lon_float = 0; 									// Количество цифр после запятой в долготе с GPS приемника(парсинг NMEA)
u8											n_alt_float = 0;									// Количество цифр после запятой в высоте с GPS приемника(парсинг NMEA)

u8 											turn_in_place = 0;									// Флаг поворота на месте

volatile time_t 							total_cnt;											// Счетчик миллисекунд

volatile time_t								timeout_USART_wait = 0;								// Таймер для ожидания USART1 байт

volatile time_t 							RECEIVE0_timer = 0;									// Таймер для посылок с NTRIP сервера
volatile  time_t 							RECEIVE1_timer = 0;									// Таймер для посылок с сервера

volatile time_t 							get_server1_timer = 0;								// Таймер для обработанных посылок с NTRIP сервера
volatile time_t 							get_server2_timer = 0;								// Таймер для обработанных посылок с сервера

volatile time_t								RC_timer = 0;

u8 											RC_OFF_cnt = 5;

u32 									   	k = 0;                        						// Переменные для счетчиков
                                               
																				
u8                                         	GPS_Usart_Symbol = 0;           					// Переменная для хранения текущего символа USART2
											// Переменная для хранения текущего символа USART1
                                                

u8											SIM1_rssi = 99;										// 0 -115 dBm или меньше
																								// 1 -111 dBm
																								// 2...30 -110... -54 dBm
																								// 31 -52 dBm или больше
																								// 99 не известна или не определена
u16											mnc_operator = 0;									// MNC код оператора
u8 											percent_signal = 0;									// Уровень сигнала в процентах

u8 											isConnectionOpened = 0;								// Флаг открытого соединения NTRIP сервера
u8 											isConnectionOpened2 = 0;							// Флаг открытого соединения сервера
u8											isGprsActive = 0;									// Флаг активного GPRS
u8											NTRIP_CONNECT_OK = 0;								// Флаг успешного подключения к NTRIP серверу
u8											REMOTE_CONNECT_OK = 0; 								// Флаг успешного подключения к серверу

u64 										u64_lat1 = 0;										// uint64 переменная для широты от второго ГНСС модема					
double	 									lat1_multiplier = 0;								// множитель широты

u64 										u64_lon1 = 0;										// uint64 переменная для долготы от второго ГНСС модема
double	 									lon1_multiplier = 0;								// множитель долготы

u8											isGpsDriveAllowed = 1;								// Разрешение на автономное движение
u8											isRcDriveAllowed = 0;								// Разрешение на дистанционное движение
u16 										packet_parcel_count = 0;
u8 											packet_parcel_countX = 0;
double 										coord_GPS1_latitude = 0;							// Итоговая широта первого ГНСС модема
double 										coord_GPS2_latitude = 0;							// Итоговая широта второго ГНСС модема
double 										coord_TARGET_latitude = 0;							// Итоговая широта цели					
double 										coord_GPS1_longitude = 0;							// Итоговая долгота первого ГНСС модема	
double 										coord_GPS2_longitude = 0;							// Итоговая долгота второго ГНСС модема	
double 										coord_TARGET_longitude = 0;							// Итоговая долгота цели	


float 										HEADING = 0;										// Азимут
float 										ANGLE_btw_TARGET_N_DOZER = 0;						// Угол между машиной и целью
float 										DELTA_ANGLE_btw_CURSE_N_HEADING = 0;				// Разниаца между Азимутом и углом к цели
float										SPEED_ENGINE = 0;

double 										fi;
double 										pi = 3.14159265359;

double 										shirota_lau_rad;       // Пересчет координат цели в радианы
double 										dolgota_lau_rad;
double 										shirota_tar_rad;
double 										dolgota_tar_rad;

u16 debug_cnt;
u16 debug_cnt2;
u8 next;
volatile unsigned long ulHighFrequencyTimerTicks;
u32 TIM3_cnt = 0;

u8 speed_level = 0;

u8 left_btn = 0, right_btn = 0;
u8 init = 0;
u8											server_CLOSED = 0;
u8 											RING = 0;

u16											f5_debug = 0;
u16											c5_debug = 0;
volatile time_t 							delta_timeout_USART = 0;
volatile time_t								delta_timeout_RTK = 0;
volatile time_t								delta_timeout_SERVER = 0;
volatile time_t								delta_timeout_RC = 0;
u8											NTRIP_Server_comma_found = 0;
u8 											isGprsDeactNeed = 0;
u8											Current_Route_Point = 0;
u8											Num_Route_Point_DONE = 0;
u8											RC_SEND = 0;
u8											RC_BTN_SEND = 0;
volatile time_t 									    tick_counter;											// Счетчик миллисекунд
u8 											start_bit;
u8											overflow = 0;


u8 sendingMSG[6];
size_t freemem = 0;
#if DEBUG
	char buffer_vTask[500];
	u32 vTask_timer = 0;

#endif

// ********************************************************************************************************************************************

volatile time_t calculated_time = 0;
volatile time_t delta_calc = 0;
volatile time_t number1;
u8 negative_flag;
u8 ies;
u8 char_cnt;
u8 sizeofc = 0;
char mnc_operator_array[6] = {0, 0, 0, 0, 0, 0};
u8 NE_DATA_FLAG = 0;
u8 sendbuffer[11] = {'H', 'E', 'L', 'L', 'O', ' ', 'W', 'O', 'R', 'L', 'D'};


char 			print_buffer					[PRINT_BUF] ;        // Буфер для sprintf

volatile u8 	GPS_DATA						[GPS_BUF]	;  		// Буффер на GPS данные
volatile char	GSM_DATA						[GSM_BUF] 	; 		// USART1 буфер
u8				CAN_BASKET_LAT					[CAN_BUF] 	;		// Корзина для широты в CAN
u8				CAN_BASKET_LON					[CAN_BUF]	;		// Корзина для долготы в CAN
u8				CAN_BASKET_ALT					[CAN_BUF]	;		// Корзина для высоты в CAN
u8				CAN_BASKET_RC_DRIVING			[CAN_BUF]	;		// Корзина для дистанционки в CAN				
u8				CAN_BASKET_RC_DIR				[CAN_BUF]	;		// Корзина для дистанционки в CAN		
u8				CAN_BASKET_RC_SPEED_ENGINE		[CAN_BUF]	;		// Корзина для дистанционки в CAN
u8				CAN_BASKET_RC_START_BUTTON		[CAN_BUF]	;		// Корзина для дистанционки в CAN

char			size_of_RTK_parcel_arr			[4]			;		// Массив для хранения размера принятой NTRIP посылки

char			RECEIVE_SERVER_BUF				[SERVER_BUF];		// Буфер для посылок с сервера

u8 				parsed_buffer					[255] 		;
u8 				zzzbuffer						[20] 		;
u8 				Calling_number					[12] 		;



// Cтруктуры
// ********************************************************************************************************************************************
CanRxMsg 										RxMessage;
GPIO_InitTypeDef 					            GPIO_InitStructure;             // Инициализация структур
DMA_InitTypeDef 					            DMA_InitStructure;
NVIC_InitTypeDef 					            NVIC_InitStructure;
USART_InitTypeDef 				                USART_InitStructure;
TIM_TimeBaseInitTypeDef							TIM_InitStructure;
// ********************************************************************************************************************************************

// Задачи
// ********************************************************************************************************************************************
osThreadId defaultTaskHandle;
osThreadId myLedTaskHandle;
osThreadId myGsmTaskHandle;
osThreadId myGpsTaskHandle;
osThreadId mySysTickTaskHandle;
osThreadId myMobileApplicationHandle;
osThreadId mySendingServerDataHandle;
osThreadId myControlHandle;
osThreadId myUsartHandle;

osMessageQId SendQueueHandle;
osMessageQId UsartQueueHandle;

// ********************************************************************************************************************************************

volatile time_t								timer_CAN_coords = 0;
volatile time_t								delta_timeout_Can_coords = 0;
u8 crc8(u8 *pcBlock, u8 len)    
{
   u8 crc = 0x00;
   u8 i,j;
   u8 b;
   
   for(j = 0; j < len; j++)
   {
       b = *pcBlock++;
   
           i = 8;
           do
               {
                   if((b ^ crc) & 0x01)
                       {
                           crc =((crc ^ 0x18) >> 1) | 0x80;
                       }
                   else
                       {
                           crc >>= 1;
                       }
                   b >>= 1;
               }
           while(--i);
   }                
return crc;
}

double Azimuth_Calculating (double latitude1, double longitude1, double latitude2, double longitude2)
{
	// Формула на сайте https://planetcalc.ru/713/
	double deltaLon = 0, e = 0;
	double a = 6378240, b = 6356860;
	
	double latitude1_rad 	= latitude1 	* DEG_TO_RAD;       // Пересчет координат цели в радианы
	double longitude1_rad 	= longitude1 	* DEG_TO_RAD;
	double latitude2_rad 	= latitude2 	* DEG_TO_RAD;
	double longitude2_rad 	= longitude2 	* DEG_TO_RAD;
	double deltaLon_rad = 0;
	
	double alfa = 0;
	
	if (fabs(longitude2 - longitude1) <= 180) 	deltaLon = longitude2 - longitude1;
	if ((longitude2 - longitude1) < -180) 		deltaLon = 360 + longitude2 - longitude1;
	if ((longitude2 - longitude1) > 180) 		deltaLon = longitude2 - longitude1 - 360;
	
	deltaLon_rad = deltaLon * DEG_TO_RAD;
	e = sqrt(1 - (b * b /a / a));

	alfa = atan2(deltaLon_rad, (log((tan((pi / 4) + (latitude2_rad / 2)) * pow((1 - e * sin(latitude2_rad)) / (1 + e * sin(latitude2_rad)), e / 2))) - log((tan((pi / 4) + (latitude1_rad / 2)) * pow((1 - e * sin(latitude1_rad)) / (1 + e * sin(latitude1_rad)), e / 2)))));
	
	alfa = alfa * RAD_TO_DEG;
	if (alfa < 0) alfa = 360 + alfa;

	return alfa;
}

void blink(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u8 count, u32 duration_ms)
{
	u8 i = 0;
	for (i = 0; i < count; i++)
	{
		GPIO_SetBits(GPIOx, GPIO_Pin);
		osDelay(duration_ms);
		GPIO_ResetBits(GPIOx, GPIO_Pin);
		osDelay(duration_ms);
	}
}

i64 char_to_int (char *cti)													// Функция перевода из строки в число
{
	negative_flag = 0;
	ies = 0;
	number1 = 0;
	char_cnt = 0;

	while (cti[char_cnt] != 0)
	{
		for (ies = 0; ies < 10; ies++)
		{
			if (cti[0] == '-') negative_flag = 1;
			if (cti[char_cnt] == char_map[ies]) 
			{
				number1 = number1 * 10 + ies ;
				break;
			}
		}
		char_cnt++;
	}
	
	if (negative_flag) number1 *= -1;
	
	return number1;
}

u16 UCS2Char(char *ucs2, char *simbol)
{
	u8 s;
	u16 UCS2Code = 0x0000;
	u8 UCSLen = strlen(ucs2);
	//u8 CHRLen = UCSLen / 4;
	u8 nr = 0;
	// If UCS2 string with incorrect length
	//if ((CHRLen * 4) != UCSLen) return 0;
	// process
	for (s = 0; s < UCSLen; s = s + 4) 
	{
		if (ucs2[0+s] >= '0' && ucs2[0+s] <= '9') UCS2Code = (ucs2[0+s] - '0');
		if (ucs2[0+s] >= 'A' && ucs2[0+s] <= 'F') UCS2Code = (ucs2[0+s] + 10 - 'A');
		if (ucs2[1+s] >= '0' && ucs2[1+s] <= '9') UCS2Code = (UCS2Code << 4) + (ucs2[1+s] - '0');
		if (ucs2[1+s] >= 'A' && ucs2[1+s] <= 'F') UCS2Code = (UCS2Code << 4) + (ucs2[1+s] + 10 - 'A');
		if (ucs2[2+s] >= '0' && ucs2[2+s] <= '9') UCS2Code = (UCS2Code << 4) + (ucs2[2+s] - '0');
		if (ucs2[2+s] >= 'A' && ucs2[2+s] <= 'F') UCS2Code = (UCS2Code << 4) + (ucs2[2+s] + 10 - 'A');
		if (ucs2[3+s] >= '0' && ucs2[3+s] <= '9') UCS2Code = (UCS2Code << 4) + (ucs2[3+s] - '0');
		if (ucs2[3+s] >= 'A' && ucs2[3+s] <= 'F') UCS2Code = (UCS2Code << 4) + (ucs2[3+s] + 10 - 'A');

		if ((UCS2Code >= 32) && (UCS2Code <= 126)) simbol[nr] = UCS2Code;
		if ((UCS2Code >= 1040) && (UCS2Code <= 1103)) simbol[nr] = UCS2Code - 848;
		nr++;
	}
  return nr;
}

void clear_RXBuffer(u8 *RX_BUFER, u16 size)           				// Функция очистки буфера
{
    u16 i = 0;
    for (i = 0; i < size; i++) RX_BUFER[i] = 0;
}

void Init_RCC(void)	                                            				// Функция инициализации тактирования
{
	u32 rcc_delay;
    RCC_DeInit( );
    RCC_HSEConfig( RCC_HSE_ON );
    RCC_WaitForHSEStartUp( );
    RCC_PREDIV1Config( RCC_PREDIV1_Div1 );
    RCC_PLLConfig( RCC_PLLSource_HSE, RCC_PLLMul_5 );
    RCC_PLLCmd( ENABLE );
    RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
    RCC_HCLKConfig( RCC_SYSCLK_Div1 );
    RCC_PCLKConfig( RCC_HCLK_Div1	 );
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 | RCC_APB2Periph_SPI1, ENABLE);	
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOA,  ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN | RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM3, ENABLE);
	for (rcc_delay = 0; rcc_delay < 1000; rcc_delay++);
	SystemCoreClockUpdate();
	
}

void Init_TIM(void)
{
	TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Period = 10;
	TIM_InitStructure.TIM_Prescaler = 399;
	TIM_TimeBaseInit(TIM3, &TIM_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel              = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority      = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd           = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}



void Init_USART_2(void)                                         				// Функция инициализацмии USART2
{
	NVIC_InitStructure.NVIC_IRQChannel              = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority      = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd           = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);
    
    //TX UART
    GPIO_InitStructure.GPIO_Pin                     = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode                    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed                   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType                   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd                    = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX UART
    GPIO_InitStructure.GPIO_Pin                     = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode                    = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure the USART2 */
    USART_InitStructure.USART_BaudRate              = 115200;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);	
    USART_OverrunDetectionConfig(USART2, USART_OVRDetection_Disable);
		
    USART_Init(USART2, &USART_InitStructure);
 		
    USART_Cmd(USART2, ENABLE);
		
}

void Init_CAN(void)                                             // Функция инициализации CAN интерфейса
{
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;


    GPIO_PinAFConfig( GPIOA, GPIO_PinSource11, GPIO_AF_4 );
    GPIO_PinAFConfig( GPIOA, GPIO_PinSource12, GPIO_AF_4 );


    GPIO_InitStructure.GPIO_Pin                         = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode 	                    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed                       = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType                       = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd 	                    = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CAN rikystir init */
    CAN_DeInit(CAN);
    CAN_StructInit(&CAN_InitStructure);
    /* CAN cell init */
    CAN_InitStructure.CAN_TTCM 		                    = DISABLE;
    CAN_InitStructure.CAN_ABOM 		                    = DISABLE;
    CAN_InitStructure.CAN_AWUM 		                    = DISABLE;
    CAN_InitStructure.CAN_NART 		                    = DISABLE;
    CAN_InitStructure.CAN_RFLM 		                    = DISABLE;
    CAN_InitStructure.CAN_TXFP 		                    = DISABLE;
    CAN_InitStructure.CAN_Mode 		                    = CAN_Mode_Normal;

    CAN_InitStructure.CAN_SJW 		                    = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1 		                    = CAN_BS1_13tq; 
    CAN_InitStructure.CAN_BS2 		                    = CAN_BS2_2tq; 
    CAN_InitStructure.CAN_Prescaler                     = 10; 
    CAN_Init(CAN, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber            = 1;
    CAN_FilterInitStructure.CAN_FilterMode              = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale             = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdLow             = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdHigh            =  0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow         = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh        = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment    = CAN_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation        = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
    
    CAN_ITConfig(CAN, CAN_IT_FMP0, ENABLE); 

    NVIC_InitStructure.NVIC_IRQChannel                  = CEC_CAN_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority          = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd               = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void CEC_CAN_IRQHandler(void)
{
	u8 CAN_DATA[8];
	QUEUE_t msg;
	BaseType_t xHigherPriorityTaskWoken;
	
	msg.data[0] = '$';
	msg.data[1] = PACK_ID_dozerParams;

	
	if (CAN_GetITStatus(CAN, CAN_IT_FMP0) == SET) 
	{                   

		RxMessage.DLC 		= 	0x00;
		RxMessage.ExtId 	= 	0x00;
		RxMessage.FMI 		= 	0x00;
		RxMessage.IDE 		= 	0x00;
		RxMessage.RTR 		= 	0x00;
		RxMessage.StdId 	= 	0x00;
		RxMessage.Data [0] 	= 	0x00;
		RxMessage.Data [1] 	= 	0x00;
		RxMessage.Data [2] 	= 	0x00;
		RxMessage.Data [3] 	= 	0x00;
		RxMessage.Data [4] 	= 	0x00;
		RxMessage.Data [5] 	= 	0x00;
		RxMessage.Data [6] 	= 	0x00;
		RxMessage.Data [7] 	= 	0x00;
		
		CAN_Receive(CAN, CAN_FIFO0, &RxMessage);
		
		CAN_DATA [0] 	= 	RxMessage.Data [0];
		CAN_DATA [1] 	= 	RxMessage.Data [1];
		CAN_DATA [2] 	= 	RxMessage.Data [2];
		CAN_DATA [3] 	= 	RxMessage.Data [3];
		CAN_DATA [4] 	= 	RxMessage.Data [4];
		CAN_DATA [5] 	= 	RxMessage.Data [5];
		CAN_DATA [6] 	= 	RxMessage.Data [6];
		CAN_DATA [7] 	= 	RxMessage.Data [7];
		
			
		for(u8 i = 0; i < 8; i++)
		{
			msg.data[i + 4] = CAN_DATA[i];
		}
	
		if (RxMessage.StdId == CAN_LAT1_EXT_ID)
		{
			timer_CAN_coords = tick_counter;
			coord_GPS2_latitude = 0;
			u64_lat1 = 0;
			lat1_multiplier = 100000000;
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lat1 = u64_lat1 + ((u64)CAN_DATA[i] << (i * 8));
			}
						
			coord_GPS2_latitude = (double)u64_lat1 / lat1_multiplier - 90;			
		}
		
		if (RxMessage.StdId == CAN_LON1_EXT_ID)
		{	
			timer_CAN_coords = tick_counter;			
			coord_GPS2_longitude = 0;
			u64_lon1 = 0;
			lon1_multiplier = 100000000;
			
			for(u8 i = 0; i <= 4; i++)
			{
				u64_lon1 = u64_lon1 + ((u64)CAN_DATA[i] << (i * 8));
			}
			
			coord_GPS2_longitude = (double)u64_lon1 / lon1_multiplier - 180;	
		}
		
		switch(RxMessage.ExtId)
		{
			case CAN_Tx_DM1_Angles:
										msg.data[2] = 0x11;
										msg.data[3] = 0x05;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_DM1_Heights:
										msg.data[2] = 0x12;
										msg.data[3] = 0x05;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_201_ID:
										msg.data[2] = 0x01;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_205_ID:
										msg.data[2] = 0x05;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_207_ID:
										msg.data[2] = 0x07;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_209_ID:
										msg.data[2] = 0x09;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_210_ID:
										msg.data[2] = 0x10;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_215_ID:
										msg.data[2] = 0x15;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_216_ID:
										msg.data[2] = 0x16;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_221_ID:
										msg.data[2] = 0x21;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_2BD_ID:
										msg.data[2] = 0xBD;
										msg.data[3] = 0x02;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_303_ID:
										msg.data[2] = 0x03;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_308_ID:
										msg.data[2] = 0x08;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_311_ID:
										msg.data[2] = 0x11;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_313_ID:
										msg.data[2] = 0x13;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_Tx_Modem_314_ID:
										msg.data[2] = 0x14;
										msg.data[3] = 0x03;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_F004_ID:
										msg.data[2] = 0x04;
										msg.data[3] = 0xF0;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEEE_ID:
										msg.data[2] = 0xEE;
										msg.data[3] = 0xFE;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEEF_ID:
										msg.data[2] = 0xEF;
										msg.data[3] = 0xFE;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FEFC_ID:
										msg.data[2] = 0xFC;
										msg.data[3] = 0xFE;
										msg.data[12] = crc8(&msg.data[1], 11);
			
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
			
			case CAN_EE_FFA0_ID:
										msg.data[2] = 0xA0;
										msg.data[3] = 0xFF;
										msg.data[12] = crc8(&msg.data[1], 11);
							
			xQueueSendFromISR(SendQueueHandle, &msg.data, &xHigherPriorityTaskWoken);
			break;
		}
	}
}

void TIM3_IRQHandler (void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		ulHighFrequencyTimerTicks++;
	}
}
u16 frame_err_cnt = 0;
u8 int_cnt = 0;
u16 fully = 0;


void USART2_IRQHandler(void)                                    				// Обработчик прерываний USART, чтение данных с GPS модуля
{	
    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
    {		
        GPS_Usart_Symbol = USART_ReceiveData(USART2);           				// Получение символов из USART
		if (GPS_Usart_Symbol == '1') next = 1;
		GPS_DATA[k++] = GPS_Usart_Symbol;
    }
}



void CAN_Send(u8 *data, u32 can_id)                                   // Функция отправки CAN сообщений
{
    CanTxMsg TxMessage;
	if (can_id <= 0x7FF)
	{
		TxMessage.StdId = can_id;
		TxMessage.ExtId = CAN_EXT_ID;
		TxMessage.IDE = CAN_Id_Standard;
	}
	else
	{
		TxMessage.StdId = CAN_STD_ID;		
		TxMessage.ExtId = can_id;
		TxMessage.IDE = CAN_Id_Extended;
	}
    				
	TxMessage.RTR = CAN_RTR_Data;					
    TxMessage.DLC = 8;								

    TxMessage.Data[0] = data[0];			
    TxMessage.Data[1] = data[1];
    TxMessage.Data[2] = data[2];
    TxMessage.Data[3] = data[3];
    TxMessage.Data[4] = data[4];		
    TxMessage.Data[5] = data[5];
    TxMessage.Data[6] = data[6];
    TxMessage.Data[7] = data[7];

    CAN_Transmit(CAN, &TxMessage);
	
	osDelay(1);
}

void Init_GPS(void)                                             				// Функция инициализации GPS приемника
{
    GPIO_InitStructure.GPIO_Pin 	                = GPS_PPS_PIN;
    GPIO_InitStructure.GPIO_Mode 	                = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed                   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType                   = GPIO_OType_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    Init_USART_2();
}

void Parse_To_GPS_Points(u8 *ArrayData)
{
	enum List_of_Points_Type _type;
	u8 i = 0;
	u8 index = ArrayData[3] % RoutePoints_COUNT;
	double latitude = 0;
	double longitude = 0;

	u64 coor_lat[5];
	u64 coor_lon[5];

	coor_lat[0] = ArrayData[4];
	coor_lat[1] = ArrayData[5];
	coor_lat[2] = ArrayData[6];
	coor_lat[3] = ArrayData[7];
	coor_lat[4] = ArrayData[8];
	
	coor_lon[0] = ArrayData[9];
	coor_lon[1] = ArrayData[10];
	coor_lon[2] = ArrayData[11];
	coor_lon[3] = ArrayData[12];
	coor_lon[4] = ArrayData[13];
	
	
	if (ArrayData[2] == PARCEL_ID_SaveCoord) _type = SAVE_ZONE;
	if (ArrayData[2] == PARCEL_ID_PointCoord) _type = ROUTE;
	
	if (_type == SAVE_ZONE)
	{
		for(i = 0; i <= 4; i++)
		{
			latitude = latitude + (coor_lat[i] << (i * 8));
			longitude = longitude + (coor_lon[i] << (i * 8));
		}
		coordSaveZone[index].latitude = latitude / 100000000 - 90;
		coordSaveZone[index].longitude = longitude / 100000000 - 180;
	}	
	
	if (_type == ROUTE)
	{
		for(i = 0; i <= 4; i++)
		{
			latitude = latitude + (coor_lat[i] << (i * 8));
			longitude = longitude + (coor_lon[i] << (i * 8));
		}
		coordRoute[index].latitude = latitude / 100000000 - 90;
		coordRoute[index].longitude = longitude / 100000000 - 180;
		
	}
	
}

void Parse_To_Total_Num_Coords(u8 *ArrayData)
{
	u8 _type = 0;
	
	if (ArrayData[2] == PARCEL_ID_SaveCoord) _type = SAVE_ZONE;
	if (ArrayData[2] == PARCEL_ID_PointCoord) _type = ROUTE;
	if (_type == SAVE_ZONE)
	{
		total_NUM_Points_SaveZone = ArrayData[3];
	}
	
	if (_type == ROUTE)
	{
		total_NUM_Points_Route = ArrayData[3];
	}
}

void Parse_To_Mode(u8 *ArrayData)
{
	if (ArrayData[2] == PARCEL_ID_Mode)
	{
		MODE_CONTROL = (enum List_of_MODES_of_CONTROL)ArrayData[3];
		
		if (MODE_CONTROL == MANUAL_CONTROL_ON)
		{
			RC_timer = tick_counter;
			delta_timeout_RC = 0;
		}
	}
}

void Parse_To_RC_Command(u8 *ArrayData)
{
	u8 left_y1 = 0;
	u8 left_y2 = 0;
	u8 left_x1 = 0;
	u8 left_x2 = 0;
	u8 right_y1 = 0;
	u8 right_y2 = 0;
	u8 right_x1 = 0;
	u8 right_x2 = 0;
	
	
		
	switch (ArrayData[4]) 
	{
		case 0: left_btn = 0x00;	
				right_btn = 0x00;    
		break;
		case 1: left_btn = 0x08;	
				right_btn = 0x00;    
		break;
		case 2: left_btn = 0x10;	
				right_btn = 0x00; 	
		break;
		case 4: left_btn = 0x00;	
				right_btn = 0x08; 	
		break;
		case 8: left_btn = 0x00;	
				right_btn = 0x10; 	
		break;
		default: left_btn = 0x00; 	
				right_btn = 0x00;    
		break;
	}
	
	if (ArrayData[6] == 100) 
	{
		left_y1 = 0;
		left_y2 = 0;
	}
	else if(ArrayData[6] > 100) 
	{
		left_y1 = ArrayData[5] - 100;
		left_y2 = 255 - left_y1;
	}
	else 
	{
		left_y2 = 100 - ArrayData[5];
		left_y1 = 255 - left_y2;
	}
/************************************************************/
	if (ArrayData[5] == 100) 
	{
		left_x1 = 0;
		left_x2 = 0;
	}
	else if(ArrayData[5] > 100) 
	{
		left_x1 = ArrayData[5] - 100;
		left_x2 = 255 - left_x1;
	}
	else 
	{
		left_x2 = 100 - ArrayData[5];
		left_x1 = 255 - left_x2;
	}
/************************************************************/
	if (ArrayData[8] == 100) 
	{
		right_y1 = 0;
		right_y2 = 0;
	}
	else if(ArrayData[8] > 100) 
	{
		right_y1 = ArrayData[8] - 100;
		right_y2 = 255 - right_y1;
	}
	else 
	{
		right_y2 = 100 - ArrayData[8];
		right_y1 = 255 - right_y2;
	}
/************************************************************/
	if (ArrayData[7] == 100) 
	{
		right_x1 = 0;
		right_x2 = 0;
	}
	else if(ArrayData[7] > 100) 
	{
		right_x1 = ArrayData[7] - 100;
		right_x2 = 255 - right_x1;
	}
	else 
	{
		right_x2 = 100 - ArrayData[7];
		right_x1 = 255 - right_x2;
	}
/************************************************************/
   
	speed_level = ArrayData[9];                    
	if(speed_level >= 255) speed_level = 255;
	
	CAN_BASKET_RC_DRIVING[0] = left_y1;    	
	CAN_BASKET_RC_DRIVING[1] = left_x1;    	
	CAN_BASKET_RC_DRIVING[2] = left_y2;    	
	CAN_BASKET_RC_DRIVING[3] = left_x2;    	                    
	CAN_BASKET_RC_DRIVING[4] = 0xFF;        	
	CAN_BASKET_RC_DRIVING[5] = left_btn;    	
	CAN_BASKET_RC_DRIVING[6] = 0x00;        	
	CAN_BASKET_RC_DRIVING[7] = 0xFF;        	
				
	CAN_BASKET_RC_DIR[0] = right_y1;    
	CAN_BASKET_RC_DIR[1] = right_x1;    
	CAN_BASKET_RC_DIR[2] = right_y2;    
	CAN_BASKET_RC_DIR[3] = right_x2;    
	CAN_BASKET_RC_DIR[4] = 0xFF;        
	CAN_BASKET_RC_DIR[5] = right_btn;
	CAN_BASKET_RC_DIR[6] = 0x00;        
	CAN_BASKET_RC_DIR[7] = 0xFF;        
	
	CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF; 			
	CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF; 			
	CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;     		
	CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;	
	CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;        	
	CAN_BASKET_RC_SPEED_ENGINE[5] = 0xFF; 			
	CAN_BASKET_RC_SPEED_ENGINE[6] = 0x01;        	
	CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;        	
	
	CAN_BASKET_RC_START_BUTTON[0] = 0x00;
	CAN_BASKET_RC_START_BUTTON[1] = 0x00;
	CAN_BASKET_RC_START_BUTTON[2] = 0x00;
	CAN_BASKET_RC_START_BUTTON[3] = 0x00;
	CAN_BASKET_RC_START_BUTTON[4] = 0x00;
	CAN_BASKET_RC_START_BUTTON[5] = 0x00;
	CAN_BASKET_RC_START_BUTTON[6] = 0x00;
	
	if((ArrayData[10] == 0xF8) & (start_bit == 0)) 
	{
		start_bit = 0x01;
		CAN_BASKET_RC_START_BUTTON[7] = 0xF1;
		RC_BTN_SEND = 1;
	}                        
	if((ArrayData[10] == 0x00) & (start_bit == 1)) 
	{
		start_bit = 0x00;
		CAN_BASKET_RC_START_BUTTON[7] = 0xF0;
		RC_BTN_SEND = 1;
	}
	
	RC_timer = tick_counter;
	delta_timeout_RC = 0;
	
	RC_SEND = 1;
}

void Parse_To_Error(u8* ArrayData)
{
}

void Parce_To_CAN_Command(u8* ArrayData)
{
	u32 ext_id = 0;
	u8 id0 = ArrayData[2];
	u8 id1 = ArrayData[3];
	u8 localCAN_BASKET[8];
	
	ext_id = CAN_EXT_ID_MASK | id0 << 8 | id1 << 16;
	localCAN_BASKET[0] = ArrayData[4];
	localCAN_BASKET[1] = ArrayData[5];
	localCAN_BASKET[2] = ArrayData[6];
	localCAN_BASKET[3] = ArrayData[7];
	localCAN_BASKET[4] = ArrayData[8];
	localCAN_BASKET[5] = ArrayData[9];
	localCAN_BASKET[6] = ArrayData[10];
	localCAN_BASKET[7] = ArrayData[11];
	
	CAN_Send(localCAN_BASKET, ext_id);
	
}

void GET_CONFIRM_MSG (u8 *ArrayData, u8 *OutputArray)
{
	OutputArray[0] = ArrayData[0];
	OutputArray[1] = 0xFE;
	OutputArray[2] = ArrayData[1];
	OutputArray[3] = ArrayData[2];
	OutputArray[4] = ArrayData[3];
	OutputArray[5] = crc8((u8 *)&OutputArray[1], 4);
}



u16 g5_debug = 0;
u16 a5_debug = 0;
u16 b5_debug = 0;

void getting_data(void)
{
	QUEUE_t msg;

	u16 size = 0;
	u8 confirm = 0;
	
	clear_RXBuffer(parsed_buffer, 255);
	
	while(parcel_count || search_RECEIVE1 || incData_from_SERVER2)
	{
		if (tick_counter - RECEIVE1_timer > 5000) break;
		if (parcel_count == 0) 
		{
			continue;
		}
		g5_debug++;
		incData_from_SERVER2 = 0;

		packet_parcel_countX = GET_DATA_PARCEL(size, RECEIVE_SERVER_BUF, parsed_buffer);
		
		if (packet_parcel_countX) packet_parcel_count += packet_parcel_countX;

		confirm = 0;
		if (parsed_buffer[0] == '$')
		{
			a5_debug++;
			switch(parsed_buffer[1])
			{
				case (u8)PACK_ID_tNum: 			Parse_To_Total_Num_Coords(parsed_buffer);
												size = PACK_LEN_tNum + 1;
												confirm = 1;
				break;
				case (u8)PACK_ID_Coords: 		Parse_To_GPS_Points(parsed_buffer);
												size = PACK_LEN_Coords + 1;
												confirm = 1;
				break;	
				case (u8)PACK_ID_Mode: 			Parse_To_Mode(parsed_buffer);
												size = PACK_LEN_Mode + 1;
												confirm = 1;
				break;
				case (u8)PACK_ID_RC: 			confirm = 0;
												if (parsed_buffer[2] == PARCEL_ID_RC) 	
													Parse_To_RC_Command(parsed_buffer);
												else if (parsed_buffer[2] == 0 && parsed_buffer[3] == 0)							
													Parce_To_CAN_Command(parsed_buffer);
												else
												{
													Parce_To_CAN_Command(parsed_buffer);
													confirm = 1;
												}
												size = PACK_LEN_RC + 1;
												
				break;
				case (u8)PACK_ID_Err: 			Parse_To_Error(parsed_buffer);
												size = PACK_LEN_Err + 1;
												confirm = 1;
				break;
				
				default : b5_debug++;
			}
			
			debug_cnt++;
			if (confirm) 
			{
				GET_CONFIRM_MSG(parsed_buffer, msg.data);
				xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
				c5_debug++;
				clear_RXBuffer(parsed_buffer, 255);
			}
		}
		else
		{
			__nop();
		}

			
		parcel_count--;
	}

	clear_RXBuffer((u8 *)RECEIVE_SERVER_BUF, SERVER_BUF);
	s1_buf_cnt = 0;
	packet_parcel_count = 0;

	
	
	
	delta_calc = tick_counter - calculated_time;
}

void gsm_at_parse (char *result, volatile char *GSM_TEMP, char *left_mask, char *right_mask) 	// *GSM_TEMP = наша команда
{	
	u8 i = 0, j = 0, del = 0;								                            	// *left_mask = символы перед нужной нам частью строки
	char *startstring; 											            	// *right_mask = символы после нужной нам части строки
	char *endstring;
	u8 sofleft = 0;
	u16 sofstart = 0;
	u16 sofend = 0;
	sofleft = strlen(left_mask);
    startstring = strstr((const char *)GSM_TEMP, left_mask); // ищем left_mask
    startstring = startstring + sofleft; 

	endstring = strstr(startstring, right_mask);
	sofstart = strlen(startstring);
	sofend = strlen(right_mask);
	for (; i < sofstart; i++)
	{
		if (startstring[i] == endstring[j])
		{			
			if (j < sofend) j++;
			else 
			{
				for(del = i; del >= i - j; del--)
				{
					result[del] = 0;
				}
				break;
			}
		}
		
		else 
		{
			j = 0;
			if (startstring[i] == endstring[j]) j++;
			
		}
		result[i] = startstring[i];
	}
}



i32 	lat_int;
i32 	lon_int;
u32 	lat_float;
u32 	lon_float;
double* address;
void Parse_Nmea_Messages (void)													// Парсинг NMEA сообщений от GPS модуля
{

	u32    	j = 0; 
	u32    	g = 0;
	u32    	gh = 0;
	u8 		statComma = 0;
	u8 		float_flag = 0;
	u32		mul = 1;
	
	for (j = 0; j <= k; j++)
	{
		if (GPS_DATA[j] == 0x0A) 												// Ищем конец сообщения
		{

			statComma = 0;														// Обнуляем счетчик запятых
			while (GPSFixData.Latitude_float[n_lat_float] != 0 && GPSFixData.Latitude_float[n_lat_float] != 0x2E) 
			{
				n_lat_float++;	// Считаем количество знаков после запятой у широты
				mul *= 10;
			}
			while (GPSFixData.Longitude_float[n_lon_float] != 0 && GPSFixData.Longitude_float[n_lon_float] != 0x2E) n_lon_float++;	// Считаем количество знаков после запятой у долготы
			while (GPSFixData.Altitude_float[n_alt_float] != 0 && GPSFixData.Altitude_float[n_alt_float] != 0x2E) n_alt_float++;	// Считаем количество знаков после запятой у высоты
			lat_int =  char_to_int(GPSFixData.Latitude_int);					// Переводим в число целую часть широты
			lat_float = char_to_int(GPSFixData.Latitude_float);					// Переводим в число десятичную часть широты
			lon_int =  char_to_int(GPSFixData.Longitude_int);					// Переводим в число целую часть долготы
			lon_float = char_to_int(GPSFixData.Longitude_float);				// Переводим в число десятичну часть долготы
			GPS_al  = char_to_int(GPSFixData.alt_full);							// Получаем конечное INT значение высоты
			if (GPS_al < 0) GPS_al = 0;
			if (GPS_al > 9999999) GPS_al = 0;
			GPS_lat = (double)((u32)lat_int / 100) + (((double)((u32)lat_int % 100) + (double)lat_float / mul) / 60);			// Получаем конечное INT значение широты
			GPS_lon = (double)((u32)lon_int / 100) + (((double)((u32)lon_int % 100) + (double)lon_float / mul) / 60); 			// Получаем конечное INT значение долготы
		}
		
		if (GPS_DATA[j] != 0x2C)												// Ищим запятую
		{
			switch(statComma)													// По количеству запятых определяем нужный параметр в сообщении
			{
				case 1: GPSFixData._time[g++] = GPS_DATA[j]; break;				// Записывам время

				case 2:  														// Записываем широту
					  if (float_flag == 0) GPSFixData.Latitude_int[g++] = GPS_DATA[j]; 
					  else GPSFixData.Latitude_float[g++] = GPS_DATA[j];
					  
					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 3: GPSFixData.NS = GPS_DATA[j]; break;						// Записываем Север/Юг

				case 4: 														// Записываем долготу
					  if (float_flag == 0) GPSFixData.Longitude_int[g++] = GPS_DATA[j]; 
					  else GPSFixData.Longitude_float[g++] = GPS_DATA[j];
					  
					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;

				case 5: GPSFixData.EW = GPS_DATA[j]; break;						// Записываем Запад/Восток

				case 6: GPSFixData.ReceiverMode = GPS_DATA[j]; break;			// Записываем код качества сигнала

				case 7: GPSFixData.SatelliteNum[g++] = GPS_DATA[j]; break;		// Записываем количество видимых спутников

				case 9: 														// Записываем высоту
					GPSFixData.alt_full[gh++] = GPS_DATA[j];

					  if (GPS_DATA[j] == 0x2E) { float_flag = 1; g = 0; }
				break;
			}
		}
		else
		{
			statComma++;														// Счетчик запятых
			float_flag = 0;
			g = 0;
			gh = 0;
		}
	}			
}


time_t time_to_send_rc;
time_t Dtime_to_send_rc;
void run_RC_drive(void)
{	
	 
	if (RC_SEND)
	{
		Dtime_to_send_rc = tick_counter - time_to_send_rc;
		RC_SEND = 0;
		
		CAN_Send(CAN_BASKET_RC_DRIVING, 		CAN_Remote_Control_A_ID);
		CAN_Send(CAN_BASKET_RC_DIR, 			CAN_Remote_Control_B_ID);
		CAN_Send(CAN_BASKET_RC_SPEED_ENGINE, 	CAN_Remote_Control_C_ID);
		if (RC_BTN_SEND)
		{
			CAN_Send(CAN_BASKET_RC_START_BUTTON, 	CAN_Remote_Control_sButton_ID);
			RC_BTN_SEND = 0;
		}
		
		time_to_send_rc = tick_counter;
	}
}

void run_gps_drive(void)  // Функция работы автоматического движения по gps координатам
{
	enum List_of_MOVING _curse;
	QUEUE_t msg;
	coord_TARGET_latitude = coordRoute[Current_Route_Point].latitude;
	coord_TARGET_longitude = coordRoute[Current_Route_Point].longitude;
		
	if (coord_TARGET_latitude != 0 && coord_TARGET_longitude !=0 &&	coord_GPS2_latitude != 0 && coord_GPS2_longitude != 0)
	{
		ANGLE_btw_TARGET_N_DOZER = Azimuth_Calculating(coord_GPS1_latitude, coord_GPS1_longitude, coord_TARGET_latitude, coord_TARGET_longitude);
		// Задание допустимого радиуса цели
		if (((	pow((coord_GPS1_latitude - coord_TARGET_latitude), 2) + pow((coord_GPS1_longitude - coord_TARGET_longitude), 2)) > 0.0000000000000032) && !next)
		{
			if ((ANGLE_btw_TARGET_N_DOZER < 180) && (HEADING < 180))                 // Вычисление направления поворота движения
			{
				DELTA_ANGLE_btw_CURSE_N_HEADING = fabs(ANGLE_btw_TARGET_N_DOZER - HEADING);
				
				if ((ANGLE_btw_TARGET_N_DOZER - HEADING) < -5)			_curse = LEFT_DRIVE;
				else if ((ANGLE_btw_TARGET_N_DOZER - HEADING) > 5)		_curse = RIGHT_DRIVE;
				else													_curse = FORWARD_DRIVE;
			}

			if ((ANGLE_btw_TARGET_N_DOZER < 180) && (HEADING > 180))
			{
				if ((360 - HEADING + ANGLE_btw_TARGET_N_DOZER) < 180)
				{
																		_curse = RIGHT_DRIVE;
					DELTA_ANGLE_btw_CURSE_N_HEADING = 360 - HEADING + ANGLE_btw_TARGET_N_DOZER;;
				}
				else
				{
																		_curse = LEFT_DRIVE;
					DELTA_ANGLE_btw_CURSE_N_HEADING = HEADING - ANGLE_btw_TARGET_N_DOZER;
				}
			}

			if ((ANGLE_btw_TARGET_N_DOZER > 180) && (HEADING < 180))
			{
				
				if ((360 - ANGLE_btw_TARGET_N_DOZER + HEADING) < 180)    
				{
																		_curse = LEFT_DRIVE;
					DELTA_ANGLE_btw_CURSE_N_HEADING = 360 - ANGLE_btw_TARGET_N_DOZER + HEADING;
				}
				else
				{
																		_curse = RIGHT_DRIVE;
					DELTA_ANGLE_btw_CURSE_N_HEADING = ANGLE_btw_TARGET_N_DOZER - HEADING;
				} 
			
			}

			if ((ANGLE_btw_TARGET_N_DOZER > 180) && (HEADING > 180))
			{
				DELTA_ANGLE_btw_CURSE_N_HEADING = fabs(ANGLE_btw_TARGET_N_DOZER - HEADING);
				if ((ANGLE_btw_TARGET_N_DOZER - HEADING) > 5)			_curse = RIGHT_DRIVE;
				else if ((ANGLE_btw_TARGET_N_DOZER - HEADING) < -5)		_curse = LEFT_DRIVE;
				else                                          			_curse = FORWARD_DRIVE;
			}
		}
		else if (Num_Route_Point_DONE < total_NUM_Points_Route)
		{
			next = 0;
			msg.data[0] = '$';
			msg.data[1] = 0x13;
			msg.data[2] = Num_Route_Point_DONE;
			msg.data[3] = crc8(&msg.data[1], 2);
			
			xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
			
			coordRoute[Current_Route_Point].latitude = 0;
			coordRoute[Current_Route_Point].longitude = 0;
			Current_Route_Point++;
			Num_Route_Point_DONE++;
			
			if (Current_Route_Point == RoutePoints_COUNT) Current_Route_Point = 0;
			if (Num_Route_Point_DONE == total_NUM_Points_Route) 
			{
				MODE_CONTROL = STOP_AUTO;
			}
		}
		else
			_curse = NO_MOVEMENT;
		
		switch (_curse)                                       // Вычисление значений ШИМ для гусениц
		{
		  case NO_MOVEMENT:
						CAN_BASKET_RC_DRIVING[0] = STOP_DRIVE;
						CAN_BASKET_RC_DRIVING[1] = STOP_DRIVE;
						CAN_BASKET_RC_DRIVING[2] = STOP_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = STOP_DRIVE;
						turn_in_place = 0;
		  break;

		  case LEFT_DRIVE:
					if (DELTA_ANGLE_btw_CURSE_N_HEADING < 140 && !turn_in_place)
					{
						// Left steer
						CAN_BASKET_RC_DRIVING[0] = SLOW_DRIVE;                  
						CAN_BASKET_RC_DRIVING[2] = SLOW_DRIVE_CRC;
						// Forward command
						CAN_BASKET_RC_DRIVING[1] = SLOW_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = SLOW_DRIVE_CRC;
					}
					else 
					{
						turn_in_place = 1;
						// Left steer
						CAN_BASKET_RC_DRIVING[0] = FULL_DRIVE;
						CAN_BASKET_RC_DRIVING[2] = FULL_DRIVE_CRC;
						// Forward command
						CAN_BASKET_RC_DRIVING[1] = SLOW_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = SLOW_DRIVE_CRC;
					}
		  break;

		  case RIGHT_DRIVE:
					if (DELTA_ANGLE_btw_CURSE_N_HEADING < 140 && !turn_in_place)
					{
						// Left steer
						CAN_BASKET_RC_DRIVING[2] = SLOW_DRIVE;                  
						CAN_BASKET_RC_DRIVING[0] = SLOW_DRIVE_CRC;
						// Forward command
						CAN_BASKET_RC_DRIVING[1] = SLOW_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = SLOW_DRIVE_CRC;
					}
					else 
					{
						turn_in_place = 1;
						// Left steer
						CAN_BASKET_RC_DRIVING[2] = FULL_DRIVE;
						CAN_BASKET_RC_DRIVING[0] = FULL_DRIVE_CRC;
						// Forward command
						CAN_BASKET_RC_DRIVING[1] = SLOW_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = SLOW_DRIVE_CRC;
					}
		  break;

		  case FORWARD_DRIVE:
						CAN_BASKET_RC_DRIVING[1] = SLOW_DRIVE;
						CAN_BASKET_RC_DRIVING[3] = SLOW_DRIVE_CRC;
						CAN_BASKET_RC_DRIVING[0] = STOP_DRIVE;
						CAN_BASKET_RC_DRIVING[2] = STOP_DRIVE;
						turn_in_place = 0;
								
		  break;
		}
		
		
		
		CAN_BASKET_RC_DRIVING[4] = 0xFF;
		CAN_BASKET_RC_DRIVING[5] = left_btn;
		CAN_BASKET_RC_DRIVING[6] = 0x00;
		CAN_BASKET_RC_DRIVING[7] = 0xFF;
		
		CAN_BASKET_RC_DIR[0] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[1] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[2] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[3] = STOP_DRIVE;
		CAN_BASKET_RC_DIR[4] = 0xFF;
		CAN_BASKET_RC_DIR[5] = right_btn;
		CAN_BASKET_RC_DIR[6] = 0x00;
		CAN_BASKET_RC_DIR[7] = 0xFF;
		
		SPEED_ENGINE = (RPM_ENGINE - 750) / 5.68;
		CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[3] = speed_level;
		CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[5] = 0xFF;
		CAN_BASKET_RC_SPEED_ENGINE[6] = 0x01;
		CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;
		
		CAN_Send(CAN_BASKET_RC_DRIVING, 		CAN_Remote_Control_A_ID);
		CAN_Send(CAN_BASKET_RC_DIR, 			CAN_Remote_Control_B_ID);
		CAN_Send(CAN_BASKET_RC_SPEED_ENGINE, 	CAN_Remote_Control_C_ID);
		
		osDelay(100);
	}
}

int main(void)
{ 
    Init_RCC();																	// Инициализация RCC
	
	
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 550);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	
	osThreadDef(myLedTask, StartLedTask, osPriorityNormal, 0, 128);
	myLedTaskHandle = osThreadCreate(osThread(myLedTask), NULL);

	
	osThreadDef(myGsmTask, StartGsmTask, osPriorityNormal, 0, 550);
	myGsmTaskHandle = osThreadCreate(osThread(myGsmTask), NULL);

	
	osThreadDef(myGpsTask, StartGpsTask, osPriorityNormal, 0, 64);
	myGpsTaskHandle = osThreadCreate(osThread(myGpsTask), NULL);

	
	osThreadDef(mySysTickTask, StartmySysTickTask, osPriorityRealtime, 0, 128);
	mySysTickTaskHandle = osThreadCreate(osThread(mySysTickTask), NULL);
	
	osThreadDef(myMobileApplicationTask, StartmyMobileApplicationTask, osPriorityNormal, 0, 128);
	myMobileApplicationHandle = osThreadCreate(osThread(myMobileApplicationTask), NULL);
	
	osThreadDef(mySendingServerData, StartmySendingServerDataTask, osPriorityNormal, 0, 128);
	mySendingServerDataHandle = osThreadCreate(osThread(mySendingServerData), NULL);
	
	osThreadDef(myControl, StartmyControlTask, osPriorityNormal, 0, 128);
	myControlHandle = osThreadCreate(osThread(myControl), NULL);
	
	osThreadDef(myUsartTask, StartUsartTask, osPriorityAboveNormal, 0, 128);
	myUsartHandle = osThreadCreate(osThread(myUsartTask), NULL);
		
	osMessageQDef(SendQueue, MAIL_SIZE, QUEUE_t);
	SendQueueHandle = osMessageCreate(osMessageQ(SendQueue), NULL);
	
	osMessageQDef(UsartQueue, 200, USART_q);
	UsartQueueHandle = osMessageCreate(osMessageQ(UsartQueue), NULL);
	
	osKernelStart();
	
	
    while(1)
    {

	}
}

u8 GET_DATA_PARCEL(u8 size, char *xBuf, u8 *result_mas1)
{
	
	u16 inc = 0, ti = 0, threshold = 0;
	u8 pack_length1 = 0;

	u8 dollar_flag1 = 0;
	u8 crc_calc = 0;
	u8 crc_received = 0;
	clear_RXBuffer(zzzbuffer, 20);
	
//	if (xBuf[0] == 0) return ERROR;
	
	for(ti = 0; ti < SERVER_BUF; ti++)
	{
//		if (xBuf[ti] == 'E') Xsearch++;
//		else if (Xsearch == 1 && xBuf[ti] == ',') Xsearch++;
//		else if (Xsearch == 2 && xBuf[ti] == '1') Xsearch++;
//		else Xsearch = 0;

//		if (Xsearch == 3) 
//		{
//			parcel_found = 1;
//			Xsearch = 0;
//		}
		
		if (xBuf[ti] == '$') 
		{
			dollar_flag1 = 1;
			threshold = ti;
			break;
		}
	}
	
	if (ti == SERVER_BUF)
	{		
		return ERROR;
	}
		
	switch (xBuf[threshold + 1])
	{
		case (u8)PACK_ID_tNum: pack_length1 = PACK_LEN_tNum;
		break;
		case (u8)PACK_ID_Coords: pack_length1 = PACK_LEN_Coords;
		break;
		case (u8)PACK_ID_Mode: pack_length1 = PACK_LEN_Mode;
		break;
		case (u8)PACK_ID_RC: pack_length1 = PACK_LEN_RC;
		break;
		case (u8)PACK_ID_Err: pack_length1 = PACK_LEN_Err;
		break;
	
		
		default: pack_length1 = 0;
	}
	xBuf[threshold] = 0;
	for (inc = 0; inc < pack_length1; inc++)
	{
		zzzbuffer[inc] = xBuf[threshold + inc + 1];
		xBuf[threshold + inc + 1] = 0;
	}
	if (dollar_flag1)
	{
		crc_received = zzzbuffer[pack_length1 - 1];
		crc_calc = crc8(zzzbuffer, pack_length1 - 1);
		
		if (crc_calc == crc_received) 
		{
			result_mas1[0] = 0x24;
			for(inc = 0; inc < pack_length1; inc++)
			{
				result_mas1[inc+1] = zzzbuffer[inc];
			}
			return pack_length1 + threshold + 1;
		}
		else return ERROR;
	}
	else return ERROR;
}

u8 MyCoords[13];
double myLat = 55.216422;
double myLon = 61.441115;
void SENDING_COORDS(void)
{
	QUEUE_t msg;
	u64 mulLat = (myLat + 90) * pow(10, 8);
	u64 mulLon = (myLon + 180) * pow(10, 8);
	clear_RXBuffer(MyCoords, 13);
	
	msg.data[0] = '$';
	msg.data[1] = 0x12;
	for(u8 i = 0; i <= 4; i++)
	{
		msg.data[i + 2] = mulLat >> (8 * i) & 0xFF;
	}
	
	for(u8 i = 0; i <= 4; i++)
	{
		msg.data[i + 7] = mulLon >> (8 * i) & 0xFF;
	}
	
	msg.data[12] = (u16)roundf(HEADING) & 0xFF;
	msg.data[13] = (u16)roundf(HEADING) >> 8;
	msg.data[14] = char_to_int(&GPSFixData.ReceiverMode);
	msg.data[15] = char_to_int(GPSFixData.SatelliteNum);
	msg.data[16] = MODE_CONTROL;
	msg.data[17] = crc8(&msg.data[1], 16);
	xQueueSend(SendQueueHandle, &msg.data, osWaitForever);
}

void IWDG_ON(void)
{
	/* Включение LSI */
	RCC_LSICmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET){}
 
	/* Активация Watchdog для выхода из Standby режима*/
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256); 
	IWDG_SetReload(0xFFF);	// Количество тиков Watchdog
		
	IWDG_ReloadCounter();
	IWDG_Enable();
}

u8 Items_num_Queue = 0;


u8 count_of_RTK_missing = 0;
u8 count_of_USART_missing = 0;
u8 count_of_SERVER_missing = 0;
u8 dsa[13] = {0x24, 0x06, 0x04, 0x08, 1, 1, 1, 1, 1, 1, 1, 1, 0x4D};

void StartDefaultTask(void const * argument)
{
	
	Init_CAN();
	#if SCHEDULER_STATUS
		Init_TIM();
	#endif
	Init_GSM();																// Инициализация USART порта GPS модуля
	
	Init_GPS();																	// Инициализация GPS модуля

	GPIO_SetBits(GPIOB, SIM_CH_PIN);
	send_Uart_AT(USART2, (u8*)"DEVICE WAS RELOADED\r\n");
	send_Uart_AT(USART2, (u8*)">>>>>>>>>>>>>>>>>>>\r\n\r\n");
//	GSM_Off();
	GSM_Restart();																// Запуск GSM модуля

	USART1->ICR |= 0x02;														// Обнуление флага, который иногда появляется на USART (не разобрался с чем связано)
																				// но из за него иногда перестает работать USART!

	GSM_conf();			                      									// Конфигурируем GSM модуль, до тех пор, 
																				// пока не получим положительный ответ от модуля
	//IWDG_ON();
	
	GPRS_Connection_ON();                 										// Настройка параметров GPRS подключения
	isGprsActive = 1;
																				// Подключение к NTRIP серверу
	NTRIP_CONNECT_OK = CONNECT_TO_NTRIP(NTRIP_SERVER_CON_ID, NTRIP_SERVER_ADDRESS, NTRIP_SERVER_PORT, NTRIP_SERVER_POINT, NTRIP_SERVER_LOGIN, NTRIP_SERVER_PASSWORD); 
				        	
	REMOTE_CONNECT_OK = CONNECT_TO_SERVER(REMOTE_SERVER_CON_ID, REMOTE_SERVER_ADDRESS, REMOTE_SERVER_PORT);			// Подключение к удаленному серверу

	while(!osDelay(100))
	{
		
		if (!NTRIP_CONNECT_OK)													// Если соединение с сервером потеряно иницилизируем повторное подключение
			NTRIP_CONNECT_OK = CONNECT_TO_NTRIP(NTRIP_SERVER_CON_ID, NTRIP_SERVER_ADDRESS, NTRIP_SERVER_PORT, NTRIP_SERVER_POINT, NTRIP_SERVER_LOGIN, NTRIP_SERVER_PASSWORD);	
																				// Подключение
		
		
		if (!REMOTE_CONNECT_OK)													// Если соединение с сервером потеряно иницилизируем повторное подключение
			REMOTE_CONNECT_OK = CONNECT_TO_SERVER(REMOTE_SERVER_CON_ID, REMOTE_SERVER_ADDRESS, REMOTE_SERVER_PORT); 												
																				// Подключение	


			
		n_lat_float = n_lon_float = n_alt_float = 0;							// Обнуляем счетчики десятичных цифр
		

				
		
	} 
}

void StartLedTask(void const * argument)
{
	while(!osDelay(1))
	{	if (NTRIP_CONNECT_OK || REMOTE_CONNECT_OK)
		{
			if (NTRIP_CONNECT_OK && REMOTE_CONNECT_OK)
			{
				blink(GPIOB, debug_pin, 2, 100);
				osDelay(500);
			}
			else
			{
				blink(GPIOB, debug_pin, 2, 100);
				osDelay(1000);
			}

		}
		else if (isGprsActive)
		{
			GPIO_SetBits(GPIOB, debug_pin);
			osDelay(500);
			GPIO_ResetBits(GPIOB, debug_pin);
			osDelay(500);
		}
		else
		{
			GPIO_SetBits(GPIOB, debug_pin);
			osDelay(1000);
			GPIO_ResetBits(GPIOB, debug_pin);
			osDelay(1000);
		}
	} 
}
u8 queue_cnt = 0;
void StartGsmTask(void const * argument)
{
	while(1)
	{	
		if (!incData_from_SERVER1 && !incData_from_SERVER2)
			osDelay(1);
		
		if (NTRIP_CONNECT_OK)
		{
			if (incData_from_SERVER1)
			{	
				incData_from_SERVER1 = 0;
				debug_cnt++;	
				get_server1_timer = tick_counter;
			}
			
			if (tick_counter - get_server1_timer > 2000)
			{
				get_server1_timer = tick_counter; 
			}
							
		}
		
		if (REMOTE_CONNECT_OK)
		{
			if (incData_from_SERVER2)
			{
				getting_data();
				get_server2_timer = tick_counter;
				debug_cnt2++;		
			}
		}			
	}
}

u8 global_buf[PACK_LEN_dozerParams * 21];


u32 check_ext_id = 0;
u32 def_ext_id = 0;
enum List_of_CAN_places place;
void StartmySendingServerDataTask(void const * argument)
{
	
	QUEUE_t msg;
	time_t can_send_timer;
	clear_RXBuffer(global_buf, PACK_LEN_dozerParams * 21);
	while(1)
	{
//		if (FAIL_cnt > 1)
//		{
//			osDelay(2000);
//			FAIL_cnt = 0;
//		}
		
		Items_num_Queue = uxQueueMessagesWaiting(SendQueueHandle);
		if (xQueueReceive(SendQueueHandle, &msg, 0) == pdTRUE && REMOTE_CONNECT_OK)
		{
			if (msg.data[1] == PACK_ID_dozerParams)
			{
				check_ext_id = 0x18000003 | (msg.data[2] << 8) | (msg.data[3] << 16);
				
				switch(check_ext_id)
				{
					case CAN_Tx_Modem_201_ID:
						place = CAN_Tx_Modem_201_place;
					break;
					
					case CAN_Tx_Modem_205_ID:
						place = CAN_Tx_Modem_205_place;
					break;
					
					case CAN_Tx_Modem_207_ID:
						place = CAN_Tx_Modem_207_place;
					break;
				
					case CAN_Tx_Modem_209_ID:
						place = CAN_Tx_Modem_209_place;
					break;
					
					case CAN_Tx_Modem_210_ID:
						place = CAN_Tx_Modem_210_place;
					break;
					
					case CAN_Tx_Modem_215_ID:
						place = CAN_Tx_Modem_215_place;
					break;
					
					case CAN_Tx_Modem_216_ID:
						place = CAN_Tx_Modem_216_place;
					break;
					
					case CAN_Tx_Modem_221_ID:
						place = CAN_Tx_Modem_221_place;
					break;
					
					case CAN_Tx_Modem_2BD_ID:
						place = CAN_Tx_Modem_2BD_place;
					break;
					
					case CAN_Tx_Modem_303_ID:
						place = CAN_Tx_Modem_303_place;
					break;
					
					case CAN_Tx_Modem_308_ID:
						place = CAN_Tx_Modem_308_place;
					break;
					
					case CAN_Tx_Modem_311_ID:
						place = CAN_Tx_Modem_311_place;
					break;
					
					case CAN_Tx_Modem_313_ID:
						place = CAN_Tx_Modem_313_place;
					break;
					
					case CAN_Tx_Modem_314_ID:
						place = CAN_Tx_Modem_314_place;
					break;
					
					case CAN_Tx_DM1_Angles:
						place = CAN_Tx_DM1_Angles_place;
					break;
					
					case CAN_Tx_DM1_Heights:
						place = CAN_Tx_DM1_Heights_place;
					break;
						
					default: check_ext_id = (msg.data[2]) | (msg.data[3] << 8);
				}
				
				switch(check_ext_id)
				{
					case CAN_EE_F004_ID:
						place = CAN_EE_F004_place;
					break;
					
					case CAN_EE_FEEE_ID:
						place = CAN_EE_FEEE_place;
					break;
					
					case CAN_EE_FEEF_ID:
						place = CAN_EE_FEEF_place;
					break;
					
					case CAN_EE_FEFC_ID:
						place = CAN_EE_FEFC_place;
					break;
					
					case CAN_EE_FFA0_ID:
						place = CAN_EE_FFA0_place;
					break;
				}
				
				for(u8 i = 0; i < PACK_LEN_dozerParams; i++)
				{
					global_buf[PACK_LEN_dozerParams * place + i] = msg.data[i];
				}
				
				if (tick_counter - can_send_timer > 200)
				{
					can_send_timer = tick_counter;
					CIPSEND(1, PACK_LEN_dozerParams * 21, global_buf, 2000);
				}
			}
			
			if (msg.data[1] == PACK_ID_myCoords)
			CIPSEND(1, PACK_LEN_myCoords, msg.data, 2000);
			
			if (msg.data[1] == PACK_ID_Confirm || msg.data[1] == PACK_ID_Err)
			CIPSEND(1, PACK_LEN_Confirm, msg.data, 2000);
			
			if (msg.data[1] == PACK_ID_CompletePoint)
			CIPSEND(1, PACK_LEN_CompletePoint, msg.data, 2000);
			
		}
		else
		{	
			osDelay(1);
		}
	}
}

void StartmyMobileApplicationTask(void const * argument)
{
	time_t current_time2 = tick_counter;
	while(!osDelay(1))
	{
		if (REMOTE_CONNECT_OK  && tick_counter - current_time2 > 1000)
		{
			SENDING_COORDS();
			current_time2 = tick_counter;
		}
		
		if (RING)
		{
			if (strstr((const char *)GSM_DATA, PHONE_NUMBER1) || strstr ((const char *)GSM_DATA, PHONE_NUMBER2))
			{
				MODE_CONTROL = DISCONNECT_RC;
				RING = 0;
			}
		}
	}
}

u32 GPS_count = 0;
void StartGpsTask(void const * argument)
{
	
	while(!osDelay(1))
	{
		if (GPS_DATA[k-1] == 0x0A && GPS_DATA[k-2] == 0x0D) 
		{
			Parse_Nmea_Messages();
			coord_GPS1_latitude = GPS_lat;
			coord_GPS1_longitude = GPS_lon;
			GPS_count++;	
			k = 0;
		
			clear_RXBuffer((u8 *)GPSFixData.alt_full, sizeof(GPSFixData.alt_full));
		}
			coord_GPS1_latitude = myLat;
			coord_GPS1_longitude = myLon;
	}
}

void STOP_MOVING(u8 param)
{
	CAN_BASKET_RC_DRIVING[0] = 0;
	CAN_BASKET_RC_DRIVING[1] = 0;
	CAN_BASKET_RC_DRIVING[2] = 0;
	CAN_BASKET_RC_DRIVING[3] = 0;
	CAN_BASKET_RC_DRIVING[4] = 0xFF;
	CAN_BASKET_RC_DRIVING[5] = 0;
	CAN_BASKET_RC_DRIVING[6] = 0x00;
	CAN_BASKET_RC_DRIVING[7] = 0xFF;
	
	CAN_BASKET_RC_DIR[0] = 0;
	CAN_BASKET_RC_DIR[1] = 0;
	CAN_BASKET_RC_DIR[2] = 0;
	CAN_BASKET_RC_DIR[3] = 0;
	CAN_BASKET_RC_DIR[4] = 0xFF;
	CAN_BASKET_RC_DIR[5] = 0;
	CAN_BASKET_RC_DIR[6] = 0x00;
	CAN_BASKET_RC_DIR[7] = 0xFF;
	
	SPEED_ENGINE = (RPM_ENGINE - 750) / 5.68;
	CAN_BASKET_RC_SPEED_ENGINE[0] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[1] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[2] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[3] = (u8)SPEED_ENGINE;
	CAN_BASKET_RC_SPEED_ENGINE[4] = 0xFF;
	CAN_BASKET_RC_SPEED_ENGINE[5] = 0xFF;
	
	if (param)
		CAN_BASKET_RC_SPEED_ENGINE[6] = 0x01;
	else
		CAN_BASKET_RC_SPEED_ENGINE[6] = 0x00;
	
	CAN_BASKET_RC_SPEED_ENGINE[7] = 0x01;
	
	CAN_Send(CAN_BASKET_RC_DRIVING, 		CAN_Remote_Control_A_ID);
	CAN_Send(CAN_BASKET_RC_DIR, 			CAN_Remote_Control_B_ID);
	CAN_Send(CAN_BASKET_RC_SPEED_ENGINE, 	CAN_Remote_Control_C_ID);
}

void StartmyControlTask(void const * argument)
{
	
	while(!osDelay(1))
	{	
		
		HEADING = Azimuth_Calculating(coord_GPS1_latitude, coord_GPS1_longitude, coord_GPS2_latitude, coord_GPS2_longitude);
		
		switch (MODE_CONTROL)
		{
			case MANUAL_CONTROL_ON:		run_RC_drive();
										RC_OFF_cnt = 0;
			break;
			
			case MANUAL_CONTROL_OFF:	MODE_CONTROL = IDLE_MODE;
										RC_OFF_cnt = 0;
			break;
			
			case START_AUTO:			run_gps_drive();
										RC_OFF_cnt = 0;
			break;
			
			case PAUSE_AUTO:			STOP_MOVING(1);
										RC_OFF_cnt = 0;
										osDelay(100);
			break;
			
			case RESUME_AUTO:
										MODE_CONTROL = START_AUTO;
										RC_OFF_cnt = 0;
			break;
			
			case STOP_AUTO:
										Num_Route_Point_DONE = 0;
										Current_Route_Point = 0;
										total_NUM_Points_Route = 0;
										coord_TARGET_latitude = 0;
										coord_TARGET_longitude = 0;
										ANGLE_btw_TARGET_N_DOZER = 0;
										HEADING = 0;
			
										for(uint8_t i = 0; i < RoutePoints_COUNT; i++)
										{
											coordRoute[i].latitude = 0;
											coordRoute[i].longitude = 0;
										}

																				
										MODE_CONTROL = IDLE_MODE;
										RC_OFF_cnt = 0;
			break;
			
			case DISCONNECT_RC:			if (RC_OFF_cnt < 4)
										{
											STOP_MOVING(0);
											
											RC_OFF_cnt++;
										}
										osDelay(100);
																	
			break; 
			
			case IDLE_MODE: 			STOP_MOVING(1);
										osDelay(100);
										RC_OFF_cnt = 0;
			break;
		}		
	}
}

void StartmySysTickTask(void const * argument)
{
	
	while(!osDelay(1))
	{
		tick_counter++;
		
		freemem = xPortGetFreeHeapSize();
		#if DEBUG
			if (tick_counter - vTask_timer + 1 > 2000)
			{
				
				#if SCHEDULER_STATUS
					vTaskGetRunTimeStats(buffer_vTask);
				#else
					vTaskList(buffer_vTask);
				#endif
				vTask_timer = tick_counter;
			}
		#endif
		
		if(USART_GetFlagStatus(USART1, USART_FLAG_FE) == SET)
		{
			USART1->ICR = 0x02;
			frame_err_cnt++;
		}
		delta_timeout_USART = tick_counter - timeout_USART_wait;					// Таймаут ожидания ответа от сервера
		delta_timeout_RTK = tick_counter - RECEIVE0_timer;
		delta_timeout_SERVER = tick_counter - RECEIVE1_timer;
		delta_timeout_RC = tick_counter - RC_timer;
		delta_timeout_Can_coords = tick_counter - timer_CAN_coords;
		
		if ((MODE_CONTROL == MANUAL_CONTROL_ON && delta_timeout_RC > 2000))
		{
			MODE_CONTROL = DISCONNECT_RC;
		}
		
		if (server_CLOSED)
		{
			isConnectionOpened = 0;
			NTRIP_CONNECT_OK = 0;
			server_CLOSED = 0;
		}
		
		if (delta_timeout_Can_coords > 1000)
		{
			coord_GPS2_latitude = 0;
			coord_GPS2_longitude = 0;
		}
		
		if (NTRIP_CONNECT_OK && delta_timeout_RTK > 10000) 											
		{										// Обнуляем флаг успешного подключения
			NTRIP_CONNECT_OK = 0;
		}
		if (REMOTE_CONNECT_OK && delta_timeout_SERVER > 10000)
		{
												// Обнуляем флаг успешного подключения
			NTRIP_CONNECT_OK = 0;
			REMOTE_CONNECT_OK = 0;
			isGprsDeactNeed = 1;	
			MODE_CONTROL = DISCONNECT_RC;			
		}
		
		if (delta_timeout_USART > 20000)
		{
			__nop();
		}
	}
}

void Error_Handler(void)
{
	GPIO_SetBits(GPIOB, debug_pin);
	osDelay(50);
	GPIO_ResetBits(GPIOB, debug_pin);
	osDelay(50);
}


	 
