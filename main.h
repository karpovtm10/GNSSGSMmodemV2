#ifndef __main
#define __main
#define GIT										0
#define MAIL_SIZE 								(u32) 50					// Размер очереди
#define DEBUG 									0							// Отладка
#if DEBUG
#define SCHEDULER_STATUS						1							// Статус планировщика. 0 - диспетчер задач, 1 - время работы задач
#endif
#define	RoutePoints_COUNT						10							// Количество точек для сохранения маршрута
#define	SaveZonePoints_COUNT					10							// Количество точек для сохранения зоны безопасности

#define waiting 								1000						// Таймаут ожидания для общения с GSM модулем
#define TIMEOUT_RTK								650							// Время таймаута ожидания новой посылки GPS
#define TIMEOUT_TIME                            10000                       // Время таймаута ожидания ответа GSM

#define CAN_EXT_ID_MASK							0x18000003
#define CAN_STD_ID								0x000						// Стандартный идентификатор
#define CAN_EXT_ID								0x00000000					// Расширенный идентификатор
#define CAN_LAT_ID								0x18060103					// Идентификатор широты для Контроллера ГСТ
#define CAN_LON_ID								0x18060203					// Идентификатор долготы для Контроллера ГСТ
#define CAN_ALT_ID								0x18051003					// Идентификатор высоты для Контроллера ГСТ

#define CAN_LAT1_EXT_ID							0x174						// Идентификатор широты от второго ГНСС модема
#define CAN_LON1_EXT_ID							0x175						// Идентификатор долготы от второго ГНСС модема
#define CAN_Remote_Control_A_ID					0x195						// Посылка от джойстика движения
#define CAN_Remote_Control_B_ID					0x196						// Посылка от джойстика навесного оборудования
#define CAN_Remote_Control_C_ID					0x198						// Посылка с оборотами ДВС
#define CAN_Remote_Control_sButton_ID			0x300

#define RPM_ENGINE								1600						// Обороты 750...2200

#define NTRIP_SERVER_ADDRESS                    "mo.youcors.com"            // Адрес NTRIP сервера
#define NTRIP_SERVER_PORT                       "2101"                      // Порт NTRIP сервера

#define NTRIP_CASTER                            "mo.youcors.com"            // Адрес кастера
#define NTRIP_SERVER_LOGIN                      "august24client"             // Логин точки доступа на NTRIP сервере
#define NTRIP_SERVER_PASSWORD                   "123456"                	// Пароль точки доступа на NTRIP сервере
#define NTRIP_SERVER_POINT                      "MPAugust24"                 // Имя точки доступа на NTRIP сервере

#define REMOTE_SERVER_ADDRESS					"192.162.100.33"			// Адрес сервера для мобильного приложения
#define REMOTE_SERVER_PORT                      "12488" 					// Порт сервера для мобильного приложения

#define PHONE_NUMBER1							"+79822979396"
#define PHONE_NUMBER2							"+79822814709"


#define RFM22_LENGTH_BUFFER						16                          // Длинна буфера радио модуля
#define USART2_RX_BUF_SIZE 						8                           // Длина буфера GPS данных

#define AT(x)									Write_GSM_Command((u8*)x)	// Сокращение функции для записи АТ команд

#define	GPS_USART_RX_PIN						GPIO_Pin_3					// Пин GPS USART RX
#define	GPS_USART_TX_PIN						GPIO_Pin_2					// Пин GPS USART TX
#define	GPS_PPS_PIN								GPIO_Pin_6					// Пин GPS PPS

#define GSM_GPIO								GPIOA
#define	GSM_USART_RX_PIN						GPIO_Pin_10
#define GSM_USART_TX_PIN						GPIO_Pin_9
#define GSM_PWR_KEY_PIN							GPIO_Pin_8
#define debug_pin								GPIO_Pin_1
#define SIM_CH_PIN								GPIO_Pin_0

#define	GSM_PWR_ON								GPIO_SetBits(GPIOB, GSM_PWR_KEY_PIN);
#define	GSM_PWR_OFF								GPIO_ResetBits(GPIOB, GSM_PWR_KEY_PIN);


#define GSM_BUF									100
#define PRINT_BUF								255
#define STATUS_BUF								255
#define GPS_BUF									255
#define	SERVER_BUF								500
#define NTRIP_BUF								1024
#define	CAN_BUF									8

#define DEG_TO_RAD 								0.01745329252
#define RAD_TO_DEG								57.29577951308

#define FULL_DRIVE								100
#define STOP_DRIVE								125
#define SLOW_DRIVE								50

#define	FULL_DRIVE_CRC							255 - FULL_DRIVE
#define STOP_DRIVE_CRC							255 - STOP_DRIVE
#define SLOW_DRIVE_CRC							255 - SLOW_DRIVE

#define NTRIP_SERVER_CON_ID						0
#define REMOTE_SERVER_CON_ID					1

#define PACK_ID_tNum							0x01
#define PACK_ID_Coords							0x02
#define PACK_ID_Mode							0x05
#define PACK_ID_RC								0x06
#define PACK_ID_ALIVE							0xA1
#define PACK_ID_Err								0xFF
#define PACK_ID_Confirm							0xFE

#define PACK_ID_myCoords						0x12
#define PACK_ID_dozerParams						0xA0
#define PACK_ID_CompletePoint					0x13

#define PACK_LEN_tNum							4
#define PACK_LEN_Coords							14
#define PACK_LEN_Mode							4
#define PACK_LEN_RC								12
#define PACK_LEN_Err							5
#define PACK_LEN_Confirm						6
#define PACK_LEN_myCoords						18
#define PACK_LEN_dozerParams					13
#define PACK_LEN_CompletePoint					4
#define PACK_LEN_ALIVE							5

#define PARCEL_ID_tNumSave						0x01
#define PARCEL_ID_tNumCoord						0x02
#define PARCEL_ID_SaveCoord						0x01
#define PARCEL_ID_PointCoord					0x02
#define PARCEL_ID_Mode							0x01
#define PARCEL_ID_RC							0x00
#define PARCEL_ID_Err							0xFF
#define PARCEL_ID_myCoord						0x01

#define CAN_Tx_Modem_201_ID						0x18020103
#define CAN_Tx_Modem_205_ID						0x18020503
#define CAN_Tx_Modem_207_ID						0x18020703
#define CAN_Tx_Modem_209_ID						0x18020903
#define CAN_Tx_Modem_210_ID						0x18021003
#define CAN_Tx_Modem_215_ID						0x18021503
#define CAN_Tx_Modem_216_ID						0x18021603
#define CAN_Tx_Modem_221_ID						0x18022103
#define CAN_Tx_Modem_2BD_ID						0x1802BD03

#define CAN_Tx_Modem_303_ID						0x18030303
#define CAN_Tx_Modem_308_ID						0x18030803
#define CAN_Tx_Modem_311_ID						0x18031103
#define CAN_Tx_Modem_313_ID						0x18031303
#define CAN_Tx_Modem_314_ID						0x18031403

#define CAN_EE_F004_ID							0xF004
#define CAN_EE_FEEE_ID							0xFEEE
#define CAN_EE_FEEF_ID							0xFEEF
#define CAN_EE_FEFC_ID							0xFEFC
#define CAN_EE_FFA0_ID							0xFFA0

#define CAN_Tx_DM1_Angles						0x18051103
#define CAN_Tx_DM1_Heights						0x18051203

enum List_of_Points_Type
{
	SAVE_ZONE = 1,				
	ROUTE									
};

enum List_of_MOVING
{
	NO_MOVEMENT,         					
	LEFT_DRIVE,          					
	RIGHT_DRIVE,         					
	FORWARD_DRIVE,       					
	BACKWARD_DRIVE      					
};

enum List_of_MODES_of_CONTROL
{
	IDLE_MODE,
	MANUAL_CONTROL_ON,
	MANUAL_CONTROL_OFF,
	START_AUTO,
	PAUSE_AUTO,
	RESUME_AUTO,
	STOP_AUTO,
	DISCONNECT_RC = 0xFF
};

enum List_of_CAN_places
{
	CAN_Tx_Modem_201_place,					
	CAN_Tx_Modem_205_place,					
	CAN_Tx_Modem_207_place,					
	CAN_Tx_Modem_209_place,					
	CAN_Tx_Modem_210_place,					
	CAN_Tx_Modem_215_place,				
	CAN_Tx_Modem_216_place,					
	CAN_Tx_Modem_221_place,					
	CAN_Tx_Modem_2BD_place,					
	CAN_Tx_Modem_303_place,					
	CAN_Tx_Modem_308_place,					
	CAN_Tx_Modem_311_place,					
	CAN_Tx_Modem_313_place,					
	CAN_Tx_Modem_314_place,					

	CAN_EE_F004_place,						
	CAN_EE_FEEE_place,						
	CAN_EE_FEEF_place,						
	CAN_EE_FEFC_place,						
	CAN_EE_FFA0_place,						

	CAN_Tx_DM1_Angles_place,				
	CAN_Tx_DM1_Heights_place				
};


typedef int64_t time_t;
typedef int64_t i64;
typedef uint8_t u8;
typedef int32_t i32;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
extern osMessageQId UsartQueueHandle;

extern u8 											isGprsDeactNeed;
extern volatile time_t								delta_timeout_RTK;
extern volatile time_t								delta_timeout_SERVER;
extern volatile time_t 								get_server1_timer;
extern volatile time_t 								get_server2_timer;

extern u8 											RING;


extern u8											search_DOLLAR;
extern u8											search_RECEIVE0;
extern u8											search_RECEIVE1;


extern volatile time_t 								RECEIVE0_timer;									// Таймер для посылок с NTRIP сервера
extern volatile  time_t 							RECEIVE1_timer;									// Таймер для посылок с сервера
extern u8											overflow;
extern u16											f5_debug;

extern u8											NTRIP_CONNECT_OK;								// Флаг успешного подключения к NTRIP серверу
extern u8											REMOTE_CONNECT_OK; 								// Флаг успешного подключения к серверу

extern u8 											parcel_count;
extern u8											server_CLOSED;

extern u16											s1_buf_cnt;										// Счетчик буфера сервера


extern u8											incData_from_SERVER1;							// Флаг полученной посылки с первого сервера
extern u8											incData_from_SERVER2;							// Флаг полученной посылки с NTRIP сервера
extern volatile time_t 								calculated_time;

extern char			size_of_RTK_parcel_arr			[4];
extern char			RECEIVE_SERVER_BUF				[SERVER_BUF];


typedef struct          						// Структура с сервисными данными
{
	u8 data [18];           					// Данные
} QUEUE_t;

typedef struct          						// Структура с сервисными данными
{
	u8 data [1];           						// Данные
} USART_q;

extern enum List_of_MODES_of_CONTROL				MODE_CONTROL;
extern u16											mnc_operator;
extern u8 											percent_signal;
extern u8											SIM1_rssi;


extern volatile time_t tick_counter;
extern volatile time_t								timeout_USART_wait;
extern volatile char GSM_DATA [GSM_BUF];
extern char 			print_buffer					[PRINT_BUF];
extern char mnc_operator_array[6];
// Функции
// ********************************************************************************************************************************************
void 											Error_Handler(void);
void 											blink(GPIO_TypeDef* GPIOx, u16 GPIO_Pin, u8 count, u32 duration_ms);
void 											Init_RCC(void);

void 											Init_USART_2(void);
void 											Init_CAN(void);
void 											Init_GPS(void);
void send_Uart_SERV(USART_TypeDef* USARTx, u8 *s, u16 size);
void send_Uart_AT(USART_TypeDef* USARTx, u8 *s);
void send_Uart(USART_TypeDef* USARTx, u8 c);
void 											StartDefaultTask(void const * argument);
void 											StartLedTask(void const * argument);
void 											StartGsmTask(void const * argument);
void 											StartGpsTask(void const * argument);
void 											StartmySysTickTask(void const * argument);
void 											StartmyMobileApplicationTask(void const * argument);
void 											StartmySendingServerDataTask(void const * argument);
void 											StartmyControlTask(void const * argument);

void 											CAN_Send(u8 *data, u32 can_id); 
void 											Parse_Nmea_Messages (void);
void 											run_gps_drive(void);
void 											gsm_at_parse (char *result, volatile char *GSM_TEMP, char *left_mask, char *right_mask);

void 											GPRS_Connection_ON(void);

void											delay(u32 mTime);
void 											clear_RXBuffer(u8 *RX_BUFER, u16 size);
void 											getting_data(void);
double 											Azimuth_Calculating (double latitude1, double longitude1, double latitude2, double longitude2);
u8 												Query_Signal_Quality(void);
ErrorStatus 									GSM_conf(void);

ErrorStatus 									Connect_To_Server(u8 CON_ID);
u16 											UCS2Char(char *ucs2, char *simbol);

i64 											char_to_int (char * c);
u8 												GET_DATA_PARCEL(u8 size, char *xBuf, u8 *result_mas1);

// ********************************************************************************************************************************************

extern NVIC_InitTypeDef 					            NVIC_InitStructure;
extern USART_InitTypeDef 				                USART_InitStructure;
extern GPIO_InitTypeDef 					            GPIO_InitStructure;

#endif //__main
