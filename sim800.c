#include "stm32f0xx.h"
#include "cmsis_os.h"
#include "main.h"
#include "sim800.h"
#include "base64.h"
#include <stdio.h> 
#include <string.h>
USART_q                                     GSM_Usart_Symbol;
u32											gsm = 0;
u8											SEND_OK_waiting = 0;								// Флаг ожидания успешной отправки
u16											s2_buf_cnt = 0;										// Счетчик буфера NTRIP сервера
u16											size_of_RTK_parcel = 0;								// Размер принятой NTRIP посылки
u16											size_of_RTK_parcel_cnt = 0;							// Счетчик Массива
u16 										rtk_length = 0;
u8											good_end_of_AT = 0;
u8											bad_end_of_AT = 0;
u8 											SEND_OK = 0;
u8 											SEND_FAIL = 0;
u32											FAIL_cnt = 0;
u8											ready_to_send = 0;
u8											SUCCESS_NTRIP = 0;
u8											search_SUCCESS_NTRIP = 0;
u8											search_RING = 0;
u8											search_ARROW = 0;
u8											search_OK = 0;
u8											search_SEND_OK = 0;
u8											search_SEND_FAIL = 0;
u8											search_CONNECT = 0;
u8											search_ERROR = 0;
u8											search_CLOSED = 0;

u8 											search = 0;
u8 											search2 = 0;

u8											search_DOLLAR = 0;
u8											search_RECEIVE0 = 0;
u8											search_RECEIVE1 = 0;
u8											continue_debug = 0;
u8 											parcel_count;
u8 											pack_length = 0;

u8											received_pack_bytes = 0;
u16											received_rtk_bytes = 0;

void Init_GSM(void)                                             				// Функция инициализаци GSM модуля
{
    GPIO_InitStructure.GPIO_Pin 	                = GSM_PWR_KEY_PIN | debug_pin | SIM_CH_PIN; // debug_pin - GPIOB pin 2 отладочный светодиод
    GPIO_InitStructure.GPIO_Mode 	                = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed                   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType                   = GPIO_OType_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	


    Init_USART_1();
}

void send_Uart(USART_TypeDef* USARTx, u8 c)                				// Отправка симовла по USART, GPS
{
    while(USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
    USART_SendData(USARTx, c);
}

void send_Uart_AT(USART_TypeDef* USARTx, u8 *s)           					// Отправка строки AT команд
{
	vTaskSuspendAll();
    while (*s != 0) send_Uart(USARTx, *s++);
	xTaskResumeAll();
}

void send_Uart_SERV(USART_TypeDef* USARTx, u8 *s, u16 size)           					// Отправка строки AT команд
{
	u16 i = 0;
	vTaskSuspendAll();
    while (i < size) 
	{
//		send_Uart(USART2, s[i]);
//		if (USARTx == USART1)
			send_Uart(USARTx, s[i]);
		i++;
	}
	xTaskResumeAll();
}

void USART1_IRQHandler(void)                                    				// Обработчик прерываний USART, чтение данных с GSM модуля
{
    if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
    {	
		BaseType_t xHigherPriorityTaskWoken;
		IWDG_ReloadCounter();		
        GSM_Usart_Symbol.data[0] = USART_ReceiveData(USART1);           				// Получение символов из USART
		xQueueSendFromISR(UsartQueueHandle, &GSM_Usart_Symbol, &xHigherPriorityTaskWoken);
	
		timeout_USART_wait = tick_counter;		
    }


}
void Init_USART_1(void)                                         				// Функция инициализацмии USART1
{	
	NVIC_InitStructure.NVIC_IRQChannel              = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority      = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd           = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);
    
    //TX UART
    GPIO_InitStructure.GPIO_Pin                     = GSM_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode                    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed                   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType                   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd                    = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //RX UART
    GPIO_InitStructure.GPIO_Pin                     = GSM_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode                    = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure the USART2 */
    USART_InitStructure.USART_BaudRate              = 115200;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
		
	USART_OverrunDetectionConfig(USART1, USART_OVRDetection_Disable);
    USART_Init(USART1, &USART_InitStructure);
		
    USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
		
}

void Init_USART_1_high_speed(void)                                         				// Функция инициализацмии USART1
{	
	 USART_Cmd(USART1, DISABLE);

    /* Configure the USART2 */
    USART_InitStructure.USART_BaudRate              = 230400;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Rx | USART_Mode_Tx;
		
	USART_OverrunDetectionConfig(USART1, USART_OVRDetection_Disable);
    USART_Init(USART1, &USART_InitStructure);
		
    USART_Cmd(USART1, ENABLE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);	
		
}

void GSM_Off(void)                                              				// Выключение GSM модуля
{
    GSM_PWR_ON;
    osDelay(1800);
	GSM_PWR_OFF;
}

void GSM_On(void)                                               				// Включение GSM модуля
{
    GSM_PWR_ON;
    osDelay(1000);
	GSM_PWR_OFF;
}

void GSM_Restart(void)                                          				// Перезапуск GSM модуля
{
	GSM_Off();
	osDelay(1800);
	GSM_On();
	osDelay(2800);
}

ErrorStatus Write_GSM_Command(u8 *command)                       		// Отправка команд на GSM модуль
{
	time_t current_time;
	// Запоминаем время начала отсчета, для отслеживания времени ожидания
	

	clear_RXBuffer((u8*)GSM_DATA, GSM_BUF);
	gsm = 0;
	good_end_of_AT = 0;
	bad_end_of_AT = 0;
	sprintf(print_buffer, "%s\r", command);
	send_Uart_AT(USART1, (u8*)print_buffer); // Отправляем команду в USART
	clear_RXBuffer((u8 *)print_buffer, PRINT_BUF);
    current_time = tick_counter;
	while (tick_counter - current_time < waiting && !good_end_of_AT && !bad_end_of_AT); 


	if (good_end_of_AT) 
	{ 
		good_end_of_AT = 0;
		bad_end_of_AT = 0;
//		osDelay(1); 
		return SUCCESS; 
	}
	else
	{
		good_end_of_AT = 0;
		bad_end_of_AT = 0;
		Error_Handler();
		return ERROR;
	}
}

void Send_SMS(char *phone_number, char *text)								// Отправка СМС
{
	time_t current_time = 0;
	sprintf(print_buffer,"AT+CMGS=\"%s\"\r", phone_number);
	send_Uart_AT(USART1, (u8*)print_buffer); // Отправляем команду в USART
	clear_RXBuffer((u8 *)print_buffer, PRINT_BUF);
	current_time = tick_counter;
	
	while (tick_counter - current_time < 50 && !ready_to_send);
	ready_to_send = 0;
	
	sprintf(print_buffer, "%s%c", text, 0x1A);
	AT(print_buffer);
}

void Get_Ip_adress(void)                       								// Получение IP адреса
{
	time_t current_time;
	// Запоминаем время начала отсчета, для отслеживания времени ожидания
	
	clear_RXBuffer((u8*)GSM_DATA, GSM_BUF);
	
	gsm = 0;
	send_Uart_AT(USART1, (u8*)("AT+CIFSR\r")); // Отправляем команду в USART
	
    current_time = tick_counter;
	while (tick_counter - current_time < 200);
}

ErrorStatus Bring_Up_GPRS(void)                       							// Поднять GPRS соединения
{
	time_t current_time;
	// Запоминаем время начала отсчета, для отслеживания времени ожидания
	clear_RXBuffer((u8*)GSM_DATA, GSM_BUF);
	gsm = 0;
	good_end_of_AT = 0;
	bad_end_of_AT = 0;
	send_Uart_AT(USART1, (u8 *)"AT+CIICR\r"); // Отправляем команду в USART

	
    current_time = tick_counter;
	while ((tick_counter - current_time < 20000) && !bad_end_of_AT && !good_end_of_AT) ;

	
	if (good_end_of_AT) 
	{
		good_end_of_AT = 0;
		bad_end_of_AT = 0;		
		osDelay(200); 
		return SUCCESS; 
	}
	else 
	{
		good_end_of_AT = 0;
		bad_end_of_AT = 0;
		return ERROR;
	}
	
}



ErrorStatus GET_NET_STATUS(void)                       		// Отправка команд на GSM модуль
{
	time_t current_time;
	// Запоминаем время начала отсчета, для отслеживания времени ожидания
	clear_RXBuffer((u8*)GSM_DATA, GSM_BUF);
	gsm = 0;
	send_Uart_AT(USART1, (u8*)"AT+CIPSTATUS\r\n"); // Отправляем команду в USART
	
    current_time = tick_counter;
	while (tick_counter - current_time < waiting);
	
	return SUCCESS; 
}

void CIPSEND(u8 SERVERnum, u16 size, u8 *data, u32 timer)
{
	time_t current_time = tick_counter;
	while(tick_counter - current_time < 300 && SEND_OK_waiting);
	SEND_OK_waiting = 1;
//	SEND_FAIL = 1;
	sprintf(print_buffer, "AT+CIPSEND=%u,%u\r", SERVERnum, size);
	send_Uart_AT(USART1, (u8*)print_buffer); // Отправляем команду в USART
	clear_RXBuffer((u8 *)print_buffer, PRINT_BUF);
    current_time = tick_counter;
	
	while (tick_counter - current_time < 20 && !ready_to_send);
	ready_to_send = 0;
	current_time = tick_counter;
	send_Uart_SERV(USART1, data, size);
	while(tick_counter - current_time < timer && !SEND_OK)
	{
		if (SEND_FAIL)
		{
//			osDelay(1000);
//			sprintf(print_buffer, "AT+CIPSEND=%u,%u\r", SERVERnum, size);
//			send_Uart_AT(USART1, (u8*)print_buffer); // Отправляем команду в USART
//	
//			current_time = tick_counter;
//	
//			while (tick_counter - current_time < 50 && !ready_to_send);
//			ready_to_send = 0;
//			current_time = tick_counter;
//			send_Uart_SERV(USART1, data, size);
			SEND_FAIL = 0;
			break;
		}
			
	}
	
	SEND_OK = 0;
	SEND_OK_waiting = 0;
	good_end_of_AT = 0;
	bad_end_of_AT = 0;

}

u16 Get_MNC_code(void)
{

	u16 code = 0;
	
	while(	code != 25001 && 
			code != 25002 && 
			code != 25020 && 
			code != 25025 && 
			code != 25099)												// Ожидаем подключения к сети
	{
		clear_RXBuffer((u8 *)mnc_operator_array, 6);
		AT("AT+COPS?");															// Запрос данных оператора
		osDelay(10);
		gsm_at_parse(mnc_operator_array, GSM_DATA, "+COPS: 0,2,\"", "\"");		// Парсинг данных для вычисления MNC кода оператора
		code = char_to_int(mnc_operator_array);
		osDelay(100);
	}
	
	return code;
}

u8 Query_Signal_Quality(void)                                 				// Запрос качества сигнала
{
	char rssi_sim1_array[50];				// информация о значении силы сигнала
    // Запоминаем время начала отсчета, для отслеживания времени ожидания
    AT("AT+CSQ"); 							// Отправка команды на запрос качества сигнала

    gsm_at_parse(rssi_sim1_array, GSM_DATA, "+CSQ:", ","); // Парсинг данных качества сигнала

    // Сохраняем значения силы сигнала и частоты ошибок
    return char_to_int(rssi_sim1_array);
}


ErrorStatus GSM_conf(void)                                          						// Конфигурация GSM модуля
{
	AT("ATZ0");																	// Сброс по умолчанию
	AT("AT");																	// Проверка связи с SIM модулем
	AT("AT+IPR=230400");														// Проверка связи с SIM модулем
	
	Init_USART_1_high_speed();
	
	AT("AT");
	
	AT("AT+CIPSHUT");															// Разрываем IP соединения
	AT("ATE0");																	// Отключаем эхо в UART
	while(!AT("AT+CPIN?")); 													// Проверка готовности сим карты
	AT("AT+CMGF=1"); 															// Установка текстового формата SMS сообщений
	AT("AT+CSCS=\"GSM\"");														// Выбор 7 битного GSM алфавита
	AT("AT+DDET=1");															// Включить контроль обнаружения тонального сигнала		
	AT("AT+COPS=0,2");															// 0 - Автоматический выбор оператора, 2 - Показ MNC кода оператора
	
	mnc_operator = Get_MNC_code();					
	SIM1_rssi = Query_Signal_Quality();											// Данные сигнала в RSSI 	0: -115 dBm или меньше
																				// 							1: -111 dBm
																				// 							2:...30 -110... -54 dBm
																				// 							31: -52 dBm или больше
																				// 							99: не известна или не определена
	
	percent_signal = SIM1_rssi * 3;												// Данные сигнала в %
	
	
	
	AT("AT+CIPMODE=0");     													// Передача данных в ручном режиме - 0, в прозрачном режиме - 1
	AT("AT+CIPMUX=1");      													// Запуск Multi Ip соединения
	AT("AT+CIPHEAD=1");															// Добавить IP-Head в начале пакета в формате +RECEIVE,<id>,<data length>:
	AT("AT+CIPATS=0");															// Убрать таймер автоматической отправки
	AT("AT+CIPSRIP=0");															// Не показывать удаленный IP-адрес и порт при получении данных в формате +RECV FROM:<IP ADDRESS>:<PORT>
	
	AT("AT+CLIP=1");															// Показ номера телефона после RING при входящем звонке
	
    return SUCCESS; // Если все запросы прошли успешно, возвращаем флаг успешного конфигурирования 
}

void GPRS_Connection_ON(void)                  									// Включение GPRS соединения
{
	do
	{	

		AT("AT+CIPSHUT");														// Разрываем IP соединения		
		osDelay(1000);
//		GET_NET_STATUS();														// Запрос текущего статуса
			
		// Подключение к GPRS мобильного оператора
		switch (mnc_operator)
		{
			case 25001: // MTC
				sprintf(print_buffer, "AT+CSTT=\"%s\",\"%s\",\"%s\"", "internet.mts.ru", "mts", "mts");
				break;
			case 25002:	// Megafon
				sprintf(print_buffer, "AT+CSTT=\"%s\",\"%s\",\"%s\"", "internet", "gdata", "gdata");
				break;
			case 25020:	// Tele2
				sprintf(print_buffer, "AT+CSTT=\"%s\",\"%s\",\"%s\"", "internet.tele2.ru", "", "");
				break;
			case 25025:	// MTC
				sprintf(print_buffer, "AT+CSTT=\"%s\",\"%s\",\"%s\"", "internet.mts.ru", "mts", "mts");
				break;
			case 25099:	// Beeline
				sprintf(print_buffer, "AT+CSTT=\"%s\",\"%s\",\"%s\"", "internet.beeline.ru", "beeline", "beeline");
				break;	
		}
		
		AT(print_buffer);
		clear_RXBuffer((u8 *)print_buffer, PRINT_BUF);
		osDelay(3000);
//		GET_NET_STATUS(); 														// Запрос текущего статуса
	}
	while (Bring_Up_GPRS() != SUCCESS);          									// Поднять беспроводное соединение через GPRS
																				// В случае неудачи повторить процедуру установки связи
	
//	GET_NET_STATUS(); 															// Запрос текущего статуса

	Get_Ip_adress();          													// Получить локальный IP адрес

//	GET_NET_STATUS();															// Запрос текущего статуса
	
//	AT("AT+CIPPING=\"192.162.100.33\"");
//	while(1);
}
u8 error_count = 0;
ErrorStatus CONNECT_TO_SERVER(u8 id, char *address, char *port)                           						// Подключение к NTRIP серверу
{
	ErrorStatus result = ERROR;
	u8 isConOpened = 0;
	char buffer[255];
	u8 isGprsDeactNeed_local = 0;
	
	clear_RXBuffer((u8 *)buffer, 255);
	
	while (!result)
	{
		sprintf(buffer, "AT+CIPCLOSE=%u,1", id);
		AT(buffer);
		clear_RXBuffer((u8 *)buffer, 255);
		if (isGprsDeactNeed_local) GPRS_Connection_ON();
		isGprsDeactNeed_local = 0;			
																// Запуск TCP соединения
		sprintf(buffer, "AT+CIPSTART=%u,\"UDP\",\"%s\",%s", id, address, port);
				
		if (AT(buffer)) 
		{
			clear_RXBuffer((u8 *)buffer, 255);
			isConOpened = 1;
		}
		else
		{
			error_count++;
			if (error_count > 10)
			{
				isGprsDeactNeed_local = 1;
				error_count = 0;
				REMOTE_CONNECT_OK = 0;
				NTRIP_CONNECT_OK = 0;
				MODE_CONTROL = DISCONNECT_RC;
			}		
		}
		if (isConOpened)
		{
			MODE_CONTROL = IDLE_MODE;
			result = SUCCESS;
			error_count = 0;
			id ? (RECEIVE1_timer = tick_counter) : (RECEIVE0_timer = tick_counter);
			id ? (get_server2_timer = tick_counter) : (get_server1_timer = tick_counter);
			delta_timeout_SERVER = 0;
			break;
		}
			
		
	}
			
	return result;
}

ErrorStatus CONNECT_TO_NTRIP(u8 id, char *address, char *port, char *mount_point, char *login, char *password)                           						// Подключение к NTRIP серверу
{
	ErrorStatus result = ERROR;
	time_t current_time = 0;
	char buffer[255];
	char LoginPassword_base64[50];
	char size_array[4];
	u8 error_count = 0;
	
	
	clear_RXBuffer((u8 *)buffer, 255);
	clear_RXBuffer((u8 *)LoginPassword_base64, 50);
	clear_RXBuffer((u8 *)size_array, 4);
	
	sprintf(buffer, "%s:%s", login, password);
	base64_encode(buffer, LoginPassword_base64);
	clear_RXBuffer((u8 *)buffer, 255);
	
	while (!result)
	{
		received_rtk_bytes = 0;
		clear_RXBuffer((u8 *)size_of_RTK_parcel_arr, 4);
		
		if (isGprsDeactNeed) 
		{	osDelay(3000);
			AT("AT+CIPCLOSE=0,1");
			osDelay(1000);
			AT("AT+CIPCLOSE=1,1");
			osDelay(1000);
			GPRS_Connection_ON();
			isGprsDeactNeed = 0;
		}
		else
		{	
			osDelay(3000);
			sprintf(buffer, "AT+CIPCLOSE=%u,1", id);
			AT(buffer);
		}
		
		osDelay(3000);
		
		clear_RXBuffer((u8 *)buffer, 255);
		if (isGprsDeactNeed) continue;														// Запуск TCP соединения
		sprintf(buffer, "AT+CIPSTART=%u,\"TCP\",\"%s\",%s", id, address, port);
		
		if (AT(buffer))
		{
			if (isGprsDeactNeed) continue;
			clear_RXBuffer((u8 *)buffer, 255);
			osDelay(5000);
			// Отправка GET запроса на подключение к NTRIP кастеру как клиент
						
			sprintf(buffer, "GET /%s HTTP/1.1\r\nHost: %s\r\nNtrip-Version: Ntrip/2.0\r\nUser-Agent: NTRIPOEMComnavK700/2.0\r\nAuthorization: Basic %s\r\nConnection: Close\r\n\r\n", 
			mount_point, address, LoginPassword_base64);
			
			SUCCESS_NTRIP = 0;
			if (!isGprsDeactNeed) CIPSEND(0, strlen(buffer), (u8 *)buffer, 2000);
			current_time = tick_counter;
			
			while (tick_counter - current_time < 10000 && !isGprsDeactNeed)
			{
				if (SUCCESS_NTRIP) 
				{
					SUCCESS_NTRIP = 0;

					clear_RXBuffer((u8 *)size_of_RTK_parcel_arr, 4);
					
					good_end_of_AT = 0;
					bad_end_of_AT = 0;
					result = SUCCESS;
					osDelay(100);
					RECEIVE0_timer = tick_counter;
					delta_timeout_RTK = 0;
					get_server1_timer = tick_counter;
					error_count = 0;
					break;
				}
			}
			
		}
		else
		{
			error_count++;
			if (error_count > 2)
			{
				isGprsDeactNeed = 1;
				error_count = 0;
				REMOTE_CONNECT_OK = 0;
				NTRIP_CONNECT_OK = 0;
				MODE_CONTROL = DISCONNECT_RC;
			}				
		
		}
	}
			
	return result;
}


void StartUsartTask(void const * argument)
{
	USART_q Symbol;
	while(1)
	{
//		osDelay(1000);
		if (xQueueReceive(UsartQueueHandle, &Symbol, 0) == pdTRUE)
		{
			GSM_DATA[gsm++] = Symbol.data[0];
			if (gsm == GSM_BUF) gsm = 0;
			if (search_RECEIVE0)
			{

				if (Symbol.data[0] != ':' && size_of_RTK_parcel_cnt < 4) 
				{
					size_of_RTK_parcel_arr[size_of_RTK_parcel_cnt++] = Symbol.data[0];		
				}
				else
				{
					rtk_length = char_to_int(size_of_RTK_parcel_arr);
					if (rtk_length > NTRIP_BUF)
					{
						rtk_length = 0;
					}
					search_RECEIVE0 = 0;
					size_of_RTK_parcel_cnt = 0;
					clear_RXBuffer((u8 *)size_of_RTK_parcel_arr, 4);
				}
					
			}
			
			if (rtk_length)
			{
				received_rtk_bytes++;
				if (received_rtk_bytes > 3)
				{
					s2_buf_cnt++;
					if (NTRIP_CONNECT_OK) 
						send_Uart(USART2, *Symbol.data);
				}
				
				if (s2_buf_cnt == NTRIP_BUF) 
					s2_buf_cnt = 0;
						
				if (s2_buf_cnt == rtk_length)
				{
					rtk_length = 0;
					s2_buf_cnt = 0;
					received_rtk_bytes = 0;
					incData_from_SERVER1 = 1;
				}
			}
			

			
			if (search_DOLLAR)
			{
				
				switch (Symbol.data[0])
				{
					case (u8)PACK_ID_tNum: pack_length = PACK_LEN_tNum;
					break;
					case (u8)PACK_ID_Coords: pack_length = PACK_LEN_Coords;
					break;
					case (u8)PACK_ID_Mode: pack_length = PACK_LEN_Mode;
					break;
					case (u8)PACK_ID_RC: pack_length = PACK_LEN_RC;
					break;
					case (u8)PACK_ID_Err: pack_length = PACK_LEN_Err;
					break;
					case (u8)PACK_ID_ALIVE: pack_length = PACK_LEN_ALIVE;
					break;
				
					
					default: pack_length = 0;
				}
				search_DOLLAR = 0;
			}
			
			
			
			// Идентификатор посылки от 0 сервера
			switch (Symbol.data[0])
			{
				case '+' : search++; break;
				case 'R' : search == 1 ? (search++) : (search = 0); break;
				case 'E' : search == 2 || search == 4 || search == 7 ? (search++) : (search = 0); break;
				case 'C' : search == 3 ? (search++) : (search = 0); break;
				case 'I' : search == 5 ? (search++) : (search = 0); break;
				case 'V' : search == 6 ? (search++) : (search = 0); break;
				case ',' : search == 8 || search == 10 ? (search++) : (search = 0); break;
				case '0' : search == 9 ? (search++) : (search = 0); break;
				
				default: search = 0;
			}
			if (search == 11) 
			{ 
				search_RECEIVE0 = 1;			
				search = 0;
				RECEIVE0_timer = tick_counter;
			}
			// ************************************
			
			// Идентификатор посылки от 1 сервера
			switch (Symbol.data[0])
			{
				case '+' : search2++; break;
				case 'R' : search2 == 1 ? (search2++) : (search2 = 0); break;
				case 'E' : search2 == 2 || search2 == 4 || search2 == 7 ? (search2++) : (search2 = 0); break;
				case 'C' : search2 == 3 ? (search2++) : (search2 = 0); break;
				case 'I' : search2 == 5 ? (search2++) : (search2 = 0); break;
				case 'V' : search2 == 6 ? (search2++) : (search2 = 0); break;
				case ',' : search2 == 8 ? (search2++) : (search2 = 0); break;
				case '1' : search2 == 9 ? (search2++) : (search2 = 0); break;


				
				default: search2 = 0;
			}
			if (search2 == 10) 
			{ 
				search_RECEIVE1 = 1;
				search2 = 0;
				RECEIVE1_timer = tick_counter;
			}
			
			if (search_RECEIVE1)
			{
				if (Symbol.data[0] == 0x24)
				{	
					RECEIVE_SERVER_BUF[s1_buf_cnt++] = Symbol.data[0];
					search_DOLLAR = 1;
					search_RECEIVE1 = 0;
				}
			}
			
			if (pack_length)
			{
				RECEIVE_SERVER_BUF[s1_buf_cnt++] = Symbol.data[0];
				if (s1_buf_cnt == SERVER_BUF) 
				{
					s1_buf_cnt = 0;
					overflow++;
				}
				received_pack_bytes++;
				if (received_pack_bytes == pack_length)
				{
					incData_from_SERVER2 = 1;
					parcel_count++;		
					f5_debug++;
					calculated_time = tick_counter;
					search_DOLLAR = 0;
					received_pack_bytes = 0;
					pack_length = 0;
				}
			}
			// ************************************	

			// Идентификатор хорошего ответа
			switch (Symbol.data[0])
			{
				case 'O' : search_OK++; break;
				case 'K' : search_OK == 1 ? (search_OK++) : (search_OK = 0); break;
			
				default: search_OK = 0;
			}
			if (search_OK == 2)  
			{ 
				search_OK = 0;
				good_end_of_AT = 1;
			}
			// ************************************
			
			// Идентификатор хорошего ответа
			switch (Symbol.data[0])
			{
				case 'C' : search_CONNECT == 0 || search_CONNECT == 4 ? (search_CONNECT++) : (search_CONNECT = 0); break;
				case 'O' : search_CONNECT == 1 ? (search_CONNECT++) : (search_CONNECT = 0); break;
				case 'N' : search_CONNECT == 2 || search_CONNECT == 3 ? (search_CONNECT++) : (search_CONNECT = 0); break;
				case 'E' : search_CONNECT == 3 ? (search_CONNECT++) : (search_CONNECT = 0); break;
				case 'T' : search2 == 5 ? (search_CONNECT++) : (search_CONNECT = 0); break;
				
				default: search_CONNECT = 0;
			}
			// ************************************			
			if (search_CONNECT == 6) 
			{ 
				search_CONNECT = 0;
				good_end_of_AT = 1;
			}
			// ************************************	
			
			// Идентификатор хорошего ответа
			switch (Symbol.data[0])
			{
				case '>' : search_ARROW++; break;
				case ' ' : search_ARROW == 1 ? (search_ARROW++) : (search_ARROW = 0); break;
				
				default: search_ARROW = 0;
			}
			if (search_ARROW == 2) 
			{
				ready_to_send = 1;
				search_ARROW = 0;
			}
			// ************************************	
			
			// Идентификатор плохого ответа
			switch (Symbol.data[0])
			{
				case 'E' : search_ERROR++; break;
				case 'R' : search_ERROR == 1 || search_ERROR == 2 || search_ERROR == 4 ? (search_ERROR++) : (search_ERROR = 0); break;
				case 'O' : search_ERROR == 3 ? (search_ERROR++) : (search_ERROR = 0); break;
			
				default: search_ERROR = 0;
			}
			if (search_ERROR == 5) 
			{ 
				search_ERROR = 0;
				bad_end_of_AT = 1;
			}
			// ************************************
			
			// Идентификатор плохого ответа
			switch (Symbol.data[0])
			{
				case 'C' : search_CLOSED++; break;
				case 'L' : search_CLOSED == 1 ? (search_CLOSED++) : (search_CLOSED = 0); break;
				case 'O' : search_CLOSED == 2 ? (search_CLOSED++) : (search_CLOSED = 0); break;
				case 'S' : search_CLOSED == 3 ? (search_CLOSED++) : (search_CLOSED = 0); break;
				case 'E' : search_CLOSED == 4 ? (search_CLOSED++) : (search_CLOSED = 0); break;
				case 'D' : search_CLOSED == 5 ? (search_CLOSED++) : (search_CLOSED = 0); break;
				
				default: search_CLOSED = 0;
			}
			if (search_CLOSED == 6) 
			{ 
				server_CLOSED = 1;
				search_CLOSED = 0;
			}
			// ************************************	
			
			// Идентификатор удачной отправки
			switch (Symbol.data[0])
			{
				case 'S' : search_SEND_OK++; break;
				case 'E' : search_SEND_OK == 1 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				case 'N' : search_SEND_OK == 2 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				case 'D' : search_SEND_OK == 3 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				case ' ' : search_SEND_OK == 4 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				case 'O' : search_SEND_OK == 5 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				case 'K' : search_SEND_OK == 6 ? (search_SEND_OK++) : (search_SEND_OK = 0); break;
				
				default: search_SEND_OK = 0;
			}
			if (search_SEND_OK == 7) 
			{ 
				SEND_OK = 1; 
				search_SEND_OK = 0;
			}
			// ************************************
			
			// Идентификатор неудачной отправки
			switch (Symbol.data[0])
			{
				case 'S' : search_SEND_FAIL++; break;
				case 'E' : search_SEND_FAIL == 1 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'N' : search_SEND_FAIL == 2 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'D' : search_SEND_FAIL == 3 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case ' ' : search_SEND_FAIL == 4 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'F' : search_SEND_FAIL == 5 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'A' : search_SEND_FAIL == 6 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'I' : search_SEND_FAIL == 7 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				case 'L' : search_SEND_FAIL == 8 ? (search_SEND_FAIL++) : (search_SEND_FAIL = 0); break;
				
				default: search_SEND_FAIL = 0;
			}
			if (search_SEND_FAIL == 9) 
			{ 
				SEND_FAIL = 1; 
				search_SEND_FAIL = 0;
				FAIL_cnt++;
			}
			// ************************************
			
			// Идентификатор неудачной отправки
			switch (Symbol.data[0])
			{
				case 'H' : search_SUCCESS_NTRIP++; break;
				case 'T' : search_SUCCESS_NTRIP == 1 || search_SUCCESS_NTRIP == 2 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case 'P' : search_SUCCESS_NTRIP == 3 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case '/' : search_SUCCESS_NTRIP == 4 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case '1' : search_SUCCESS_NTRIP == 5 || search_SUCCESS_NTRIP == 7 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case '.' : search_SUCCESS_NTRIP == 6 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case ' ' : search_SUCCESS_NTRIP == 8 || search_SUCCESS_NTRIP == 12 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case '2' : search_SUCCESS_NTRIP == 9 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case '0' : search_SUCCESS_NTRIP == 10 || search_SUCCESS_NTRIP == 11 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case 'O' : search_SUCCESS_NTRIP == 13 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				case 'K' : search_SUCCESS_NTRIP == 14 ? (search_SUCCESS_NTRIP++) : (search_SUCCESS_NTRIP = 0); break;
				
				default: search_SUCCESS_NTRIP = 0;
			}
			if (search_SUCCESS_NTRIP == 15) 
			{ 
				SUCCESS_NTRIP = 1; 
				search_SUCCESS_NTRIP = 0;
			}
			// ************************************
			
			// Идентификатор посылки от 0 сервера
			switch (Symbol.data[0])
			{
				case 'R' : search_RING++; break;
				case 'I' : search_RING == 1 ? (search_RING++) : (search_RING = 0); break;
				case 'N' : search_RING == 2 ? (search_RING++) : (search_RING = 0); break;
				case 'G' : search_RING == 3 ? (search_RING++) : (search_RING = 0); break;

				
				default: search_RING = 0;
			}
			if (search_RING == 4) 
			{ 
				RING = 1;			
				search_RING = 0;
			}
			// ************************************
			
		}
		else osDelay(1);
	}
}
