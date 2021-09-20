#ifndef __sim800
#define __sim800

void GSM_Off(void);
void GSM_On(void) ;
void GSM_Restart(void);
ErrorStatus Write_GSM_Command(u8 *command);
void StartUsartTask(void const * argument);
void Init_USART_1(void);
void Init_USART_1_high_speed(void);
void Send_SMS(char *phone_number, char *text);
void USART1_IRQHandler(void);
void Init_GSM(void);
void Get_Ip_adress(void) ;
ErrorStatus Bring_Up_GPRS(void);
ErrorStatus GET_NET_STATUS(void);
void CIPSEND(u8 SERVERnum, u16 size, u8 *data, u32 timer);
ErrorStatus CONNECT_TO_NTRIP(u8 id, char *address, char *port, char *mount_point, char *login, char *password);
ErrorStatus CONNECT_TO_SERVER(u8 id, char *address, char *port) ;
void GPRS_Connection_ON(void);
ErrorStatus GSM_conf(void);
u8 Query_Signal_Quality(void);
u16 Get_MNC_code(void);
void send_Uart(USART_TypeDef* USARTx, u8 c);
void send_Uart_AT(USART_TypeDef* USARTx, u8 *s);
void send_Uart_SERV(USART_TypeDef* USARTx, u8 *s, u16 size) ;

#endif //__main
