/*
 * sp5KV4_8CH_tkGprs_ssOFFLINE.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

static int gTR_B00(void);
static int gTR_B01(void);
static int gTR_B02(void);
static int gTR_B03(void);
static int gTR_B04(void);
static int gTR_B05(void);
static int gTR_B06(void);
static int gTR_B07(void);
static int gTR_B08(void);
static int gTR_B09(void);
static int gTR_B10(void);
static int gTR_B11(void);
static int gTR_B12(void);
static int gTR_B13(void);
static int gTR_B14(void);
static int gTR_B15(void);
static int gTR_B16(void);
static int gTR_B17(void);
static int gTR_B18(void);
static int gTR_B19(void);
static int gTR_B20(void);
static int gTR_B21(void);
static int gTR_B22(void);

// Eventos locales al estado OnOFFLINE.
typedef enum {
	b_ev_GSMBAND_OK = 0,
	b_ev_CTIMER_NOT_0,
	b_ev_P_TRYES_NOT_0,
	b_ev_Q_TRYES_NOT_0,
	b_ev_CREGRSP_OK,
	b_ev_WKMONITOR_SQE,
	b_ev_IPASSIGNED
} t_ssOff_eventos;

#define ssON_EVENT_COUNT 7


//------------------------------------------------------------------------------------
/*
 *  FUNCIONES DEL ESTADO ONOFFLINE:
 *  El modem esta prendido y hay que configurarlo hasta lograr pedir una IP.
 *
 */
//------------------------------------------------------------------------------------
void SM_onOffline(void)
{
	// Maquina de estados del estado ONOFFLINE

s08 b_eventos[ssON_EVENT_COUNT];
u08 i;

	// Evaluo solo los eventos del estado OFF.
	// Inicializo la lista local de eventos.
	for ( i=0; i < ssON_EVENT_COUNT; i++ ) {
		b_eventos[i] = FALSE;
	}

	// ev_BAND_OK. NET gprsBand is correct.
	if ( GPRS_flags.gsmBandOK == TRUE  ) { b_eventos[b_ev_GSMBAND_OK] = TRUE; }
	// evCTIMER_IS_0
	if ( GPRS_counters.cTimer > 0 ) { b_eventos[b_ev_CTIMER_NOT_0] = TRUE; }
	// evPTRYES_IS_0
	if ( GPRS_counters.pTryes > 0 ) { b_eventos[b_ev_P_TRYES_NOT_0] = TRUE; }
	// evQTRYES_IS_0
	if ( GPRS_counters.qTryes > 0 ) { b_eventos[b_ev_Q_TRYES_NOT_0] = TRUE; }
	// ev_CREGRSP_OK		CREGrsp == +CREG 0,1
	if ( modem_response == MRSP_CREG ) { b_eventos[b_ev_CREGRSP_OK] = TRUE; }
	// ev_WKMONITOR_SQE
	if ( systemVars.wrkMode == WK_MONITOR_SQE ) { b_eventos[b_ev_WKMONITOR_SQE] = TRUE; }
	// ev_IPASSIGNED		E2IPA: 000
	if ( modem_response == MRSP_E2IPA ) { b_eventos[b_ev_IPASSIGNED] = TRUE; }

	switch ( tkGprs_subState ) {
	case gSST_ONOFFLINE_00:
		tkGprs_subState = gTR_B00();
		break;
	case gSST_ONOFFLINE_01:
		tkGprs_subState = gTR_B01();
		break;
	case gSST_ONOFFLINE_02:
		if ( b_eventos[b_ev_GSMBAND_OK] )  {
			tkGprs_subState = gTR_B03();
		} else {
			tkGprs_subState = gTR_B02();
		}
		break;
	case gSST_ONOFFLINE_03:
		if ( b_eventos[b_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_B04();
		} else {
			tkGprs_subState = gTR_B05();
		}
		break;
	case gSST_ONOFFLINE_04:
		if ( b_eventos[b_ev_CREGRSP_OK] )  {
			tkGprs_subState = gTR_B09();
		} else {
			tkGprs_subState = gTR_B06();
		}
		break;
	case gSST_ONOFFLINE_05:
		if ( b_eventos[b_ev_P_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_B07();
		} else {
			tkGprs_subState = gTR_B08();
		}
		break;
	case gSST_ONOFFLINE_06:
		if ( b_eventos[b_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_B10();
		} else {
			tkGprs_subState = gTR_B11();
		}
		break;
	case gSST_ONOFFLINE_07:
		if ( b_eventos[b_ev_WKMONITOR_SQE] )  {
			tkGprs_subState = gTR_B12();
		} else {
			tkGprs_subState = gTR_B13();
		}
		break;
	case gSST_ONOFFLINE_08:
		tkGprs_subState = gTR_B14();
		break;
	case gSST_ONOFFLINE_09:
		if ( b_eventos[b_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_B15();
		} else {
			tkGprs_subState = gTR_B16();
		}
		break;
	case gSST_ONOFFLINE_10:
		if ( b_eventos[b_ev_IPASSIGNED] )  {
			tkGprs_subState = gTR_B20();
		} else {
			tkGprs_subState = gTR_B17();
		}
		break;
	case gSST_ONOFFLINE_11:
		if ( b_eventos[b_ev_P_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_B19();
		} else {
			tkGprs_subState = gTR_B18();
		}
		break;
	case gSST_ONOFFLINE_12:
		if ( b_eventos[b_ev_Q_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_B21();
		} else {
			tkGprs_subState = gTR_B22();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_onoffline: subState  (%d) NOT DEFINED\r\n\0"),tkGprs_subState);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		tkGprs_state = gST_OFF;
		tkGprs_subState = gSST_OFF_Entry;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
static int gTR_B00(void)
{

	// Configuro el modem.

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	//
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS configure:\r\n\0"), u_now() );
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT&D0&C1\r\0", sizeof("AT&D0&C1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();


	// Configuro la secuencia de escape +++AT
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPS=2,8,2,1020,1,15\r\0", sizeof("AT*E2IPS=2,8,2,1020,1,15\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Envio: Los envio en modo texto
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CMGF=1\r\0", sizeof("AT+CMGF=1\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS Recepcion: No indico al TE ni le paso el mensaje
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT+CNMI=1,0\r\0", sizeof("AT+CNMI=1,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// SMS indicacion: Bajando el RI por 100ms.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2SMSRI=100\r\0", sizeof("AT*E2SMSRI=100\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Deshabilito los mensajes
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*E2IPEV=0,0\r\0", sizeof("AT*E2IPEV=0,0\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	g_printExitMsg("B00\0");
	return(gSST_ONOFFLINE_01);
}
//------------------------------------------------------------------------------------
static int gTR_B01(void)
{
	// Configuro la banda.

char bandBuffer[32];
char *ts = NULL;
u08 modemBand;
size_t xBytes = 0;

	// Vemos si la banda configurada es la correcta. Si no la reconfiguro.

	// Leo la banda que tiene el modem configurada
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT*EBSE?\r\0", sizeof("AT*EBSE?\r\0") );

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Extraigo de la respuesta la banda
	memcpy(bandBuffer, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(bandBuffer) );
	ts = strchr(bandBuffer, ':');
	ts++;
	modemBand = atoi(ts);

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("mBAND=%d,sBAND=%d\r\n\0"),modemBand, systemVars.gsmBand);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	GPRS_flags.gsmBandOK = TRUE;
	if ( modemBand != systemVars.gsmBand ) {
		// Debo reiniciar el modem
		GPRS_flags.gsmBandOK = FALSE;	// Para que luego el modem se resetee.

		// Reconfiguro.
		xBytes = snprintf_P( gprs_printfBuff,CHAR256,PSTR("AT*EBSE=%d\r\0"),systemVars.gsmBand );
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff));
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		// Guardo el profile
		FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
		g_flushRXBuffer();
		FreeRTOS_write( &pdUART0, "AT&W\r\0", sizeof("AT&W\r\0") );
		vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS Reconfiguro GSM_BAND a modo %d:\r\n\0"), u_now(),systemVars.gsmBand);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	}

	g_printExitMsg("B01\0");
	return(gSST_ONOFFLINE_02);
}
//------------------------------------------------------------------------------------
static int gTR_B02(void)
{
	// Debo reiniciar el modem para que tome la nueva banda

	tkGprs_state = gST_OFF;

	g_printExitMsg("B02\0");
	return(gSST_OFF_Entry);
}
//------------------------------------------------------------------------------------
static int gTR_B03(void)
{

	// Trato de atachearme a la red

	GPRS_counters.cTimer = 6;	// a intervalos de 6s entre consultas
	GPRS_counters.pTryes = 10;	// Pregunto hasta 10 veces

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	modem_response = MRSP_NONE;
	FreeRTOS_write( &pdUART0, "AT+CREG?\r\0", sizeof("AT+CREG?\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("B03\0");
	return(gSST_ONOFFLINE_03);
}
//------------------------------------------------------------------------------------
static int gTR_B04(void)
{
	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("B04\0");
	return(gSST_ONOFFLINE_03);
}
//------------------------------------------------------------------------------------
static int gTR_B05(void)
{
	// Chequeo la respuesta

size_t pos;

	// Leo y Evaluo la respuesta al comando AT+CREG ( home network )
	if ( g_strstr("+CREG: 0,1\0", &pos ) == TRUE ) {
		modem_response = MRSP_CREG;
	}
	//( roaming !!!. Se usa en Concordia )
	//if ( systemVars.roaming == TRUE ) {
	//	if (g_strstr("+CREG: 0,5\0", &pos ) == TRUE ) {
	//		GPRSrsp = RSP_CREG;
	//	}
	//}

	g_printRxBuffer();

	g_printExitMsg("B05\0");
	return(gSST_ONOFFLINE_04);
}
//------------------------------------------------------------------------------------
static int gTR_B06(void)
{

	g_printExitMsg("B06\0");
	return(gSST_ONOFFLINE_05);
}
//------------------------------------------------------------------------------------
static int gTR_B07(void)
{

	--GPRS_counters.pTryes;
	GPRS_counters.cTimer = 6;	// a intervalos de 6s entre consultas

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	FreeRTOS_write( &pdUART0, "AT+CREG?\r\0", sizeof("AT+CREG?\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("B07\0");
	return(gSST_ONOFFLINE_03);
}
//------------------------------------------------------------------------------------
static int gTR_B08(void)
{
	// Debo reiniciar el modem.

	tkGprs_state = gST_OFF;

	g_printExitMsg("B08\0");
	return(gSST_ONOFFLINE_01);
}
//------------------------------------------------------------------------------------
static int gTR_B09(void)
{

	// Leo el SQE
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: query SQE:\r\n\0"), u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	GPRS_counters.cTimer = 5;	// Espero 5s desde que doy el comando hasta que pregunto.

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	modem_response =  MRSP_NONE;
	FreeRTOS_write( &pdUART0, "AT+CSQ\r\0", sizeof("AT+CSQ\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("B09\0");
	return(gSST_ONOFFLINE_06);
}
//------------------------------------------------------------------------------------
static int gTR_B10(void)
{
	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("B10\0");
	return(gSST_ONOFFLINE_06);
}
//------------------------------------------------------------------------------------
static int gTR_B11(void)
{

	// Chequeo la respuesta al SQE

size_t pos;
char csqBuffer[32];
char *ts = NULL;

	if ( g_strstr("CSQ:\0", &pos ) == TRUE ) {

		g_printRxBuffer();

		memcpy(csqBuffer, g_getRxBuffer(), sizeof(csqBuffer) );
		ts = strchr(csqBuffer, ':');
		ts++;
		systemVars.csq = atoi(ts);
		systemVars.dbm = 113 - 2 * systemVars.csq;

		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\nCSQ=%d,DBM=%d\r\n\0"),systemVars.csq,systemVars.dbm);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	}

	g_printExitMsg("B11\0");
	return(gSST_ONOFFLINE_07);
}
//------------------------------------------------------------------------------------
static int gTR_B12(void)
{

	GPRS_counters.cTimer = 15;	// Repregunto c/15s.
	modem_response =  MRSP_NONE;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	FreeRTOS_write( &pdUART0, "AT+CSQ\r\0", sizeof("AT+CSQ\r\0") );
	vTaskDelay( (portTickType)( 100 / portTICK_RATE_MS ) );

	g_printExitMsg("B12\0");
	return(gSST_ONOFFLINE_06);
}
//------------------------------------------------------------------------------------
static int gTR_B13(void)
{

	// Fijo el APN

size_t xBytes;

	// APN
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS set APN:\r\n\0"), u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	xBytes = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT+CGDCONT=1,\"IP\",\"%s\"\r\0"),systemVars.apn);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);

	modem_response =  MRSP_NONE;
	g_flushRXBuffer();
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );
	vTaskDelay( (portTickType)( 1000 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	GPRS_counters.qTryes = 3;	// Pregunto hasta 3 la IP

	g_printExitMsg("B13\0");
	return(gSST_ONOFFLINE_08);
}
//------------------------------------------------------------------------------------
static int gTR_B14(void)
{
	// Pido la IP

	GPRS_counters.cTimer = 10;	// espero 10s antes de consultar
	GPRS_counters.pTryes = 6;	// Pregunto hasta 6 veces antes de reenviar el comando

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n[%s] GPRS ask IP:\r\n\0"), u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);

	GPRS_flags.modemResponse = MRSP_NONE;
	FreeRTOS_write( &pdUART0, "AT*E2IPA=1,1\r\0", sizeof("AT*E2IPA=1,1\r\0") );

	g_printExitMsg("B14\0");
	return(gSST_ONOFFLINE_09);
}
//------------------------------------------------------------------------------------
static int gTR_B15(void)
{
	// Espero 1 segundo
	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("B15\0");
	return(gSST_ONOFFLINE_09);
}
//------------------------------------------------------------------------------------
static int gTR_B16(void)
{

size_t pos;

	// Leo y Evaluo la respuesta al comando AT+CREG ( home network )
	if ( g_strstr("E2IPA: 000\0", &pos ) == TRUE ) {
		modem_response = MRSP_E2IPA;
	}

	g_printRxBuffer();

	g_printExitMsg("B16\0");
	return(gSST_ONOFFLINE_10);
}
//------------------------------------------------------------------------------------
static int gTR_B17(void)
{

	--GPRS_counters.pTryes;

	g_printExitMsg("B17\0");
	return(gSST_ONOFFLINE_11);
}
//------------------------------------------------------------------------------------
static int gTR_B18(void)
{

	--GPRS_counters.qTryes;

	g_printExitMsg("B18\0");
	return(gSST_ONOFFLINE_12);
}
//------------------------------------------------------------------------------------
static int gTR_B19(void)
{

	GPRS_counters.cTimer = 10;	// Reinicio cTimer

	g_printExitMsg("B19\0");
	return(gSST_ONOFFLINE_09);
}
//------------------------------------------------------------------------------------
static int gTR_B20(void)
{
	// Tengo la IP asignada: salgo

char *ts = NULL;
int i=0;
char c;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	g_flushRXBuffer();
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	FreeRTOS_write( &pdUART0, "AT*E2IPI=0\r\0", sizeof("AT*E2IPI=0\r\0") );
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	g_printRxBuffer();

	// Extraigo la IP del token. Voy a usar el buffer  de print ya que la respuesta
	// puede ser grande.
	memcpy(gprs_printfBuff, FreeRTOS_UART_getFifoPtr(&pdUART0), sizeof(gprs_printfBuff) );

	ts = strchr( gprs_printfBuff, '\"');
	ts++;
	while ( (c= *ts) != '\"') {
		systemVars.dlgIp[i++] = c;
		ts++;
	}
	systemVars.dlgIp[i++] = '\0';
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\nIPADDRESS=[%s]\r\n\0"),systemVars.dlgIp);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Cambio de estado
	GPRS_flags.socketStatus = SOCKET_CLOSED;
	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("B20\0");
	return(gSST_SOCKET_00);
}
//------------------------------------------------------------------------------------
static int gTR_B21(void)
{

	g_printExitMsg("B21\0");
	return(gSST_ONOFFLINE_08);
}
//------------------------------------------------------------------------------------
static int gTR_B22(void)
{
	// Apago.

	tkGprs_state = gST_OFF;

	g_printExitMsg("B22\0");
	return(gSST_OFF_Entry);
}
//------------------------------------------------------------------------------------

