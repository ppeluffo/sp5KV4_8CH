/*
 * sp5KV4_8CH_tkGprs_socket.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

static int gTR_C00(void);
static int gTR_C01(void);
static int gTR_C02(void);
static int gTR_C03(void);
static int gTR_C04(void);
static int gTR_C05(void);
static int gTR_C06(void);
static int gTR_C07(void);
static int gTR_C08(void);
static int gTR_C09(void);
static int gTR_C10(void);
static int gTR_C11(void);
static int gTR_C12(void);
static int gTR_C13(void);
static int gTR_C14(void);

// Eventos locales al estado SOCKET
typedef enum {
	c_ev_CTIMER_NOT_0 = 0,
	c_ev_P_TRYES_NOT_0,
	c_ev_Q_TRYES_NOT_0,
	c_ev_SOCK_IS_OPEN,
	c_ev_SOCK_ERROR

} t_ssSock_eventos;

#define ssSOCK_EVENT_COUNT 5


//------------------------------------------------------------------------------------
/*
 *  FUNCIONES DEL ESTADO OPENSOCKET:
 *  Se abre un socket TCP hacia el servidor.
 *
 */
//------------------------------------------------------------------------------------
void SM_onOpenSocket(void)
{
	// Maquina de estados del estado SOCKET

s08 c_eventos[ssSOCK_EVENT_COUNT];
u08 i;

	// Evaluo solo los eventos del estado OFF.
	// Inicializo la lista local de eventos.
	for ( i=0; i < ssSOCK_EVENT_COUNT; i++ ) {
		c_eventos[i] = FALSE;
	}

	// evCTIMER_IS_0
	if ( GPRS_counters.cTimer > 0 ) { c_eventos[c_ev_CTIMER_NOT_0] = TRUE; }
	// evPTRYES_IS_0
	if ( GPRS_counters.pTryes > 0 ) { c_eventos[c_ev_P_TRYES_NOT_0] = TRUE; }
	// evQTRYES_IS_0
	if ( GPRS_counters.qTryes > 0 ) { c_eventos[c_ev_Q_TRYES_NOT_0] = TRUE; }
	// ev_SOCK_IS_OPEN	CONNECT
	if ( GPRS_flags.socketStatus == SOCKET_OPEN ) { c_eventos[c_ev_SOCK_IS_OPEN] = TRUE; }
	// ev_SOCK_ERROR
	if ( GPRS_flags.modemResponse == MRSP_ERROR ) { c_eventos[c_ev_SOCK_ERROR] = TRUE; }
	//
	// MSG_RELOAD
	if ( GPRS_flags.msgReload == TRUE ) {
		void g_reloadConfig(void);
		return;
	}

	switch ( tkGprs_subState ) {
	case gSST_SOCKET_00:
		tkGprs_subState = gTR_C00();
		break;
	case gSST_SOCKET_01:
		if ( c_eventos[c_ev_SOCK_IS_OPEN] )  {
			tkGprs_subState = gTR_C01();
		} else {
			tkGprs_subState = gTR_C04();
		}
		break;
	case gSST_SOCKET_02:
		if ( c_eventos[c_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_C03();
		} else {
			tkGprs_subState = gTR_C02();
		}
		break;
	case gSST_SOCKET_03:
		tkGprs_subState = gTR_C05();
		break;
	case gSST_SOCKET_04:
		if ( c_eventos[c_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_C06();
		} else {
			tkGprs_subState = gTR_C07();
		}
		break;
	case gSST_SOCKET_05:
		if ( c_eventos[c_ev_SOCK_IS_OPEN] )  {
			tkGprs_subState = gTR_C13();
		} else if ( c_eventos[c_ev_SOCK_ERROR] ) {
			tkGprs_subState = gTR_C14();
		} else {
			tkGprs_subState = gTR_C08();
		}
		break;
	case gSST_SOCKET_06:
		if ( c_eventos[c_ev_P_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_C11();
		} else {
			tkGprs_subState = gTR_C09();
		}
		break;
	case gSST_SOCKET_07:
		if ( c_eventos[c_ev_Q_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_C12();
		} else {
			tkGprs_subState = gTR_C10();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_socket: subState  (%d) NOT DEFINED\r\n\0"),tkGprs_subState);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		tkGprs_state = gST_OFF;
		tkGprs_subState = gSST_OFF_00;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
static int gTR_C00(void)
{

	GPRS_counters.cTimer = 30;	// Espero hasta 30s que el socket este cerrado.

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n%s: GPRS open SOCKET:\r\n\0"), u_now());
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Cierro el socket por las dudas.
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	// Paso primero a modo comando
	FreeRTOS_write( &pdUART0, "+++AT\r\0", sizeof("+++AT\r\0") );
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );
	// Y mando el comando de cerrar el socket. Si esta cerrado va a responder ERROR pero no importa
	FreeRTOS_write( &pdUART0, "AT*E2IPC=1\r\0", sizeof("AT*E2IPC=1\r\0") );
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	// No muestro la respuesta del modem ya que si esta cerrado va a indicar ERROR y
	// puede confundir al operador.
	//g_printRxBuffer();

	g_printExitMsg("C00\0");
	return(gSST_SOCKET_01);
}
//------------------------------------------------------------------------------------
static int gTR_C01(void)
{

	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("C02\0");
	return(gSST_SOCKET_02);
}
//------------------------------------------------------------------------------------
static int gTR_C02(void)
{

	// No cerro en 30s salgo a apagar el modem.

	tkGprs_state = gST_OFF;

	g_printExitMsg("C02\0");
	return(gSST_OFF_00);
}
//------------------------------------------------------------------------------------
static int gTR_C03(void)
{
	//g_printExitMsg("C03\0");
	return(gSST_SOCKET_01);
}
//------------------------------------------------------------------------------------
static int gTR_C04(void)
{

	GPRS_counters.qTryes = 3;	// Reintento 3 veces abrir el socket.

	g_printExitMsg("C04\0");
	return(gSST_SOCKET_03);
}
//------------------------------------------------------------------------------------
static int gTR_C05(void)
{
	// Envio el comando AT para abrir el socket

size_t xBytes;

	GPRS_counters.cTimer = 5;	// Consulto c/5s si abrio el socket
	GPRS_counters.pTryes = 6;	// Lo hago 6 veces.

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	GPRS_flags.modemResponse  = MRSP_NONE;
	xBytes = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("AT*E2IPO=1,\"%s\",%s\r\n\0"),systemVars.serverAddress,systemVars.serverPort);
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Debug
	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs: OPEN SOCKET (%d)\r\n\0"),tickCount, GPRS_counters.qTryes );
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("C05\0");
	return(gSST_SOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_C06(void)
{
	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("C06\0");
	return(gSST_SOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_C07(void)
{
	// Leo y Evaluo la respuesta al comando AT*E2IPO ( open socket )
	// La respuesta correcta debe ser CONNECT
	// La evalua la tarea tkGprsRX !!! pero de todos modos lo confirmo aqui.

size_t pos;

	GPRS_flags.modemResponse  = MRSP_NONE;

	if ( g_strstr("CONNECT\0", &pos ) == TRUE ) {
		GPRS_flags.modemResponse = MRSP_CREG;
		GPRS_flags.socketStatus = SOCKET_OPEN;
	}

	if ( g_strstr("ERROR\0", &pos ) == TRUE ) {
		GPRS_flags.modemResponse = MRSP_ERROR;
		GPRS_flags.socketStatus = SOCKET_CLOSED;
	}

	g_printRxBuffer();

	g_printExitMsg("C07\0");
	return(gSST_SOCKET_05);
}
//------------------------------------------------------------------------------------
static int gTR_C08(void)
{

	--GPRS_counters.pTryes;

	g_printExitMsg("C08\0");
	return(gSST_SOCKET_06);
}
//------------------------------------------------------------------------------------
static int gTR_C09(void)
{

	--GPRS_counters.qTryes;

	g_printExitMsg("C09\0");
	return(gSST_SOCKET_07);
}
//------------------------------------------------------------------------------------
static int gTR_C10(void)
{

	// Salgo a apagar el modem.

	tkGprs_state = gST_OFF;

	g_printExitMsg("C10\0");
	return(gSST_OFF_00);
}
//------------------------------------------------------------------------------------
static int gTR_C11(void)
{
	// Espero otros 5s
	GPRS_counters.cTimer = 5;

	g_printExitMsg("C11\0");
	return(gSST_SOCKET_04);
}
//------------------------------------------------------------------------------------
static int gTR_C12(void)
{
	// Vuelvo a reintentar abrir el socket
	g_printExitMsg("C12\0");
	return(gSST_SOCKET_03);
}
//------------------------------------------------------------------------------------
static int gTR_C13(void)
{
	// El socket abrio: cambio de estado.

	tkGprs_state = gST_DATA;

	g_printExitMsg("C13\0");
	return(gSST_DATA_00);
}
//------------------------------------------------------------------------------------
static int gTR_C14(void)
{
	// El socket indico ERROR.

	--GPRS_counters.qTryes;

	g_printExitMsg("C14\0");
	return(gSST_SOCKET_07);
}
//------------------------------------------------------------------------------------



