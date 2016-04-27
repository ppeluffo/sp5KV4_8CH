/*
 * sp5KV4_8CH_ssDATA.c
 *
 *  Created on: 25 de abr. de 2016
 *      Author: pablo
 */

#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

static int gTR_D00(void);
static int gTR_D01(void);
static int gTR_D02(void);
static int gTR_D03(void);
static int gTR_D04(void);
static int gTR_D05(void);
static int gTR_D06(void);
static int gTR_D07(void);
static int gTR_D08(void);
static int gTR_D09(void);
static int gTR_D10(void);
static int gTR_D11(void);
static int gTR_D12(void);
static int gTR_D13(void);
static int gTR_D14(void);
static int gTR_D15(void);
static int gTR_D16(void);
static int gTR_D17(void);
static int gTR_D18(void);

// Eventos locales al estado DATA
typedef enum {
	d_ev_CTIMER_NOT_0 = 0,
	d_ev_MEM_RCD4TX,
	d_ev_SOCK_IS_OPEN,
	d_ev_SOCK_IS_CLOSED,
	d_ev_RSP_ERROR,
	d_ev_MORE_RCS4TX,
	d_ev_FRAME_CONFIRMED,
	d_ev_MORE_RCD4DEL,

} t_ssData_eventos;

#define ssDATA_EVENT_COUNT 8

//------------------------------------------------------------------------------------
/*
 *  FUNCIONES DEL ESTADO DATA:
 *  Se trasmite un frame al servidor.
 *
 */
//------------------------------------------------------------------------------------
void SM_Data(void)
{
	// Maquina de estados del estado SOCKET

s08 d_eventos[ssDATA_EVENT_COUNT];
u08 i;

	// Evaluo solo los eventos del estado OFF.
	// Inicializo la lista local de eventos.
	for ( i=0; i < ssDATA_EVENT_COUNT; i++ ) {
		d_eventos[i] = FALSE;
	}

	// evCTIMER_IS_0
	if ( GPRS_counters.cTimer > 0 ) { d_eventos[d_ev_CTIMER_NOT_0] = TRUE; }
	// ev_MEM_RCD4TX
	if ( GPRS_flags.memRcds4Tx == TRUE ) { d_eventos[d_ev_MEM_RCD4TX] = TRUE; }
	// ev_SOCK_IS_OPEN	CONNECT
	if ( GPRS_flags.socketStatus == SOCKET_OPEN ) { d_eventos[d_ev_SOCK_IS_OPEN] = TRUE; }
	// ev_SOCK_IS_CLOSED
	if ( GPRS_flags.socketStatus == SOCKET_CLOSED ) { d_eventos[d_ev_SOCK_IS_CLOSED] = TRUE; }
	// ev_RSP_ERROR
	if ( GPRS_flags.modemResponse == MRSP_ERROR ) { d_eventos[d_ev_RSP_ERROR] = TRUE; }
	// ev_MORE_RCS4TX
	if ( GPRS_flags.moreRcds4Tx == TRUE ) { d_eventos[d_ev_MORE_RCS4TX] = TRUE; }
	// ev_SRV_RSP_OK
	if ( GPRS_flags.modemResponse == MRSP_FRAMEOK ) { d_eventos[d_ev_FRAME_CONFIRMED] = TRUE; }
	// ev_MORE_RCD4DEL
	if ( GPRS_flags.memRcds4Del == TRUE ) { d_eventos[d_ev_MORE_RCD4DEL] = TRUE; }

	// MSG_RELOAD
	if ( GPRS_flags.msgReload == TRUE ) {
		void g_reloadConfig(void);
		return;
	}

	switch ( tkGprs_subState ) {
	case gSST_DATA_00:
		tkGprs_subState = gTR_D00();
		break;
	case gSST_DATA_01:
		tkGprs_subState = gTR_D01();
		break;
	case gSST_DATA_02:
		if ( d_eventos[d_ev_MEM_RCD4TX] )  {
			tkGprs_subState = gTR_D05();
		} else {
			tkGprs_subState = gTR_D02();
		}
		break;
	case gSST_DATA_03:
		if ( d_eventos[d_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_D03();
		} else {
			tkGprs_subState = gTR_D04();
		}
		break;
	case gSST_DATA_04:
		if ( d_eventos[d_ev_SOCK_IS_OPEN] )  {
			tkGprs_subState = gTR_D07();
		} else {
			tkGprs_subState = gTR_D06();
		}
		break;
	case gSST_DATA_05:
		tkGprs_subState = gTR_D08();
		break;
	case gSST_DATA_06:
		if ( d_eventos[d_ev_MORE_RCS4TX] )  {
			tkGprs_subState = gTR_D09();
		} else {
			tkGprs_subState = gTR_D10();
		}
		break;
	case gSST_DATA_07:
		tkGprs_subState = gTR_D11();
		break;
	case gSST_DATA_08:
		if ( d_eventos[d_ev_FRAME_CONFIRMED] )  {
			tkGprs_subState = gTR_D13();
		} else if ( d_eventos[d_ev_SOCK_IS_CLOSED] ) {
			tkGprs_subState = gTR_D16();
		} else if (d_eventos[d_ev_RSP_ERROR] ) {
			tkGprs_subState = gTR_D17();
		} else if ( d_eventos[d_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_D12();
		} else {
			tkGprs_subState = gTR_D18();
		}
		break;
	case gSST_DATA_09:
		if ( d_eventos[d_ev_MORE_RCD4DEL] )  {
			tkGprs_subState = gTR_D14();
		} else {
			tkGprs_subState = gTR_D15();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_data: subState  (%d) NOT DEFINED\r\n\0"),tkGprs_subState);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		tkGprs_state = gST_OFF;
		tkGprs_subState = gSST_OFF_00;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
static int gTR_D00(void)
{

	g_printExitMsg("D00\0");
	return(gSST_DATA_01);
}
//------------------------------------------------------------------------------------
static int gTR_D01(void)
{
	// Veo si hay registros en memoria para trasmitir.

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);

	GPRS_flags.memRcds4Tx = TRUE;
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS  ) {
		GPRS_flags.memRcds4Tx = FALSE;
	}

	g_printExitMsg("D01\0");
	return(gSST_DATA_02);
}
//------------------------------------------------------------------------------------
static int gTR_D02(void)
{
	// No hay datos para trsmitir. Espero 30s

	GPRS_counters.cTimer = 30;

	g_printExitMsg("D02\0");
	return(gSST_DATA_03);
}
//------------------------------------------------------------------------------------
static int gTR_D03(void)
{
	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("D03\0");
	return(gSST_DATA_03);
}
//------------------------------------------------------------------------------------
static int gTR_D04(void)
{

	// Vuelvo a chequear si hay datos para trasmitir.

	g_printExitMsg("D04\0");
	return(gSST_DATA_01);
}
//------------------------------------------------------------------------------------
static int gTR_D05(void)
{
	// Hay datos para trasmitir.

	g_printExitMsg("D05\0");
	return(gSST_DATA_04);;
}
//------------------------------------------------------------------------------------
static int gTR_D06(void)
{
	// El socket esta cerrado. Salgo para abrirlo

	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("D06\0");
	return(gSST_SOCKET_00);

}
//------------------------------------------------------------------------------------
static int gTR_D07(void)
{
	// El socket esta abierto. Trasmito el HEADER y me preparo para
	// comenzar a trasmitir datos.

u16 pos;
//StatBuffer_t pxFFStatBuffer;

	FF_seek(); // Ajusta la posicion del puntero de lectura al primer registro a leer
	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	// HEADER:
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);
	g_flushRXBuffer();

	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));
	pos = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("GET " ));
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s"), systemVars.serverScript );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("?DLGID=%s"), systemVars.dlgId );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&PASSWD=%s"), systemVars.passwd );
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// DebugMsg
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	GPRS_counters.txRcdsInWindow = FRAMEXTXWINDOW;	// Cantidad de registros maximo a trasmitir en una ventana

	// Imprimo stats de memoria
//	FF_stat(&pxFFStatBuffer);
//	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n[%s] GPRS SOW(%d) FSstat: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0"), u_now(),GPRS_counters.txRcdsInWindow,pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
//	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("D07\0");
	return(gSST_DATA_05);
}
//------------------------------------------------------------------------------------
static int gTR_D08(void)
{
	// Chequeo la condicion detener la trasmision de mas registros.

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n[%s] GPRS Wfr(%d) FSstat: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0"), u_now(),GPRS_counters.txRcdsInWindow,pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	GPRS_flags.moreRcds4Tx = TRUE;

	// 1 -La memoria esta vacia en absoluto ( Free = MAX )
	// o en lectura, es decir que lei todos los registros ocupados.
	//
	if ( pxFFStatBuffer.rcdsFree == FF_MAX_RCDS) {
		// Memoria vacia en absoluto
		GPRS_flags.moreRcds4Tx = FALSE;
	} else if ( pxFFStatBuffer.rcdsFree == 0 ) {
		// Memoria llena
		GPRS_flags.moreRcds4Tx = TRUE;
	} else if ( pxFFStatBuffer.RD == pxFFStatBuffer.HEAD ) {
		// Memoria con datos pero todos trasmitidos
		GPRS_flags.moreRcds4Tx = FALSE;
	}

	// 2 - Complete una window
	if ( --GPRS_counters.txRcdsInWindow == 0 )
		GPRS_flags.moreRcds4Tx = FALSE;


	g_printExitMsg("D08\0");
	return(gSST_DATA_06);
}
//------------------------------------------------------------------------------------
static int gTR_D09(void)
{
	// Agrego un recd. al frame

u16 pos;
u08 channel;
frameData_t Aframe;
StatBuffer_t pxFFStatBuffer;

	// Leo memoria
	FF_fread( &Aframe, sizeof(Aframe));
	FF_stat(&pxFFStatBuffer);

	// Siempre trasmito los datos aunque vengan papasfritas.
	// Armo el frame
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));

	// Indice de la linea
	pos = snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("&CTL=%d"), pxFFStatBuffer.RD );

	// Calidad del frame.
	//pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&ST=%d"), pxFFStatBuffer.errno );

	// Aqui indico si los datos leidos de memoria son correctos o hubo un error.
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("&LINE=") );
	// Fecha y hora
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ), PSTR("%02d%02d%02d,"),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );
	// Valores analogicos
	for ( channel = 0; channel < NRO_ANALOG_CHANNELS; channel++) {
		pos += snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("%s>%.2f,"),systemVars.aChName[channel],Aframe.analogIn[channel] );
	}
	// Datos digitales
	for ( channel = 0; channel < NRO_DIGITAL_CHANNELS; channel++ ) {
		pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR("%sP>%d"), systemVars.dChName[channel],Aframe.dIn.pulses[channel] );
		if ( channel != (NRO_DIGITAL_CHANNELS - 1) ) {
			pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR(","));
		}
	}

	// Trasmito por el modem.
	FreeRTOS_write( &pdUART0, gprs_printfBuff, pos );

	// Imprimo
	FreeRTOS_write( &pdUART1, "TX->{\0", sizeof("TX->{\0") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ), PSTR("\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
	// Agrego mem.stats
	if (pxFFStatBuffer.errno > 0 ) {
		snprintf_P( gprs_printfBuff,  sizeof(gprs_printfBuff), PSTR(" ERROR (%d) MEM(%d) [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.errno, GPRS_counters.txRcdsInWindow, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	} else {
		snprintf_P( gprs_printfBuff, sizeof(gprs_printfBuff), PSTR(" MEM(%d) [%d/%d/%d][%d/%d]\r\n\0"), GPRS_counters.txRcdsInWindow, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}

	g_printExitMsg("D09\0");
	return(gSST_DATA_05);
}
//------------------------------------------------------------------------------------
static int gTR_D10(void)
{

	// Trasmito la cola

u16 pos = 0;
//StatBuffer_t pxFFStatBuffer;

	// TAIL ( No mando el close) :
//	g_flushRXBuffer();
//	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	memset( gprs_printfBuff, '\0', sizeof(gprs_printfBuff));

	pos = snprintf_P( gprs_printfBuff, ( sizeof(gprs_printfBuff) - pos ),PSTR(" HTTP/1.1\n") );
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("Host: www.spymovil.com\n" ));
	pos += snprintf_P( &gprs_printfBuff[pos], ( sizeof(gprs_printfBuff) - pos ),PSTR("\n\n\0" ));

	// Trasmito
	FreeRTOS_write( &pdUART0, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// DebugMsg
	snprintf_P( &gprs_printfBuff[pos],( sizeof(gprs_printfBuff) - pos ),PSTR("\r\n\0" ));
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	// Solo espero hasta 10s la respuesta.
	GPRS_counters.cTimer = 10;

	g_printExitMsg("D10\0");
	return(gSST_DATA_07);
}
//------------------------------------------------------------------------------------
static int gTR_D11(void)
{
	// Chequeo la respuesta del server.

size_t pos;

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

//	GPRS_flags.modemResponse = MRSP_NONE;
	// Leo y Evaluo la respuesta del server al frame.
	if ( g_strstr("RX_OK\0", &pos ) == TRUE ) {
		GPRS_flags.modemResponse = MRSP_FRAMEOK;
	}

	g_printRxBuffer();

	g_printExitMsg("D11\0");
	return(gSST_DATA_08);
}
//------------------------------------------------------------------------------------
static int gTR_D12(void)
{

	--GPRS_counters.cTimer;

	//g_printExitMsg("D12\0");
	return(gSST_DATA_07);
}
//------------------------------------------------------------------------------------
static int gTR_D13(void)
{
	// Respuesta RX_OK del servidor.

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);
	if ( FCB.ff_stat.rcds4del > 0 ) {
		GPRS_flags.memRcds4Del = TRUE;
	} else {
		GPRS_flags.memRcds4Del = FALSE;
	}


	g_printExitMsg("D13\0");
	return(gSST_DATA_09);
}
//------------------------------------------------------------------------------------
static int gTR_D14(void)
{

StatBuffer_t pxFFStatBuffer;

	FF_del();
	FF_stat(&pxFFStatBuffer);

	if ( FCB.ff_stat.rcds4del > 0 ) {
		GPRS_flags.memRcds4Del = TRUE;
	} else {
		GPRS_flags.memRcds4Del = FALSE;
	}

	tickCount = xTaskGetTickCount();
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("FSstat: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0"),pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );

	g_printExitMsg("D14\0");
	return(gSST_DATA_09);
}
//------------------------------------------------------------------------------------
static int gTR_D15(void)
{
	// Luego de trasmitir y borrar un window muestro el fstat

StatBuffer_t pxFFStatBuffer;

	FF_stat(&pxFFStatBuffer);
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\n[%s] GPRS FSstat: [wrPtr=%d,rdPtr=%d,delPtr=%d][Free=%d,4del=%d]\r\n\0"), u_now(),pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("D15\0");
	return(gSST_SOCKET_00);
}
//------------------------------------------------------------------------------------
static int gTR_D16(void)
{
	// El socket se cerro sin llegar la respuesta

	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("D16\0");
	return(gSST_SOCKET_00);
}
//------------------------------------------------------------------------------------
static int gTR_D17(void)
{

	// Frame trasmitido OK y memoria actualizada.

	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("D17\0");
	return(gSST_SOCKET_00);
}
//------------------------------------------------------------------------------------
static int gTR_D18(void)
{

	// Frame trasmitido OK y memoria actualizada.

	tkGprs_state = gST_ONOPENSOCKET;

	g_printExitMsg("D18\0");
	return(gSST_SOCKET_00);
}
//------------------------------------------------------------------------------------

