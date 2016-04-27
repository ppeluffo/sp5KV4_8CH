/*
 * sp5KV4_8CH_tkGprs_ssOFF.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */


#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

static int gTR_A00(void);	// Subestado OFF
static int gTR_A01(void);
static int gTR_A02(void);
static int gTR_A03(void);
static int gTR_A04(void);
static int gTR_A05(void);
static int gTR_A06(void);
static int gTR_A07(void);
static int gTR_A08(void);
static int gTR_A09(void);
static int gTR_A10(void);
static int gTR_A11(void);
static int gTR_A12(void);
static int gTR_A13(void);
static int gTR_A14(void);
static int gTR_A15(void);

// Eventos locales al estado OFF.
typedef enum {
	a_ev_CTIMER_NOT_0 = 0,
	a_ev_P_TRYES_NOT_0,
	a_ev_Q_TRYES_NOT_0,
	a_ev_M_RSP_OK,		// La respuesta del modem es OK
	a_ev_WKM_SERVICE,
} t_ssOff_eventos;

#define ssOFF_EVENT_COUNT 6

//------------------------------------------------------------------------------------
/*
 *  FUNCIONES DEL ESTADO OFF:
 *  El modem esta apagado y sale hasta quedar prendido
 *
 */
//------------------------------------------------------------------------------------
void SM_off(void)
{
	// Maquina de estados del estado OFF.( MODEM APAGADO)

s08 a_eventos[ssOFF_EVENT_COUNT];
u08 i;

	// Inicializo la lista local de eventos.
	for ( i=0; i < ssOFF_EVENT_COUNT; i++ ) {
		a_eventos[i] = FALSE;
	}

	// Evaluo solo los eventos del estado OFF.
	if ( GPRS_counters.cTimer > 0 ) { a_eventos[a_ev_CTIMER_NOT_0] = TRUE; }
	// evPTRYES_IS_0
	if ( GPRS_counters.pTryes > 0 ) { a_eventos[a_ev_P_TRYES_NOT_0] = TRUE; }
	// evQTRYES_IS_0
	if ( GPRS_counters.qTryes > 0 ) { a_eventos[a_ev_Q_TRYES_NOT_0] = TRUE; }
	// ev_GPRSRSP_OK
	if ( GPRS_flags.modemResponse == MRSP_OK ) { a_eventos[a_ev_M_RSP_OK] = TRUE; }
	// ev_WKM_SERVICE
	if ( ( systemVars.wrkMode == WK_SERVICE ) || ( systemVars.wrkMode == WK_MONITOR_FRAME ) ) { a_eventos[a_ev_WKM_SERVICE] = TRUE; }

	// MSG_RELOAD
	if ( GPRS_flags.msgReload == TRUE ) {
		g_reloadConfig();
		return;
	}

	switch ( tkGprs_subState ) {
	case gSST_OFF_00:
		tkGprs_subState = gTR_A00();
		break;
	case gSST_OFF_01:
		if ( a_eventos[a_ev_WKM_SERVICE] ) {
			tkGprs_subState = gTR_A15();
		} else if ( a_eventos[a_ev_P_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_A04();
		} else {
			tkGprs_subState = gTR_A01();
		}
		break;
	case gSST_OFF_02:
		if ( a_eventos[a_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_A05();
		} else {
			tkGprs_subState = gTR_A06();
		}
		break;
	case gSST_OFF_03:
		if ( a_eventos[a_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_A07();
		} else {
			tkGprs_subState = gTR_A08();
		}
		break;
	case gSST_OFF_04:
		if ( a_eventos[a_ev_Q_TRYES_NOT_0] )  {
			tkGprs_subState = gTR_A09();
		} else {
			tkGprs_subState = gTR_A10();
		}
		break;
	case gSST_OFF_05:
		if ( a_eventos[a_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_A11();
		} else {
			tkGprs_subState = gTR_A12();
		}
		break;
	case gSST_OFF_06:
		if ( a_eventos[a_ev_M_RSP_OK] )  {
			tkGprs_subState = gTR_A14();
		} else {
			tkGprs_subState = gTR_A13();
		}
		break;
	case gSST_OFF_07:
		if ( a_eventos[a_ev_CTIMER_NOT_0] )  {
			tkGprs_subState = gTR_A02();
		} else {
			tkGprs_subState = gTR_A03();
		}
		break;
	default:
		snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("\r\ntkGprs::ERROR sst_off: subState  (%d) NOT DEFINED\r\n\0"),tkGprs_subState);
		FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
		tkGprs_state = gST_OFF;
		tkGprs_subState = gSST_OFF_00;
		break;
	}

}
/*------------------------------------------------------------------------------------*/
static int gTR_A00(void)
{

	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo el sistema aqui

	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	//
	GPRS_counters.pTryes = HW_TRYES;

	// Inicializamos las variables de trabajo.
	GPRS_flags.msgReload = FALSE;

	g_printExitMsg("A00\0");
	return(gSST_OFF_01);
}
//------------------------------------------------------------------------------------
static int gTR_A01(void)
{
	// Si no pude prender el modem luego de HWTRYES reintentos, voy
	// a este estado a esperar un nuevo ciclo.

	// Inicializo el contador de segundos para la cantidad a esperar para
	// comenzar a rediscar.
	GPRS_counters.cTimer = SECS_AWAIT_NEWCICLE;
	MODEM_HWpwrOff();

	g_printExitMsg("A01\0");
	return(gSST_OFF_07);

}
//------------------------------------------------------------------------------------
static int gTR_A02(void)
{

	// Espero 1 segundo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("A02\0");
	return(gSST_OFF_07);
}
//------------------------------------------------------------------------------------
static int gTR_A03(void)
{

	// Termine de esperar: vuelvo a rediscar
	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;
	//
	GPRS_counters.pTryes = HW_TRYES;

	// Inicializamos las variables de trabajo.
	GPRS_flags.msgReload = FALSE;

	g_printExitMsg("A03\0");
	return(gSST_OFF_01);

}
//------------------------------------------------------------------------------------
static int gTR_A04(void)
{

	// Apago el modem y espero que se descargue todo

	strncpy_P(systemVars.dlgIp, PSTR("000.000.000.000\0"),16);
	systemVars.csq = 0;
	systemVars.dbm = 0;

	--GPRS_counters.pTryes;
	//
	MODEM_HWpwrOff();
	//
	// Vamos a espera 5secs.
	GPRS_counters.cTimer = 5;
	//
	g_printExitMsg("A04\0");
	return(gSST_OFF_02);

}
//------------------------------------------------------------------------------------
static int gTR_A05(void)
{
	// Espero 1 segundo durante un ciclo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("A05\0");
	return(gSST_OFF_02);
}
//------------------------------------------------------------------------------------
static int gTR_A06(void)
{

	// Prendo el modem y espero que se estabilize la fuente
	MODEM_HWpwrOn();
	//
	// Vamos a espera 5secs.
	GPRS_counters.cTimer = 5;
	//
	g_printExitMsg("A06\0");
	return(gSST_OFF_03);

}
//------------------------------------------------------------------------------------
static int gTR_A07(void)
{
	// Espero 1 segundo durante un ciclo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("A07\0");
	return(gSST_OFF_03);
}
//------------------------------------------------------------------------------------
static int gTR_A08(void)
{
	// Inicializo el contador de reintentos de switchear el modem.
	// Se prende haciendo un toggle en el pin 14, que lo prende o lo apaga.

	GPRS_counters.qTryes = SW_TRYES;

	g_printExitMsg("A08\0");
	return(gSST_OFF_04);
}
//------------------------------------------------------------------------------------
static int gTR_A09(void)
{
	// Genero un toggle en el pin SW del modem

	--GPRS_counters.qTryes;

	// Genero el toggle
	MODEM_SWswitchHIGH();
	vTaskDelay( (portTickType)( 500 / portTICK_RATE_MS ) );
	MODEM_SWswitchLOW();
	//
	// Espero 10s
	GPRS_counters.cTimer = 10;

	g_printExitMsg("A09\0");
	return(gSST_OFF_05);
}
//------------------------------------------------------------------------------------
static int gTR_A10(void)
{
	// Reintente el max. toggles: vuelvo al inicio a apagarlo y prenderlo

	g_printExitMsg("A10\0");
	return(gSST_OFF_01);
}
//------------------------------------------------------------------------------------
static int gTR_A11(void)
{
	// Espero 1 segundo durante un ciclo

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
	--GPRS_counters.cTimer;

	//g_printExitMsg("A11\0");
	return(gSST_OFF_05);
}
//------------------------------------------------------------------------------------
static int gTR_A12(void)
{
	// Envio un comando AT y espero 1s para evaluar la respuesta

size_t pos;

	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_RX_BUFFER, NULL);
	FreeRTOS_ioctl( &pdUART0,ioctl_UART_CLEAR_TX_BUFFER, NULL);

	g_flushRXBuffer();
	FreeRTOS_write( &pdUART0, "AT\r\0", sizeof("AT\r\0") );

	vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );

	// Leo y Evaluo la respuesta al comando AT
	GPRS_flags.modemResponse = MRSP_NONE;
	if ( g_strstr("OK\0", &pos ) == TRUE ) {
		GPRS_flags.modemResponse = MRSP_OK;
	}

	// Muestro el resultado.
	g_printRxBuffer();

	g_printExitMsg("A12\0");
	return(gSST_OFF_06);
}
//------------------------------------------------------------------------------------
static int gTR_A13(void)
{
	// EL modem no respondio al comando AT.
	// Reintento prenderlo

	g_printExitMsg("A13\0");
	return(gSST_OFF_04);
}
//------------------------------------------------------------------------------------
static int gTR_A14(void)
{
	// El modem respondio: esta prendido.
	// Cambio de estado a gST_ONOFFLINE.

	tkGprs_state = gST_ONOFFLINE;

	g_printExitMsg("A14\0");
	return(gSST_ONOFFLINE_00);
}
//------------------------------------------------------------------------------------
static int gTR_A15(void)
{
	// El datalogger esta en modo service. No prendo el modem y espero

	GPRS_counters.cTimer = 60;

	g_printExitMsg("A15\0");
	return(gSST_OFF_07);
}
//------------------------------------------------------------------------------------
