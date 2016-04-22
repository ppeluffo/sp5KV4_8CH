/*
 * sp5KV3_tkAnalogIn.c
 *
 *  Created on: 14/4/2015
 *      Author: pablo
 */


#include "sp5KV4_8CH.h"

static char aIn_printfBuff[CHAR256];
TimerHandle_t pollingTimer;

// Estados
typedef enum {	tkdST_INIT 		= 0,
				tkdST_STANDBY	= 1,
				tkdST_STATE1	= 2,
				tkdST_POLL01	= 3,
				tkdST_POLL02	= 4,
				tkdST_POLL03	= 5
} t_tkData_state;

// Eventos
typedef enum {
	evINIT = 0, 			// Init
	evRELOADCONFIG,			// EV_f_reload_IS_TRUE
	evMSGFORCEPOLL,			// EV_msg_POLL
	evTOPOLLTIMER,			// EV_TO_pollTimer
	evWRKMODENORMAL,		// EV_wrkMode_NORMAL
	evNROPOLEOSNOT0			// EV_nroPoleos_NOT_0
} t_tkData_eventos;

#define dEVENT_COUNT		6

static s08 dEventos[dEVENT_COUNT];

// transiciones
static int trD00(void);
static int trD01(void);
static int trD02(void);
static int trD03(void);
static int trD04(void);
static int trD05(void);
static int trD06(void);
static int trD07(void);
static int trD08(void);
static int trD09(void);

static u08 tkAIN_state = tkdST_INIT;		// Estado
static u32 tickCount;						// para usar en los mensajes del debug.
static double rAIn[NRO_ANALOG_CHANNELS];	// Almaceno los datos de conversor A/D
static frameData_t Aframe;

static struct {
	s08 starting;			// flag que estoy arrancando
	s08 msgReload;			// flags de los mensajes recibidos.
	s08 msgForcePoll;		// flags de los forzar un poleo.
	s08 TOpollTimer;		// time out del timer de poleo.
} AN_flags;

#define CICLOS_POLEO		3		// ciclos de poleo para promediar.

static struct {
	s16 secs2poll;			// contador de segundos hasta el siguiente poleo ( habilitacion de la flag )
	s08 nroPoleos;			// Contador del nro de poleos
} AN_counters;

// Funciones generales
static void pv_AINgetNextEvent(void);
static void pv_AINfsm(void);
static void pv_AINprintExitMsg(u08 code);
static void pv_AinLoadParameters( void );
void pv_pollTimerCallback( TimerHandle_t pxTimer );

//--------------------------------------------------------------------------------------
 void tkAnalogIn(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("starting tkAnalogIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

	tkAIN_state = tkdST_INIT;		// Estado inicial.
	AN_flags.starting = TRUE;		// Evento inicial ( arranque ).
	AN_flags.msgReload = FALSE;		// No tengo ningun mensaje de reload pendiente.
	AN_flags.msgForcePoll = FALSE;

	// Arranco el timer de poleo.
	if ( xTimerStart( pollingTimer, 0 ) != pdPASS )
		u_panic(P_AIN_TIMERSTART);

	//
	for( ;; )
	{

		u_clearWdg(WDG_AIN);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {

			if ( ( ulNotifiedValue & TKA_PARAM_RELOAD ) != 0 ) {
				// Mensaje de reload configuration.
				AN_flags.msgReload = TRUE;
			}

			if ( ( ulNotifiedValue & TKA_READ_FRAME ) != 0 ) {
				// Mensaje de polear un frame ( estando en modo servicio )
				AN_flags.msgForcePoll = TRUE;
			}
		}

		// Analizo los eventos.
		pv_AINgetNextEvent();
		// Corro la maquina de estados.
		pv_AINfsm();
	}

}
/*------------------------------------------------------------------------------------*/
void tkAnalogInit(void)
{
	// Esta funcion se utiliza  antes de arrancar el FRTOS de modo que cree
	// el timer que necesitamos en este modulo
	// Expira c/1sec

	pollingTimer = xTimerCreate (  "POLL_T",
	                     /* The timer period in ticks, must be greater than 0. */
	                     ( 1000 / portTICK_PERIOD_MS) ,
	                     /* The timers will auto-reload themselves when they expire. */
	                     pdTRUE,
	                     /* Assign each timer a unique id equal to its array index. */
	                     ( void * ) NULL,
	                     /* Each timer calls the same callback when it expires. */
						 pv_pollTimerCallback
	                   );

	if ( pollingTimer == NULL )
		u_panic(P_OUT_TIMERCREATE);
}
//------------------------------------------------------------------------------------
void pv_pollTimerCallback( TimerHandle_t pxTimer )
{
	// Como el timer esta en reload c/1 sec, aqui contamos los secs para polear
	// en la variable secs2poll.
	AN_counters.secs2poll--;

	if ( AN_counters.secs2poll <= 0 ) {	 	// programacion defensiva: uso <=

		// Habilito un nuevo poleo
		AN_flags.TOpollTimer = TRUE;

		// Reajusto el timer
		if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
			AN_counters.secs2poll = 15;
		} else {
			AN_counters.secs2poll = systemVars.timerPoll;
		}

	}
}
//--------------------------------------------------------------------------------------
static void pv_AINgetNextEvent(void)
{
// Evaluo todas las condiciones que generan los eventos que disparan las transiciones.
// Tenemos un array de eventos y todos se evaluan.

u08 i;

	// Inicializo la lista de eventos.
	for ( i=0; i < dEVENT_COUNT; i++ ) {
		dEventos[i] = FALSE;
	}

	// Evaluo los eventos
	// EV00: INIT
	if ( AN_flags.starting == TRUE ) { dEventos[evINIT] = TRUE; }

	// EV01: EV_MSGreload:
	if ( AN_flags.msgReload == TRUE ) { dEventos[evRELOADCONFIG] = TRUE;	}

	// EV02: EV_msg_POLL:
	if ( AN_flags.msgForcePoll == TRUE ) { dEventos[evMSGFORCEPOLL] = TRUE;	}

	// EV03: EV_f_start2poll
	if ( AN_flags.TOpollTimer == TRUE ) { dEventos[evTOPOLLTIMER] = TRUE; }

	// EV04: EV_wrkMode_NORMAL
	if ( systemVars.wrkMode == WK_NORMAL ) { dEventos[evWRKMODENORMAL] = TRUE; }

	// EV05: EV_nroPoleos_NOT_0
	if ( AN_counters.nroPoleos != 0 ) { dEventos[evNROPOLEOSNOT0] = TRUE; }

}
/*------------------------------------------------------------------------------------*/
static void pv_AINfsm(void)
{
	// El manejar la FSM con un switch por estado y no por transicion me permite
	// priorizar las transiciones.
	// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
	//

	switch ( tkAIN_state ) {
	case tkdST_INIT:
		tkAIN_state = trD00();	// TR00
		break;

	case tkdST_STANDBY:
		if ( dEventos[evRELOADCONFIG] == TRUE  ) { tkAIN_state = trD01();break; }
		if ( dEventos[evTOPOLLTIMER] == TRUE  ) { tkAIN_state = trD02();break; }
		if ( dEventos[evMSGFORCEPOLL] == TRUE  ) { tkAIN_state = trD03();break; }
		break;

	case tkdST_STATE1:
		if ( dEventos[evWRKMODENORMAL] == TRUE  ) {
			tkAIN_state = trD04();
		} else {
			tkAIN_state = trD05();
		}
		break;

	case tkdST_POLL01:
		tkAIN_state = trD06();
		break;

	case tkdST_POLL02:
		if ( dEventos[evNROPOLEOSNOT0] == TRUE  ) { tkAIN_state = trD07();break; }
		tkAIN_state = trD08();
		break;

	case tkdST_POLL03:
		tkAIN_state = trD09();
		break;


	default:
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("tkAnalogIn::ERROR state NOT DEFINED..\r\n\0"));
		FreeRTOS_write( &pdUART1, aIn_printfBuff,sizeof(aIn_printfBuff) );
		tkAIN_state  = tkdST_INIT;
		break;

	}
}
/*------------------------------------------------------------------------------------*/
static int trD00(void)
{
	// Evento inicial. Solo salta al primer estado operativo.
	// Inicializo el sistema aqui
	// tkdST_INIT->tkdST_STANDBY

	// Init (load parameters) & start pollTimer
	AN_flags.starting = FALSE;

	pv_AinLoadParameters();

	// En 15s hago un poleo.
	AN_counters.secs2poll = 15;

	pv_AINprintExitMsg(0);
	return(tkdST_STANDBY);
}
/*------------------------------------------------------------------------------------*/
static int trD01(void)
{
	// MSG de autoreload
	// tkdST_STANDBY->tkdST_STANDBY

	AN_flags.msgReload = FALSE;

	// Init (load parameters) & start pollTimer
	pv_AinLoadParameters();

	// En 15s hago un poleo.
	AN_counters.secs2poll = 15;

	pv_AINprintExitMsg(1);
	return(tkdST_STANDBY);
}
/*------------------------------------------------------------------------------------*/
static int trD02(void)
{
	// tkdST_STANDBY->tkdST_STATE1

	// Borro la falg para quedar armado para el siguiente ciclo
	AN_flags.TOpollTimer = FALSE;

	pv_AINprintExitMsg(2);
	return(tkdST_STATE1);
}
/*------------------------------------------------------------------------------------*/
static int trD03(void)
{
	// tkdST_STANDBY -> tkdST_POLL01

	//AN_flags.msgForcePoll = FALSE;
	// Todavia no lo borro asi discrimino al mostrar el mensaje

	pv_AINprintExitMsg(3);
	return(tkdST_POLL01);

}
/*------------------------------------------------------------------------------------*/
static int trD04(void)
{
	// tkdST_STANDBY -> tkdST_POLL01

	pv_AINprintExitMsg(4);
	return(tkdST_POLL01);

}
/*------------------------------------------------------------------------------------*/
static int trD05(void)
{
	// tkdST_STANDBY -> tkdST_POLL01

	pv_AINprintExitMsg(4);
	return(tkdST_POLL01);

}
/*------------------------------------------------------------------------------------*/
static int trD06(void)
{
	// tkdST_POLL01 -> tkdST_POLL02

	// Inicializa las variables para el poleo
u08 i = 0;

	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		rAIn[i] = 0;
	}

	AN_counters.nroPoleos = CICLOS_POLEO;

	pv_AINprintExitMsg(6);
	return(tkdST_POLL02);

}
/*------------------------------------------------------------------------------------*/
static int trD07(void)
{
	// tkdST_POLL02->tkdST_POLL02
	// Poleo

u16 adcRetValue;
s08 retS;
u08 i;

	// El tiempo entre poleos es de 1s
	if ( AN_counters.nroPoleos > 0 ) {
		vTaskDelay( ( TickType_t)( 1000 / portTICK_RATE_MS ) );
		AN_counters.nroPoleos--;
	}

	//
	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::trD05:\r\n\0"), tickCount);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );

	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		retS = ADS7828_read(i, &adcRetValue);
		rAIn[i] += adcRetValue;
		snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("\tch%02d,val=%d,r%02d=%.0f\r\n\0"),i,adcRetValue,i, rAIn[i]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	pv_AINprintExitMsg(7);
	return(tkdST_POLL02);

}
/*------------------------------------------------------------------------------------*/
static int trD08(void)
{
	// tkdST_POLL02->tkdST_POLL03
	// Promedio, convierto a magnitudes.
	// Si estoy en modo normal, guardo en memoria.

double I,M;
u16 D;
u08 i;
size_t bWrite;
StatBuffer_t pxFFStatBuffer;

	// Promedio
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		rAIn[i] /= CICLOS_POLEO;
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD08 AvgCh[%d]=%.02f\r\n\0"), tickCount, i, rAIn[i]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}

	// Convierto de ADC a magnitudes.
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		// Calculo la corriente medida en el canal
		I = rAIn[i] * systemVars.Imax[i] / 4096;
		// Calculo la pendiente
		M = 0;
		D = systemVars.Imax[i] - systemVars.Imin[i];
		if ( D != 0 ) {
			M = ( systemVars.Mmax[i]  -  systemVars.Mmin[i] ) / D;
			rAIn[i] = systemVars.Mmin[i] + M * ( I - systemVars.Imin[i] );
		} else {
			// Error: denominador = 0.
			rAIn[i] = -999;
		}
	}

	// DEBUG
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		tickCount = xTaskGetTickCount();
		snprintf_P( aIn_printfBuff,CHAR128,PSTR(".[%06lu] tkAnalogIn::trD08 MagCh[%d]=%.02f\r\n\0"), tickCount, i, rAIn[i]);
		u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
	}


	// Paso al systemVars.
	while ( xSemaphoreTake( sem_SYSVars, ( TickType_t ) 1 ) != pdTRUE )
		taskYIELD();

	// Inserto el timeStamp.
	RTC_read(&Aframe.rtc);

	// Paso los datos al systemVars.
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		Aframe.analogIn[i] = rAIn[i];
	}

	// Leo los datos digitales y los pongo en 0.
	u_readDigitalCounters( &Aframe.dIn, TRUE );

	// Convierto los pulsos a los valores de la magnitud.
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		Aframe.dIn.pulses[i] *=  systemVars.magPP[i];
	}

	xSemaphoreGive( sem_SYSVars );


	// BD SAVE FRAME solo en modo normal.
	if ( systemVars.wrkMode == WK_NORMAL ) {
		bWrite = FF_fwrite( &Aframe, sizeof(Aframe));
		FF_stat(&pxFFStatBuffer);
		// Error de escritura ??
		if ( bWrite != sizeof(Aframe) ) {
			snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR("WR ERROR: (%d)\r\n\0"),pxFFStatBuffer.errno);
			FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );
		}
	}

	pv_AINprintExitMsg(8);
	return(tkdST_POLL03);
}
/*------------------------------------------------------------------------------------*/
static int trD09(void)
{
	// tkdST_POLL01 -> tkdST_STANDBY

	// muestro los resultados.

u16 pos = 0;
u08 i;
StatBuffer_t pxFFStatBuffer;

	if ( AN_flags.msgForcePoll == TRUE ) {
		pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("service::{" ));
		AN_flags.msgForcePoll = FALSE;
	} else {
		pos = snprintf_P( aIn_printfBuff, sizeof(aIn_printfBuff), PSTR("frame::{" ));
	}

	// Fecha y hora
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ),PSTR( "%04d%02d%02d,"),Aframe.rtc.year,Aframe.rtc.month,Aframe.rtc.day );
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%02d%02d%02d} ["),Aframe.rtc.hour,Aframe.rtc.min, Aframe.rtc.sec );

	// Datos analogicos
	for ( i = 0; i < NRO_ANALOG_CHANNELS; i++ ) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("%s=%.02f"),systemVars.aChName[i],Aframe.analogIn[i] );
		if ( i != (NRO_ANALOG_CHANNELS - 1) ) {
			pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR(","));
		}
	}
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("]\r\n"));

	// Datos digitales
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++ ) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("[%s::P=%d] "), systemVars.dChName[i],Aframe.dIn.pulses[i] );
	}
	pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("\r\n"));

	// Memoria
	FF_stat(&pxFFStatBuffer);

	// En modo normal o monitor frame muestro el dato
	if ( systemVars.wrkMode == WK_NORMAL ) {
		// En modo normal agrego el mem.stats
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("MEM [%d/%d/%d][%d/%d]\r\n\0"), pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
		FreeRTOS_write( &pdUART1, "POLL->\0", sizeof("POLL->\0") );
	} else if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("\r\n\0"));
		FreeRTOS_write( &pdUART1, "MON->\0", sizeof("MON->\0") );
	} else {
		pos += snprintf_P( &aIn_printfBuff[pos], ( sizeof(aIn_printfBuff) - pos ), PSTR("\r\n\0"));
	}

	// Imprimo.
	FreeRTOS_write( &pdUART1, aIn_printfBuff, sizeof(aIn_printfBuff) );

	pv_AINprintExitMsg(9);
	return(tkdST_STANDBY);

}
/*------------------------------------------------------------------------------------*/
static void pv_AINprintExitMsg(u08 code)
{

	tickCount = xTaskGetTickCount();
	snprintf_P( aIn_printfBuff,sizeof(aIn_printfBuff),PSTR(".[%06lu] tkAnalogIn::exit TR%02d\r\n\0"), tickCount,code);
	u_debugPrint(D_DATA, aIn_printfBuff, sizeof(aIn_printfBuff) );
}
/*------------------------------------------------------------------------------------*/
static void pv_AinLoadParameters( void )
{
	// Dependiendo del modo de trabajo normal, service, idle, monitor, setea el
	// timer de poleo.

	// En modo monitor poleo c/15s
	if ( systemVars.wrkMode == WK_MONITOR_FRAME ) {
		AN_counters.secs2poll = 15;
		return;
	}

	// En todos los otros casos, poleo con timerpoll
	AN_counters.secs2poll = systemVars.timerPoll;
	return;
}
/*------------------------------------------------------------------------------------*/
s16 u_readTimeToNextPoll(void)
{
s16 retVal = -1;

	// Lo determina en base al time elapsed y el timerPoll.
	// El -1 indica un modo en que no esta poleando.
	if ( ( systemVars.wrkMode == WK_NORMAL ) || ( systemVars.wrkMode == WK_MONITOR_FRAME )) {
		retVal = AN_counters.secs2poll;
	}

	return (retVal);
}
/*------------------------------------------------------------------------------------*/
void u_readAnalogFrame (frameData_t *dFrame)
{

	memcpy(dFrame, &Aframe, sizeof(Aframe) );
}
/*------------------------------------------------------------------------------------*/
