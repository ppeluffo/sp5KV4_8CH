/*
 * sp5KV3_tkControl.c
 *
 *  Created on: 7/4/2015
 *      Author: pablo
 *
 *  Tareas de control generales del SP5K
 *  - Recibe un mensaje del timer del led para indicar si debe prender o apagarlo.
 */

#include "sp5KV4_8CH.h"

static char ctl_printfBuff[CHAR128];

static void pv_flashLeds(void);
static void pv_wdgInit(void);
static void pv_checkWdg(void );
static void pv_dailyReset(void);

//------------------------------------------------------------------------------------
void tkControl(void * pvParameters)
{

( void ) pvParameters;
TickType_t xLastWakeTime;
const TickType_t sleepTime = ( 1000 / portTICK_RATE_MS );
u16 ffRcd;
StatBuffer_t pxFFStatBuffer;

	vTaskDelay( ( TickType_t)( 500 / portTICK_RATE_MS ) );

	MCP_init();			// Esto prende la terminal.
	pv_wdgInit();

	// inicializo la memoria EE ( fileSysyem)
	ffRcd = FF_fopen();
	FF_stat(&pxFFStatBuffer);
	if ( pxFFStatBuffer.errno != pdFF_ERRNO_NONE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit ERROR (%d)[%d]\r\n\0"),ffRcd, pxFFStatBuffer.errno);
	} else {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("FSInit OK\r\nMEMsize=%d, wrPtr=%d,rdPtr=%d,delPtr=%d,Free=%d,4del=%d\r\n\0"),FF_MAX_RCDS, pxFFStatBuffer.HEAD,pxFFStatBuffer.RD, pxFFStatBuffer.TAIL,pxFFStatBuffer.rcdsFree,pxFFStatBuffer.rcds4del);
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// load systemVars
	if  ( u_loadSystemParams() == TRUE ) {
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config OK.\r\n\0") );
	} else {
		u_loadDefaults();
		u_saveSystemParams();
		snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Load config ERROR: defaults !!\r\n\0") );
	}
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("starting tkControl..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );

	// Habilito arrancar las otras tareas
	startTask = TRUE;

	// Loop
	for( ;; )
	{

		u_clearWdg(WDG_CTL);

		// Ejecuto la rutina sincronicamente c/1s
		// Wait for 1000ms.
		vTaskDelayUntil( &xLastWakeTime, sleepTime );
		xLastWakeTime = xTaskGetTickCount();

		pv_flashLeds();
		pv_checkWdg();
		pv_dailyReset();

	}
}
//------------------------------------------------------------------------------------
static void pv_dailyReset(void)
{
	// Se invoca c/1 sec.
	// Cuento hasta 60 para tener 1 minuto.
	// Se usa para resetear el micro 1 vez al dia.

static s08 rCounter = 60;
static s16 resetCounter = T_DAILYRESET;

	if ( rCounter-- > 0 )
		return;

	rCounter = 60;
	if ( resetCounter-- > 0)
		return;

	// Una vez por dia me reseteo.
	snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Going to daily reset..\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
	// RESET
	u_reset();

}
//------------------------------------------------------------------------------------
static void pv_flashLeds(void)
{
	// Ejecuto c/2 secs.
	// Prendo los leds por 20ms y los apago.

static u08 l_timer = 1;

	// El led de KA lo prendo y apago c 1/s
	u_prenderLed(LED_KA);
	vTaskDelay( 1 );
	u_apagarLed(LED_KA);

	// El led de modem lo prendo y apago c 2s.
	if (l_timer-- > 0 )
		return;

	l_timer = 1;
	if (u_modemPwrStatus() == MDM_PRENDIDO )
		u_prenderLed(LED_MODEM);

	// no es necesario ya que lo que demora las MCP son suficientes.
	vTaskDelay( 1 );
	u_apagarLed(LED_MODEM);

}
//------------------------------------------------------------------------------------
static void pv_wdgInit(void)
{
u08 pos = 0;

	pos = snprintf_P( ctl_printfBuff,sizeof(ctl_printfBuff),PSTR("Watchdog init (0x%X"),wdgStatus.resetCause);
	if (wdgStatus.resetCause & 0x01 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" PORF"));
	}
	if (wdgStatus.resetCause & 0x02 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" EXTRF"));
	}
	if (wdgStatus.resetCause & 0x04 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" BORF"));
	}
	if (wdgStatus.resetCause & 0x08 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" WDRF"));
	}
	if (wdgStatus.resetCause & 0x10 ) {
		pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" JTRF"));
	}
	pos += snprintf_P( &ctl_printfBuff[pos],sizeof(ctl_printfBuff),PSTR(" )\r\n\0"));
	FreeRTOS_write( &pdUART1, ctl_printfBuff, sizeof(ctl_printfBuff) );
}
//------------------------------------------------------------------------------------
static void pv_checkWdg(void )
{
	// Cada tarea periodicamente pone su wdg flag en 0. Esto hace que al chequearse c/3s
	// deban estar todas en 0 para asi resetear el wdg del micro.

static u08 l_timer = 2;

	if (l_timer-- > 0 )
		return;

	l_timer = 1;
	wdt_reset();

	if ( systemWdg == 0 ) {
		wdt_reset();
		systemWdg = WDG_CTL + WDG_CMD + WDG_CSG + WDG_DIN + WDG_AIN + WDG_GPRSRX + WDG_GPRSTX;
	}
}
//------------------------------------------------------------------------------------
