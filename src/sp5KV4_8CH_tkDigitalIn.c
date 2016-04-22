/*
 * sp5KV3_tkDigitalIn.c
 *
 *  Created on: 13/4/2015
 *      Author: pablo
 *
 * En los sp5KV4_8CH los latches estan conectados directamente a pines
 * del micro, no se usa mas el MCP.
 * La configuracion de los pines se hace main::pv_initMPU()
 *
 */


#include "sp5KV4_8CH.h"

static void pv_clearQ(void);
static void pv_pollQ(void);

static char dIn_printfBuff[CHAR64];	// Buffer de impresion
static dinData_t digIn;				// Estructura local donde cuento los pulsos.

/*------------------------------------------------------------------------------------*/
void tkDigitalIn(void * pvParameters)
{

( void ) pvParameters;
u08 i = 0;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR("starting tkDigitalIn..\r\n\0"));
	FreeRTOS_write( &pdUART1, dIn_printfBuff, sizeof(dIn_printfBuff) );

	// Inicializo los latches borrandolos
	pv_clearQ();
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		digIn.level[i] = 0;
		digIn.pulses[i] = 0;
	}

	for( ;; )
	{
		u_clearWdg(WDG_DIN);

		// Espero hasta 250ms por un mensaje.
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

		// Solo poleo las entradas en modo normal. En modo service no para
		// poder manejarlas por los comandos de servicio.
		if ( systemVars.wrkMode == WK_NORMAL) {
			pv_pollQ();
		}
	}

}
/*------------------------------------------------------------------------------------*/
void u_readDigitalCounters( dinData_t *dIn , s08 resetCounters )
{
	// copio los valores de los contadores en la estructura dIn.
	// Si se solicita, luego se ponen a 0.
u08 i = 0;
//u32 tickCount;

	memcpy( dIn, &digIn, sizeof(dinData_t)) ;

//	tickCount = xTaskGetTickCount();
//	snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] DEBYG: D0={%d,%d}, D1={%d,%d}, D2={%d,%d}, D3={%d,%d}\r\n\0"), tickCount,digIn.level[0],digIn.pulses[0],digIn.level[1],digIn.pulses[1],digIn.level[2],digIn.pulses[2],digIn.level[3],digIn.pulses[3] );
//	u_debugPrint(( D_BASIC + D_DIGITAL ), dIn_printfBuff, sizeof(dIn_printfBuff) );


	if ( resetCounters == TRUE ) {
		for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
			digIn.level[i] = 0;
			digIn.pulses[i] = 0;
		}
	}
}
/*------------------------------------------------------------------------------------*/
static void pv_pollQ(void)
{

s08 debugQ = FALSE;
u32 tickCount = 0;
u08 i = 0;

	// Leo las entradas digitales
	digIn.level[0] = D0_IN_PIN & _BV(D0_IN);
	digIn.level[1] = D1_IN_PIN & _BV(D1_IN);
	digIn.level[2] = D2_IN_PIN & _BV(D2_IN);
	digIn.level[3] = D3_IN_PIN & _BV(D3_IN);

	// Levels
	debugQ = FALSE;
	for ( i = 0; i < NRO_DIGITAL_CHANNELS; i++) {
		// Flancos de bajada
		if (digIn.level[i] == 0 ) { digIn.pulses[i]++ ; debugQ = TRUE;}
	}

	if ( ((systemVars.debugLevel & D_DIGITAL) != 0) && debugQ ) {
		tickCount = xTaskGetTickCount();
		snprintf_P( dIn_printfBuff,sizeof(dIn_printfBuff),PSTR(".[%06lu] tkDigitalIn: D0={%d}, D1={%d}, D2={%d}, D3={%d}\r\n\0"), tickCount,digIn.pulses[0],digIn.pulses[1],digIn.pulses[2],digIn.pulses[3] );
		u_debugPrint(( D_BASIC + D_DIGITAL ), dIn_printfBuff, sizeof(dIn_printfBuff) );
	}

	// Siempre borro los latches para evitar la posibilidad de quedar colgado.
	pv_clearQ();
	return;

}
/*------------------------------------------------------------------------------------*/
static void pv_clearQ(void)
{
	// Pongo un pulso 1->0->1 en Q0/Q1 pin para resetear el latch
	// En reposo debe quedar en H.
	cbi( D0_CLR_PORT, D0_CLR );
	cbi( D1_CLR_PORT, D1_CLR );
	cbi( D2_CLR_PORT, D2_CLR );
	cbi( D3_CLR_PORT, D3_CLR );

	vTaskDelay( ( TickType_t)( 10 / portTICK_RATE_MS ) );
	//taskYIELD();
	//_delay_us(5);
	//asm("nop");

	sbi( D0_CLR_PORT, D0_CLR );
	sbi( D1_CLR_PORT, D1_CLR );
	sbi( D2_CLR_PORT, D2_CLR );
	sbi( D3_CLR_PORT, D3_CLR );

}
/*------------------------------------------------------------------------------------*/
