/*
 * sp5K_tkGprsRX.c
 *
 *  Created on: 21/05/2014
 *      Author: root
 *
 *
 *  V4.0.7:
 *  Paso la tarea de gprsRX a un archivo separado.
 *  V4.0.6:
 *  Agrego una tarea que procesa los caracteres recibidos del GPRS asi me detecta el estado
 *  del socket.
 *  Cuando DCD=1 el socket esta cerrado y cuando DCD=0 esta abierto.
 *  Ocurre que a veces el nivel de cerrado son 2V y no se detecta, por lo que creo que esta
 *  abierto cuando en realidad esta cerrado.
 *  Por lo tanto el detectar el socket OPEN es no safe ya que puede estar cerrado con 2V y el
 *  micro leerlo como un 0 y pensar que esta OPEN cuando en realidad esta CLOSE.
 *  El CLOSE es safe ya que si detecto un 1 ( aunque no siempre ), no tengo dudas que este cerrado
 *  Esto me permite controlar el socket cerrado como un OR de 2 condiciones: NO CARRIER o DCD == 1
 *  El problema que veo es que el DCD puede ser 1 antes de la respuesta NO CARRIER.
 *
 *
 */
#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

//-------------------------------------------------------------------------------------

void tkGprsTx(void * pvParameters)
{

( void ) pvParameters;
BaseType_t xResult;
uint32_t ulNotifiedValue;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("starting tkGprsTx..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );

	tkGprs_state = gST_OFF;
	tkGprs_subState = gSST_OFF_00;
	//
	for( ;; )
	{
		u_clearWdg(WDG_GPRSTX);

		// Espero hasta 100ms por un mensaje.
		xResult = xTaskNotifyWait( 0x00, ULONG_MAX, &ulNotifiedValue, ((TickType_t) 100 / portTICK_RATE_MS ) );
		// Si llego un mensaje, prendo la flag correspondiente.
		if ( xResult == pdTRUE ) {
			if ( ( ulNotifiedValue & TKG_PARAM_RELOAD ) != 0 ) {
				GPRS_flags.msgReload = TRUE;
			}
		}

		// El manejar la FSM con un switch por estado y no por transicion me permite
		// priorizar las transiciones.
		// Luego de c/transicion debe venir un break así solo evaluo de a 1 transicion por loop.
		//
		switch ( tkGprs_state ) {
		case gST_OFF:
			SM_off();
			break;
		case gST_ONOFFLINE:
			SM_onOffline();
			break;
		case gST_ONOPENSOCKET:
			SM_onOpenSocket();
			break;
		case gST_DATA:
			SM_Data();
			break;
		default:
			snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR("tkGprs::ERROR state NOT DEFINED\r\n\0"));
			FreeRTOS_write( &pdUART1, gprs_printfBuff, sizeof(gprs_printfBuff) );
			// Estado inicial.
			tkGprs_state = gST_OFF;
			tkGprs_subState = gSST_OFF_00;
			break;
		}

	}
}

//------------------------------------------------------------------------------------

void tkGprsRx(void * pvParameters)
{
// Esta tarea lee y procesa las respuestas del GPRS.

u08 c;
size_t pos;

( void ) pvParameters;

	while ( !startTask )
		vTaskDelay( ( TickType_t)( 100 / portTICK_RATE_MS ) );

	snprintf_P( gprsRX_printfBuff,sizeof(gprsRX_printfBuff),PSTR("starting tkGprsRx..\r\n\0"));
	FreeRTOS_write( &pdUART1, gprsRX_printfBuff, sizeof(gprsRX_printfBuff) );

	g_setSocketStatus(SOCKET_CLOSED);
	g_setModemResponse(MRSP_NONE);
	g_flushRXBuffer();

	// loop
	for( ;; )
	{
		u_clearWdg(WDG_GPRSRX);

		// el read se bloquea 50ms. lo que genera la espera.
		while ( FreeRTOS_read( &pdUART0, &c, 1 ) == 1 ) {
			gprsRx.buffer[gprsRx.ptr] = c;
			// Avanzo en modo circular
			gprsRx.ptr = ( gprsRx.ptr  + 1 ) % ( UART0_RXBUFFER_LEN );

			// Los comandos vienen terminados en CR
			if (c == '\r') {

				if ( g_strstr("OK\r", &pos ) == TRUE ) {
				//	FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_OK\r\n\0", sizeof("DEBUG ** MRSP_OK\r\n\0") );
				// No podemos asumir que el socket este cerrado ya que en las respuestas HTTP puede venir
				// un OK\r.
				}

				if ( g_strstr("ERROR\r", &pos ) == TRUE ) {
					//FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_ERROR\r\n\0", sizeof("DEBUG ** MRSP_ERROR\r\n\0") );
				}

				if ( g_strstr("CONNECT", &pos ) == TRUE ) {
					g_setSocketStatus(SOCKET_OPEN);
				//	FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_CONNECT\r\n\0", sizeof("DEBUG ** MRSP_CONNECT\r\n\0") );
				}

				if ( g_strstr("NO CARRIER", &pos ) == TRUE ) {
					g_setSocketStatus(SOCKET_CLOSED);
				//	FreeRTOS_write( &pdUART1, "DEBUG ** MRSP_NO CARRIER\r\n\0", sizeof("DEBUG ** MRSP_NO CARRIER\r\n\0") );
				}
			}
		}
	}
}
/*------------------------------------------------------------------------------------*/
