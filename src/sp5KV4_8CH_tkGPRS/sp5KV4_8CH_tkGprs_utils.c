/*
 * sp5KV4_8CH_tkGprs_utils.c
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#include "sp5KV4_8CH.h"
#include "sp5KV4_8CH_tkGprs.h"

//------------------------------------------------------------------------------------
void g_reloadConfig(void)
{
	GPRS_flags.msgReload = FALSE;
	tkGprs_state = gST_OFF;
	tkGprs_subState = gSST_OFF_00;
}
//------------------------------------------------------------------------------------
void g_flushRXBuffer(void)
{
	g_setModemResponse(MRSP_NONE);
	memset(gprsRx.buffer,0, UART0_RXBUFFER_LEN );
	gprsRx.ptr = 0;

}
//------------------------------------------------------------------------------------
s08 g_strstr(char *rsp, size_t *pos)
{
	// Busca el string rsp en el buffer local de recepcion del gprs.
	// Devuelve la posicion relativa

u16 i;
char *p,*q, *res = NULL;
s08 inString = FALSE;
s08 rollover = FALSE;
s08 retS = FALSE;

	// Busca un substring en un string circular
	p = (char *)(gprsRx.buffer);
	q = rsp;

	// Chequeo que el substrig no sea vacio.
    if ( *rsp == 0) {
    	*pos = 0;
		goto quit;
    }

	// Recorro el string base. Lo recorro todo ya que como es circular, puede
	// tener un \0 en el medio.
    i = 0;
	inString = FALSE;
    while(1) {
		if ( *p == *q ) {
			if ( inString == FALSE ) {
				res = p;	// Guardo la primer posicion de coincidencia.
				*pos = i;
			}
			inString = TRUE;
			q++;
			if ( *q == '\0')
				// Esta es la unica condicion de salida valida.
				break;
		} else {
			// Reinicio las condiciones de busqueda
			inString = FALSE;
			q = rsp;
			*pos = 0;
			if ( rollover )	// Ya di vuelta y no coincide.
				break;
		}
		// Avanzo
		p++;
		i++;
		if ( i == UART0_RXBUFFER_LEN ) {
			// Llegue al final. Rollover.
			i = 0;
			p = (char *)(gprsRx.buffer);
			rollover = TRUE;
		}
    }

    if ( ! inString) {
 		 // FAIL
		res = NULL;
  	}

 quit:

	if ( res != NULL) {
		// FOUND
		retS = TRUE;
	}
	return(retS);

}
//------------------------------------------------------------------------------------
void g_printRxBuffer(void)
{

	// Imprime la respuesta a un comando.
	// Utiliza el buffer de RX.

	if ( (systemVars.debugLevel & D_GPRS) != 0) {
		tickCount = xTaskGetTickCount();
		snprintf_P( gprsRX_printfBuff,sizeof(gprsRX_printfBuff),PSTR(".[%06lu] tkGprs: Rsp=\r\n\0"),tickCount  );
		FreeRTOS_write( &pdUART1, gprsRX_printfBuff, sizeof(gprsRX_printfBuff) );

		// Imprimo todo el buffer de RX ( 640b). Sale por \0.
		FreeRTOS_write( &pdUART1, gprsRx.buffer, UART0_RXBUFFER_LEN );
		// Agrego un CRLF por las dudas
		FreeRTOS_write( &pdUART1, "\r\n\0", sizeof("\r\n\0") );
	}
}
//------------------------------------------------------------------------------------
char *g_getRxBuffer(void)
{
	return (gprsRx.buffer);
}
//------------------------------------------------------------------------------------
s08 u_modemPwrStatus(void)
{
	return(GPRS_flags.modemPwrStatus);
}
//--------------------------------------------------------------------------------------
void g_setSocketStatus( u08 socketStatus)
{
	GPRS_flags.socketStatus = socketStatus;

}
//-------------------------------------------------------------------------------------
void g_setModemResponse( u08 modemStatus)
{
	GPRS_flags.modemResponse = modemStatus;
}
//-------------------------------------------------------------------------------------
void g_printExitMsg(char *code)
{
	tickCount = xTaskGetTickCount();
	memset( gprs_printfBuff,'\0',sizeof(gprs_printfBuff));
	snprintf_P( gprs_printfBuff,sizeof(gprs_printfBuff),PSTR(".[%06lu] tkGprs: exit %s\r\n\0"), tickCount,code);
	u_debugPrint(D_GPRS, gprs_printfBuff, sizeof(gprs_printfBuff) );
}
//------------------------------------------------------------------------------------
