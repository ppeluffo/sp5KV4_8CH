/*
 * sp5KV4_8CH_tkGprs.h
 *
 *  Created on: 22 de abr. de 2016
 *      Author: pablo
 */

#ifndef SP5KV4_8CH_TKGPRS_SP5KV4_8CH_TKGPRS_H_
#define SP5KV4_8CH_TKGPRS_SP5KV4_8CH_TKGPRS_H_

u32 tickCount;

// Estados
typedef enum { gST_INIT = 0,
				gST_OFF,
				gST_ONOFFLINE,
				gST_ONOPENSOCKET,
				gST_DATA
} t_tkGprs_state;

t_tkGprs_state tkGprs_state;

// Subestados.
typedef enum {
				// Estado OFF
				gSST_OFF_00 = 0,
	            gSST_OFF_01,
				gSST_OFF_02,
				gSST_OFF_03,
				gSST_OFF_04,
				gSST_OFF_05,
				gSST_OFF_06,
				gSST_OFF_07,

				gSST_ONOFFLINE_00,
				gSST_ONOFFLINE_01,
				gSST_ONOFFLINE_02,
				gSST_ONOFFLINE_03,
				gSST_ONOFFLINE_04,
				gSST_ONOFFLINE_05,
				gSST_ONOFFLINE_06,
				gSST_ONOFFLINE_07,
				gSST_ONOFFLINE_08,
				gSST_ONOFFLINE_09,
				gSST_ONOFFLINE_10,
				gSST_ONOFFLINE_11,
				gSST_ONOFFLINE_12,

	            gSST_SOCKET_00,
				gSST_SOCKET_01,
				gSST_SOCKET_02,
				gSST_SOCKET_03,
				gSST_SOCKET_04,
				gSST_SOCKET_05,
				gSST_SOCKET_06,
				gSST_SOCKET_07,

	            gSST_DATA_00,
				gSST_DATA_01,
				gSST_DATA_02,
				gSST_DATA_03,
				gSST_DATA_04,
				gSST_DATA_05,
				gSST_DATA_06,
				gSST_DATA_07,
				gSST_DATA_08,
				gSST_DATA_09,

} t_tkGprs_subState;

t_tkGprs_subState tkGprs_subState;

typedef enum { SOCKET_CLOSED = 0, SOCKET_OPEN } t_socket;
typedef enum { MRSP_NONE = 0, MRSP_OK , MRSP_ERROR, MRSP_CONNECT, MRSP_NOCARRIER, MRSP_E2IPA, MRSP_CREG, MRSP_FRAMEOK } t_modemResponse;

struct {
	s08 msgReload;
	t_modemResponse modemResponse;
	t_modemStatus modemPwrStatus;
	t_socket socketStatus;
	s08 gsmBandOK;
	s08 memRcds4Tx;
	s08 moreRcds4Tx;
	s08 memRcds4Del;

} GPRS_flags;

struct {
	s08 cTimer;
	s08 pTryes;
	s08 qTryes;
	u08 txRcdsInWindow;
} GPRS_counters;

struct {
	char buffer[UART0_RXBUFFER_LEN];
	u16 ptr;
} gprsRx;

#define HW_TRYES	3			// Reintentos de prender HW el modem
#define SW_TRYES	3			// Reintentos de hacer toggle del SW del modem
#define SECS_AWAIT_NEWCICLE	15	// Espera entre ciclos si el modem no prendio.
#define FRAMEXTXWINDOW	10		// Registros por frameWindow.

char gprs_printfBuff[CHAR256];
char gprsRX_printfBuff[CHAR256];

// Acciones Generales
void gTR_reloadConfig(void);
void g_printExitMsg(char *code);

void SM_off(void);
void SM_onOffline(void);
void SM_onOpenSocket(void);
void SM_Data(void);

void g_setSocketStatus( u08 socketStatus);
void g_setModemResponse( u08 modemStatus);
void g_printRxBuffer(void);
void g_flushRXBuffer(void);
s08 g_strstr(char *rsp, size_t *pos);
char *g_getRxBuffer(void);
void g_reloadConfig(void);


#endif /* SP5KV4_8CH_TKGPRS_SP5KV4_8CH_TKGPRS_H_ */
