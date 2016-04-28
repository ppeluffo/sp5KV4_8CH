/*
 * sp5K.h
 *
 * Created on: 27/12/2013
 *      Author: root
 */

#ifndef SP5K_H_
#define SP5K_H_

#include <avr/io.h>			/* include I/O definitions (port names, pin names, etc) */
//#include <avr/signal.h>		/* include "signal" names (interrupt names) */
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <string.h>
#include <compat/deprecated.h>
#include <util/twi.h>
#include <util/delay.h>
#include <ctype.h>
#include <util/delay.h>
#include <avr/cpufunc.h>

#include "sp5Klibs/avrlibdefs.h"
#include "sp5Klibs/avrlibtypes.h"
#include "sp5Klibs/global.h"			// include our global settings
#include "file_sp5K.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"

#include "FRTOS-IO.h"

#include "cmdline.h"
#include "sp5K_i2c.h"
#include "sp5K_uart.h"

#include "mcp_sp5K.h"
#include "ads7828_sp5K.h"
#include "rtc_sp5K.h"

// DEFINICION DEL TIPO DE SISTEMA
//----------------------------------------------------------------------------
#define SP5K_REV "4.1.4"
#define SP5K_DATE "@ 20160428a"

#define SP5K_MODELO "sp5KV4_8CH HW:avr1284P R5.0"
#define SP5K_VERSION "FW:FRTOS8"

#define CHAR64		64
#define CHAR128	 	128
#define CHAR256	 	256

//----------------------------------------------------------------------------
// TASKS
/* Stack de las tareas */
#define tkCmd_STACK_SIZE		512
#define tkControl_STACK_SIZE	512
#define tkDigitalIn_STACK_SIZE	512
#define tkAIn_STACK_SIZE		512
#define tkGprsTx_STACK_SIZE		512
#define tkGprsRx_STACK_SIZE		512

/* Prioridades de las tareas */
#define tkCmd_TASK_PRIORITY	 		( tskIDLE_PRIORITY + 1 )
#define tkControl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkDigitalIn_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkAIn_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsTx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )
#define tkGprsRx_TASK_PRIORITY 		( tskIDLE_PRIORITY + 1 )

/* Prototipos de tareas */
void tkCmd(void * pvParameters);
void tkControl(void * pvParameters);
void tkDigitalIn(void * pvParameters);
void tkControlInit(void);
void tkAnalogIn(void * pvParameters);
void tkAnalogInit(void);
void tkGprsTx(void * pvParameters);
void tkGprsRx(void * pvParameters);

TaskHandle_t xHandle_tkCmd, xHandle_tkControl, xHandle_tkDigitalIn, xHandle_tkAIn, xHandle_tkGprsTx, xHandle_tkGprsRx;

s08 startTask;
typedef struct {
	u08 resetCause;
	u08 mcusr;
} wdgStatus_t;

wdgStatus_t wdgStatus;

// Mensajes entre tareas
#define TKA_PARAM_RELOAD		0x01	// to tkAnalogIN: reload
#define TKA_READ_FRAME			0x02	// to tkAnalogIN: (mode service) read a frame

#define TKG_PARAM_RELOAD		0x01	// to tkGprsIN: reload
#define TKG_PARAM_NO_DIAL		0x02	// to tkGprsIN: dial_not_allowed
#define TKG_PARAM_CAN_DIAL		0x04	// to tkGprsIN: dial_allowed

#define TKC_PARAM_RELOAD		0x01	// to tkConsIN: reload

//------------------------------------------------------------------------------------

xSemaphoreHandle sem_SYSVars;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

typedef enum { WK_IDLE = 0, WK_NORMAL, WK_SERVICE, WK_MONITOR_FRAME, WK_MONITOR_SQE  } t_wrkMode;
typedef enum { MDM_PRENDIDO = 0, MDM_APAGADO } t_modemStatus;

#define NRO_DIGITAL_CHANNELS	4
#define NRO_ANALOG_CHANNELS		8

#define DLGID_LENGTH		12
#define APN_LENGTH			32
#define PORT_LENGTH			7
#define IP_LENGTH			24
#define SCRIPT_LENGTH		64
#define PASSWD_LENGTH		15
#define PARAMNAME_LENGTH	5

typedef struct {
	u08 level[NRO_DIGITAL_CHANNELS];	// 4
	u16 pulses[NRO_DIGITAL_CHANNELS];	// 8
} dinData_t;		// 12 bytes

typedef struct {
	// size = 7+12+32 = 51 bytes
	RtcTimeType_t rtc;						// 7
	dinData_t dIn;							// 12
	double analogIn[NRO_ANALOG_CHANNELS];	// 32

} frameData_t;	// 38 bytes

typedef struct {
	// Variables de trabajo.
	// Tamanio: 302 bytes para 3 canales.

	u08 dummyBytes;
	u08 initByte;

	char dlgId[DLGID_LENGTH];
	char apn[APN_LENGTH];
	char serverPort[PORT_LENGTH];
	char serverAddress[IP_LENGTH];
	char serverIp[IP_LENGTH];
	char dlgIp[IP_LENGTH];
	char serverScript[SCRIPT_LENGTH];
	char passwd[PASSWD_LENGTH];

	u08 csq;
	u08 dbm;

	u16 timerPoll;

	t_wrkMode wrkMode;

	u08 logLevel;		// Nivel de info que presentamos en display.
	u08 debugLevel;		// Indica que funciones debugear.
	u08 gsmBand;

	//  Canales analogicos
	char aChName[NRO_ANALOG_CHANNELS][PARAMNAME_LENGTH];
	u08 Imin[NRO_ANALOG_CHANNELS];				// Coeficientes de conversion de I->magnitud (presion)
	u08 Imax[NRO_ANALOG_CHANNELS];
	u08 Mmin[NRO_ANALOG_CHANNELS];
	double Mmax[NRO_ANALOG_CHANNELS];

	// Canales digitales
	char dChName[NRO_DIGITAL_CHANNELS][PARAMNAME_LENGTH];
	double magPP[NRO_DIGITAL_CHANNELS];

} systemVarsType;	// 315 bytes

systemVarsType systemVars,tmpSV;

#define EEADDR_SV 32		// Direccion inicio de la EE de escritura del systemVars.

//------------------------------------------------------------------------------------

// Led de placa analogica ( PD6 )
#define LED_KA_PORT		PORTD
#define LED_KA_PIN		PIND
#define LED_KA_DDR		DDRD
#define LED_KA			6

#define LED_MODEM_PORT		PORTD
#define LED_MODEM_PIN		PIND
#define LED_MODEM_DDR		DDRD
#define LED_MODEM			7

void u_prenderLed(u08 led);
void u_apagarLed(u08 led);

// DIN PINES
#define D0_IN_PORT		PORTB
#define D0_IN_DDR		DDRB
#define D0_IN_PIN		PINB
#define D0_IN			4
#define D0_CLR_PORT		PORTB
#define D0_CLR_DDR		DDRB
#define D0_CLR_PIN		PINB
#define D0_CLR			3

#define D1_IN_PORT		PORTB
#define D1_IN_DDR		DDRB
#define D1_IN_PIN		PINB
#define D1_IN			1
#define D1_CLR_PORT		PORTB
#define D1_CLR_DDR		DDRB
#define D1_CLR_PIN		PINB
#define D1_CLR			2

#define D2_IN_PORT		PORTB
#define D2_IN_DDR		DDRB
#define D2_IN_PIN		PINB
#define D2_IN			0
#define D2_CLR_PORT		PORTA
#define D2_CLR_DDR		DDRA
#define D2_CLR_PIN		PINA
#define D2_CLR			0

#define D3_IN_PORT		PORTA
#define D3_IN_DDR		DDRA
#define D3_IN_PIN		PINA
#define D3_IN			2
#define D3_CLR_PORT		PORTA
#define D3_CLR_DDR		DDRA
#define D3_CLR_PIN		PINA
#define D3_CLR			1

// DEBUG
typedef enum { D_NONE = 0, D_BASIC = 1, D_DATA = 2, D_GPRS = 4, D_MEM = 8, D_DIGITAL = 16, D_DEBUG = 32 } t_debug;
typedef enum { OFF = 0, ON = 1 } t_onOff;

//------------------------------------------------------------------------------------
// FUNCIONES DE USO GENERAL.
//------------------------------------------------------------------------------------
void u_readDigitalCounters( dinData_t *dIn , s08 resetCounters );
void u_panic( u08 panicCode );
s08 u_configAnalogCh( u08 channel, char *chName, char *s_iMin, char *s_iMax, char *s_mMin, char *s_mMax );
s08 u_configDigitalCh( u08 channel, char *chName, char *s_magPP );
s08 u_configTimerPoll(char *s_tPoll);
void u_clearWdg( u08 wdgId );
s08 u_saveSystemParams(void);
s08 u_loadSystemParams(void);
void u_loadDefaults(void);
char *u_now(void);

char nowStr[32];

void u_readAnalogFrame (frameData_t *dFrame);
s16 u_readTimeToNextPoll(void);

s08 u_modemPwrStatus(void);
s08 u_wrRtc(char *s);

void u_debugPrint(u08 debugCode, char *msg, u16 size);
void pvMCP_init_MCP1(u08 modo);

void u_reset(void);

//------------------------------------------------------------------------------------
// PANIC CODES
#define P_OUT_TIMERSTART	1
#define P_OUT_TIMERCREATE	2
#define P_AIN_TIMERSTART	3
#define P_GPRS_TIMERSTART	4
#define P_CTL_TIMERCREATE	5
#define P_CTL_TIMERSTART	6

//------------------------------------------------------------------------------------
// WATCHDOG
u08 systemWdg;

#define WDG_CTL			0x01
#define WDG_CMD			0x02
#define WDG_DIN			0x04
#define WDG_CSG			0x08
#define WDG_AIN			0x10
#define WDG_GPRSTX		0x20
#define WDG_GPRSRX		0x40

//------------------------------------------------------------------------------------
// TERMINAL
// Pin de control de fuente de la terminal ( PD7)
#define TERMSW_PORT		PORTD
#define TERMSW_PIN		PIND
#define TERMSW_BIT		7
#define TERMSW_DDR		DDRD
#define TERMSW_MASK		0x80

char debug_printfBuff[CHAR128];

#define T_DAILYRESET		720

//------------------------------------------------------------------------------------
// DCD
// Como el MCP23018 a veces no detecta el nivel del modem, cableamos
// el DCD a PB3
// Pin de control de fuente de la terminal ( PB3)
#define DCD_PORT		PORTB
#define DCD_PIN			PINB
#define DCD_BIT			3
#define DCD_DDR			DDRB
#define DCD_MASK		0x8
//
//------------------------------------------------------------------------------------

#endif /* SP5K_H_ */
