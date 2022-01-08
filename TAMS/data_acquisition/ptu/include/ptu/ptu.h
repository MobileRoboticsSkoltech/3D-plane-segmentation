#ifndef PTU_H
#define PTU_H
/*** Conditionally include the required serial interface declarations. ***/
/*** If your compiler doesn't have the macro symbol defined, you can   ***/
/*** manually select the right include file yourself.                  ***/
#include "ptu/linuxser.h"
#include "ptu/opcodes.h"

namespace ptu{

/* return status codes */
#define PTU_OK				0
#define PTU_ILLEGAL_COMMAND_ARGUMENT    1
#define PTU_ILLEGAL_COMMAND		2
#define PTU_ILLEGAL_POSITION_ARGUMENT	3
#define PTU_ILLEGAL_SPEED_ARGUMENT	4
#define PTU_ACCEL_TABLE_EXCEEDED	5
#define PTU_DEFAULTS_EEPROM_FAULT	6
#define PTU_SAVED_DEFAULTS_CORRUPTED	7
#define PTU_LIMIT_HIT        		8
#define PTU_CABLE_DISCONNECTED		9
#define PTU_ILLEGAL_UNIT_ID	    	10
#define PTU_ILLEGAL_POWER_MODE		11
#define PTU_RESET_FAILED	     	12
#define PTU_NOT_RESPONDING		13
#define PTU_FIRMWARE_VERSION_TOO_LOW    14

/********************************************************************
 *****                                                          *****
 *****		For all of these commands, a non-zero return status *****
 ***** 		indicates an error, and it returns that error code. *****
 *****															*****
 ********************************************************************/

/* open_host_port(<portname>) ==> <portstream> */
extern portstream_fd open_host_port(const char *);

/* Set's the baud rate of the port prior to opening only */
extern char set_baud_rate(int baudrate);

/* close_host_port(<portstream>) ==> <status> */
extern char close_host_port(portstream_fd);

typedef short int PTU_PARM_PTR;

/* reset_PTU_parser(<timeout_in_msec>) ==> [PTU_OK|PTU_NOT_RESPONDING] */
extern char reset_PTU_parser(long);

/* set_desired([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT],
 [<position>|<speed>|<acceleration>],
 [RELATIVE|ABSOLUTE]) ==> <status>
 set_desired([PAN|TILT],
 HOLD_POWER_LEVEL,
 <power mode>,
 NULL) ==> <status>
 set_desired([PAN|TILT],
 [HOLD_POWER_LEVEL,MOVE_POWER_LEVEL],
 [PTU_REG_POWER|PTU_LOW_POWER|PTU_OFF_POWER],
 ABSOLUTE) ==> <status>                              */
extern char set_desired(char, char, PTU_PARM_PTR *, char);

/* get_current([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|RESOLUTION]) ==> <value> */
extern long get_current(char, char);

/* get_desired([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|RESOLUTION]) ==> <value> */
extern long get_desired(char, char);

/* set_mode(COMMAND_EXECUTION_MODE, [EXECUTE_IMMEDIATELY|EXECUTE_UPON_IMMEDIATE_OR_AWAIT]) ==> <status>
 set_mode(ASCII_VERBOSE_MODE, [VERBOSE|TERSE|QUERY_MODE]) ==> <status>
 set_mode(ASCII_ECHO_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
 set_mode(POSITION_LIMITS_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
 set_mode(DEFAULTS,[SAVE_CURRENT_SETTINGS|RESTORE_SAVED_SETTINGS|RESTORE_FACTORY_SETTINGS]) ==> <status> */
extern char set_mode(char, char);

/* halt([ALL|PAN|TILT]) ==> <status>	*/
extern char halt(char);

/* await_completion() ==> <status> */
extern char await_completion(void);

/* reset_PTU() ==> <status> */
extern char reset_ptu(void);
extern char reset_ptu_pan(void);
extern char reset_ptu_tilt(void);

/* firmware_version() ==> <version ID string> */
extern char* firmware_version(void);

/*** multiple unit support ***/
typedef unsigned short int UID_fd;

/* in general, should not be used or required... */
extern char select_host_port(portstream_fd);

/* select_unit(<portstream>, <unit ID>) ==> <status> */
extern char select_unit(UID_fd);

extern char set_unit_id(UID_fd); // This call should be made only
// when one unit is on the current
// host serial port

/*** sets all pan-tilt dynamic motion parameters within a single command ***/
extern char set_PTU_motion(short int, short int, unsigned short int, unsigned short int);
extern char get_PTU_motion(short int *, short int *, short int *, short int *);

/*** sample textual commands ***/
extern char config_CHA(void);
extern char talkto_CHA(void);
extern char talkto_PTUcontroller(void);

/*** asynchronous event status handling functions ***/

typedef unsigned char (*event_handler_fn_ptr_type)(unsigned char);

extern unsigned char default_async_event_handler(unsigned char);
/* Call this function to set the function handler for asynchronous events.
 Defaults to a null function. Example call:
 if ( set_async_event_handler( (unsigned char (*) (unsigned char)) default_async_event_handler ) )
 printf("ASYNCH handler installed properly!");
 else printf("ERROR: ASYNCH handler not installed properly!");
 */
extern unsigned char set_async_event_handler(void(*)(unsigned char));

/********************* function call constants ***********************/

#define PAN                             1
#define TILT                            2

#define POSITION                        1
#define SPEED			        2
#define ACCELERATION                    3
#define BASE                            4
#define UPPER_SPEED_LIMIT               5
#define LOWER_SPEED_LIMIT               6
#define MINIMUM_POSITION                7
#define MAXIMUM_POSITION                8
#define HOLD_POWER_LEVEL                9
#define MOVE_POWER_LEVEL                10
#define RESOLUTION                      11
#define ISM_DRIFT                       12

/* specifies changes relative to current position */
/* (Had to add conditional compilation since WIN32 already defines these values */
#ifndef RELATIVE
#define RELATIVE	1
#endif
#ifndef ABSOLUTE
#define ABSOLUTE	2 
#endif

#define QUERY		NULL

/* power modes */
#define PTU_HI_POWER	1
#define PTU_REG_POWER	2
#define PTU_LOW_POWER	3
#define PTU_OFF_POWER	4

/* PTU mode types */
#define COMMAND_EXECUTION_MODE               1
#define ASCII_VERBOSE_MODE                   2
#define ASCII_ECHO_MODE                      3
#define POSITION_LIMITS_MODE                 4
#define DEFAULTS                             5
#define SPEED_CONTROL_MODE                   6 /* v1.9.7 and higher */
#define PAN_CONTINUOUS_MODE                  7

#define EXECUTE_IMMEDIATELY                  1  /* default */
#define EXECUTE_UPON_IMMEDIATE_OR_AWAIT      2

#define VERBOSE				1  /* default */
#define TERSE				0  

#define ON_MODE				1  /* default */
#define OFF_MODE                      	0

#define SAVE_CURRENT_SETTINGS		0
#define RESTORE_SAVED_SETTINGS		1
#define RESTORE_FACTORY_SETTINGS	2

#define QUERY_MODE			3

#define ALL		3

/* OEM support for TTL controls */
extern unsigned char set_TTL_outputs(unsigned char);
extern unsigned char get_TTL_values(void);

/* advanced calls for doubling pan and tilt position query rates by overlapping comms */
extern char set_desired_abs_positions(signed short int*, signed short int*);
extern char get_current_positions(signed short int*, signed short int*);

/* OEM support for trigger controls */
extern unsigned char TriggerOn(signed short int, signed short int, signed short int);
extern unsigned char TriggerOff(void);
extern signed short int TriggersPending(void);

/* ISM controls */
/* use only in pure velocity mode */
/* <SET_DESIRED_VELOCITIES opcode><pvel signed 2B int><tvel signed 2B int><checksum: xor of int bytes> ==> <status byte> */
unsigned char ptu_set_desired_velocities(signed short int, signed short int);
unsigned char execute_set_desired_velocities(signed short int*, signed short int*);

/* a way to set the pan tilt's positions while in ISM mode */
/* this will command the ISM to slew to this pan tilt position and to begin stabilizing that angle */
char set_ISM_desired_abs_positions(signed short int *Ppos, signed short int *Tpos);

/* a way to get the pan and tilt's positions while in ISM mode */
/* this will command the ISM to return the latest known positios of the pan-tilt head in pan tilt coordinates. */
char get_ISM_desired_abs_positions(signed short int *Ppos, signed short int *Tpos);

}//endof namespace ptu
#endif
