#include <stdio.h>         
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include "ptu/cpiver.h"
#include "ptu/ptu.h"

namespace ptu
{

/* this code supports controller firmware versions equal or above */
#define PTU_modelVersion						1
#define PTU_codeVersion						    7
#define PTU_revision 							6

static char err;

static portstream_fd current_host_port;

static char speed_control_mode = PTU_INDEPENDENT_SPEED_CONTROL_MODE;

/* trace_printf: redirected/portable printf tracing
 * note: this should be done inside a portability layer, much like the serial code */
int trace_printf(const char *format, ...)
{
  va_list args;
  int retc;

  va_start(args, format);
  retc = vprintf(format, args);
  va_end(args);

  return retc;
}

/*** asynchronous event handling ***/
/* This mechanism is invoked when a pan/tilt limit is hit or a cable disconnect is detected */

/* The default asynchronous event handler does nothing. Useful for defining
 your own callback function. Returns TRUE when called (used to determine
 if the handler has been installed properly). */
unsigned char default_async_event_handler(unsigned char async_event)
{
  trace_printf("\n\n\n\ndefault_async_event_handler called: ");
  switch (async_event)
  {
    case PAN_LIMIT_HIT:
      trace_printf("PAN_LIMIT_HIT\n\n\n");
      break;
    case TILT_LIMIT_HIT:
      trace_printf("TILT_LIMIT_HIT\n\n\n");
      break;
    case CABLE_DISCONNECT_DETECTED:
      trace_printf("CABLE_DISCONNECT_DETECTED\n\n\n");
      break;
    default:
      trace_printf("ERROR (unknown async_event signalled)\n\n\n");
      break;
  }
  return (TRUE);
}

static event_handler_fn_ptr_type async_event_handler_fn_ptr =
    (unsigned char(*)(unsigned char))default_async_event_handler;

/* Call this function to set the function handler for asynchronous events.
 Defaults to a null function. Example call:
 if ( set_async_event_handler( (unsigned char (*) (unsigned char)) default_async_event_handler ) )
 printf("ASYNCH handler installed properly!");
 else printf("ERROR: ASYNCH handler not installed properly!");
 */
unsigned char set_async_event_handler(void(*async_event_handler)(unsigned char))
{
  async_event_handler_fn_ptr = (unsigned char(*)(unsigned char))default_async_event_handler;
  return ((*async_event_handler_fn_ptr)(0));
}

/************************** PTU COMMANDS *****************************/

/* open_host_port(<portname>) ==> <portstream> */
portstream_fd open_host_port(const char *portname)
{
  char out_string[10] = "    ";

  current_host_port = openserial(portname);

  /* added this to allow serial ports to clear for networked startup */
  SerialBytesOut(current_host_port, (unsigned char *)out_string, strlen(out_string));
  do_delay(2000);

  return current_host_port;
}

/* Set's the baud rate of the port prior to opening only */
extern char set_baud_rate(int baudrate)
{
  return setbaudrate(baudrate);
}

/* close_host_port(<portstream>) ==> <status> */
char close_host_port(portstream_fd portstream)
{
  current_host_port = PORT_NOT_OPENED;
  return (closeserial(portstream));
}

unsigned char GetSerialChar(char await_char)
{
  unsigned char c;

  for (;;)
  {
    err = SerialBytesIn(current_host_port, &c, 1, 10000);
    if (err < 0)
      return err;
    else if (err > 0)
      return c;
    else if (await_char != TRUE)
      return 0;
  }
}

/* Call this function to return a single byte binary command return status.
 Handles asynchronous events, and makes any required event handling callbacks. */
unsigned char get_binary_command_return_status()
{
  unsigned char status;

  status = GetSerialChar(TRUE);
  while (ASYNCHRONOUS_EVENT(status))
  { /* call the asynchronous event handler */
    (*async_event_handler_fn_ptr)(status);
    status = GetSerialChar(TRUE);
  }
  return (status);
}

char SerialOut(unsigned char outchar)
{
  return SerialBytesOut(current_host_port, &outchar, 1);
}

unsigned char SerialIn()
{
  unsigned char Cin;
  SerialBytesIn(current_host_port, &Cin, 1, 1000);
  return Cin;
}

/* use this function to switch the host port currently being controlled */
char select_host_port(portstream_fd portstream)
{
  current_host_port = portstream;
  return (0);
}

/* reset_PTU() ==> <status> */
char reset_ptu(void)
{
  unsigned char c;

  SerialOut(UNIT_RESET);
  /* strips out the limit hits looking for a return status */
  while (((c = GetSerialChar(TRUE)) == PAN_LIMIT_HIT) || (c == TILT_LIMIT_HIT) || (c == 255))
    ;
  return (c);
}

/* reset_PTU_pan() ==> <status> */
char reset_ptu_pan(void)
{
  unsigned char c;

  SerialOut(UNIT_RESET_PAN);
  /* strips out the limit hits looking for a return status */
  while (((c = GetSerialChar(TRUE)) == PAN_LIMIT_HIT) || (c == TILT_LIMIT_HIT))
    ;
  return (c);
}

/* reset_PTU_tilt() ==> <status> */
char reset_ptu_tilt(void)
{
  unsigned char c;

  SerialOut(UNIT_RESET_TILT);
  /* strips out the limit hits looking for a return status */
  while (((c = GetSerialChar(TRUE)) == PAN_LIMIT_HIT) || (c == TILT_LIMIT_HIT))
    ;
  return (c);
}

/* an internal function that verifies that the PTU controller firmware
 supports this binary interface. TRUE when OK, otherwise error code.
 This function communicates in ASCII to ensure proper unit communication.
 Added 10 sec timeout on read to allow fall-out from non-responsive PTU. */
/* updated 7/3/03 */
char firmware_version_OK(void)
{
  unsigned char *tmpStr;
  char c1, c2;
  int modelVersion, codeVersion, revision;
  unsigned char versionSTR[256];
  int charsRead = 0;
  int status;
  unsigned char initString[] = "    v ";

  do_delay(500);
  status = FlushInputBuffer(current_host_port);
  if (status != TRUE)
  {
    trace_printf("\nERROR(firmware_version_OK): FlushInputBuffer failed with status (%d)\n", status);
    return FALSE;
  }
  tmpStr = initString;
  if ((status = SerialBytesOut(current_host_port, tmpStr, 6)) != TRUE)
  {
    trace_printf("\nERROR(firmware_version_OK): SerialBytesOut error %d\n", status);
    return (FALSE);
  }
  do_delay(500);

  switch (ReadSerialLine(current_host_port, versionSTR, 10000, &charsRead))
  {
    case TRUE:
      break;
    case TIMEOUT_CHAR_READ:
      trace_printf("\nERROR(firmware_version_OK): timeout on ReadSerialLine (%d read)\n", charsRead);
      return (TIMEOUT_CHAR_READ);
    default:
      trace_printf("\nERROR(firmware_version_OK): ReadSerialLine error\n");
      return (FALSE);
  }

  /* parse to the beginning of the version ID (fix for ft mode on 7/28/99) */
  tmpStr = versionSTR;
  while (tolower(*tmpStr) != '*')
    tmpStr++;
  while (tolower(*tmpStr) != 'v')
    tmpStr++;
  tmpStr++;
  sscanf((char *)tmpStr, "%1d %c %2d %c %2d", &modelVersion, &c1, &codeVersion, &c2, &revision);

  if ( /* ensure the version numbers seem reasonable as version numbers, and
   the version number is high enough */
  ((modelVersion < 999) && (modelVersion > PTU_modelVersion)) || ((codeVersion < 999) && (modelVersion
      == PTU_modelVersion) && (codeVersion > PTU_codeVersion)) || ((revision < 9999) && (modelVersion
      == PTU_modelVersion) && (codeVersion == PTU_codeVersion) && (revision >= PTU_revision)))
  {
    trace_printf("\n\nController firmware v%d.%d.%d is compatible\n\n", modelVersion, codeVersion, revision);
    return (TRUE); /* the controller firmware is compatible */
  }
  else
  {
    trace_printf(
                 "\n\nPTU Controller firmware version v%d.%d.%d is NOT compatible:\n\tversion v%d.%d.%d and higher is required\n   charsRead='%d'\n",
                 modelVersion, codeVersion, revision, PTU_modelVersion, PTU_codeVersion, PTU_revision, charsRead);
    return (FALSE);
  }
}

/* Flushes the PTU parser so that pending command parses are terminated,
 and the call blocks until the PTU is ready to accept the next command
 or it times out before the PTU responds that it is OK. */
/* reset_PTU_parser(<timeout_in_msec>) ==> [PTU_OK|PTU_NOT_RESPONDING] */
char reset_PTU_parser(long timeout_in_msec)
{
  long elapsed_time = 250;
  char status;

  do_delay(500); /* allows pending PTU commands to complete */
  SerialOut(' ');
  SerialOut(' ');
  SerialOut(' '); /* terminates any pending parses */
  do_delay(250); /* allows the return of any PTU info from the termination */
  FlushInputBuffer(current_host_port); /* flushes the PTU return info */

  if (firmware_version_OK() != TRUE)
    return (PTU_NOT_RESPONDING);

  /* now make sure the PTU is responding to commands */
  for (;;)
  { /* issue a command and ensure it looks legal */
    SerialOut(PAN_HOLD_POWER_QUERY);
    /* do_delay(250);  */
    status = GetSerialChar(FALSE);
    if ((status >= PTU_REG_POWER) & (status <= PTU_OFF_POWER))
    { /* things look OK, so flush and unblock */
      FlushInputBuffer(current_host_port); /* flushes the PTU return info */
      return (PTU_OK);
    }
    else
    { /* there's a problem, so flush, delay, and retry */
      FlushInputBuffer(current_host_port); /* flushes the PTU return info */
      do_delay(500);
      elapsed_time += 750;
      if (elapsed_time > timeout_in_msec)
        return (PTU_NOT_RESPONDING);
    }
  }
}

/* set_desired([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|UPPER|LOWER|ISM_DRIFT],
 [<position>|<speed>|<acceleration>],
 [RELATIVE|ABSOLUTE]) ==> <status>
 set_desired([PAN|TILT],
 [HOLD_POWER_LEVEL,MOVE_POWER_LEVEL],
 <power mode>,
 ABSOLUTE) ==> <status>                              */
char set_desired(char axis, char kinematic_property, PTU_PARM_PTR *value, char movement_mode)
{
  unsigned short int uvalue;
  long lvalue;
  char cvalue;

  switch (kinematic_property)
  {
    case POSITION:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
              SerialOut(PAN_SET_REL_POS);
              break;
            case ABSOLUTE:
              SerialOut(PAN_SET_ABS_POS);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
              SerialOut(TILT_SET_REL_POS);
              break;
            case ABSOLUTE:
              SerialOut(TILT_SET_ABS_POS);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      PutSignedShort(current_host_port, (signed short *)value);
      break;
    case SPEED:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
              SerialOut(PAN_SET_REL_SPEED);
              PutSignedShort(current_host_port, (signed short *)value);
              break;
            case ABSOLUTE:
              SerialOut(PAN_SET_ABS_SPEED);
              if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
              {
                uvalue = *((unsigned short *)value);
                PutUnsignedShort(current_host_port, &uvalue);
              }
              else
              {
                PutSignedShort(current_host_port, (signed short *)value);
              }
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
              SerialOut(TILT_SET_REL_SPEED);
              PutSignedShort(current_host_port, (signed short *)value);
              break;
            case ABSOLUTE:
              SerialOut(TILT_SET_ABS_SPEED);
              if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
              {
                uvalue = *((unsigned short *)value);
                PutUnsignedShort(current_host_port, &uvalue);
              }
              else
              {
                PutSignedShort(current_host_port, (signed short *)value);
              }
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      break;
    case ISM_DRIFT: /*** ISM supported added 16Feb2006 ***/
      switch (axis)
      {
        case PAN:
          SerialOut(ISM_SET_PAN_DRIFT_RATE);
          break;
        case TILT:
          SerialOut(ISM_SET_TILT_DRIFT_RATE);
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      PutSignedShort(current_host_port, (signed short *)value);
      break;
    case ACCELERATION:
      lvalue = *((long *)value);
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
              lvalue += get_current(PAN, ACCELERATION);
              SerialOut(PAN_SET_ACCEL);
              break;
            case ABSOLUTE:
              SerialOut(PAN_SET_ACCEL);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
              lvalue += get_current(TILT, ACCELERATION);
              SerialOut(TILT_SET_ACCEL);
              break;
            case ABSOLUTE:
              SerialOut(TILT_SET_ACCEL);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      PutSignedLong(current_host_port, &lvalue);
      break;
    case BASE:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(PAN_SET_BASE_SPEED);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(TILT_SET_BASE_SPEED);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      uvalue = *((unsigned short int*)value);
      PutUnsignedShort(current_host_port, &uvalue);
      break;
    case UPPER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(PAN_SET_UPPER_SPEED_LIMIT);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(TILT_SET_UPPER_SPEED_LIMIT);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      uvalue = *((unsigned short int*)value);
      PutUnsignedShort(current_host_port, &uvalue);
      break;
    case LOWER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(PAN_SET_LOWER_SPEED_LIMIT);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(TILT_SET_LOWER_SPEED_LIMIT);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      uvalue = *((unsigned short int*)value);
      PutUnsignedShort(current_host_port, &uvalue);
      break;
    case HOLD_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(PAN_SET_HOLD_POWER);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(TILT_SET_HOLD_POWER);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      cvalue = *((unsigned char*)value);
      SerialOut((unsigned char)cvalue);
      break;
    case MOVE_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(PAN_SET_MOVE_POWER);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        case TILT:
          switch (movement_mode)
          {
            case RELATIVE:
            case ABSOLUTE:
              SerialOut(TILT_SET_MOVE_POWER);
              break;
            default:
              return (PTU_ILLEGAL_COMMAND_ARGUMENT);
          }
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      cvalue = *((unsigned char*)value);
      SerialOut((unsigned char)cvalue);
      break;
    default:
      return (PTU_ILLEGAL_COMMAND_ARGUMENT);
  }

  return (get_binary_command_return_status()); /* return the command execution status */

}

/* await_completion() ==> <status> */
char await_completion(void)
{
  SerialOut(AWAIT_COMMAND_COMPLETION);
  return (get_binary_command_return_status());
}

/* get_current([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|
 UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
 MINIMUM_POSITION|MAXIMUM_POSITION|
 RESOLUTION|
 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|
 ISM_DRIFT]) ==> <value> */
long get_current(char axis, char kinematic_property)
{
  unsigned short int uvalue;
  signed short int value;
  long long_value;
  unsigned char Cin;

  /* check for pending asynchronous event */
  if (PeekByte(current_host_port, &Cin))
  {
    Cin = SerialIn();
    if (ASYNCHRONOUS_EVENT( Cin ))
    { /* call the asynchronous event handler */
      (*async_event_handler_fn_ptr)(Cin);
    }
  }

  /* process the command now */
  switch (kinematic_property)
  {
    case POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_CURRENT_POS_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_CURRENT_POS_QUERY);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case SPEED:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_CURRENT_SPEED_QUERY);
          if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
            goto get_and_return_unsigned_short_int;
          else
            goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_CURRENT_SPEED_QUERY);
          if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
            goto get_and_return_unsigned_short_int;
          else
            goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case ISM_DRIFT: /*** added ISM support 16Feb2006 ***/
      switch (axis)
      {
        case PAN:
          SerialOut(ISM_GET_PAN_DRIFT_RATE);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(ISM_GET_TILT_DRIFT_RATE);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case ACCELERATION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_ACCEL_QUERY);
          goto get_and_return_long;
        case TILT:
          SerialOut(TILT_ACCEL_QUERY);
          goto get_and_return_long;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case BASE:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_BASE_SPEED_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_BASE_SPEED_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case UPPER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_UPPER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_UPPER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case LOWER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_LOWER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_LOWER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case MINIMUM_POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MIN_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_MIN_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case MAXIMUM_POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MAX_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_MAX_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case RESOLUTION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_RESOLUTION_QUERY);
          goto get_and_return_long;
        case TILT:
          SerialOut(TILT_RESOLUTION_QUERY);
          goto get_and_return_long;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case HOLD_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_HOLD_POWER_QUERY);
          goto get_and_return_char;
        case TILT:
          SerialOut(TILT_HOLD_POWER_QUERY);
          goto get_and_return_char;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case MOVE_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MOVE_POWER_QUERY);
          goto get_and_return_char;
        case TILT:
          SerialOut(TILT_MOVE_POWER_QUERY);
          goto get_and_return_char;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    default:
      return (PTU_ILLEGAL_COMMAND_ARGUMENT);
  }

  get_and_return_unsigned_short_int: err = GetUnsignedShort(current_host_port, &uvalue, -1);
  long_value = uvalue;
  return (long_value);

  get_and_return_signed_short_int: err = GetSignedShort(current_host_port, &value, -1);
  long_value = value;
  return (long_value);

  get_and_return_long: err = GetSignedLong(current_host_port, &long_value, -1);
  return (long_value);

  get_and_return_char: long_value = (long)GetSerialChar(TRUE);
  return (long_value);

}

/* get_desired([PAN|TILT],
 [POSITION|SPEED|ACCELERATION|BASE|
 UPPER_SPEED_LIMIT|LOWER_SPEED_LIMIT|
 MINIMUM_POSITION|MAXIMUM_POSITION|
 RESOLUTION|
 HOLD_POWER_LEVEL|MOVE_POWER_LEVEL|
 ISM_DRIFT]) ==> <ptr to value> */
long get_desired(char axis, char kinematic_property)
{
  unsigned short int uvalue;
  signed short int value;
  long long_value;
  unsigned char Cin;

  /* check for pending asynchronous event */
  if (PeekByte(current_host_port, &Cin))
  {
    Cin = SerialIn();
    if (ASYNCHRONOUS_EVENT( Cin ))
    { /* call the asynchronous event handler */
      (*async_event_handler_fn_ptr)(Cin);
    }
  }

  switch (kinematic_property)
  {
    case POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_DESIRED_POS_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_DESIRED_POS_QUERY);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case SPEED:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_DESIRED_SPEED_QUERY);
          if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
            goto get_and_return_unsigned_short_int;
          else
            goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_DESIRED_SPEED_QUERY);
          if (speed_control_mode == PTU_INDEPENDENT_SPEED_CONTROL_MODE)
            goto get_and_return_unsigned_short_int;
          else
            goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case ISM_DRIFT: /* added 16Feb2006 for ISM */
      switch (axis)
      {
        case PAN:
          SerialOut(ISM_GET_PAN_DRIFT_RATE);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(ISM_GET_TILT_DRIFT_RATE);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case ACCELERATION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_ACCEL_QUERY);
          goto get_and_return_long;
        case TILT:
          SerialOut(TILT_ACCEL_QUERY);
          goto get_and_return_long;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case BASE:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_BASE_SPEED_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_BASE_SPEED_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case UPPER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_UPPER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_UPPER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case LOWER_SPEED_LIMIT:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_LOWER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        case TILT:
          SerialOut(TILT_LOWER_SPEED_LIMIT_QUERY);
          goto get_and_return_unsigned_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case MINIMUM_POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MIN_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_MIN_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        default:
          break;
      }
    case MAXIMUM_POSITION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MAX_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        case TILT:
          SerialOut(TILT_MAX_POSITION_QUERY);
          goto get_and_return_signed_short_int;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case RESOLUTION:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_RESOLUTION_QUERY);
          goto get_and_return_long;
        case TILT:
          SerialOut(TILT_RESOLUTION_QUERY);
          goto get_and_return_long;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case HOLD_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_HOLD_POWER_QUERY);
          goto get_and_return_char;
        case TILT:
          SerialOut(TILT_HOLD_POWER_QUERY);
          goto get_and_return_char;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    case MOVE_POWER_LEVEL:
      switch (axis)
      {
        case PAN:
          SerialOut(PAN_MOVE_POWER_QUERY);
          goto get_and_return_char;
        case TILT:
          SerialOut(TILT_MOVE_POWER_QUERY);
          goto get_and_return_char;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    default:
      return (PTU_ILLEGAL_COMMAND_ARGUMENT);

  }

  get_and_return_unsigned_short_int: err = GetUnsignedShort(current_host_port, &uvalue, -1);
  long_value = uvalue;
  return (long_value);

  get_and_return_signed_short_int: err = GetSignedShort(current_host_port, &value, -1);
  long_value = value;
  return (long_value);

  get_and_return_long: err = GetSignedLong(current_host_port, &long_value, -1);
  return (long_value);

  get_and_return_char: long_value = (long)GetSerialChar(TRUE);
  return (long_value);

}

/* set_mode(COMMAND_EXECUTION_MODE,
 [EXECUTE_IMMEDIATELY|EXECUTE_UPON_IMMEDIATE_OR_AWAIT]) ==> <status>
 set_mode(ASCII_VERBOSE_MODE, [VERBOSE|TERSE|QUERY_MODE]) ==> <status>
 set_mode(ASCII_ECHO_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
 set_mode(POSITION_LIMITS_MODE, [ON_MODE|OFF_MODE|QUERY_MODE]) ==> <status>
 set_mode(DEFAULTS,[SAVE_CURRENT_SETTINGS|RESTORE_SAVED_SETTINGS|RESTORE_FACTORY_SETTINGS]) ==> <status>
 *** below is only supported by PTU firmware versions 1.9.7 and higher.                        ***
 *** This call must be made before pure velocity speed control mode made be used by CPI calls. ***
 set_mode(SPEED_CONTROL_MODE, [PTU_INDEPENDENT_SPEED_CONTROL_MODE |
 PTU_PURE_VELOCITY_SPEED_CONTROL_MODE | QUERY_MODE] ==> <status>
 */
char set_mode(char mode_type, char mode_parameter)
{
  switch (mode_type)
  {
    case DEFAULTS:
    {
      switch (mode_parameter)
      {
        case SAVE_CURRENT_SETTINGS:
          SerialOut(SAVE_DEFAULTS);
          goto return_status;
        case RESTORE_SAVED_SETTINGS:
          SerialOut(RESTORE_SAVED_DEFAULTS);
          goto return_status;
        case RESTORE_FACTORY_SETTINGS:
          SerialOut(RESTORE_FACTORY_DEFAULTS);
          goto return_status;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    }
    case ASCII_ECHO_MODE:
    {
      switch (mode_parameter)
      {
        case ON_MODE:
          SerialOut(ENABLE_ECHO);
          goto return_status;
        case OFF_MODE:
          SerialOut(DISABLE_ECHO);
          goto return_status;
        case QUERY_MODE:
          SerialOut(ECHO_QUERY);
          goto return_status;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    }
    case COMMAND_EXECUTION_MODE:
      switch (mode_parameter)
      {
        case EXECUTE_IMMEDIATELY:
          SerialOut(SET_IMMEDIATE_COMMAND_MODE);
          break;
        case EXECUTE_UPON_IMMEDIATE_OR_AWAIT:
          SerialOut(SET_SLAVED_COMMAND_MODE);
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
      break;
    case ASCII_VERBOSE_MODE:
    {
      switch (mode_parameter)
      {
        case VERBOSE:
          SerialOut(SET_VERBOSE_ASCII_ON);
          break;
        case TERSE:
          SerialOut(SET_VERBOSE_ASCII_OFF);
          break;
        case QUERY_MODE:
          SerialOut(VERBOSE_QUERY);
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    }
      break;
    case POSITION_LIMITS_MODE:
    {
      switch (mode_parameter)
      {
        case ON_MODE:
          SerialOut(ENABLE_POSITION_LIMITS);
          break;
        case OFF_MODE:
          SerialOut(DISABLE_POSITION_LIMITS);
          break;
        case QUERY_MODE:
          SerialOut(POSITION_LIMITS_QUERY);
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    }
      break;
    case SPEED_CONTROL_MODE:
    {
      switch (mode_parameter)
      {
        case PTU_INDEPENDENT_SPEED_CONTROL_MODE:
          speed_control_mode = PTU_INDEPENDENT_SPEED_CONTROL_MODE;
          SerialOut(SET_INDEPENDENT_CONTROL_MODE);
          break;
        case PTU_PURE_VELOCITY_SPEED_CONTROL_MODE:
          speed_control_mode = PTU_PURE_VELOCITY_SPEED_CONTROL_MODE;
          SerialOut(SET_PURE_VELOCITY_CONTROL_MODE);
          break;
        case QUERY_MODE:
          SerialOut(QUERY_SPEED_CONTROL_MODE);
          break;
        default:
          return (PTU_ILLEGAL_COMMAND_ARGUMENT);
      }
    }
      break;
    default:
      return (PTU_ILLEGAL_COMMAND_ARGUMENT);
  }
  return_status: return (get_binary_command_return_status()); /* return <status> */
}

/* halt([ALL|PAN|TILT]) ==> <status>	*/
char halt(char halt_type)
{
  switch (halt_type)
  {
    case PAN:
      SerialOut(HALT_PAN);
      break;
    case TILT:
      SerialOut(HALT_TILT);
      break;
    default:
      SerialOut(HALT);
      break;
  }
  return (get_binary_command_return_status());
}

/* firmware_version() ==> <version ID string> */
char* firmware_version(void)
{
  static unsigned char version_ID_string[256];
  int charsRead;

  SerialOut(FIRMWARE_VERSION_QUERY);
  do_delay(1000);
  ReadSerialLine(current_host_port, version_ID_string, 0, &charsRead);
  return ((char *)version_ID_string);
}

//Function Enables Stabilization and sends a command
void main2()
{
  short desired_speed;
  //enable stabilization
  select_unit(128);
  //set constant azimuth velocity
  desired_speed = 1000; //1000 positions / sec
  set_desired(PAN, SPEED, &desired_speed, ABSOLUTE);
  //Let it move for a few seconds
  do_delay(3000);
  //disable stabilization
  select_unit(0);

  //set_desired(PAN, SPEED,
}

/* modified 5/12/99  */
/* delay added 21NOV2006 */
/* Addresses the desired network device
 *    unit_ID - this is an 8-byte integer representing the ID of the device to address
 */
char select_unit(UID_fd unit_ID)
{
  char UID_select[10];

  sprintf(UID_select, "  _%d ", unit_ID);
  SerialBytesOut(current_host_port, (unsigned char*)UID_select, strlen(UID_select));
  do_delay(25);

  /* the below only works for PTU firmware v1.9.11r3 and above */
  /* SerialOut(SELECT_UNIT_ID); */
  /* PutUnsignedShort(current_host_port, &unit_ID); */

  return TRUE;
}

/*  modified 10/19/98  */
char set_unit_id(UID_fd unit_ID)
{
  SerialOut(SET_UNIT_ID);
  PutUnsignedShort(current_host_port, &unit_ID);
  return GetSerialChar(TRUE);
}

/* added 5/28/2004 */
char set_PTU_motion(short int desired_pan_pos, short int desired_tilt_pos, unsigned short int desired_pan_speed,
                    unsigned short int desired_tilt_speed)
{
  unsigned char opcode = SET_DESIRED_PTU_MOTION, status;

  SerialBytesOut(current_host_port, &opcode, 1);
  PutSignedShort(current_host_port, &desired_pan_pos);
  PutSignedShort(current_host_port, &desired_tilt_pos);
  PutUnsignedShort(current_host_port, &desired_pan_speed);
  PutUnsignedShort(current_host_port, &desired_tilt_speed);
  SerialBytesIn(current_host_port, &status, 1, 500);
  return (status);
}

/* added 5/28/2004 */
char get_PTU_motion(short int *desired_pan_pos, short int *desired_tilt_pos, short int *desired_pan_speed,
                    short int *desired_tilt_speed)
{
  unsigned char opcode = QUERY_CURRENT_PTU_MOTION;

  SerialBytesOut(current_host_port, &opcode, 1);
  GetSignedShort(current_host_port, desired_pan_pos, 500);
  GetSignedShort(current_host_port, desired_tilt_pos, 500);
  GetSignedShort(current_host_port, desired_pan_speed, 500);
  GetSignedShort(current_host_port, desired_tilt_speed, 500);
  return (PTU_OK);
}

/***************/

/* Configures EIO channel CHA. added 10/12/2004 */
char config_CHA()
{
  char status;
  char commandResponse[256];
  int charsRead;
  SerialStringOut(current_host_port, (unsigned char *)"   @A(9600,8,N,n) ");
  if ((status = ReadSerialLine(current_host_port, (unsigned char*)commandResponse, 1000, &charsRead)))
  {
    if (commandResponse[strlen(commandResponse) - 2] == '*')
      return (TRUE);
    else
      return (FALSE);
  }
  else
    return (status);
}

static char CHA_comm_active = FALSE;
/* establishes point-to-point communication between host computer and EIO CHA device */
char talkto_CHA()
{
  char status;
  char commandResponse[256];
  int charsRead;
  SerialStringOut(current_host_port, (unsigned char *)"   @A ");
  if ((status = ReadSerialLine(current_host_port, (unsigned char *)commandResponse, 1000, &charsRead)))
  {
    if (commandResponse[strlen(commandResponse) - 2] == '*')
    {
      CHA_comm_active = TRUE;
      return (TRUE);
    }
    else
      return (FALSE);
  }
  else
    return (status);
}

/* establishes point-to-point communication between host computer and PTU controller. */
/* talkto_CHA should be called prior to this function */
char talkto_PTUcontroller()
{
  char status;
  char commandResponse[256];
  int charsRead;
  if (CHA_comm_active != TRUE)
    return (FALSE);
  SerialStringOut(current_host_port, (unsigned char *)"@ ");
  if ((status = ReadSerialLine(current_host_port, (unsigned char *)commandResponse, 1000, &charsRead)))
  {
    if (commandResponse[strlen(commandResponse) - 2] == '*')
    {
      CHA_comm_active = FALSE;
      return (TRUE);
    }
    else
      return (FALSE);
  }
  else
    return (status);
}

char set_pure_velocities(signed short int *pan_speed, signed short int *tilt_speed)
{
  char status1, status2;
  SerialOut(PAN_SET_ABS_SPEED);
  PutSignedShort(current_host_port, (signed short int *)pan_speed);
  SerialOut(TILT_SET_ABS_SPEED);
  PutSignedShort(current_host_port, (signed short int *)tilt_speed);
  if ((status1 = get_binary_command_return_status()) != PTU_OK)
  {
    trace_printf("\n\nSTATUS1 failure (%d)\n", status1);
  }
  if ((status2 = get_binary_command_return_status()) != PTU_OK)
  {
    trace_printf("\n\nSTATUS2 failure (%d)\n", status2);
  }
  return (status1 & status2);
}

/***************/

/* TTL controls added 2/22/2006, required firmware version 2.12.12r5 or higher */

/* call to set D300 ports OP2..5 in format: X X X X OP5 OP4 OP3 OP2 */
/* Returns byte with: 0 IP4 IP3 IP2 OP5 OP4 OP3 OP2 */
unsigned char set_TTL_outputs(unsigned char in_TTL_control_byte)
{
  unsigned char outbyte;

  SerialOut(TTL_CONTROLS);
  outbyte = (in_TTL_control_byte & 15) | 128;
  SerialOut(outbyte);
  /* get the return data now */
  return ((unsigned char)GetSerialChar(TRUE));
}

/* Returns byte with: 0 IP4 IP3 IP2 OP5 OP4 OP3 OP2 */
unsigned char get_TTL_values()
{
  unsigned char outbyte;

  SerialOut(TTL_CONTROLS);
  outbyte = 0;
  SerialOut(outbyte);
  /* get the return data now */
  return ((unsigned char)GetSerialChar(TRUE));
}

/*****************/
/* Trigger controls added for firmware version 2.12.13d31 or higher */

/* call to set pan triggers */
/* startPanPos: the pan position at which the first trigger will occur        */
/* deltaPos:    the number of pan positions between triggers. Value must be   */
/*              two or higher. The CMOS TTL trigger is set (active high) when */
/*				a trigger position is reached. The trigger is unset on the    */
/*				next pan position.                                            */
/* numTriggers: the number of times a trigger is output starting from the     */
/*              first trigger, given the axis has moved deltaPos*numTriggers  */
/*              positions                                                     */

/* Notes: The system matches for an exact position and triggers when that     */
/* position is reached. "Trigger start position" must be reached to begin     */
/* triggering. The triggers occur in sequence. If any trigger position in a   */
/* sequence is not explicitly moved through, the trigger is essentially stuck */
/* waiting for the next trigger position. The trigger off and new trigger set */
/* commands change this status. If a trigger is activated, and the pan is at  */
/* startPanPos, then a trigger will be automatically set to ACTIVE ON, and it */
/* will stay on until the pan axis has moved one position.                    */

/* returns status */
unsigned char TriggerOn(signed short int startPanPos, signed short int deltaPos, signed short int numTriggers)
{ /* send the command */
  SerialOut(TRIGGER_CONTROLS);
  SerialOut(TRIGGER_OP_on);
  PutSignedShort(current_host_port, (signed short int *)&startPanPos);
  PutSignedShort(current_host_port, (signed short int *)&deltaPos);
  PutSignedShort(current_host_port, (signed short int *)&numTriggers);
  /* get the return data now */
  return ((unsigned char)GetSerialChar(TRUE));
}

/* Returns status */
unsigned char TriggerOff()
{ /* send the command */
  SerialOut(TRIGGER_CONTROLS);
  SerialOut(TRIGGER_OP_off);
  /* get the return data now */
  return ((unsigned char)GetSerialChar(TRUE));
}

/* Returns status */
signed short int TriggersPending()
{
  signed short int triggersPending;
  /* send the command */
  SerialOut(TRIGGER_CONTROLS);
  SerialOut(TRIGGER_OP_numPending);
  /* get the return data now */
  GetSignedShort(current_host_port, &triggersPending, -1);
  return (triggersPending);
}

/*****************/

/* a faster way to send absolute position commands that overlaps command execution */
char set_desired_abs_positions(signed short int *Ppos, signed short int *Tpos)
{
  char err1, err2;
  /* send the pan position command C1 */
  SerialOut(PAN_SET_ABS_POS);
  PutSignedShort(current_host_port, Ppos);
  /* send the tilt position command C2 */
  SerialOut(TILT_SET_ABS_POS);
  PutSignedShort(current_host_port, Tpos);
  /* read the response R1 from the command C1 */
  err1 = get_binary_command_return_status(); /* R1 response from command C1 */
  err2 = get_binary_command_return_status(); /* R2 response from command C2 */
  return (err1 || err2);
}

/* get_current_positions - get current pan and tilt position
 * This command exploits port buffering to pipeline commands.
 * Returns zero on error.
 */
char get_current_positions(signed short int *Ppos, signed short int *Tpos)
{
  char err1, err2;
  /* send the pan position query command C1 */
  SerialOut(PAN_CURRENT_POS_QUERY);
  /* send the tilt position query command C2 */
  SerialOut(TILT_CURRENT_POS_QUERY);
  /* read the response R1 from the command C1 */
  err1 = GetSignedShort(current_host_port, Ppos, -1);
  err2 = GetSignedShort(current_host_port, Tpos, -1);
  return (err1 || err2);
}

/* a way to set the pan tilt's positions while in ISM mode */
/* this will command the ISM to slew to this pan tilt position and to begin stabilizing that angle */
char set_ISM_desired_abs_positions(signed short int *Ppos, signed short int *Tpos)
{
  /* send the pan position command C1 */
  SerialOut(PAN_TILT_SET_ABS_POS);
  PutSignedShort(current_host_port, Ppos);
  PutSignedShort(current_host_port, Tpos);
  return (get_binary_command_return_status());
}

/* a way to get the pan and tilt's positions while in ISM mode */
/* this will command the ISM to return the latest known positios of the pan-tilt head in pan tilt coordinates. */
char get_ISM_desired_abs_positions(signed short int *Ppos, signed short int *Tpos)
{
  char err1, err2;
  SerialOut(PAN_TILT_GET_ABS_POS);
  err1 = GetSignedShort(current_host_port, Ppos, -1);
  err2 = GetSignedShort(current_host_port, Tpos, -1);
  return (err1 || err2);
}

/*****************/

unsigned char checksum_on_2_2B(unsigned char *val1, unsigned char *val2)
{
  unsigned char checksum;
  checksum = *val1 + *(val1 + 1) + *val2 + *(val2 + 1);
  return (checksum);
}

unsigned char ptu_set_desired_velocities(signed short int pspeed, signed short int tspeed)
{
  return execute_set_desired_velocities(&pspeed, &tspeed);
}

/* used by ISM to send two signed velocities (CV mode) with status byte */
/* use only in pure velocity mode */
/* <SET_DESIRED_VELOCITIES opcode><pvel signed 2B int><tvel signed 2B int><checksum: xor of int bytes> ==> <status byte> */
unsigned char execute_set_desired_velocities(signed short int *Pspeed, signed short int *Tspeed)
{
  unsigned char checksum, status;

  /* send the command */
  SerialOut(SET_DESIRED_VELOCITIES);
  PutSignedShort(current_host_port, Pspeed);
  PutSignedShort(current_host_port, Tspeed);
  checksum = checksum_on_2_2B((unsigned char *)Pspeed, (unsigned char *)Tspeed);
  SerialOut(checksum);
  /* get the command response */
  status = get_binary_command_return_status(); // GetSerialChar(FALSE);
  return (status);
}
}//namesapce ptu
