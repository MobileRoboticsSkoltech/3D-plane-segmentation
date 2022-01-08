#include <stdio.h>      
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "ptu/cpiver.h"
#include "ptu/ptu.h"
#include <unistd.h>
#include <string.h>

using namespace ptu;

extern char set_pure_velocities(signed short *pan_speed, signed short *tilt_speed);

/********** MAIN PROGRAM *********/
#define TEST_ERROR			-1

unsigned short int uval, uval1, uval2, uval_in;
signed short int val, val1, val2, val3, val4, val_in;
long lval, lval_in, lval1, lval2;
char status, cval, cval_in;
char tmpChar;

int Sleep(int usec)
{
  return usleep(usec);
}

char return_error()
{
  printf("(Enter 'c' to continue): ");
  tmpChar = 'f';
  while (tmpChar != 'c')
    tmpChar = (char)tolower(getchar());
  return (TEST_ERROR);
}

static char error_string[50];
void set_error_string(char status_code)
{
  switch (status_code)
  {
    case PTU_OK:
      strcpy(error_string, "PTU_OK");
      break;
    case PTU_ILLEGAL_COMMAND:
      strcpy(error_string, "PTU_ILLEGAL_COMMAND");
      break;
    case PTU_ILLEGAL_POSITION_ARGUMENT:
      strcpy(error_string, "PTU_ILLEGAL_POSITION_ARGUMENT");
      break;
    case PTU_ILLEGAL_SPEED_ARGUMENT:
      strcpy(error_string, "PTU_ILLEGAL_SPEED_ARGUMENT");
      break;
    case PTU_ACCEL_TABLE_EXCEEDED:
      strcpy(error_string, "PTU_ACCEL_TABLE_EXCEEDED");
      break;
    case PTU_DEFAULTS_EEPROM_FAULT:
      strcpy(error_string, "PTU_DEFAULTS_EEPROM_FAULT");
      break;
    case PTU_SAVED_DEFAULTS_CORRUPTED:
      strcpy(error_string, "PTU_SAVED_DEFAULTS_CORRUPTED");
      break;
    case PTU_LIMIT_HIT:
      strcpy(error_string, "PTU_LIMIT_HIT");
      break;
    case PTU_CABLE_DISCONNECTED:
      strcpy(error_string, "PTU_CABLE_DISCONNECTED");
      break;
    case PTU_ILLEGAL_UNIT_ID:
      strcpy(error_string, "PTU_ILLEGAL_UNIT_ID");
      break;
    case PTU_ILLEGAL_POWER_MODE:
      strcpy(error_string, "PTU_ILLEGAL_POWER_MODE");
      break;
    case PTU_RESET_FAILED:
      strcpy(error_string, "PTU_RESET_FAILED");
      break;
    case PTU_ILLEGAL_PARAMETERS:
      strcpy(error_string, "PTU_ILLEGAL_PARAMETERS");
      break;
    case PTU_DUART_ERROR:
      strcpy(error_string, "PTU_DUART_ERROR");
      break;
    case PTU_ERROR:
      strcpy(error_string, "PTU_ERROR");
      break;
    case NOT_SUPPORTED_BY_THIS_FIRMWARE_VERSION:
      strcpy(error_string, "NOT_SUPPORTED_BY_THIS_FIRMWARE_VERSION");
      break;
    case PTU_TILT_VANE_OUT_OF_RANGE_ERROR:
      strcpy(error_string, "PTU_TILT_VANE_OUT_OF_RANGE_ERROR");
      break;
    default:
      strcpy(error_string, "UNDEFINED_RETURN_STATUS_ERROR");
      break;
  }
}

char return_error_status(char *failed_binary_op, char return_status)
{
  set_error_string(return_status);
  printf("! %s failed with status %d (%s)\n", failed_binary_op, return_status, error_string);
  return (return_error());
}

char test_verbose_modes(void)
{ /* test the ASCII verbose modes */

  if ((status = set_mode(ASCII_VERBOSE_MODE, TERSE)) == TRUE)
  {
    return (return_error_status("SET_VERBOSE_ASCII_OFF", status));
  }
  else
    printf("\nSET_VERBOSE_ASCII_OFF executed\n");
  if ((status = set_mode(ASCII_VERBOSE_MODE, QUERY_MODE)) != TERSE)
  {
    return (return_error_status("SET_VERBOSE_ASCII_OFF", status));
  }
  else
    printf("SET_VERBOSE_ASCII_OFF verified\n");

  if ((status = set_mode(ASCII_VERBOSE_MODE, VERBOSE)) == TRUE)
  {
    return (return_error_status("SET_VERBOSE_ASCII_ON", status));
  }
  else
    printf("\nSET_VERBOSE_ASCII_ON executed\n");
  if ((status = set_mode(ASCII_VERBOSE_MODE, QUERY_MODE)) != VERBOSE)
  {
    return (return_error_status("SET_VERBOSE_ASCII_ON", status));
  }
  else
    printf("SET_VERBOSE_ASCII_ON verified\n");
  return (PTU_OK);
}

char test_ASCII_echo_modes(void)
{ /* test the ASCII echo modes */
  if ((status = set_mode(ASCII_ECHO_MODE, OFF_MODE)) == TRUE)
  {
    return (return_error_status("DISABLE_ECHO", status));
  }
  else
    printf("\nDISABLE_ECHO executed\n");
  if ((status = set_mode(ASCII_ECHO_MODE, QUERY_MODE)) != OFF_MODE)
  {
    return (return_error_status("DISABLE_ECHO", status));
  }
  else
    printf("DISABLE_ECHO verified\n");

  if ((status = set_mode(ASCII_ECHO_MODE, ON_MODE)) == TRUE)
  {
    return (return_error_status("ENABLE_ECHO", status));
  }
  else
    printf("ENABLE_ECHO executed\n");
  if ((status = set_mode(ASCII_ECHO_MODE, QUERY_MODE)) != ON_MODE)
  {
    return (return_error_status("ENABLE_ECHO", status));
  }
  else
    printf("ENABLE_ECHO verified\n");
  return (PTU_OK);
}

char test_position_limits(void)
{ /* test the position limits modes */

  if ((status = set_mode(POSITION_LIMITS_MODE, OFF_MODE)) == TRUE)
  {
    return (return_error_status("DISABLE_POSITION_LIMITS", status));
  }
  else
    printf("\nDISABLE_POSITION_LIMITS executed\n");
  if ((status = set_mode(POSITION_LIMITS_MODE, QUERY_MODE)) != OFF_MODE)
  {
    return (return_error_status("DISABLE_POSITION_LIMITS", status));
  }
  else
    printf("DISABLE_POSITION_LIMITS verified\n");

  if ((status = set_mode(POSITION_LIMITS_MODE, ON_MODE)) == TRUE)
  {
    return (return_error_status("ENABLE_POSITION_LIMITS", status));
  }
  else
    printf("ENABLE_POSITION_LIMITS executed\n");
  if ((status = set_mode(POSITION_LIMITS_MODE, QUERY_MODE)) != ON_MODE)
  {
    return (return_error_status("ENABLE_POSITION_LIMITS", status));
  }
  else
    printf("ENABLE_POSITION_LIMITS verified\n");
  return (PTU_OK);
}

char test_speed_commands(void)
{
  halt(ALL);
  do_delay(1000);

  /* verify relative speed queries return the right value */
  lval_in = get_current(PAN, SPEED);
  if (lval_in != 0)
  {
    printf("PAN_CURRENT_SPEED_QUERY failed: (%ld != 0)\n", lval_in);
    return (return_error());
  }
  printf("\nPAN_CURRENT_SPEED_QUERY issued and verified\n");

  lval_in = get_current(TILT, SPEED);
  if (lval_in != 0)
  {
    printf("TILT_CURRENT_SPEED_QUERY failed: (%ld != 0)\n", lval_in);
    return (return_error());
  }
  printf("TILT_CURRENT_SPEED_QUERY issued and verified\n");

  /* absolute speed command tests */
  uval = 1900;

  if ((status = set_desired(PAN, SPEED, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_ABS_SPEED", status));
  }
  else
    printf("\nPAN_SET_ABS_SPEED executed\n");
  lval_in = get_desired(PAN, SPEED);
  if (lval_in != (long)uval)
  {
    printf("PAN_DESIRED_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_DESIRED_SPEED_QUERY issued and verified\n");

  if ((status = set_desired(TILT, SPEED, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("TILT_SET_ABS_SPEED", status));
  }
  else
    printf("TILT_SET_ABS_SPEED executed\n");
  lval_in = get_desired(TILT, SPEED);
  if (lval_in != (long)uval)
  {
    printf("TILT_DESIRED_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_DESIRED_SPEED_QUERY issued and verified\n");

  /* relative speed command tests (when current_speed=0) */
  halt(ALL);
  do_delay(1000);
  val = uval = 1900;

  /* relative pan speed command test */
  lval_in = get_current(PAN, SPEED);
  if ((status = set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, RELATIVE)) == TRUE)
    return (return_error_status("PAN_SET_REL_SPEED", status));
  else
    printf("\nPAN_SET_REL_SPEED executed\n");
  /* relative tilt speed command test */
  lval2 = get_desired(PAN, SPEED);
  if (lval2 != (lval_in + val))
    return (return_error_status("PAN_DESIRED_SPEED_QUERY", -1));
  else
    printf("PAN_DESIRED_SPEED_QUERY issued and verified\n");

  /* query current tilt speed test */
  lval_in = get_current(TILT, SPEED);
  if ((status = set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, RELATIVE)) == TRUE)
    return (return_error_status("TILT_SET_REL_SPEED", status));
  else
    printf("TILT_SET_REL_SPEED executed\n");

  /* query current pan speed test */
  lval2 = get_desired(TILT, SPEED);
  if (lval2 != (lval_in + val))
  {
    return (return_error_status("TILT_DESIRED_SPEED_QUERY", -1));
  }
  else
    printf("TILT_DESIRED_SPEED_QUERY issued and verified\n");

  return (PTU_OK);
}

char test_position_commands(void)
{ /* pan position command */
  val = -1900;

  if ((status = set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_ABS_POSITION", status));
  }
  else
    printf("\nPAN_SET_ABS_POSITION executed\n");

  if ((status = await_completion()) == PTU_OK)
    printf("AWAIT_COMPLETION executed (%d)\n", status);
  else
    return (return_error_status("AWAIT_COMPLETION", status));

  lval_in = get_current(PAN, POSITION);
  if (lval_in != val)
  {
    printf("PAN_CURRENT_POSITION_QUERY failed: (%ld != %d)\n", lval_in, val);
    return (return_error());
  }
  printf("\nPAN_CURRENT_POSITION_QUERY issued and verified\n");

  lval_in = get_desired(PAN, POSITION);
  if (lval_in != val)
  {
    printf("PAN_DESIRED_POSITION_QUERY failed: (%ld != %d)\n", lval_in, val);
    return (return_error());
  }
  printf("\nPAN_DESIRED_POSITION_QUERY issued and verified\n");

  val = (short)-val;
  if ((status = set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, RELATIVE)) == TRUE)
  {
    return (return_error_status("PAN_SET_REL_POSITION", status));
  }
  else
    printf("PAN_SET_REL_POSITION executed\n");

  if ((status = await_completion()) == PTU_OK)
    printf("AWAIT_COMPLETION executed (%d)\n", status);
  else
    return (return_error_status("AWAIT_COMPLETION", status));

  /* tilt position command */
  val = -600;

  if ((status = set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("TILT_SET_ABS_POSITION", status));
  }
  else
    printf("\nTILT_SET_ABS_POSITION executed\n");

  if ((status = await_completion()) == PTU_OK)
    printf("AWAIT_COMPLETION executed (%d)\n", status);
  else
    return (return_error_status("AWAIT_COMPLETION", status));

  lval_in = get_current(TILT, POSITION);
  if (lval_in != val)
  {
    printf("TILT_CURRENT_POSITION_QUERY failed: (%ld != %d)\n", lval_in, val);
    return (return_error());
  }
  printf("\nTILT_CURRENT_POSITION_QUERY issued and verified\n");

  lval_in = get_desired(TILT, POSITION);
  if (lval_in != val)
  {
    printf("TILT_DESIRED_POSITION_QUERY failed: (%ld != %d)\n", lval_in, val);
    return (return_error());
  }
  printf("\nTILT_DESIRED_POSITION_QUERY issued and verified\n");

  val = (short)-val;
  if ((status = set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, RELATIVE)) == TRUE)
  {
    return (return_error_status("TILT_SET_ABS_POSITION", status));
  }
  else
    printf("TILT_SET_ABS_POSITION executed\n");

  if ((status = await_completion()) == PTU_OK)
    printf("AWAIT_COMPLETION executed (%d)\n", status);
  else
    return (return_error_status("AWAIT_COMPLETION", status));

  return (PTU_OK);
}

char test_power_modes(void)
{ /* test power modes */
  cval = PTU_LOW_POWER;

  set_desired(PAN, HOLD_POWER_LEVEL, (PTU_PARM_PTR *)&cval, ABSOLUTE);
  printf("\nPAN_HOLD_POWER_LEVEL set and query executed and verified\n");
  lval_in = get_current(PAN, HOLD_POWER_LEVEL);
  set_desired(PAN, HOLD_POWER_LEVEL, (PTU_PARM_PTR *)&cval, RELATIVE);
  lval2 = get_desired(PAN, HOLD_POWER_LEVEL);
  if ((lval_in != cval) || (lval2 != cval))
  {
    printf("PAN_HOLD_POWER_LEVEL set and query failed: (set,get)=(%d,%ld)\n", cval, lval_in);
    return (return_error());
  }

  set_desired(TILT, HOLD_POWER_LEVEL, (PTU_PARM_PTR *)&cval, ABSOLUTE);
  printf("TILT_HOLD_POWER_LEVEL set and query executed and verified\n");
  lval_in = get_current(TILT, HOLD_POWER_LEVEL);
  set_desired(TILT, HOLD_POWER_LEVEL, (PTU_PARM_PTR *)&cval, RELATIVE);
  lval2 = get_desired(TILT, HOLD_POWER_LEVEL);
  if ((lval_in != cval) || (lval2 != cval))
  {
    printf("TILT_HOLD_POWER_LEVEL set and query failed: (set,get)=(%d,%ld)\n", cval, lval_in);
    return (return_error());
  }

  set_desired(PAN, MOVE_POWER_LEVEL, (PTU_PARM_PTR *)&cval, ABSOLUTE);
  printf("PAN_MOVE_POWER_LEVEL set and query executed and verified\n");
  lval_in = get_current(PAN, MOVE_POWER_LEVEL);
  set_desired(PAN, MOVE_POWER_LEVEL, (PTU_PARM_PTR *)&cval, RELATIVE);
  lval2 = get_desired(PAN, MOVE_POWER_LEVEL);
  if ((lval_in != cval) || (lval2 != cval))
  {
    printf("PAN_MOVE_POWER_LEVEL set and query failed: (set,get)=(%d,%ld)\n", cval, lval_in);
    return (return_error());
  }

  set_desired(TILT, MOVE_POWER_LEVEL, (PTU_PARM_PTR *)&cval, ABSOLUTE);
  printf("TILT_MOVE_POWER_LEVEL set and query executed and verified\n");
  lval_in = get_current(TILT, MOVE_POWER_LEVEL);
  set_desired(TILT, MOVE_POWER_LEVEL, (PTU_PARM_PTR *)&cval, RELATIVE);
  lval2 = get_desired(TILT, MOVE_POWER_LEVEL);
  if ((lval_in != cval) || (lval2 != cval))
  {
    printf("TILT_MOVE_POWER_LEVEL set and query failed: (set,get)=(%d,%ld)\n", cval, lval_in);
    return (return_error());
  }

  return (PTU_OK);
}

char test_base_speed_commands(void)
{ /* PAN set base testing */
  uval = 999;

  if ((status = set_desired(PAN, BASE, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_BASE_SPEED", status));
  }
  else
    printf("\nPAN_SET_BASE_SPEED executed\n");

  lval_in = get_desired(PAN, BASE);
  if (lval_in != (long)uval)
  {
    printf("PAN_DESIRED_BASE_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_DESIRED_BASE_QUERY issued and verified\n");

  lval_in = get_current(PAN, BASE);
  if (lval_in != (long)uval)
  {
    printf("PAN_CURRENT_BASE_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_CURRENT_BASE_QUERY issued and verified\n");

  /* tilt set base testing */
  uval = 1111;

  if ((status = set_desired(TILT, BASE, (PTU_PARM_PTR *)&uval, RELATIVE)) == TRUE)
  {
    return (return_error_status("TILT_SET_BASE_SPEED", status));
  }
  else
    printf("\nTILT_SET_BASE_SPEED executed\n");

  lval_in = get_desired(TILT, BASE);
  if (lval_in != (long)uval)
  {
    printf("TILT_DESIRED_BASE_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_DESIRED_BASE_QUERY issued and verified\n");

  lval_in = get_current(TILT, BASE);
  if (lval_in != (long)uval)
  {
    printf("TILT_CURRENT_BASE_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_CURRENT_BASE_QUERY issued and verified\n");

  return (PTU_OK);
}

char test_speed_limits(void)
{ /* pan set lower speed limit testing */
  uval = 57;

  if ((status = set_desired(PAN, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_LOWER_SPEED_LIMIT", status));
  }
  else
    printf("\nPAN_SET_LOWER_SPEED_LIMIT executed\n");

  lval_in = get_desired(PAN, LOWER_SPEED_LIMIT);
  if (lval_in != (long)uval)
  {
    printf("PAN_LOWER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_LOWER_SPEED_QUERY issued and verified\n");

  if ((status = set_desired(PAN, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, RELATIVE)) == TRUE)
  {
    return (return_error_status("PAN_SET_LOWER_SPEED_LIMIT", status));
  }
  else
    printf("PAN_SET_LOWER_SPEED_LIMIT executed\n");

  lval_in = get_current(PAN, LOWER_SPEED_LIMIT);
  if (lval_in != (long)uval)
  {
    printf("PAN_LOWER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_LOWER_SPEED_QUERY issued and verified\n");

  /* tilt set lower speed limit testing */
  uval = 58;

  if ((status = set_desired(TILT, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("TILT_SET_LOWER_SPEED_LIMIT", status));
  }
  else
    printf("\nTILT_SET_LOWER_SPEED_LIMIT executed\n");

  lval_in = get_desired(TILT, LOWER_SPEED_LIMIT);
  if (lval_in != (long)uval)
  {
    printf("TILT_LOWER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_LOWER_SPEED_QUERY issued and verified\n");

  if ((status = set_desired(TILT, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, RELATIVE)) == TRUE)
  {
    return (return_error_status("TILT_SET_LOWER_SPEED_LIMIT", status));
  }
  else
    printf("TILT_SET_LOWER_SPEED_LIMIT executed\n");

  lval_in = get_current(TILT, LOWER_SPEED_LIMIT);
  if (lval_in != (long)uval)
  {
    printf("TILT_LOWER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_LOWER_SPEED_QUERY issued and verified\n");

  /* pan set upper speed limit testing */
  uval = 2801;

  if ((status = set_desired(PAN, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_UPPER_SPEED_LIMIT", status));
  }
  else
    printf("\nPAN_SET_UPPER_SPEED_LIMIT executed\n");

  lval_in = get_desired(PAN, UPPER_SPEED_LIMIT);
  if (lval_in != (long)uval)
  {
    printf("PAN_UPPER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_UPPER_SPEED_QUERY issued and verified\n");

  halt(ALL);
  do_delay(1000);
  lval_in = get_current(PAN, UPPER_SPEED_LIMIT);

  if ((status = set_desired(PAN, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, RELATIVE)) == TRUE)
  {
    return (return_error_status("PAN_SET_UPPER_SPEED_LIMIT", status));
  }
  else
    printf("PAN_SET_UPPER_SPEED_LIMIT executed\n");

  if (lval_in != (long)uval)
  {
    printf("PAN_UPPER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("PAN_UPPER_SPEED_QUERY issued and verified\n");

  /* tilt set UPPER speed limit testing */
  uval = 2832;

  if ((status = set_desired(TILT, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("TILT_SET_UPPER_SPEED_LIMIT", status));
  }
  else
    printf("\nTILT_SET_UPPER_SPEED_LIMIT executed\n");

  lval_in = get_desired(TILT, UPPER_SPEED_LIMIT);
  if (abs((int)(lval_in - (long)uval)) > 1)
  {
    printf("TILT_UPPER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_UPPER_SPEED_QUERY issued and verified\n");

  if ((status = set_desired(TILT, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, RELATIVE)) == TRUE)
  {
    return (return_error_status("TILT_SET_UPPER_SPEED_LIMIT", status));
  }
  else
    printf("TILT_SET_UPPER_SPEED_LIMIT executed\n");

  lval_in = get_current(TILT, UPPER_SPEED_LIMIT);
  if (abs((int)(lval_in - (long)uval)) > 1)
  {
    printf("TILT_UPPER_SPEED_QUERY failed: (set,get)=(%u,%ld)\n", uval, lval_in);
    return (return_error());
  }
  printf("TILT_UPPER_SPEED_QUERY issued and verified\n");

  return (PTU_OK);
}

char test_position_bounds(void)
{ /* Query minimum position testing */

  lval_in = get_desired(PAN, MINIMUM_POSITION);
  printf("\nPAN_MINIMUM_POSITION_QUERY RETURNED: %ld\n", lval_in);
  lval_in = get_desired(PAN, MAXIMUM_POSITION);
  printf("PAN_MAXIMUM_POSITION_QUERY RETURNED: %ld\n", lval_in);
  lval_in = get_desired(TILT, MINIMUM_POSITION);
  printf("TILT_MINIMUM_POSITION_QUERY RETURNED: %ld\n", lval_in);
  lval_in = get_desired(TILT, MAXIMUM_POSITION);
  printf("TILT_MAXIMUM_POSITION_QUERY RETURNED: %ld\n", lval_in);

  printf("Are the above values correct? (Enter 'y' or 'n'): ");
  for (;;)
  {
    switch (tmpChar = ((char)tolower(getchar())))
    {
      case 'y':
        printf("\nMin/Max position queries PASSED\n\n");
        return (PTU_OK);
      case 'n':
        return (TEST_ERROR);
    }
  }
}

char test_acceleration_commands(void)
{
  /* pan TEST SET AND QUERY ACCELERATION */
  lval = lval2 = 16666;

  if ((status = set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&lval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("PAN_SET_ACCELERATION", status));
  }
  else
    printf("\nPAN_SET_ACCELERATION executed\n");

  lval_in = get_desired(PAN, ACCELERATION);
  if (lval_in != lval)
  {
    printf("PAN_ACCELERATION_QUERY failed: (set,get)=(%ld,%ld)\n", lval, lval_in);
    return (return_error());
  }
  printf("PAN_ACCELERATION_QUERY issued and verified\n");

  lval = 16777;
  lval2 += lval;
  if ((status = set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&lval, RELATIVE)) == TRUE)
  {
    return (return_error_status("PAN_SET_ACCELERATION", status));
  }
  else
    printf("PAN_SET_ACCELERATION executed\n");

  lval_in = get_desired(PAN, ACCELERATION);
  if (lval_in != lval2)
  {
    printf("PAN_ACCELERATION_QUERY failed: (set,get)=(%ld,%ld)\n", lval2, lval_in);
    return (return_error());
  }
  printf("PAN_ACCELERATION_QUERY issued and verified\n");

  /* tilt TEST SET AND QUERY ACCELERATION */
  lval = lval2 = 16888;

  if ((status = set_desired(TILT, ACCELERATION, (PTU_PARM_PTR *)&lval, ABSOLUTE)) == TRUE)
  {
    return (return_error_status("TILT_SET_ACCELERATION", status));
  }
  else
    printf("\nTILT_SET_ACCELERATION executed\n");

  lval_in = get_desired(TILT, ACCELERATION);
  if (lval_in != lval)
  {
    printf("TILT_ACCELERATION_QUERY failed: (set,get)=(%ld,%ld)\n", lval, lval_in);
    return (return_error());
  }
  printf("TILT_ACCELERATION_QUERY issued and verified\n");

  lval = 16999;
  lval2 += lval;
  if ((status = set_desired(TILT, ACCELERATION, (PTU_PARM_PTR *)&lval, RELATIVE)) == TRUE)
  {
    return (return_error_status("TILT_SET_ACCELERATION", status));
  }
  else
    printf("TILT_SET_ACCELERATION executed\n");

  lval_in = get_desired(TILT, ACCELERATION);
  if (lval_in != lval2)
  {
    printf("TILT_ACCELERATION_QUERY failed: (set,get)=(%ld,%ld)\n", lval2, lval_in);
    return (return_error());
  }
  printf("TILT_ACCELERATION_QUERY issued and verified\n");

  return (PTU_OK);
}

char test_resolution_queries(void)
{
  /* test the resolution query commands */
  lval_in = get_desired(PAN, RESOLUTION);
  printf("\nPAN_RESOLUTION_QUERY RETURNED: %.3f arc min\n", (lval_in / 3600.0));
  lval_in = get_desired(TILT, RESOLUTION);
  printf("TILT_RESOLUTION_QUERY RETURNED: %.3f arc min\n", (lval_in / 3600.0));
  lval_in = get_current(PAN, RESOLUTION);

  printf("Are the above values correct? (Enter 'y' or 'n'): ");
  for (;;)
  {
    switch (tmpChar = ((char)tolower(getchar())))
    {
      case 'y':
        printf("Resolution queries PASSED\n\n");
        return (PTU_OK);
      case 'n':
        return (TEST_ERROR);
    }
  }
}

char test_command_execution_modes(void)
{ /* test the command execution modes */

  if ((status = set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY)) == TRUE)
  {
    return (return_error_status("SET_IMMEDIATE_COMMAND_MODE", status));
  }
  else
    printf("\nSET_IMMEDIATE_COMMAND_MODE executed\n");
  val = 2000;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  do_delay(2000);
  if ((lval = get_current(PAN, POSITION)) != val)
  {
    printf("! SET_IMMEDIATE_COMMAND_MODE verification failed: %ld != %d\n", lval, val);
    return (return_error());
  }
  else
    printf("SET_IMMEDIATE_COMMAND_MODE verified\n");

  if ((status = set_mode(COMMAND_EXECUTION_MODE, EXECUTE_UPON_IMMEDIATE_OR_AWAIT)) == TRUE)
  {
    return (return_error_status("SET_SLAVED_COMMAND_MODE", status));
  }
  else
    printf("SET_SLAVED_COMMAND_MODE executed\n");
  val2 = 0;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val2, ABSOLUTE);
  do_delay(500);
  if ((lval = get_current(PAN, POSITION)) != val)
  {
    printf("! SET_SLAVED_COMMAND_MODE verification failed: %ld != %d\n", lval, val);
    return (return_error());
  }
  else
    printf("SET_SLAVED_COMMAND_MODE verified\n");
  set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY);
  if ((status = set_mode(COMMAND_EXECUTION_MODE, EXECUTE_UPON_IMMEDIATE_OR_AWAIT)) == TRUE)
  {
    return (return_error_status("SET_SLAVED_COMMAND_MODE", status));
  }
  else
    printf("SET_SLAVED_COMMAND_MODE executed\n");

  return (PTU_OK);
}

char test_halt_commands(void)
{ /* test the halt commands */
  {
    short int val1 = 1900;
    short int val2 = -1900;
    unsigned short int uval = 1000;
    short int val3 = -850;
    short int val4 = 350;
    long lval, lval_in;

    set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY);
    set_desired(PAN, SPEED, (PTU_PARM_PTR *)&uval, ABSOLUTE);
    set_desired(TILT, SPEED, (PTU_PARM_PTR *)&uval, ABSOLUTE);
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val1, ABSOLUTE);
    set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val3, ABSOLUTE);
    await_completion();
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val2, ABSOLUTE);
    set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val4, ABSOLUTE);
    do_delay(500);
    if ((status = halt(ALL)) == TRUE)
    {
      return (return_error_status("HALT(ALL)", status));
    }
    else
      printf("\nHALT(ALL) executed\n");
    await_completion();
    if ((lval = get_current(PAN, POSITION)) == val2)
    {
      printf("! HALT(ALL) verification failed to stop pan axis movement (%ld)\n", lval);
      return (return_error());
    }
    else
      printf("HALT(ALL) verified\n");

    /* test halt(tilt) */
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val1, ABSOLUTE);
    set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val3, ABSOLUTE);
    await_completion();
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val2, ABSOLUTE);
    set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val4, ABSOLUTE);
    do_delay(500);
    if ((status = halt(TILT)) == TRUE)
      return (return_error_status("HALT(TILT)", status));
    else
      printf("HALT(TILT) executed\n");
    await_completion();
    if ((get_current(PAN, POSITION) != val2) || (get_current(TILT, POSITION) == val4))
    {
      printf("HALT(TILT) verification failed to stop pan axis movement or impeded tilt axis\n");
      printf("\nget_current(PAN, POSITION)=%ld/%d, get_current(TILT,POSITION)=%ld/%d\n", get_current(PAN, POSITION),
             val2, get_current(TILT, POSITION), val4);
      return (return_error());
    }
    else
      printf("HALT(TILT) verified\n");

    /* test halt(pan) */
    if (set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val1, ABSOLUTE) || set_desired(TILT, POSITION,
                                                                                   (PTU_PARM_PTR *)&val3, ABSOLUTE))
      printf("\nOrig 1 position commanding error\n");
    await_completion();
    if (set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val2, ABSOLUTE) || set_desired(TILT, POSITION,
                                                                                   (PTU_PARM_PTR *)&val4, ABSOLUTE))
      printf("\nOrig 2 position commanding error\n");
    do_delay(1000);
    if ((status = halt(PAN)) == TRUE)
    {
      return (return_error_status("HALT(PAN)", status));
    }
    else
      printf("HALT(PAN) executed\n");
    await_completion();

    lval = get_current(PAN, POSITION);
    lval_in = get_current(TILT, POSITION);
    if ((lval == val2) || (lval_in != val4))
    {
      printf("! HALT(PAN) verification failed (P:%ld == %d || T:%ld != %d)\n", lval, val2, lval_in, val4);
      return (return_error());
    }
    else
      printf("HALT(PAN) verified\n");

    return (PTU_OK);
  }
}

char test_reset(void)
{
  if ((status = reset_ptu()) != 0)
  {
    return (return_error_status("RESET_PTU", status));
  }
  else
    printf("\nRESET_PTU executed and verified\n");
  if ((status = reset_ptu_pan()) == TRUE)
  {
    return (return_error_status("RESET_PTU_PAN", status));
  }
  else
    printf("RESET_PTU_PAN executed and verified\n");
  if ((status = reset_ptu_tilt()) == TRUE)
  {
    return (return_error_status("RESET_PTU_TILT", status));
  }
  else
    printf("RESET_PTU_TILT executed and verified\n");
  return (PTU_OK);
}

char test_default_operations(void)
{
  unsigned short int uval;

  await_completion();
  lval = 9999;
  uval = 1692;
  set_desired(TILT, SPEED, (PTU_PARM_PTR *)&uval, ABSOLUTE);
  set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&lval, ABSOLUTE);
  if ((status = set_mode(DEFAULTS, SAVE_CURRENT_SETTINGS)) == TRUE)
  {
    return (return_error_status("SAVE_DEFAULTS", status));
  }
  else
    printf("\nSAVE_DEFAULTS executed\n");

  if ((status = set_mode(DEFAULTS, RESTORE_SAVED_SETTINGS)) == TRUE)
  {
    return (return_error_status("RESTORE_SAVED_DEFAULTS", status));
  }
  else
    printf("RESTORE_SAVED_DEFAULTS executed\n");
  lval_in = get_desired(PAN, ACCELERATION);
  uval1 = (unsigned short)get_desired(TILT, SPEED);
  if ((lval_in != lval) || (uval1 != uval))
  {
    printf("! RESTORE_SAVED_DEFAULTS verification failed (PA: %ld, TS: %u)\n", lval_in, uval1);
    return (return_error());
  }
  else
    printf("RESTORE_SAVED_DEFAULTS verified\n");

  if ((status = set_mode(DEFAULTS, RESTORE_FACTORY_SETTINGS)) == TRUE)
  {
    return (return_error_status("RESTORE_FACTORY_SETTINGS", status));
  }
  else
    printf("RESTORE_FACTORY_SETTINGS executed\n");
  if ((get_desired(PAN, SPEED) != 1000) || (get_desired(TILT, ACCELERATION) != 2000))
  {
    return (return_error_status("RESTORE_FACTORY_SETTINGS", 0));
  }
  else
    printf("RESTORE_FACTORY_SETTINGS verified\n");

  return (PTU_OK);
}

char rapid_cycle_test(void)
{
  int i;
  long lval = 50000L;

  val1 = 0;
  val2 = 200;
  set_desired(PAN, ACCELERATION, (PTU_PARM_PTR *)&lval, ABSOLUTE);
  uval = 6600;
  set_desired(PAN, UPPER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE);
  uval = 1200;
  set_desired(PAN, BASE, (PTU_PARM_PTR *)&uval, ABSOLUTE);
  set_mode(COMMAND_EXECUTION_MODE, EXECUTE_IMMEDIATELY);
  for (i = 0; i < 50; i++)
  {
    await_completion();
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val1, ABSOLUTE);
    await_completion();
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val2, ABSOLUTE);
  }
  return (PTU_OK);
}

char test_pure_velocity_control_mode()
{
  if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) != PTU_OK)
  {
    printf("Speed control modes not supported in this PTU firmware version\n");
    return (PTU_OK);
  }
  printf("\nSPEED_CONTROL_MODE set to PTU_PURE_VELOCITY_SPEED_CONTROL_MODE\n");

  val = 600;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  val = -1000;
  set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();
  if (((val1 = (short)get_desired(PAN, POSITION)) >= 0) || ((val2 = (short)get_desired(TILT, POSITION)) >= 0))
  {
    printf("\nSPEED_CONTROL_MODE failed: signed velocity didn't override position command ([p,t]=[%d,%d]\n\n", val1,
           val2);
    goto failed_exit;
  }
  else
  {
    printf("Absolute signed speed controls verified\n");
  }

  val = 600;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();
  val = -1000; /* so absolute speed should be 0 + -1000 = -1000 */
  set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, RELATIVE);
  set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, RELATIVE);
  await_completion();
  if ((get_desired(PAN, POSITION) >= 0) || (get_desired(TILT, POSITION) >= 0))
  {
    printf("\nSPEED_CONTROL_MODE failed: signed relative velocity didn't override position command ([p,t]=[%d,%d]\n\n",
           val1, val2);
    goto failed_exit;
  }
  else
  {
    printf("Relative signed speed controls verified\n");
  }

  set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE);
  printf("\n");
  return (PTU_OK);

  failed_exit: set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE);
  return (return_error());
}

/* Cycle down through pure velocities.
 Move home, send pur velocity command, query current velocity. When equal,
 query position, and print it out.    */
/* returns TRUE when error; otherwise FALSE */
char test_low_speed_moves(void)
{
  int i;

  set_mode(DEFAULTS, RESTORE_SAVED_SETTINGS);
  if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) != PTU_OK)
  {
    printf("Speed control modes not supported in this PTU firmware version\n");
    return TRUE;
  }

  uval = 0;
  set_desired(PAN, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE);
  set_desired(TILT, LOWER_SPEED_LIMIT, (PTU_PARM_PTR *)&uval, ABSOLUTE);
  printf("\ntest_low_speed_moves: SPEED_CONTROL_MODE set to PTU_PURE_VELOCITY_SPEED_CONTROL_MODE\n");

  for (i = -100; i < 0; i++)
  { /* rehome the PTU */
    printf(" %d ", i);
    val = 1000;
    set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
    set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
    val = 0;
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
    set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
    await_completion();

    /* printf("\ni=%d:  ", i); */
    val = i;
    set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
    set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
    Sleep(500);

    val1 = (short)get_current(PAN, SPEED);
    val2 = (short)get_current(TILT, SPEED);
    val3 = (short)get_desired(PAN, SPEED);
    val4 = (short)get_desired(TILT, SPEED);
    /* printf(" C(%d,%d) D(%d,%d) ", val1, val2, val3, val4); */

    if ((val1 != i) || (val2 != i))
    {
      printf(" Current speed ERROR! ==>C(%d,%d), D(%d,%d)\n", val1, val2, val3, val4);
      return TRUE;
    }
  }
  val = 0;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();
  set_mode(DEFAULTS, RESTORE_SAVED_SETTINGS);
  printf("\nrestored_saved_settings\n");
  return FALSE;
}

char y_or_n()
{
  for (;;)
  {
    switch (tolower(getchar()))
    {
      case 'y':
        return (TRUE);
      case 'n':
        return (FALSE);
    }
    printf("(enter 'y' or 'n'): ");
  }
}

/* crude test program, most return status not processed (needs to be added in operational code) */
char test_triggers(portstream_fd COMstream)
{
  signed short int triggersPending, readTriggersPending;
  /* go to a known start position */
  val = 0;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();
  /* ensure no triggers pending */
  TriggerOff();
  triggersPending = TriggersPending();
  if (triggersPending != 0)
  {
    printf("\nTriggers pending = %d, should be zero... ", triggersPending);
    TriggerOff();
    printf("\nExecuted TriggerOff, now triggers pending = %d (should be zero)... ", TriggersPending());
  }
  /* configure the triggers to start at pos 100, trigger every 100, and do 6 triggers */
  printf("\n\nTriggerOn(100, 100, 6)");
  TriggerOn(100, 100, 6);
  triggersPending = TriggersPending();
  /* now move to trigger positions, verifying the trigger appears and the trigger counts down properly */
  for (val = 100; val <= 600; val += 100)
  {
    printf("\n\nMoving to position %d with %d triggers pending\n", val, triggersPending);
    set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
    await_completion();
    triggersPending--;
    readTriggersPending = TriggersPending();
    printf("   ... position %d achieved, %d triggers pending now...", val, readTriggersPending);
    if (triggersPending != readTriggersPending)
      printf("\n! Triggers pending read value of %d when should be %d\n", readTriggersPending, triggersPending);
    printf("\n   position %d trigger hit? ", triggersPending + 1);
    y_or_n();
  }
  TriggerOff();
  printf("\n\nDone testing triggers...\n\n");
  return (TRUE);
}

void test_DB15_extensions(portstream_fd COMstream)
{
  set_TTL_outputs(8); // DB15 pin15
  do_delay(2000);
  test_triggers(COMstream); // DB15 pin12
  do_delay(2000);
  set_TTL_outputs(0); // DB15 pin15
  do_delay(2000);
  return;
}

char test_speed_ramping()
{
  signed short int speed;
  unsigned char status;
  char axis = PAN;
  int i, zero_count = 0;

  if (set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE) != PTU_OK)
  {
    printf("Speed control modes not supported in this PTU firmware version\n");
    return TRUE;
  }

  for (i = 0; i < 2; i++)
  {
    printf("\n\nRunning up...\n");

#define TOP_SPEED 8000

    for (speed = 1; speed < TOP_SPEED; speed++)
    { // Sleep(0);
      if ((status = set_desired(axis, SPEED, (PTU_PARM_PTR *)&speed, ABSOLUTE)) != PTU_OK)
      {
        set_error_string(status);
        printf("\nError at %d status %d (%s)", speed, status, error_string);
        Sleep(5000);
      }

      zero_count = 0;
      while ((speed - (val1 = (short)get_current(axis, SPEED))) > 0)
      {
        if (val1 == 0)
          zero_count++;
        if (zero_count > 10)
          return (-1);
        if (zero_count > 3)
          printf("  (C%d, D%d)", val1, speed);
        //printf(" (hit key to continue): "); getchar();
      }
      if ((speed % 100) == 0)
        printf("\nspeed %d... ", speed);

    }
    for (speed = TOP_SPEED; speed > -TOP_SPEED; speed--)
    {
      if ((status = set_desired(axis, SPEED, (PTU_PARM_PTR *)&speed, ABSOLUTE)) != PTU_OK)
      {
        set_error_string(status);
        printf("\nError at %d status %d (%s)", speed, status, error_string);
      }
      if ((speed % 100) == 0)
        printf("\nspeed %d... ", speed);
    }

  }
  return (-1);
}

char test_circular_motion()
{
  signed short int panSpeed = 0;
  signed short int tiltSpeed = 0;
  doubleps = 0;
  doublets = 0;
  int ix = 0;
  int total = 0;

  set_mode(SPEED_CONTROL_MODE, PTU_PURE_VELOCITY_SPEED_CONTROL_MODE);

  printf("Starting Now.\r\n");

  for (ix = 0; ix < 5000; ix++)
  {
    ps = sin((double)ix / 100) * 1000;
    ts = cos((double)ix / 100) * 4000;
    panSpeed = (signed short int)ps;
    tiltSpeed = (signed short int)ts;

    total += execute_set_desired_velocities(&panSpeed, &tiltSpeed);
  }
  panSpeed = 0;
  tiltSpeed = 0;
  execute_set_desired_velocities(&panSpeed, &tiltSpeed);
  printf("Finishing Now: %d\r\n", total);
  return -1;
}

char test_ism_commands()
{
  short speedVal;
  short inVal;
  //This enables stabilization
  select_unit(128);
  //Move the Pan-Tilt
  printf("Move the pan tilt - is it stabilized?");
  y_or_n();
  //Test Pan Speed
  speedVal = 1000;
  set_desired(PAN, SPEED, &speedVal, ABSOLUTE);
  inVal = (short)get_desired(PAN, SPEED);
  if (speedVal != inVal)
  {
    printf("Error setting pan speed, desired=%d, read=%d\r\n", speedVal, inVal);
    goto failed_exit;
  }
  do_delay(2000);
  speedVal = 0;
  set_desired(PAN, SPEED, &speedVal, ABSOLUTE);
  printf("Pan speed verified.\r\n");
  //Test Tilt Speed
  speedVal = 1000;
  set_desired(TILT, SPEED, &speedVal, ABSOLUTE);
  inVal = (short)get_desired(TILT, SPEED);
  if (speedVal != inVal)
  {
    printf("Error setting tilt speed, desired=%d, read=%d\r\n", speedVal, inVal);
    goto failed_exit;
  }
  do_delay(2000);
  speedVal = 0;
  set_desired(TILT, SPEED, &speedVal, ABSOLUTE);
  printf("Tilt speed verified.\r\n");
  //Take back out of ISM control
  select_unit(0);
  return 0;

  failed_exit: select_unit(0);
  return (return_error());
}

/************************************************************************************/
/*****                                                                          *****/
/*****                        MAIN TEST PROGRAM                                 *****/
/************************************************************************************/

int main()
{
  int COMportNum, BaudRate;
  char COMportName[256], tmpChar;
  portstream_fd COMstream;

  char COMportPrefix[15] = "/dev/ttyUSB";
  /* parse the input arguments */

  printf("\n\n\n****** PAN-TILT BINARY TEST PROGRAM, %s\n", PTU_CPI_CODE_VERSION);
  printf("****** Serial Port Interface Driver, %s\n", SERIAL_CODE_VERSION);
  printf("****** (c)2000 - 2007, Directed Perception, Inc. All Rights Reserved.\n");
  COMportName[0] = ' ';
  while (COMportName[0] == ' ')
  {
    printf("\nEnter the %s port number the PTU is attached to: ", COMportPrefix);
    scanf("%d", &COMportNum);
    printf("You selected %s%d. Is this OK? (enter 'y' or 'n'): ", COMportPrefix, COMportNum);
    tmpChar = 'f';
    while ((tmpChar != 'y') && (tmpChar != 'n'))
      tmpChar = ((char)tolower(getchar()));
    if (tmpChar == 'y')
      sprintf(COMportName, "%s%d", COMportPrefix, COMportNum);
  }
  tmpChar = 'f';
  while (tmpChar != 'y')
  {
    printf("\nEnter the baud rate the ptu is communicate at (default: 9600): ");
    scanf("%d", &BaudRate);
    printf("You selected %d. Is this OK? (enter 'y' or 'n'): ", BaudRate);
    tmpChar = 'f';
    while ((tmpChar != 'y') && (tmpChar != 'n'))
      tmpChar = ((char)tolower(getchar()));
  }

  /* initialize the serial port */
  set_baud_rate(BaudRate);
  COMstream = open_host_port(const_cast<char*>(COMportName));

  if (COMstream == PORT_NOT_OPENED)
  {
    printf("\nSerial Port setup error.\n");
    goto abnormal_exit;
  }
  printf("\nSerial port %s initialized\n", COMportName);

  //Just in case there is an ISM on the line, talk directly to PTU.
  SerialStringOut(COMstream, (unsigned char *)"   ");
  select_unit(0);

  /* begin exercising the PTU command set */

  switch (reset_PTU_parser(5000))
  {
    case PTU_OK:
      break;
    case PTU_FIRMWARE_VERSION_TOO_LOW:
      printf("\nError(reset_PTU_parser): PTU FIRMWARE VERSION MATCH ERROR\n");
      goto abnormal_exit;
    case PTU_NOT_RESPONDING:
      printf("\nError(reset_PTU_parser): PTU_NOT_RESPONDING\n");
      goto abnormal_exit;
  }
  printf("\nPending commands and input buffer flushed\n");

  printf("\n\nFirmware version: %s\nFirmware version command executed successfully\n", firmware_version());

  if (test_reset())
    goto abnormal_exit;
  if (test_position_commands())
    goto abnormal_exit;
  if (test_position_limits())
    goto abnormal_exit;
  if (test_power_modes())
    goto abnormal_exit;
  if (test_speed_commands())
    goto abnormal_exit;
  if (test_position_bounds())
    goto abnormal_exit;
  if (test_verbose_modes())
    goto abnormal_exit;
  if (test_ASCII_echo_modes())
    goto abnormal_exit;
  if (test_base_speed_commands())
    goto abnormal_exit;
  if (test_acceleration_commands())
    goto abnormal_exit;
  if (test_speed_limits())
    goto abnormal_exit;
  if (test_resolution_queries())
    goto abnormal_exit;
  if (test_command_execution_modes())
    goto abnormal_exit;
  if (test_halt_commands())
    goto abnormal_exit;
  if (test_default_operations())
    goto abnormal_exit;
  if (test_pure_velocity_control_mode())
    goto abnormal_exit;
  /* if ( test_ism_commands() ) goto abnormal_exit; */
  /*if ( test_circular_motion() ) goto abnormal_exit;*/

  printf("\n\nBINARY COMMAND SET TESTS FINISHED SUCCESSFULLY!!\n\n\n");

  /* rehome the PTU */
  set_mode(DEFAULTS, RESTORE_SAVED_SETTINGS);
  val = 0;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();

  goto exit;

  abnormal_exit: printf("\nABNORMAL EXIT: test failed\n\n");
  Sleep(5000);

  exit:
  /* rehome the PTU */
  val = 1000;
  set_desired(PAN, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, SPEED, (PTU_PARM_PTR *)&val, ABSOLUTE);
  val = 0;
  set_desired(PAN, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  set_desired(TILT, POSITION, (PTU_PARM_PTR *)&val, ABSOLUTE);
  await_completion();
  /* FlushInputBuffer(); */
  return_error();
  close_host_port(COMstream);
  return (TRUE);

}

