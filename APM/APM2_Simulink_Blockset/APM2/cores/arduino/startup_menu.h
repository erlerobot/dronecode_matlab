#ifndef _STARTUP_MENU_H
#define _STARTUP_MENU_H

#include "Arduino.h"
#include <stdbool.h>
#include <inttypes.h>
#include "FastSerial.h"

/* Constants */
#define TIMEOUT               10000
#define ANS_PRINT_TO_TERMINAL     0
#define ANS_ERASE_MEMORY          1
#define ANS_LEAVE_ALONE           2
#define ANS_NO_RESPONSE           3
#define ANS_OVERWRITE             4
#define ANS_ARM_DATA_REC          5
#define AUTO_REQ_BYTE1          0x97  // Request automatic help if available
#define AUTO_REQ_BYTE2          0x98  // Request automatic help if available


/* these are all FastSerial pointers but Matlab gets angry so they are just declared
 * here as void pointers.
 */
void    print_greeting(FastSerial* ps);
uint8_t get_data_request(FastSerial *ps);
void    print_warning(FastSerial *ps);
void    print_goodbye(FastSerial *ps);
uint8_t get_char(FastSerial* ps);
uint8_t get_char_timeout(FastSerial* ps);
uint8_t get_no_data_found_request(FastSerial* ps);
bool    check_for_auto_req(FastSerial* ps);

#endif /* Startup menu */

