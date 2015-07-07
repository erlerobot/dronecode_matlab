#include "startup_menu.h"
#include "FastSerial.h"

bool check_for_auto_req(FastSerial* ps)
{
}

void print_greeting(FastSerial* ps)
{
  ps->println("");
  ps->println("Welcome to ArduPilot 2.0!");
  ps->println("");
}

uint8_t get_no_data_found_request(FastSerial* ps)
{
  uint32_t t0;
  char c, d;

  ps->println("No data was found in memory. No new data will be recorded unless armed here.");
  ps->println("");
  ps->println(" Select:");
  ps->println("   (e) Erase memory");
  ps->println("   (l) Do nothing");
  ps->println("   (a) Arm data recording");
  ps->println("");

  c = get_char_timeout(ps);
  if (c != 0)
    {
      switch (c)
	{
	case 'e':
	  ps->println("Erasing memory...");
	  return ANS_ERASE_MEMORY;
	  break;
	case 'l':
	  ps->println("Leaving data alone...");
	  return ANS_LEAVE_ALONE;
	  break;
	case 'a':
	  ps->println("Arming data recording...");
	  return ANS_ARM_DATA_REC;
	  break;
	default:
	  ps->println("Don't know that letter.");
	}
    }

  return ANS_NO_RESPONSE;
}

uint8_t get_char_timeout(FastSerial* ps)
{
  uint32_t t0;
  uint8_t c;

  t0 = millis();
  while ((ps->available()==0) && (millis()<(t0+TIMEOUT)))
    { 
      /* waiting for user input */
    }
  if (ps->available() != 0)
    {
      c = (unsigned char)ps->read();
      return c;
    }
  
  return 0;
}

uint8_t get_char(FastSerial* ps)
{
  uint8_t c;


  while (ps->available()==0)
    { 
      /* waiting for user input */
    }
  c = (unsigned char)ps->read();
  return c;
}
      
uint8_t get_data_request(FastSerial* ps)
{

  char c, d;
  
  ps->println("You've got data! ");
  ps->println("");
  ps->println(" Select:");
  ps->println("   (p) Print data to this terminal as text");
  ps->println("   (e) Erase memory");
  ps->println("   (o) Overwrite data. May lead to corrupt data sets if new");
  ps->println("       data set is shorter than the one it is overwriting!");
  ps->println("   (l) Leave data alone. No new data will be recorded.");
  ps->println("");

  c = get_char_timeout(ps);
  if (c != 0)
    {
      switch (c)
	{
	case 'p':
	  return ANS_PRINT_TO_TERMINAL;
	  break;
	case 'e':
	  print_warning(ps);
	  ps->println("YOU ARE ABOUT TO ERASE ALL MEMORY. CONFIRM (y/n): ");
	  d = get_char(ps);

	  if ( (d == 'y') || (d == 'Y') )
	    {
	      ps->println("Erasing memory...this may take a few minutes...");
	      return ANS_ERASE_MEMORY;
	    }

	  break;
	case 'o':
	  print_warning(ps);
	  ps->println("YOU ARE ABOUT TO OVERWRITE MEMORY. CONFIRM (y/n): ");
	  d = get_char(ps);

	  if ( (d == 'y') || (d == 'Y') )
	    {
	      ps->println("Overwriting file header...");
	      return ANS_OVERWRITE;
	    }

	  break;
	case 'l':
	  return ANS_LEAVE_ALONE;
	  break;
	default:
	  ps->println("Don't know that letter.");
	}
    }

  return ANS_NO_RESPONSE;
}

void print_warning(FastSerial *ps)
{
  ps->println("            ! ! !   W A R N I N G   ! ! !                  ");
  ps->println("");
}

void print_goodbye(FastSerial *ps)
{
  ps->println("Exiting menu system. Goodbye.");
}
