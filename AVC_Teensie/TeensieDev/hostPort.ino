///////////////////////////////////////////////////////////////////////////////
// The hostPort is the serial port to the host computer - the Raspberry Pi. 
///////////////////////////////////////////////////////////////////////////////

#include "constants.h"
#include "commands.h"

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////
bool hostPortin = false;
uint16  serialin = true;

union cmdData command;

///////////////////////////////////////////////////////////////////////////////
// Function templates
///////////////////////////////////////////////////////////////////////////////
int hostPortEvent();


///////////////////////////////////////////////////////////////////////////////
// initHostPort - initialize the communications to/from the host
///////////////////////////////////////////////////////////////////////////////
void initHostPort ()
{
    RPIPORT.begin(115200);
}

///////////////////////////////////////////////////////////////////////////////
// checkForCmds - called from the main loop
///////////////////////////////////////////////////////////////////////////////

void checkForCmds (uint32_t currTimeMsec)
{
  /* original code
  if (serialin)
  {
      if (hostPortEvent() == 10) // A valid command consists of 8 (bytes)
      {
          commandDecode(&command);
      }
  }
  */ 

    if ( (uint) RPIPORT.available() >= sizeof(cmdData) ) 
    {
        // There is at least one commands worth of bytes available to read
        // So read one commands worth right now
        for (uint i = 0; i < sizeof(cmdData); i++)
        {
            command.bytes[i] = RPIPORT.read();           
        }
        
        // Check if the first two bytes correspond to the header
        if (command.words[0] == CMD_HEADER)
        {
            // Yep, the header is right so decode the command
            commandDecode(&command);
        } 
        else
        {
            // Incorrect header on command message - resync by waiting for 
            // a short while to ensure all the bytes come in and then throwing
            // all the bytes away.  The message after that should be in sync.
            telem.rejectCntr++;
            telem.rejectReason = 1;              
            
            delay(2); 
            while ( RPIPORT.available() )
            {
                RPIPORT.read();
            }
        }   
    }
}


///////////////////////////////////////////////////////////////////////////////
// void hostPortEvent() - checks if there are 8 bytes at the serial port and if
// if so reads them into command and returns the number of bytes read 
// (should always be either 0 or 8)
///////////////////////////////////////////////////////////////////////////////

int hostPortEvent()
{
    if (RPIPORT.available() >= 10) // A valid command consists of 10 (bytes)
    {
        for (int i = 0; i < 10; i++)
        {
            command.bytes[i] = RPIPORT.read();           
        }
        return (10);
    }
    return (0);
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////