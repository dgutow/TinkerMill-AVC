///////////////////////////////////////////////////////////////////////////////
// diagnostics - Functions associated with the diagnostics structure.  
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "diagnostics.h"
#include "constants.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////
Diagnostics diag;            // The diagnostics class

///////////////////////////////////////////////////////////////////////////////
// diag_init - initialize the diagnostic system
///////////////////////////////////////////////////////////////////////////////
void diag_init ()
{
   // Initialize the serial ports 
   // DBGPORT.begin(9600); 
   
   // initialize the structure
   diag.pktId          = 0x22222222;
   diag.time           = millis();
   diag.string[0]      = '\0'; 
   diag.data[0]        = 0;
   diag.data[1]        = 0;
   diag.data[2]        = 0;
   diag.data[3]        = 0;
   diag.data[4]        = 0;
   diag.data[5]        = 0;
   diag.data[6]        = 0;
   diag.data[7]        = 0;
}


///////////////////////////////////////////////////////////////////////////////
// diag_sendToHost - sends the diagnostic structure to the Rpi as a binary message
// NOTE dag make this packet 54 bytes...
///////////////////////////////////////////////////////////////////////////////
void diag_sendToHost (char* str, int32 data0 = 0, int32 data1 = 0, 
                                 int32 data2 = 0, int32 data3 = 0,
                                 int32 data4 = 0, int32 data5 = 0, 
                                 int32 data6 = 0, int32 data7 = 0)
{
   diag.pktId       = 0x22222222;    
   diag.time        = millis();
   diag.data[0]     = data0;  
   diag.data[1]     = data1;    
   diag.data[2]     = data2;     
   diag.data[3]     = data3;     
   diag.data[4]     = data4;     
   diag.data[5]     = data5;      
   diag.data[6]     = data6;      
   diag.data[7]     = data7;    
   
   // RPIPORT.write ((byte*) &diag, sizeof(diag));
}
