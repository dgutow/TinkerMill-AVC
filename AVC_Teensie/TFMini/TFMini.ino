///////////////////////////////////////////////////////////////////////////////
// Main file for the TFMini racer
///////////////////////////////////////////////////////////////////////////////

#include "TFMiniClass.h"

///////////////////////////////////////////////////////////////////////////////
// With or without the '_t'
///////////////////////////////////////////////////////////////////////////////
#define uint32  uint32_t
#define int32   int32_t
#define uint16  uint16_t
#define int16   int16_t
#define uint8   uint8_t
#define int8    int8_t


#define DBGPORT   Serial        // (USB) Serial port for textual debugging
#define TFPORT    Serial2       // Serial port connected to the TFMini 


#define TEENSIE_35
#ifdef TEENSIE_35
    const int TEENSIE_LED   = 13;   // LED on the board. teensie 3.2 - led 13
#else
    const int TEENSIE_LED   = 6;    // LED on the board. teensie 2++ - led 6
#endif

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Original Libraries/System header files
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Global variables
///////////////////////////////////////////////////////////////////////////////
// union rangeUnion rangeMsg;

///////////////////////////////////////////////////////////////////////////////
// Function templates for the taskList
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////

TFMiniClass tfmini(TFPORT);            //The TFMini

///////////////////////////////////////////////////////////////////////////////
// initialization
///////////////////////////////////////////////////////////////////////////////

void setup()
{ 
   TFPORT.begin(115200);
   DBGPORT.begin(115200);
   
  
   pinMode(TEENSIE_LED, OUTPUT);
   digitalWrite(TEENSIE_LED, 1);
   
   delay (1000); 
   DBGPORT.print("Setup complete \n");
}

///////////////////////////////////////////////////////////////////////////////
// The main loop
///////////////////////////////////////////////////////////////////////////////
int retVal;
int counter = 0;

void loop()
{                 
    delay (2);
    retVal = tfmini.getRange();
   
    if (retVal == 9)                         // We got a message
    {
           DBGPORT.print("Range = ");
           DBGPORT.print(tfmini.rangeMsg.msg.distance);
           DBGPORT.print(" nResync = ");
           DBGPORT.print(tfmini.nResync); 
           DBGPORT.print(" nDropped = ");
           DBGPORT.print(tfmini.nDropped); 
           DBGPORT.print(" strength = ");
           DBGPORT.print(tfmini.rangeMsg.msg.strength);           
           DBGPORT.print(" quality = ");
           DBGPORT.print(tfmini.rangeMsg.msg.quality);     
           DBGPORT.print("\n"); 
           counter = 0;          
           
           blinkLed (0);           
    } 
    else if (retVal == 0)
    {
        if (counter++ > 100)
        {
           DBGPORT.println("No data for 100 loops (2 msec)");    
           counter = 0;
        }
    }
    else if (retVal == -1)    
    {
        // Need to resync
        DBGPORT.print("Need to resync \n"); 
    }  
    else if (retVal == -2)                  // Lost a bit or two somewhere
    {
        // Lost a bit or two somewhere           
        DBGPORT.print("Lost Bit \n");           
    }           
           

}

///////////////////////////////////////////////////////////////////////////////
// TFMiniClass - class to get range data from a TFMini rangefinder.
///////////////////////////////////////////////////////////////////////////////
#if 0
class TFMiniClass
{
public:    

    // The structure of the msg from a TFMini
    struct rangeStruct
    {
        uint16  header;
        uint16  distance;
        uint16  strength;
        uint8   reserved;
        uint8   quality;
        uint8   checksum;
    }; 
    
    // Union used to access a msg as either an array of bytes of a rangeStruct
    union rangeUnion
    {
        struct rangeStruct     msg;
        uint8                   bytes[9];
    };

    ///////////////////////////////////////////////////////////////////////////////
    // constructor
    ///////////////////////////////////////////////////////////////////////////////     
    TFMiniClass(Stream serport)
    {
        serialPort = serport;
        serialPort.begin(115200);
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // int getRange() - checks if there are 9 bytes at the serial port and if
    // if so reads them and checks the msg for integrity.
    ///////////////////////////////////////////////////////////////////////////////
    int getRange()
    {
        if (serialPort.available() >= 9) // A valid command consists of 8 (bytes)
        {
            for (int i = 0; i < 9; i++)
            {
                rangeMsg.bytes[i] = serialPort.read();           
            }
        }
        else
        {
            return (0);
        }
        
        // We got a complete messag - check it for errors
        int err = tfmini.checkRangeMsg();    // Check if it came through OK     

        if (err == -1)                       // The header was wrong.
        {
            // need to resync
            nResync++;
            resync();
            return (err);
        }
        else if (err == -2)                  // Lost a bit or two somewhere
        {
            // Just throw this message away           
            nDropped++; 
            return (err);       
        }
        
        return (9);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // void resync() - resynchronize the message. Waits 1 msec to make sure the
    // rest of the current message arrives, then throws everything away, to start
    // fresh with the next message.   
    ///////////////////////////////////////////////////////////////////////////////
    
    void resync()
    {
        delay(1);
        while (serialPort.available())
        {
            serialPort.read();       
        }
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // void checkRangeMsg() - checks the range msg for errors.
    ///////////////////////////////////////////////////////////////////////////////
    
    int checkRangeMsg()
    {
        // Check if the first two bytes are correct.  If not we're out of sync.
        if (rangeMsg.msg.header != 0x5959)
            return -1;
        
        // Calculate the checksum.  The checksum byte is the sum of the first
        // eight bytes.
        uint8 sum = 0;
        for (int i = 0; i < 8; i++) 
        {
            sum += rangeMsg.bytes[i];
        }
        
        // Compare the checksums 
        if (sum != rangeMsg.msg.checksum)
        {
            return (-2);
        }
        
        return (0);
    }

    ///////////////////////////////////////////////////////////////////////////////
    // uint_swap - swaps the bytes.
    ///////////////////////////////////////////////////////////////////////////////
    // unsigned int uint_swap(unsigned int val)
    // {
    // unsigned int retval = (val >> 8) | (val << 8);
    // return (retval);
    // }
    
    ///////////////////////////////////////////////////////////////////////////////
    // class variables
    ///////////////////////////////////////////////////////////////////////////////  
    Stream serialPort;          // Which serial port the TFMini is connected to 
        
    union rangeUnion rangeMsg;  // The msg received from the TFMini
        
    int32 nResync = 0;          // number of messages needing resynch
    int32 nDropped= 0;          // number of messages dropped due to bad checksum   
}
#endif
///////////////////////////////////////////////////////////////////////////////
// blinkLed - blink the onboard TEENSIE_LED
///////////////////////////////////////////////////////////////////////////////
void blinkLed (uint32_t currTimeMsec)
{
  static int blinkVal = 0;
      
  if (blinkVal == 0)
  {
    digitalWrite(TEENSIE_LED, HIGH);
    blinkVal = 1;
  }
  else
  {
    digitalWrite(TEENSIE_LED, LOW);
    blinkVal = 0;
  }
}

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
