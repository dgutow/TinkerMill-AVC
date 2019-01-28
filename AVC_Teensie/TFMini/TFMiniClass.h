///////////////////////////////////////////////////////////////////////////////
// Main file for the TFMini ranger
///////////////////////////////////////////////////////////////////////////////

#ifndef TFMINI_CLASS
#define TFMINI_CLASS

///////////////////////////////////////////////////////////////////////////////
// TFMiniClass - class to get range data from a TFMini rangefinder.
///////////////////////////////////////////////////////////////////////////////
class TFMiniClass
{
public:    

    // The structure of the msg from a TFMini
    struct rangeStruct
    {
        uint16_t  header;
        uint16_t  distance;
        uint16_t  strength;
        uint8_t   reserved;
        uint8_t   quality;
        uint8_t   checksum;
    }; 
    
    // Union used to access a msg as either an array of bytes of a rangeStruct
    union rangeUnion
    {
        struct rangeStruct     msg;
        uint8_t                bytes[9];
    };

    ///////////////////////////////////////////////////////////////////////////////
    // constructor
    ///////////////////////////////////////////////////////////////////////////////     
    TFMiniClass(HardwareSerial& serport)
    {
        serialPort = serport;
        serialPort.begin(115200);
        
        serialPort.write((uint8_t)0x42);
        serialPort.write((uint8_t)0x57);
        serialPort.write((uint8_t)0x02);
        serialPort.write((uint8_t)0x00);
        serialPort.write((uint8_t)0x00);
        serialPort.write((uint8_t)0x00);
        serialPort.write((uint8_t)0x01);
        serialPort.write((uint8_t)0x06);       
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // int getRange() - checks if there are 9 bytes at the serial port and if
    // if so reads them and checks the msg for integrity.
    ///////////////////////////////////////////////////////////////////////////////
    int32_t getRange()
    {
        if (serialPort.available() >= 9) // A valid command consists of 9 bytes
        {
            for (int32_t i = 0; i < 9; i++)
            {
                rangeMsg.bytes[i] = serialPort.read();           
            }
        }
        else
        {
            return (0);
        }
        
        // We got a complete messag - check it for errors
        int32_t err = checkRangeMsg();    // Check if it came through OK     

        if (err == -1)                    // The header was wrong.
        {
            // need to resync
            nResync++;
            resync();
            return (err);
        }
        else if (err == -2)               // Lost a bit or two somewhere
        {
            // Just throw this message away           
            nDropped++; 
            return (err);       
        }
        
        return (9);                       // Everything worked, return #bytes
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
    // void reset() - resets the serial port  
    ///////////////////////////////////////////////////////////////////////////////
    void reset()
    {
        serialPort.end();   
        serialPort.begin(115200);
    }
    
    ///////////////////////////////////////////////////////////////////////////////
    // void checkRangeMsg() - checks the range msg for errors.
    ///////////////////////////////////////////////////////////////////////////////
    int32_t checkRangeMsg()
    {
        // Check if the first two bytes are correct.  If not we're out of sync.
        if (rangeMsg.msg.header != 0x5959)
            return -1;
        
        // Calculate the checksum.  The checksum byte is the sum of the first
        // eight bytes.
        uint8_t sum = 0;
        for (int32_t i = 0; i < 8; i++) 
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
    HardwareSerial &serialPort = Serial2; // Which serial port the TFMini is connected to 
        
    union rangeUnion rangeMsg;      // The msg received from the TFMini
        
    int32_t nResync = 0;            // number of messages needing resynch
    int32_t nDropped= 0;            // number of messages dropped due to bad checksum   
    
};              // end class

#endif          // TFMINI_CLASS