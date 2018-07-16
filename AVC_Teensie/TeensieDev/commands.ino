///////////////////////////////////////////////////////////////////////////////
// commands.c - the command decoder
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "constants.h"
#include "telemetry.h"
#include "commands.h"
#include "heartbeat.h"
#include "vehicle.h"

#ifdef TEENSIE_35
//	#include "scanner.h"
//	#include "sideSensors.h"
//	#include "nineDof.h"
#endif
// #include "bling.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - defined in the main file
///////////////////////////////////////////////////////////////////////////////
extern Telemetry telem;                // The telemetry class

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Function templates
///////////////////////////////////////////////////////////////////////////////
uint16_t setMode (int16_t mode);

///////////////////////////////////////////////////////////////////////////////
// commandDecode
///////////////////////////////////////////////////////////////////////////////
void commandDecode(cmdData* command)
{
    uint16_t header = command->words[0];
    char cmd = (char) command->words[1];
    uint16_t param1 = command->words[2];
    uint16_t param2 = command->words[3]; 
    // uint16_t param3 = command->words[4]; never used
    
    telem.rejectReason = 5;         // Initialize to unknown in case of bug 
                                    // This should get overwritten!  
    uint32_t incAccpt = false;      // increment command accept counter or not
    
    if (header != 0x5454)
    {
        // Incorrect header on command message
        telem.rejectCntr++;
        telem.rejectReason = 1;
        return;
    }      
  
    switch (cmd)
    {
        case 'M':                   // Move command
            incAccpt = veh_move ((int32_t) param1, param2);
            DBGPORT.println ("commandDecode: Move Command");     
            break;

        case 'T':                   // Turn
            incAccpt = veh_turn ((int32_t) param1);
            DBGPORT.println ("commandDecode: Turn Command");
            break;

        case 'E':                   // Estop command
            incAccpt = veh_estop ();
            telem.currMode = ESTOP;
            telem.bist = COMMANDED;
            DBGPORT.println ("commandDecode: EStop Command");      
            break; 

        case 'H':                   // Heartbeat
            incAccpt = hb_command (param1);
            DBGPORT.println ("commandDecode: Heartbeat Command");
            break;

        case 'L':                   // lighting command
            DBGPORT.println ("commandDecode: Lighting command");
            incAccpt = true;
            break;

        case 'P':                   // Speed PID parms
            DBGPORT.println ("commandDecode: speed parms Command");
            incAccpt = true;
            break;

        case 'Q':                   // Steering PID parms
            DBGPORT.println ("commandDecode: speed parms command");
            incAccpt = true;
            break;

        case 'D':                   // Goto mode
            incAccpt = setMode (param1);
            DBGPORT.println ("commandDecode: goto mode Command");
            break;            

        case 'N':                   // NOP command
            DBGPORT.println ("commandDecode: NOP command");
            incAccpt = true;
            break;

        case 'S':                   // Scan speed
            DBGPORT.println ("commandDecode: Scan speed Command");
            incAccpt = true;
            break;

        case 'A':                   // Scan angle
            DBGPORT.println ("commandDecode: Scan angle command");
            incAccpt = true;
            break;

        case 'V':                   // Set Camera angle
            DBGPORT.println ("commandDecode: Camera angle Command");
            cam_setAngle  (param1);
            incAccpt = true;
            break;

        case 'C':                   // Set scan sensor
            DBGPORT.println ("commandDecode: Set scan sensor command");
            incAccpt = true;
            break;
      
        case 'B':                   // Brake command
            incAccpt = veh_brake (param1);     
            DBGPORT.println ("commandDecode: Brake command");
            break;      

        default:
            // Unrecognized command
            telem.rejectCntr++;
            telem.rejectReason = 2;
            return;
    }   //end switch

    if (incAccpt)
    {
       telem.acceptCntr++;
       telem.rejectReason = 0;      
    }
    else
    {
       telem.rejectCntr++; 
    }
    
} // end cmdDecode

///////////////////////////////////////////////////////////////////////////////
// setMode command
///////////////////////////////////////////////////////////////////////////////
uint16_t setMode (int16_t mode)
{
    switch (mode)
    {
    case BIST:
        telem.currMode = BIST;
        //todo - perform the bist in initialization mode maybe just reset the system?
        return (true);     
        
    case NORMAL:
        telem.currMode = NORMAL;  
        return (true);
        
    case ESTOP:
        veh_estop ();
        telem.currMode = ESTOP;   
        telem.bist = COMMANDED;
        return (true);
        
    default:
        // unrecognized mode - don't inc command count
        return (false);
    }
    return (false);    
}


///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// 
///////////////////////////////////////////////////////////////////////////////
