///////////////////////////////////////////////////////////////////////////////
// template - Functions associated with the <function>. 
// Written by - <name>, <date>
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Local header files
///////////////////////////////////////////////////////////////////////////////
#include "telemetry.h"
#include "vehicle.h"

///////////////////////////////////////////////////////////////////////////////
// extern global variables - these are instantiated in the main file
///////////////////////////////////////////////////////////////////////////////
extern Telemetry telem;                // The telemetry class

///////////////////////////////////////////////////////////////////////////////
// local global variables
///////////////////////////////////////////////////////////////////////////////

uint32_t global1 = 0;
uint32_t global2 = 0;

///////////////////////////////////////////////////////////////////////////////
// <tmp>_init - initialize the <template> system.  Use this to initialize only
// the devices and/or variables used within this subsystem.  Find a two or three 
// character prefix to start out each function, to replace the <tmp>...
///////////////////////////////////////////////////////////////////////////////
void <tmp>_init ()
{
    ;
}

///////////////////////////////////////////////////////////////////////////////
// <tmp>_cmd1 - called when we get the <?> command.  Each command has a 
// two byte command ID, followed by up to three two byte parameters.  Return
// true if the command is valid and accepted (is it the correct mode, etc.), 
// false otherwise. This command will be added to the command string parser
// in commands.ino
///////////////////////////////////////////////////////////////////////////////
uint32_t <tmp>_<cmd1> (uint16_t param1)
{
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// <tmp>_cmd2 - called when we get the <?> command.  Each command has a 
// two byte command ID, followed by up to three two byte parameters.  Return
// true if the command is valid and accepted (is it the correct mode, etc.), 
// false otherwise. This command will be added to the command string parser
// in commands.ino
///////////////////////////////////////////////////////////////////////////////
uint32_t <tmp>_<cmd2> (uint16_t param1)
{
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// <tmp>_cmd3 - called when we get the <?> command.  Each command has a 
// two byte command ID, followed by up to three two byte parameters.  Return
// true if the command is valid and accepted (is it the correct mode, etc.), 
// false otherwise. This command will be added to the command string parser
// in commands.ino
///////////////////////////////////////////////////////////////////////////////
uint32_t <tmp>_<cmd3> (uint16_t param1)
{
    return (true);
}

///////////////////////////////////////////////////////////////////////////////
// <tmp>_check - This function will be called repeatedly in the main loop of 
// the program at what ever speed you chose.  It will be added to the taskList[]
// structure in TeensieDev.ino.  Even if it doesn't need the currTime passed
// in it needs to have this function template.  An example of how it might be
// be used is to read the value of a sensor and enter it's value into the 
// telemetry structure/packet.  Make sure to let me (dag) know if you have to
// add anything to the telemetry.  The code below is how this type of command
// would have been used to check for the heartbeat signal and abort it it 
// didn't come within 1/2 second. 
///////////////////////////////////////////////////////////////////////////////
void <tmp>_check (uint32_t currTime)
{
    // Have we not received a heartbeat in 500 milliseconds?
    if (currTime > (lastHeartbeatTime + 500))
    {
        veh_estop();
        telem.currMode = ESTOP;
        telem.bist = HEARTBEAT;
    }
    else
    {
        lastHeartbeatTime = currTime;
    }
}
