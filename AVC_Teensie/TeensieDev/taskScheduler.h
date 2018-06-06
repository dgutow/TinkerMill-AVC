///////////////////////////////////////////////////////////////////////////////
// taskScheduler.h 
///////////////////////////////////////////////////////////////////////////////

#ifndef  TASKSCHEDULER_H
#define  TASKSCHEDULER_H

///////////////////////////////////////////////////////////////////////////////
// Function templates for the task scheduler 
///////////////////////////////////////////////////////////////////////////////   
void initExecuteTasks ();
void executeTasks (uint32_t time);

///////////////////////////////////////////////////////////////////////////////
// Struct TASK - used to schedule functions to run at specific intervals. 
///////////////////////////////////////////////////////////////////////////////
struct task
{
   // member variables
   void     (*func) (uint32_t time);   // The function to be executed
   uint32_t timeDelta;                 // the time between executions (msec)
   uint32_t timeOffset;                // Offset time for execution   
   uint32_t nextExecTime;              // The time to execute func next
};

///////////////////////////////////////////////////////////////////////////////
// taskList - An array of 'task's that are initialized to the list of functions
// to run and the interval to run them.
// - The first value (func) is the name of the function to run.  It must be
//   the name of a function which returns void and takes a uint32_t parameter
//   (the current time in millseconds)
// - The second value (timeDelta) is the time between executions of the function.
//   timeDelta is given in milliseconds. 
// - The third value (timeOffset) is the offset from the timeDelta boundary that
//   this function will run.  For example, a function with a timeDelta of 100
//   (msecs) and a timeOffset of 10 will run at times of 110, 210, 310, etc.
// - The fourth value (nextExecTime) is for internal use and should be set to
//   0.  (It is overwritten in the initExecuteTasks function)
///////////////////////////////////////////////////////////////////////////////

// taskList defined in TeensiMain.ino

#endif    //  TASKSCHEDULER_H