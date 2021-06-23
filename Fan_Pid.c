#include "RIMS.h"

// Written by: Dumitru Chiriac
// Email: dchir002@ucr.edu
// SID: 862126186
// Write up link: https://docs.google.com/document/d/1hoYBFxIa2bQKEqs4gzS9oW4QF6aMt6a2zknu5hKQwMM/edit?usp=sharing
// Demo Link: https://youtu.be/Izt8bb94mMw

//schedualler
typedef struct task {
  int state; // Current state of the task
  unsigned long period; // Rate at which the task should tick
  unsigned long elapsedTime; // Time since task's previous tick
  int (*TickFct)(int); // Function to call for task's tick
  } task;

task tasks[4];



// Tasks parameters; don't modify this code 
const unsigned char tasksNum = 4;
const unsigned long tasksPeriodGCD = 1000;
const unsigned long periodDP = 1000;
const unsigned long periodPID = 1000;
const unsigned long periodMN = 1000;
const unsigned long periodBP = 1000;

int DP_TickFct(int state);
int PID_TickFct(int state);
int MN_TickFct(int state);
int BP_TickFct(int state);

volatile int TimerFlag = 0;
void TimerISR() { 
  
  unsigned char i;
  for (i = 0; i < tasksNum; ++i) { // Heart of the scheduler code
     if ( tasks[i].elapsedTime >= tasks[i].period ) { // Ready
         //funtion pointers don't seem to work in RIMS so I modified the schedualler code
         if (i == 0){
               tasks[i].state = DP_TickFct(tasks[i].state);
               tasks[i].elapsedTime = 0;
         }
         else if (i == 1){
               tasks[i].state = PID_TickFct(tasks[i].state);
               tasks[i].elapsedTime = 0;
         }
         else if (i == 2){
               tasks[i].state = MN_TickFct(tasks[i].state);
               tasks[i].elapsedTime = 0;
         }
         else if (i == 3){
               tasks[i].state = BP_TickFct(tasks[i].state);
               tasks[i].elapsedTime = 0;
         }
     
         
     }
     tasks[i].elapsedTime += tasksPeriodGCD;
  }
  TimerFlag = 1;
}

//variables
const int goal = 100;
const int kp = 210;
const int ki = 1;
const int kd = 2500;
const int integMin = 0;
const int integMax = 200;
const int scale = 1000;

int actual = 0;
int actualPrev = 0;
int err = 0;
int integ = 0;
int der = 0;

int pos = 0;
int vel = 0;
int accel = 0;
int fanAccel = 0;
int goalReached = 0; //flag for PI to stop

//functions
int error(){
    err = goal - actual;
    return (kp * err) / scale;
}

int integral(){
    integ += err;
    if (integ > integMax) { 
        integ = integMax; 
   }
   else if (integ < integMin) { 
        integ = integMin; 
   }
   return (ki * integ) / scale;
}

int derivative(){
    der = actual - actualPrev;
    return (kd * der) / scale;
}


void ballPos(){ //calculates ball postion
    accel = fanAccel - 10; //accelaration of ball from fan minus gravity
    if(pos < 0) accel = 0;
    if(pos > 200) accel = 200;
    
    vel += accel; //velocty of ball after one sec
    pos += vel; //position of ball after one sec
    
    if(pos < 0) pos = 0;
    if(pos > 200) pos = 200;
    
}



//states
enum DP_States {DP_Start, DP_Val, DP_Pos}; //display of postion
enum PID_States {PID_Start,PID_Off, PID_Apply, PID_Cor, PID_Stop1, PID_Stop2, PID_Stop3}; //pid controller
enum MN_states {MN_Start, MN_Off, MN_Wait, MN_Up, MN_Down}; //manual control
enum BP_states {BP_Start, BP_Apply}; //ball position


/////////////////////////////////////////////////
int i = 0;
int main(){
   TimerSet(tasksPeriodGCD);
   TimerOn();
   
   //RIMS doesn't allow a pointer to function so I had to modify the timerISR schedualer
   tasks[i].state = DP_Start;
   tasks[i].period = periodDP;
   tasks[i].elapsedTime = tasks[i].period;
   
   i++;
   tasks[i].state = PID_Start;
   tasks[i].period = periodPID;
   tasks[i].elapsedTime = tasks[i].period;
   
   i++;
   tasks[i].state = MN_Start;
   tasks[i].period = periodMN;
   tasks[i].elapsedTime = tasks[i].period;
   
   i++;
   tasks[i].state = BP_Start;
   tasks[i].period = periodBP;
   tasks[i].elapsedTime = tasks[i].period;
   
   while(1){
      
      while(!TimerFlag);
      TimerFlag=0;
   }
   return 0;
}

int DP_TickFct(int state){
    switch(state){ //transitions
        case DP_Start: state = DP_Val; break;
        case DP_Val: state = (A1) ? DP_Pos : DP_Val; break;
        case DP_Pos: state = (A1) ? DP_Pos : DP_Val; break;
        default: state = DP_Start; break;
    }
    
    switch(state){
        case DP_Val: B = pos; break;
        case DP_Pos:
        if (pos / 25 != 8)
               B = (0x80 >> (pos / 25));
        else
               B = 0x01; //in the case that pos = 200
        break;
    }
    return state;
}

int PID_TickFct(int state){
    switch(state){
        case PID_Start: state = PID_Apply; break;
        case PID_Off: state = (!A0) ? PID_Off : PID_Apply; break;
        case PID_Apply: state = (pos > 98) ? PID_Cor : PID_Apply; break;
        case PID_Cor: state = PID_Stop1; break;
        case PID_Stop1: state = PID_Stop2; break;
        case PID_Stop2: state = PID_Stop3; break;
        case PID_Stop3: state = PID_Stop3; break;
        default: state = PID_Start; break;
    }
    
    switch(state){
        case PID_Cor:
        case PID_Apply:
            actualPrev = pos;
            fanAccel = error() + integral() + derivative();
            break;
        case PID_Stop1:
            fanAccel = 14; break; //stop1-3 cleanly stops the velocity of the ball to zero and at 100.
        case PID_Stop2:
            fanAccel = 8; break;
        case PID_Stop3:
            fanAccel = 10; break;
    }
    return state;
}

int MN_TickFct(int state){
    switch(state){ //transisitions
        case MN_Start: state = MN_Off; break;
        case MN_Off: state = (A0) ? MN_Wait : MN_Off; break;
        case MN_Wait:
            if (!A0) state = MN_Off;
            else if (A6 && !A7) state = MN_Up;
            else if (!A6 && A7) state = MN_Down;
            else state = MN_Wait;
            break;
        case MN_Up: state = (A6 && !A7) ? MN_Up : MN_Wait; break;
        case MN_Down: state = (!A6 && A7) ? MN_Down : MN_Wait; break; 
    }
    
    switch(state){
        case MN_Wait: fanAccel = 10; break;
        case MN_Up: fanAccel++; break;
        case MN_Down: 
            if (fanAccel > 0) fanAccel--;
            else fanAccel = 0;
            break;
    }
    return state;
}

int BP_TickFct(int state){
    switch(state){ //transitions
        case BP_Start: state = BP_Apply; break;
        case BP_Apply: state = BP_Apply; break;
        default: state = BP_Start; break;
        
    }
    
    switch(state){
        case BP_Apply: ballPos(); actual = pos; break;
    }
    return state;
}
