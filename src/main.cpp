#include <Arduino.h>
#include <cmath>

// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}

/*
STATES: 
1. idle
2. 2_wheel_drive
3. 4_wheel_drive
4. 


Basic Code Outline: 

int engage_centerlock() {
  numTries = 5; 

  //setting numTries to the right value 
  if (fws < 0.5) {  //some small value 
    double wheelSpeedDiff = abs(fws - bws); 
    if (wheelSpeedDiff > 0.5)  {  //Number given by PT
      // [FAILED TO ENGAGE CENTERLOCK]
      numTries = 0; 
    } else {
      numTries = 1; 
    }
  }

  if (numTries > 0) {
     
    numTries--; 
  } 

}
 
*/