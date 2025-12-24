// Machine Design State Machine - Final code written by Manuella Kwawu, Kelsey Anthony, Raquel Conard, Sophya Elkihel\// updated 12-15-2025
// Based off of the state machine developed in the University of Michigan ME350 Course
// Edited by Dr. Yesilevskiy with permission of instructors

//////////////////////////////////////////////
// DEFINE CONSTANTS AND GLOBAL VARIABLES:   //
//////////////////////////////////////////////

//** State Machine: **//
// CONSTANTS: 
// Definition of states in the state machine
const int CALIBRATE     = 1;
const int FIRST_BUTTON  = 2;
const int SECOND_BUTTON = 3;
const int THIRD_BUTTON  = 4;
const int WAIT = 5;

// VARIABLES:
// Global variable that keeps track of the state:
// Start the state machine in calibration state:
int  state = CALIBRATE;
int old_state = CALIBRATE;

//** Computation of position and velocity: **//
// CONSTANTS: 
// Settings for velocity computation:
const int  MIN_VEL_COMP_COUNT = 2;     // [encoder counts] Minimal change in motor position that must happen between two velocity measurements
const long MIN_VEL_COMP_TIME  = 10000; // [microseconds] Minimal time that must pass between two velocity measurements
// VARIABLES:
volatile int motorPosition = 0; // [encoder counts] Current motor position (Declared 'volatile', since it is updated in a function called by interrupts)
volatile int encoderStatus = 0; // [binary] Past and Current A&B values of the encoder  (Declared 'volatile', since it is updated in a function called by interrupts)
// The rightmost two bits of encoderStatus will store the encoder values from the current iteration (A and B).
// The two bits to the left of those will store the encoder values from the previous iteration (A_old and B_old).
float motorVelocity        = 0; // [encoder counts / seconds] Current motor velocity 
int previousMotorPosition  = 0; // [encoder counts] Motor position the last time a velocity was computed 
long previousVelCompTime   = 0; // [microseconds] System clock value the last time a velocity was computed 

//** High-level behavior of the controller:  **//
// CONSTANTS:
// Target positions:
const int CALIBRATION_VOLTAGE     =  9;                 // [Volt] Motor voltage used during the calibration process
const int FIRST_BUTTON_POSITION   =  0;                 // [encoder counts] Motor position corresponding to first button position
const int SECOND_BUTTON_POSITION  =  -1350;                 // [encoder counts] Motor position corresponding to second button position
const int THIRD_BUTTON_POSITION   =  -3000;                 // [encoder counts] Motor position corresponding to third button position
const int LOWER_BOUND         = 0;              // [encoder counts] Position of the left end stop
const int UPPER_BOUND         = THIRD_BUTTON_POSITION;  // [encoder counts] Position of the right end stop
const int TARGET_BAND         = 350;                      // [encoder counts] "Close enough" range when moving towards a target.
// Timing:
const long  WAIT_TIME         = 40; // [microseconds] Time waiting at each location
// VARIABLES:
unsigned long startWaitTime; // [microseconds] System clock value at the moment the WAIT state started

//** PID Controller  **//
// CONSTANTS:
const float KP             = 0.175;  // [Volt / encoder counts] P-Gain
const float KD             = 0.005;  // [Volt * seconds / encoder counts] D-Gain
const float KI             = 0.09; // [Volt / (encoder counts * seconds)] I-Gain
const float SUPPLY_VOLTAGE = 12.0;  // [Volt] Supply voltage at the HBridge
const float BASE_CMD       = 0;   // [Volt] Voltage needed to overcome friction
// VARIABLES:
int  targetPosition  = 0;   // [encoder counts] desired motor position
float positionError  = 0;   // [encoder counts] Position error
float integralError  = 0;   // [encoder counts * seconds] Integrated position error
float velocityError  = 0;   // [encoder counts / seconds] Velocity error
float desiredVoltage = 0;   // [Volt] Desired motor voltage
int   motorCommand   = 0;   // [0-255] PWM signal sent to the motor
unsigned long executionDuration = 0;  // [microseconds] Time between this and the previous loop execution.  Variable used for integrals and derivatives
unsigned long lastExecutionTime = 0;  // [microseconds] System clock value at the moment the loop was started the last time


// Gravity Compensation Lookup Table: 
// CONSTANTS: 
const float FF_BALANCED_POSITION   = 0;  // [encoder counts] Position at which the device is fully balanced. 
const float FF_VOLTAGE_LOWER_BOUND = 0; // [Volt] Voltage to be applied at the left endstop 
const float FF_VOLTAGE_UPPER_BOUND = 0; // [Volt] Voltage to be applied at the right endstop 

// to check if button is pressed
bool buttonPressedThisWait = false;


//** Pin assignment: **//
// CONSTANTS:
const int PIN_NR_ENCODER_A        = 2;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ENCODER_B        = 3;  // Never change these, since the interrupts are attached to pin 2 and 3
const int PIN_NR_ON_OFF_SWITCH    = 5;
const int PIN_NRL_LIMIT_SWITCH    = 8;
const int PIN_NR_PWM_OUTPUT       = 11;  // ENA on the H-bridge
const int PIN_NR_PWM_DIRECTION_1  = 12;  // IN1 on the H-bridge
const int PIN_NR_PWM_DIRECTION_2  = 13;  // IN2 on the H-bridge

const int BLUE_BUTTON = A0;
const int RED_BUTTON = A1;
const int YELLOW_BUTTON = A2;
const int SOLENOID_PIN = 7;
// End of CONSTANTS AND GLOBAL VARIABLES


//////////////////////////////////////////////////////////////////////////////////////////
// The setup() function is called when a sketch starts. Use it to initialize variables, //
// pin modes, start using libraries, etc. The setup function will only run once, after  //
// each powerup or reset of the Arduino board:                                          //
//////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // Declare which digital pins are inputs and which are outputs:
  pinMode(PIN_NR_ENCODER_A,        INPUT);
  pinMode(PIN_NR_ENCODER_B,        INPUT); 
  pinMode(PIN_NR_ON_OFF_SWITCH,    INPUT);
  pinMode(PIN_NRL_LIMIT_SWITCH,    INPUT);
  pinMode(PIN_NR_PWM_OUTPUT,       OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_1,  OUTPUT);
  pinMode(PIN_NR_PWM_DIRECTION_2,  OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  // Turn on the pullup resistors on the encoder channels
  digitalWrite(PIN_NR_ENCODER_A, HIGH);  
  digitalWrite(PIN_NR_ENCODER_B, HIGH);

  // Make sure the solenoid is turned off
  digitalWrite(SOLENOID_PIN, LOW);

  // Activate interrupt for encoder pins.
  // If either of the two pins changes, the function 'updateMotorPosition' is called:
  attachInterrupt(0, updateMotorPosition, CHANGE);  // Interrupt 0 is always attached to digital pin 2
  attachInterrupt(1, updateMotorPosition, CHANGE);  // Interrupt 1 is always attached to digital pin 3

  // Begin serial communication for monitoring.
  Serial.begin(115200);
  Serial.println("Start Executing Program.");

  // Set initial output to the motor to 0
  analogWrite(PIN_NR_PWM_OUTPUT, 0);
}
// End of function setup()


////////////////////////////////////////////////////////////////////////////////////////////////
// After going through the setup() function, which initializes and sets the initial values,   //
// the loop() function does precisely what its name suggests, and loops consecutively,        //
// allowing your program to sense and respond. Use it to actively control the Arduino board.  //
//////////////////////////////////////////////////////////////////////////////////////////////// 
void loop() {
  // Determine the duration it took to execute the last loop. This time is used 
  // for integration and for monitoring the loop time via the serial monitor.
  executionDuration = micros() - lastExecutionTime;
  lastExecutionTime = micros();

  // Speed Computation:
  if ((abs(motorPosition - previousMotorPosition) > MIN_VEL_COMP_COUNT) || (micros() - previousVelCompTime) > MIN_VEL_COMP_TIME){
    // If at least a minimum time interval has elapsed or
    // the motor has travelled through at least a minimum angle ... 
    // .. compute a new value for speed:
    // (speed = delta angle [encoder counts] divided by delta time [seconds])
    motorVelocity = (double)(motorPosition - previousMotorPosition) * 1000000 / 
                            (micros() - previousVelCompTime);
    // Remember this encoder count and time for the next iteration:
    previousMotorPosition = motorPosition;
    previousVelCompTime   = micros();
  }
  
  //******************************************************************************//
  // The state machine:
  switch (state) {
    //****************************************************************************//
    // In the CALIBRATE state, we move the mechanism to a position outside of the 
    // work space (towards the limit switch).  Once the limit switch is on and 
    // the motor stopped turning, we know that we are against the end stop
    case CALIBRATE:
      // We don't have to do anything here since this state is only used to set
      // a fixed output voltage.  This happens further below (search CALIBRATION_VOLTAGE to see where)
      
      // Decide what to do next:
      if (digitalRead(PIN_NRL_LIMIT_SWITCH)==HIGH && motorVelocity==0) { 
        // We reached the endstop.  Update the motor position to the limit:
        // (NOTE: If the limit switch is on the right, this must be UPPER_BOUND)
        motorPosition = LOWER_BOUND;  
        // Reset the error integrator:
        integralError = 0;

        // Start waiting timer:
        startWaitTime = micros();

        //Tranistion into WAIT state
        //Record which state you came from
        old_state = CALIBRATE;
        state = WAIT;

      } 
      // Otherwise we continue calibrating
      break;

    //****************************************************************************//
    // In the FIRST_BUTTON state, we move the button-pressing mechanism to the FIRST_BUTTON_POSITION 
    // Once the position is reached (with some error) and the motor stops turning, we know that we are at
    // the first button position.
    case FIRST_BUTTON:
      // Set the target position to the second button position:
      targetPosition = FIRST_BUTTON_POSITION;

      // Decide what to do next:
      if (motorPosition>=targetPosition-TARGET_BAND && motorPosition<=targetPosition+TARGET_BAND && motorVelocity==0) {
        // We reached the button.  
        // Start waiting timer:
        startWaitTime = micros();
        //Tranistion into WAIT state
        //Record which state you came from
        old_state = FIRST_BUTTON;
        state = WAIT;
        
      } 
      // Otherwise we continue moving towards the first button.
      break;

    //****************************************************************************//
    // In the SECOND_BUTTON state, we move the button-pressing mechanism to the SECOND_BUTTON_POSITION 
    // Once the position is reached (with some error) and the motor stops turning, we know that we are at
    // the second button position.
    case SECOND_BUTTON:
      // Set the target position to the second button position:
      targetPosition = SECOND_BUTTON_POSITION;

      // Decide what to do next:
      if (motorPosition>=targetPosition-TARGET_BAND && motorPosition<=targetPosition+TARGET_BAND && motorVelocity==0) {
        // We reached the button.  
        // Start waiting timer:
        startWaitTime = micros();
        //Tranistion into WAIT state
        old_state = SECOND_BUTTON;
        state = WAIT;
        
      } 
      // Otherwise we continue moving towards the second button.
      break;

    //****************************************************************************//
    // In the THIRD_BUTTON state, we move the button-pressing mechanism to the THIRD_BUTTON_POSITION 
    // Once the position is reached (with some error) and the motor stops turning, we know that we are at
    // the third button position.
    case THIRD_BUTTON:
      // Set the target position to the third button position:
      targetPosition = THIRD_BUTTON_POSITION;

      // Decide what to do next:
      if (motorPosition>=targetPosition-TARGET_BAND && motorPosition<=targetPosition+TARGET_BAND && motorVelocity==0) {
        // We reached the button.  
        // Start waiting timer:
        startWaitTime = micros();
        //Tranistion into WAIT state
        old_state = THIRD_BUTTON;
        state = WAIT;
        
      } 
      // Otherwise we continue moving towards the third button.
      break;
    /*
    case WAIT:

      if (micros() - startWaitTime > WAIT_TIME) {

          if (old_state == CALIBRATE) {
              state = SECOND_BUTTON;
              targetPosition = SECOND_BUTTON_POSITION;   // <-- REQUIRED
              integralError = 0;                         // optional but recommended
          }

          if (old_state == FIRST_BUTTON) {
              state = SECOND_BUTTON;
              targetPosition = SECOND_BUTTON_POSITION;
              integralError = 0;
          }

          if (old_state == SECOND_BUTTON) {
              state = THIRD_BUTTON;
              targetPosition = THIRD_BUTTON_POSITION;
              integralError = 0;
          }

          if (old_state == THIRD_BUTTON) {
              state = CALIBRATE;
              // calibration voltage will override PID
          }
      }

    break;

    */
    case WAIT:
        // When we just entered WAIT, press the button once
          if (!buttonPressedThisWait &&
              old_state == FIRST_BUTTON ||
              old_state == SECOND_BUTTON ||
              old_state == THIRD_BUTTON)
          {
              pressButton();   // please press the button:)
              buttonPressedThisWait = true;
          }
      
        if (micros()-startWaitTime>WAIT_TIME){
          buttonPressedThisWait = false;
          // Determine next state based on buttons
          if (old_state == CALIBRATE) {
              state = DETERMINE_STATE();       
              //integralError = 0;               // recommended
          }

          else if (old_state == FIRST_BUTTON) {
              state = DETERMINE_STATE();
              //integralError = 0;
          }

          else if (old_state == SECOND_BUTTON) {
              state = DETERMINE_STATE();
              //integralError = 0;
          }

          else if (old_state == THIRD_BUTTON) {
              state = DETERMINE_STATE();
              //integralError = 0;
          }
          

          
    
          //if (old_state == CALIBRATE){
           // Serial.println("State transition from CALIBRATE to BUTTON THAT CAME ON");
            //state = SECOND_BUTTON;
            //state = DETERMINE_STATE();
            //motorPosition = LOWER_BOUND; //ensure motor position is 0 before moving to next state
          //}
          
          //if (old_state == FIRST_BUTTON){
            //Serial.println("State transition from FIRST_BUTTON to SECOND_BUTTON");
            //state = SECOND_BUTTON;
            //motorPosition = LOWER_BOUND;  //ensure motor position is 0 before moving to next state
          //}

          //if (old_state == SECOND_BUTTON){
            //Serial.println("State transition from SECOND_BUTTON to THIRD_BUTTON");
            //state = CALIBRATE;
          //}

          //if (old_state == THIRD_BUTTON){
            //Serial.println("State transition from THIRD_BUTTON to FIRST_BUTTON");
            //state = CALIBRATE;
          //}
          
        }
    break;    
   
    //****************************************************************************//
    // We should never reach the next bit of code, which would mean that the state
    // we are currently in doesn't exist.  So if it happens, throw an error and 
    // stop the program:
    default: 
      Serial.println("Statemachine reached at state that it cannot handle.  ABORT!!!!");
      Serial.print("Found the following unknown state: ");
      Serial.println(state);
      while (1); // infinite loop to halt the program
    break;
  }
  // End of the state machine.
  //******************************************************************************//
  
 
  //******************************************************************************//
  // Position Controller
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH) {
    // If the toggle switch is on, run the controller:

    //** PID control: **//  
    // Compute the position error [encoder counts]
    positionError = targetPosition - motorPosition;
    // Compute the integral of the position error  [encoder counts * seconds]
    integralError = integralError + positionError * (float)(executionDuration) / 1000000; 
    // Compute the velocity error (desired velocity is 0) [encoder counts / seconds]
    velocityError = 0 - motorVelocity;
    // This is the actual controller function that uses the error in 
    // position and velocity and the integrated error and computes a
    // desired voltage that should be sent to the motor:
    desiredVoltage = KP * positionError +  
                     KI * integralError +
                     KD * velocityError;
 
    //** Feedforward terms: **//
    // Compensate for friction.  That is, if we now the direction of 
    // desired motion, add a base command that helps with moving in this
    // direction:
    if (positionError < -5) {
      desiredVoltage = desiredVoltage - BASE_CMD;
    }
    if (positionError > +5) {
      desiredVoltage = desiredVoltage + BASE_CMD;
    }
    
    /*
    // Gravity compensation lookup.  Here we record which voltage we need
    // to keep the device balanced at the left and at the right, and note 
    // where it is balanced passively.  The feedforward value is determined
    // by linear interpolation between these three points.
    if (motorPosition<FF_BALANCED_POSITION) {
        desiredVoltage = desiredVoltage + (FF_BALANCED_POSITION-motorPosition)/(FF_BALANCED_POSITION-LOWER_BOUND)*FF_VOLTAGE_LOWER_BOUND;
    }
    if (motorPosition>FF_BALANCED_POSITION) {
        desiredVoltage = desiredVoltage + (motorPosition-FF_BALANCED_POSITION)/(UPPER_BOUND-FF_BALANCED_POSITION)*FF_VOLTAGE_UPPER_BOUND;
    }
    */
    
    // Anti-Wind-Up
    if (abs(desiredVoltage)>SUPPLY_VOLTAGE) {
      // If we are already saturating our output voltage, it does not make
      // sense to keep integrating the error (and thus ask for even higher
      // and higher output voltages).  Instead, stop the inegrator if the 
      // output saturates. We do this by reversing the summation at the 
      // beginning of this function block:
      integralError = integralError - positionError * (float)(executionDuration) / 1000000; 
    }

    // Override the computed voltage during calibration.  In this state, we simply apply a 
    // fixed voltage to move against one of the end-stops.
    if (state == CALIBRATE){  
      // Note, if you would like to continue having the motor push against the hard stop (i.e. to try to go back to the hard stop if your linkage bounces)
      // you may find adding either portions or the entire line below to the If statement condition helpful
      // ||(state==WAIT && old_state == CALIBRATE)||(state == FIRST_BUTTON)||(state==WAIT && old_state == FIRST_BUTTON)) {
      desiredVoltage = CALIBRATION_VOLTAGE;         
  } 
  }      // End of 'if(onOffSwitch==HIGH)'SUPPLY
    
  else { 
    // Otherwise, the toggle switch is off, so do not run the controller, 
    // stop the motor...
    desiredVoltage = 0; 
    // .. and reset the integrator of the error:
    integralError = 0;
    // Produce some debugging output:
    Serial.println("The toggle switch is off.  Motor Stopped.");
  } 
  // End of  else onOffSwitch==HIGH
  
  //** Send signal to motor **//
  // Convert from voltage to PWM cycle:
  motorCommand = int(abs(desiredVoltage * 255 / SUPPLY_VOLTAGE));
  // Clip values larger than 255
  if (motorCommand > 255) {
    motorCommand = 255;
  }
  // Send motor signals out
  analogWrite(PIN_NR_PWM_OUTPUT, motorCommand);
  // Determine rotation direction
  if (desiredVoltage >= 0) {
    // If voltage is positive ...
    // ... turn forward
    digitalWrite(PIN_NR_PWM_DIRECTION_2,LOW);  // rotate forward
    digitalWrite(PIN_NR_PWM_DIRECTION_1,HIGH); // rotate forward
  } else {
    // ... otherwise turn backward:
    digitalWrite(PIN_NR_PWM_DIRECTION_2,HIGH); // rotate backward
    digitalWrite(PIN_NR_PWM_DIRECTION_1,LOW);  // rotate backward
  }
  // End of Position Controller
  //*********************************************************************//
  
  // Print out current controller state to Serial Monitor.
  printStateToSerial();
}
// End of main loop
//***********************************************************************//

int DETERMINE_STATE() {

    int blue   = digitalRead(BLUE_BUTTON);
    int red    = digitalRead(RED_BUTTON);
    int yellow = digitalRead(YELLOW_BUTTON);

    if (blue == HIGH) {
        Serial.println("Blue button pair is lit → FIRST_BUTTON");
        return FIRST_BUTTON;
    }

    if (red == HIGH) {
        Serial.println("Red button pair is lit → SECOND_BUTTON");
        return SECOND_BUTTON;
    }

    if (yellow == HIGH) {
        Serial.println("Yellow button pair is lit → THIRD_BUTTON");
        return THIRD_BUTTON;
    }

    // If no button is lit, stay where we are
    Serial.println("No button pair is lit → stay in CALIBRATE");
    return CALIBRATE;
}

void pressButton() {
  
  if (digitalRead(PIN_NR_ON_OFF_SWITCH)==HIGH){
  // Extend solenoid (press the button)
  digitalWrite(SOLENOID_PIN, HIGH);
  delay(30);  // we'll adjust depending on how long it takes to press

  // Retract solenoid
  digitalWrite(SOLENOID_PIN, LOW);
  delay(30);  // i think this one is optional? to ensure full retraction
}
  }
  

//////////////////////////////////////////////////////////////////////
// This is a function to update the encoder count in the Arduino.   //
// It is called via an interrupt whenever the value on encoder      //
// channel A or B changes.                                          //
//////////////////////////////////////////////////////////////////////
void updateMotorPosition() {
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;   
  // Use a compound bitwise OR operator (|=) to read the A channel of the encoder (pin 2)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(2);   
  // Bitwise shift left by one bit, to make room for a bit of new data:
  encoderStatus <<= 1;
  // Use a compound bitwise OR operator  (|=) to read the B channel of the encoder (pin 3)
  // and put that value into the rightmost bit of encoderStatus:
  encoderStatus |= digitalRead(3);
  // encoderStatus is truncated to only contain the rightmost 4 bits by  using a 
  // bitwise AND operator on mstatus and 15(=1111):
  encoderStatus &= 15;
  if (encoderStatus==2 || encoderStatus==4 || encoderStatus==11 || encoderStatus==13) {
    // the encoder status matches a bit pattern that requires counting up by one
    motorPosition++;         // increase the encoder count by one
  } else {
    // the encoder status does not match a bit pattern that requires counting up by one.  
    // Since this function is only called if something has changed, we have to count downwards
    motorPosition--;         // decrease the encoder count by one
  }
}
// End of function updateMotorPosition()


//////////////////////////////////////////////////////////////////////
// This function sends a status of the controller to the serial     //
// monitor.  Each character will take 85 microseconds to send, so   //
// be selective in what you write out:                              //
//////////////////////////////////////////////////////////////////////
void printStateToSerial() {
  //*********************************************************************//
  // Send a status of the controller to the serial monitor.  
  // Each character will take 85 microseconds to send, so be selective
  // in what you write out:

  /*
  Serial.print("Power switch [on/off]: ");
  Serial.print("  PWR: "); 
  Serial.print(digitalRead(PIN_NR_ON_OFF_SWITCH));
  */


  //"State Number:  [CALIBRATE = 1; FIRST_BUTTON = 2; SECOND_BUTTON = 3; THIRD_BUTTON = 4];
  Serial.print("State#: "); 
  Serial.print(state);
 
  //Serial.print("      Motor Position [encoder counts]: ");
  Serial.print("  MP: "); 
  Serial.print(motorPosition);

/*
  //Serial.print("      Motor Velocity [encoder counts / seconds]: ");
  Serial.print("  MV: "); 
  Serial.print(motorVelocity);

  //Serial.print("      Encoder Status [4 bit value]: ");
  //Serial.print("  ES: "); 
  //Serial.print(encoderStatus);

  //Serial.print("      Target Position [encoder counts]: ");
  Serial.print("  TP: "); 
  Serial.print(targetPosition);

  //Serial.print("      Position Error [encoder counts]: ");
  Serial.print("  PE: "); 
  Serial.print(positionError);

  //Serial.print("      Integrated Error [encoder counts * seconds]: ");
  Serial.print("  IE: "); 
  Serial.print(integralError);

  //Serial.print("      Velocity Error [encoder counts / seconds]: ");
  Serial.print("  VE: "); 
  Serial.print(velocityError);
*/
  //Serial.print("      Desired Output Voltage [Volt]: ");
  Serial.print("  DV: "); 
  Serial.print(desiredVoltage);
 /* 
  //Serial.print("      Motor Command [0-255]: ");
  //Serial.print("  MC: "); 
  //Serial.print(motorCommand);

  //Serial.print("      Execution Duration [microseconds]: ");
  //Serial.print("  ED: "); 
  //Serial.print(executionDuration);

*/
  // ALWAYS END WITH A NEWLINE.  SERIAL MONITOR WILL CRASH IF NOT
  Serial.println(); // new line
}
// End of Serial Out





