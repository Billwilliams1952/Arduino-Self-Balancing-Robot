/*
 * SelfBalancingRobot.ino
 * 2016 WLWilliams
 * 
 * This sketch demonstrates a self-balancing robot.
 * 
 * The Motor Shield (motor drive interface) is the SMAKN dual motor driver.
 * The Sensor is the LSM9DSO 9 degrees of freedom sensor.
 *    3 axis acceleration, three axis gyroscope, 3 axis magnetic field, and temp sensor
 * The Rotary Encoder is a KY-040.
 * The display is a <TODO: TBD>.
 * The motors are 12V motors <TODO: part number>. 
 * Wheels are <TODO: part number>.
 * The processor is an Arduino Micro.
 * The batter is a <TODO: description and part number>.
 * 
 * Several key libraries are used. Kalman and PID_V1.  
 * Kalman found at: https://github.com/TKJElectronics/KalmanFilter
 * PID_V1 found at: https://github.com/br3ttb/Arduino-PID-Library
 * As the robot design progresses, I will probably remove superflous code to minimize code 
 * size and maximize code speed.
 * 
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of
 * the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is in the public domain.
 * 
 * Code found at:
 * 
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>    // Do I really need this?
#include <Adafruit_LSM9DS0.h>   
#include <Kalman.h>             // Source: https://github.com/TKJElectronics/KalmanFilter
#include <PID_v1.h>
#include <ky-040.h>             // Rotary encoder used for setting PID's
#include <Lpf.h>                // Low Pass Filter for reading True/Magnetic North

#define   DEBUG_TERMINAL
#ifdef DEBUG_TERMINAL
  #define   PRINT(str)              Serial.print((str));
  #define   PRINTLN(str)            Serial.println((str));
#else
  #define   PRINT(str)
  #define   PRINTLN(str)
#endif

/* Math stuff */
#define RADS_TO_DEG           57.295779513  // convert radians to degrees

/* Loop timing data */
#define LOOP_TIME_MSEC        20    // 50Hz loop time to start.....
#define ONE_SEC_COUNT         (1000 / LOOP_TIME_MSEC)
#define QUARTER_SEC_COUNT     (250 / LOOP_TIME_MSEC)

/* LPF Setups */
#define LPF_BANDWIDTH_HZ       10   // 10Hz BW - 50 Hz sample rate
#define SAMPLE_TIME            ((float)LOOP_TIME_MSEC / 1000.0)

/* Robot maximums */
#define ALMOST_UPRIGHT        10    // +/- 10 deg to upright is good enough
#define PITCH_TOO_GREAT       50    // if abs(pitch) > this, then STOP!!
#define LEVEL_ANGLE            0    // Compensate if the sensor is not level

#define STUTTER_MOTOR_AMOUNT  20    // Stutter the motor when ready to raise

/* Encoder pins. One encoder is used for all three PID terms. I use my home grown
 * library ky-040.
 */
#define   ENCODER_CLK         2  
#define   ENCODER_DT          4
#define   ENCODER_SW          5
#define   MAX_ROTARIES        3
// Encoder rotary names
#define   KP_ROT              (uint8_t)0   // in numerical order to simplify code
#define   KI_ROT              (uint8_t)1
#define   KD_ROT              (uint8_t)2

/* Motor pins. These pins must support PWM using analogWrite */
#define   LEFT_WHEEL_BIA      6   // Left Wheel is MotorB PORTD (0-7)
#define   LEFT_WHEEL_BIB      9   // PORTB (8-13)
#define   RIGHT_WHEEL_AIA    10   // Right Wheel is MotorA PORTB
#define   RIGHT_WHEEL_AIB    11   // PORTB

#define   STATUS_LED         13   // PORTB 0x00100000
#define   PORTB_STATUS_LED   B00100000    // Not working?

/* Create a sensor. Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
sensors_event_t accel, mag, gyro, temp;
float heading;                // Heading calculated from mag data

/* Create an encoder with three rotaries for PID terms Kp, Ki, Kd */
ky040 pidEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, MAX_ROTARIES );
uint8_t activeRotaryPid = KP_ROT;
/* PID terms that are read from the rotary encoder pidEncoder */
float KpVal = 0.0, KiVal = 0.0, KdVal = 0.0;

/* Create the Kalman filter instances */
Kalman kalmanX;                // Create the Kalman instances
Kalman kalmanY;
float kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
float pitch, roll;             // Calculated pitch and roll values

/* Loop timing */
float  deltaTime = 1.0 / LOOP_TIME_MSEC;   // time between loop updates in seconds

/* Create PID and assign Tuning Parameters */
float targetAngle = LEVEL_ANGLE, pwmAmount;
/* Specify the links and initial tuning parameters */
PID myPID((double *)&pitch, (double *)&pwmAmount, (double *)&targetAngle, 
          (double)KpVal, (double)KiVal, (double)KdVal, DIRECT);

/* Create a low pass filter. */
LPF lpf(LPF_BANDWIDTH_HZ,SAMPLE_TIME);

/* ----------------------------------------------------------------------
 *  Initial entry. Setup/initialize the 9DOF sensor, motors, PID rotaries
 * ----------------------------------------------------------------------
 */
void setup ( void ) {
  
#ifdef DEBUG_TERMINAL
    while ( !Serial ) ;     // Wait for terminal
    /* We could wait a specific time for the terminal and if none is
     * there, just move on.
     */
    Serial.begin(115200);   // Set to highest baudrate to minimze delays
#endif

    /* Setup for a LCD display mounted on the robot */
    Serial1.begin(9600);
    /* Clear the screen, setup for fast baudrate, switch to it */

    /* Define rotaries for PID terms. 0.0 to 100.0 (divide reading by 10)
     * This may have to change.... need more resolution
     */
    pidEncoder.AddRotaryCounter(KP_ROT, 0, 0, 1000, 1, false );
    pidEncoder.AddRotaryCounter(KI_ROT, 0, 0, 1000, 1, false );
    pidEncoder.AddRotaryCounter(KD_ROT, 0, 0, 1000, 1, false );
    pidEncoder.SetRotary(activeRotaryPid);

    pinMode(STATUS_LED,OUTPUT);
    digitalWrite(STATUS_LED,HIGH);      // Drives lamp OFF
  
    /* Setup motor driver */
    pinMode(LEFT_WHEEL_BIA,OUTPUT);
    digitalWrite(LEFT_WHEEL_BIA,LOW);
    pinMode(LEFT_WHEEL_BIB,OUTPUT);
    digitalWrite(LEFT_WHEEL_BIB,LOW);
    pinMode(RIGHT_WHEEL_AIA,OUTPUT);
    digitalWrite(RIGHT_WHEEL_AIA,LOW);
    pinMode(RIGHT_WHEEL_AIB,OUTPUT);
    digitalWrite(RIGHT_WHEEL_AIB,LOW);

    /* Initialize the sensor and setup gain and integration time*/
    if( !lsm.begin() ) {
      PRINT(F("No LSM9DS0 detected. Halting..."));
      /* We need to do some kind of error display -- blink LED very
       * fast?  Write something to Serial1?
       */
      while(1);   // Halt and catch fire
    }
    PRINTLN(F("LSM9DS0 9DOF Init ok..."));
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    PRINTLN(F("LSM9DS0 9DOF Setup ok..."));
    /* 
     * Should we have a warmup delay? The user could be notified
     * by a blinking LED that turns ON when warmup is complete.
     * The user could override this function by merely starting 
     * to raise the robot.
     */
    PRINTLN(F("Accelerometer / gyro stabilization.... wait 5 seconds"));
    delay(5000);

    /* Initialize Kalman filter - first get pitch and roll */
    roll  = atan2(accel.acceleration.y, accel.acceleration.z) * RADS_TO_DEG;
    pitch = atan(-accel.acceleration.x / 
        sqrt(accel.acceleration.y * accel.acceleration.y + 
             accel.acceleration.z * accel.acceleration.z)) * RADS_TO_DEG;

    kalmanX.setAngle(roll); // Set starting angle
    kalmanY.setAngle(pitch);
    kalAngleX = roll;
    kalAngleY = pitch;
    PRINTLN(F("Kalman filter setup..."));

    /* Initialize PID filter */
    myPID.SetTunings(KpVal, KiVal, KdVal);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(LOOP_TIME_MSEC);
    myPID.SetOutputLimits(-255,255);    // PWM to motors
    PRINTLN(F("PID setup..."));
    /*
     * Stutter the wheels for a couple of seconds to let the user know
     * we are ready to turn upright
     */
    for ( uint8_t i = 0; i < 30; i++ ) {
        SetMotors(STUTTER_MOTOR_AMOUNT, STUTTER_MOTOR_AMOUNT);
        delay(10);
        SetMotors(-STUTTER_MOTOR_AMOUNT,-STUTTER_MOTOR_AMOUNT);
        delay(10);
    }
    SetMotors(0,0);   // OFF 
    
    /* Now Set Balance LED OFF. As the user moves the robot upright, 
     * increase the blink rate of the LED until within BALANCE_START.
     * Then turn LED ON and start loop.
     */
    // code here.....

    PRINTLN(F("***** Starting Balance loop"));
}

/* ----------------------------------------------------------------------
 * Main execution loop. Basic execution is as follows:
 *    Declare static variable for loop timing of 50 Hz (20 msec)
 *    Start timer
 *    Get raw sensor data, accel x, y, x - gyro x, y, z
 *    Convert to pitch and roll
 *    Run through Kalman filter for KalmanPitch KalmanRoll
 *    Run through PID to get motor drive
 *    Set motors
 *    Update display/read inputs 4 times a second
 *    Update (what??) every second
 *    Calculate deltaTime for this pass
 *    delay ( 50 - deltaTime )
 * ----------------------------------------------------------------------
 */
void loop() {
    static bool inStartup = true;
    static unsigned long startTime; 
    static uint8_t secondsCount = 0, quarterSecondsCount = 0;

    // Loop is approximately 6.05 msec ----- 
    /* Once I have this running (or at least all coded), I will do a
     * more detailed assesment of the timing. If its under 18msec,
     * then I can use micros() instead of millis() to improve time
     * accuracy for the various filters and PID.
     */
    // NO!!!!! WE WANT TO KEEP TRACK OF TOTAL LOOP TIME!!
    // Not much difference though -- 
    startTime = millis();   // Start this pass timing

    /* get latest data from 9DOF sensor */
    lsm.getEvent(&accel, &mag, &gyro, &temp);

    /* Convert accel in degrees to pitch/roll angle. */
    roll  = atan2(accel.acceleration.y, accel.acceleration.z) * RADS_TO_DEG;
    pitch = atan(-accel.acceleration.x / 
        sqrt(accel.acceleration.y * accel.acceleration.y + 
             accel.acceleration.z * accel.acceleration.z)) * RADS_TO_DEG;

    /* Now we run the pitch angle and pitch angle rate into the Kalman filter
     * to get a filtered answer that fuses accelerometer data in pitch degrees 
     * and gyro data in pitch degrees/sec based on the delta time - which is the
     * time from last time to this time.
     */

    /* This fixes the transition problem when the accelerometer angle jumps between 
     * -180 and 180 degrees 
     */
    if ( (roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90) ) {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyro.gyro.x, deltaTime);
  
    if ( abs(kalAngleX) > 90 )
        gyro.gyro.y = -gyro.gyro.y; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyro.gyro.y, deltaTime);

    if ( inStartup ) {
        /* If we're in startup, then determine when we are close enough to 
         * upright to take over autobalance.
         * THIS is where we need to give some visual / audible indication
         * that we're close enough
         */  
         if ( abs(pitch) < ALMOST_UPRIGHT ) {
            PRINTLN(F("Robot upright... starting autobalance"));
            inStartup = false;   
         }    
    } else {  // We are in autoBalance mode
        if ( abs(pitch) > PITCH_TOO_GREAT ) {
            /* we are falling over. Can't compensate. */
            SetMotors(0,0);      // Turn off motors and fall
            PRINTLN(F("***** Pitch too great. reentering Startup mode. *****"));
            /* Resetup variables, filters, etc to start over. */
            inStartup = true;
            delay(2000);        // let everything fall down.
        } else {   
             /* Computer error between pitch and targetAngle. Return a pwmAmount in
              * the range of -255 (full backwards) to 255 (full forwards)
              */
            myPID.Compute();
            /* How do we inject a turn or movement command into the PID?
             * If all well and good - set motors to PWM amounts
             */
            SetMotors(pwmAmount,pwmAmount);
        }
        
    } /* Done in AutobalanceMode */

    /* Update heading and low pass filter the result */
    heading = atan2(mag.magnetic.y,mag.magnetic.x);
    if ( heading < 0 ) heading += TWO_PI;
    heading *= RADS_TO_DEG;
    /* Need to add magnetic declination offset for our area */
    heading = lpf.NextValue(heading);

    /* Check for any procedures that are done periodically */

    if ( ++quarterSecondsCount == QUARTER_SEC_COUNT ) {
        quarterSecondsCount = 0;
        // Four times a second we need to check for switch presses and
        // update any serial displays
        CallFourTimesASecond();
    }

    if ( ++secondsCount == ONE_SEC_COUNT ) {
        secondsCount = 0;
        // Once a second, update terminal if connected?
        CallEverySecond();
    } 

    startTime = millis() - startTime;   // Loop complete - time it
    
    /* Calculate the average loop time */
    #define MAX_AVG 500
    static int loopCount = 0;
    static unsigned long avg = 0;

    avg += startTime;
    if ( ++ loopCount == MAX_AVG ) {
        loopCount = 0;
        PRINT("Average Loop: "); PRINTLN((float)avg / MAX_AVG);
        avg = 0;
    }
    
    /* Check loop timing. Adjust delay to keep near LOOP_TIME_MSEC */
    if ( startTime < LOOP_TIME_MSEC )
        delay ( LOOP_TIME_MSEC - startTime );
    else {
        /* We're running slow. What to do if this continues?
         * Perhaps add an overRunCount variable that increments.
         */
        PRINTLN(F("***** LOOP OVERRUN *****"));
    }
}

/*
 * Basic motor control. Pass a value of -255 to 255 where negative numbers
 * say the motor must be in reverse. 'Reverse' is relative - here reverese
 * means -Pitch angles, forward is +Pitch angles.
 */
void SetMotors ( int16_t leftPwmAmount, int16_t rightPwmAmount ) {

    // TODO: Consider direct port writes (PORTx) here. Saves some time
    
    if ( leftPwmAmount < 0 ) {
        // PORTB &= ~B00000010; 
        digitalWrite(LEFT_WHEEL_BIB,LOW);
        analogWrite(LEFT_WHEEL_BIA,-leftPwmAmount);
    }
    else {
        // PORTD &= ~B01000000;
        digitalWrite(LEFT_WHEEL_BIA,LOW);
        analogWrite(LEFT_WHEEL_BIB,leftPwmAmount);
    }

    if ( rightPwmAmount > 0 ) {
        // PORTB &= ~B00000100; 
        digitalWrite(RIGHT_WHEEL_AIB,LOW);
        analogWrite(RIGHT_WHEEL_AIA,-rightPwmAmount);
    }
    else {
        // PORTB &= ~B00001000; 
        digitalWrite(RIGHT_WHEEL_AIB,LOW);
        analogWrite(RIGHT_WHEEL_AIA,rightPwmAmount); 
    }                  
}

#if 0   // Simple PID --- may use this instead of PID_v1
// Only care about Pitch angle - right?? Can Pitch and Roll but not Yaw
int updatePid ( int targetPosition, int currentPosition )   {
  int error = targetPosition - currentPosition;
  
  pTerm = Kp * error;
  integrated_error += error;
  iTerm = Ki * constrain(integrated_error, -255, 255);
  dTerm = Kd * (error - last_error);
  last_error = error;
  // K is an overal gain factor......
  return -constrain(K*(pTerm + iTerm + dTerm), -255, 255);
}
#endif

/*
 * Fuction called every 250 msec. Here we check rotaries and see if they
 * have changed. If so, update associated PID terms. Update any displays
 * attached to the robot.
 */
void CallFourTimesASecond ( void ) {

    if ( pidEncoder.SwitchPressed() ) {
        if ( ++activeRotaryPid > KD_ROT ) activeRotaryPid = KP_ROT;
        pidEncoder.SetRotary(activeRotaryPid);
    }
    if ( pidEncoder.HasRotaryValueChanged(KP_ROT) ) {
        KpVal = (float)pidEncoder.GetRotaryValue(KP_ROT) / 10.0;  
    }
    if ( pidEncoder.HasRotaryValueChanged(KI_ROT) ) {
        KiVal = (float)pidEncoder.GetRotaryValue(KI_ROT) / 10.0;       
    }
    if ( pidEncoder.HasRotaryValueChanged(KD_ROT) ) {
        KdVal = (float)pidEncoder.GetRotaryValue(KD_ROT) / 10.0;       
    }
   // myPID.SetTunings(KpVal, KiVal, KdVal);
    
    /* Update lastest Pitch and Roll data */
    PRINT(F("Pitch: ")); PRINT(pitch); PRINT(F("\tRoll: ")); PRINTLN(roll);
    PRINT(F("PWM: ")); PRINTLN(pwmAmount);
    /*
     * What about an autoincrementor for PID values? While a button is pressed
     * increment the value by 0.01 every XX msec.
     */
}

/*
 * Function called every second. Blink I'm alive
 */
void CallEverySecond ( void ) {
    /* Pulse I'm alive LED */
    digitalWrite(STATUS_LED,!digitalRead(STATUS_LED));

    /* Do a battery check every 10 seconds */

    /* Heading and temperature update every second */

    PRINT(F("Heading: ")); PRINTLN(heading);
    /* Attach a servo with a pointer that points True North? */
  
    PRINT(F("Temp: ")); PRINT(temp.temperature); PRINTLN(F(" *C"));  
}
