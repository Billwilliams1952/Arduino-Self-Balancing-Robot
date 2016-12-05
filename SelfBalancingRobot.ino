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
 * The switch is a SPST pushbutton.
 * The display is a 128x32 OLED display. 
 * The motors are 12V motors <TODO: part number>. 
 * Wheels are Pololu <TODO: size and part number>.
 * The processor is an Arduino Nano.
 * The battery is a <TODO: description and part number>.
 * 
 * Several key libraries are used.  
 * 9DOF Sensor at:  https://github.com/adafruit/Adafruit_LSM9DS0_Library
 * OLED Display at: https://github.com/adafruit/Adafruit_SSD1306
 *                  https://github.com/adafruit/Adafruit-GFX-Library
 * Kalman found at: https://github.com/TKJElectronics/KalmanFilter
 * PID_V1 found at: https://github.com/br3ttb/Arduino-PID-Library
 * KY-040 found at: https://github.com/Billwilliams1952/KY-040-Encoder-Library---Arduino
 * LPF found at:    https://github.com/Billwilliams1952/Low-Pass-Filter
 * As the robot design progresses, I will probably remove superflous code to minimize code 
 * size and maximize code speed.
 * 
 * Added code for storing/retrieving PID values into EEPROM. Need a pushbutton to activate
 * the write function.
 * 
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU 
 * The Motor Shield (motor drive interface) is the SMAKN dual motor driver.
 * The Sensor is the LSM9DSO 9 degrees of freedom sensor.0.02
 *    3 axis acceleration, three axis gyroscope, 3 axis magnetic field, and temp sensor
 * The Rotary Encoder is a KY-040.
 * The display is a <TODO: TBD>.*General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version. 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details. You should have received a copy of
 * the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is in the public domain.
 * 
 * Code found at: https://github.com/Billwilliams1952/Arduino-Self-Balancing-Robot
 * 
 */

#include <SPI.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>    // Do I really need this?
#include <Adafruit_LSM9DS0.h>   
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>   // OLED display
#include <Kalman.h>             // Source: https://github.com/TKJElectronics/KalmanFilter
#include <PID_v1.h>             // Source: https://github.com/br3ttb/Arduino-PID-Library
#include <ky-040.h>             // Rotary encoder used for setting PID's
#include <Lpf.h>                // Low Pass Filter for reading True/Magnetic North

//#define     INIT_EEPROM         // Only undefine if you REALLY want to overwrite EEPROM      
//#define   FULL_DEBUG          // Lots of data printed out if using DEBUG_TERMINAL
#define     OLED_SCREEN         // OLED screen in use?
//#define   DEBUG_TERMINAL      // Terminal in use?

#ifdef DEBUG_TERMINAL
  #undef    OLED_SCREEN
  #define   PRINT(str)          Serial.print((str));
  #define   PRINTLN(str)        Serial.println((str));
  #define   CLEAR_DISPLAY
  #define   INIT_DISPLAY        Serial.begin(115200);
  #define   SHOW_DISPLAY
#elif defined ( OLED_SCREEN )
  #define   PRINT(str)          display.print((str));
  #define   PRINTLN(str)        display.println((str));
  #define   CLEAR_DISPLAY       display.clearDisplay(); display.setCursor(0,0);
  #define   INIT_DISPLAY        display.begin(SSD1306_SWITCHCAPVCC, 0x3C); \
                                display.setTextSize(1); \
                                display.setTextColor(WHITE);
  #define   SHOW_DISPLAY        display.display();
#else
  #define   CLEAR_DISPLAY           
  #define   PRINT(str)
  #define   PRINTLN(str)
  #define   INIT_DISPLAY
  #define   SHOW_DISPLAY
#endif

/* Math stuff */
#define   RADS_TO_DEG           57.295779513  // convert radians to degrees

/* Loop timing data */
#define   LOOP_TIME_MSEC        20    // 50Hz loop time to start.....
#define   SAMPLE_TIME           ((float)LOOP_TIME_MSEC / 1000.0)
#define   ONE_SEC_COUNT         (1000 / LOOP_TIME_MSEC)
#define   QUARTER_SEC_COUNT     (250 / LOOP_TIME_MSEC)

/* LPF Setups */
#define   LPF_BANDWIDTH_HZ      10    // 10Hz BW - 50 Hz sample rate

/* Robot angles */
#define   LEVEL_ANGLE            3.73 // Compensation if the sensor is not level
#define   ALMOST_UPRIGHT        10    // +/- 10 deg to upright is good enough
#define   PITCH_TOO_GREAT       50    // if abs(pitch) > this, then STOP!!
#define   INTEGRATION_GUARD     10    // Maximum allowed integration error over time
//#define USE_PITCH_AND_ROLL          // If defined, merge pitch and roll

/* Encoder pins.http://forum.arduino.cc/index.php?topic=41497.0 One encoder is used for all three PID terms. I use my home grown
 * library ky-040.
 */
#define   ENCODER_CLK         2  
#define   ENCODER_DT          4
#define   ENCODER_SW          5
#define   MAX_ROTARIES        3
// Encoder rotary names
#define   KP_ROT              0   // in numerical order to simplify code
#define   KI_ROT              1
#define   KD_ROT              2
#define   KVALUE_MULTIPLIER   0.02
#define   KP_EEPROM_ADD       0
#define   KI_EEPROM_ADD       sizeof(int)
#define   KD_EEPROM_ADD       (2 * sizeof(int))
#define   PID_EEPROM_WRITE    8   // Active LOW switch    

/* Motor pins. These pins must support PWM using analogWrite */
#define   LEFT_WHEEL_BIA      6   // Left Wheel is MotorB PORTD (0-7)
#define   LEFT_WHEEL_BIB      9   // PORTB (8-13)
#define   RIGHT_WHEEL_AIA    10   // Right Wheel is MotorA PORTB
#define   RIGHT_WHEEL_AIB    11   // PORTB
#define   STUTTER_MOTOR_AMOUNT  20    // Stutter the motor when ready to raise

/* Status indicators and switches */
#define   STATUS_LED         13   // PORTB 0x00100000
#define   PORTB_STATUS_LED   B00100000    // Not working?
#define   SELECTED           F(" <-")
#define   NOT_SELECTED       F("")

#ifdef OLED_SCREEN
    #define   OLED_RESET          4
    Adafruit_SSD1306 display(OLED_RESET);
#endif

/* Power monitor pins */
#define   BATTERY_12V        A0   // 12V Battery pack
#define   SUPPLY_7_5V        A1   // Input to Arduino Nano 7.5V

/* Create a sensor. Assign a unique base ID for this sensor   
 * Uses SDA (A4) and SCL (A5) for communication (Nano)
 */
Adafruit_LSM9DS0 sensor = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
sensors_event_t accel, mag, gyro, temp;
#ifdef DEBUG_TERMINAL
    float heading;                // Heading calculated from mag data
#endif

/* Create an encoder with three rotaries for PID terms Kp, Ki, Kd */
ky040 pidEncoder(ENCODER_CLK, ENCODER_DT, ENCODER_SW, MAX_ROTARIES );
uint8_t activeRotaryPid = KP_ROT;
/* PID terms that are read from the rotary encoder pidEncoder */
float KpVal, KiVal, KdVal;

/* Create the Kalman filter instances */
Kalman kalmanPitch;
float kalmanPitchAngle;       // Calculated angle using a Kalman filter
float pitch;                  // Calculated pitch values

#ifdef USE_PITCH_AND_ROLL
    Kalman kalmanRoll;           
    float kalmanRollAngle,roll;   // Calculated roll angle
#endif

/* Create PID and assign Tuning Parameters */
float targetAngle = 0.0, pwmAmount,
      lastError = 0.0, integratedError = 0.0;
/* Specify the links and initial tuning parameters */
PID myPID((double *)&kalmanPitchAngle, (double *)&pwmAmount, (double *)&targetAngle, 
          (double)KpVal, (double)KiVal, (double)KdVal, DIRECT);

#ifdef DEBUG_TERMINAL
    /* Create a low pass filter. */
    LPF lpf(LPF_BANDWIDTH_HZ,SAMPLE_TIME);
#endif

uint16_t batteryReading;

/* ----------------------------------------------------------------------
 *  Initial entry. Setup/initialize the 9DOF sensor, motors, PID rotaries
 * ----------------------------------------------------------------------
 */
void setup ( void ) {
  
    /* Setup motor driver - do early */
    pinMode(LEFT_WHEEL_BIA,OUTPUT);
    digitalWrite(LEFT_WHEEL_BIA,LOW);
    pinMode(LEFT_WHEEL_BIB,OUTPUT);
    digitalWrite(LEFT_WHEEL_BIB,LOW);
    pinMode(RIGHT_WHEEL_AIA,OUTPUT);
    digitalWrite(RIGHT_WHEEL_AIA,LOW);
    pinMode(RIGHT_WHEEL_AIB,OUTPUT);
    digitalWrite(RIGHT_WHEEL_AIB,LOW);

    pinMode(STATUS_LED,OUTPUT);
    digitalWrite(STATUS_LED,HIGH);          // Drives lamp OFF

    pinMode(PID_EEPROM_WRITE,INPUT_PULLUP);
    
    INIT_DISPLAY
    CLEAR_DISPLAY

    /* Setup for a LCD display mounted on the robot */
    //Serial1.begin(9600);
    /* Clear the screen, setup for fast baudrate, switch to it */

    /* Define rotaries for PID terms. 0.0 to 50.0 (divide reading by 10)/:_
     * This may have to change.... need more resolution
     */

#ifdef INIT_EEPROM
    /* First time setup EEPROM */
    EEPROM_writeInt ( KP_EEPROM_ADD, 100 );   // KpVal init to 2.0
    EEPROM_writeInt ( KI_EEPROM_ADD, 0 );     // KiVal init to 0
    EEPROM_writeInt ( KD_EEPROM_ADD, 0 );     // KdVal init to 0   
#endif

    int val = EEPROM_readInt ( KP_EEPROM_ADD );
    pidEncoder.AddRotaryCounter(KP_ROT, val, 0, 2500, 1, false ); // Value of 5 at 0.02/click
    KpVal = (float)val * KVALUE_MULTIPLIER;
    val = EEPROM_readInt ( KI_EEPROM_ADD );
    pidEncoder.AddRotaryCounter(KI_ROT, val, 0, 2500, 1, false );
    KiVal = (float)val * KVALUE_MULTIPLIER;
    val = EEPROM_readInt ( KD_EEPROM_ADD );
    pidEncoder.AddRotaryCounter(KD_ROT, val, 0, 2500, 1, false );
    KiVal = (float)val * KVALUE_MULTIPLIER;
    pidEncoder.SetRotary(activeRotaryPid);
    pidEncoder.SetChanged(KP_ROT);
 
    /* Initialize the sensor and setup gain and integration time*/
    if( !sensor.begin() ) {
      PRINT(F("No LSM9DS0 detected. Halting"));
      SHOW_DISPLAY
      
      /* Blink LED fast for 1 sec  then OFF to show no sensor
       */
      unsigned long timer = millis();
      while(1) {
          digitalWrite(STATUS_LED,HIGH); // Halt and catch fire
          delay(50);
          digitalWrite(STATUS_LED,LOW); // Halt and catch fire
          delay(50); 
          if ( millis() - timer > 1000 ) {
              delay(1000);
              timer = millis();         
          }
      }
    }
    PRINTLN(F("LSM9DS0 Init ok"));
    
    sensor.setupAccel(sensor.LSM9DS0_ACCELRANGE_2G);
    sensor.setupMag(sensor.LSM9DS0_MAGGAIN_2GAUSS);
    sensor.setupGyro(sensor.LSM9DS0_GYROSCALE_245DPS);

    /* 
     * Should we have a warmup delay? The user could be notified
     * by a blinking LED that turns ON when warmup is complete.
     * The user could override this function by merely starting 
     * to raise the robot.
     */
    PRINTLN(F("Gyro stab. wait 3"));
    SHOW_DISPLAY

    sensor.getEvent(&accel, &mag, &gyro, &temp);
    /* Initialize Kalman filter - first get pitch and roll */
    pitch = atan(-accel.acceleration.x / 
        sqrt(accel.acceleration.y * accel.acceleration.y + 
             accel.acceleration.z * accel.acceleration.z)) * RADS_TO_DEG;
    kalmanPitch.setAngle(pitch);
    kalmanPitchAngle = pitch; 
      
#ifdef USE_PITCH_AND_ROLL
    roll  = atan2(accel.acceleration.y, accel.acceleration.z) * RADS_TO_DEG;
    kalmanRoll.setAngle(roll); // Set starting angle
    kalmanRollAngle = roll;
#endif

    /* Initialize PID filter */
    myPID.SetTunings(KpVal, KiVal, KdVal);
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(LOOP_TIME_MSEC);
    myPID.SetOutputLimits(-255,255);    // PWM to motors
    
    /*
     * Stutter the wheels for a couple of seconds to let the user know
     * we are ready to turn upright
     */
    for ( uint8_t i = 0; i < 15; i++ ) {
        SetMotors(STUTTER_MOTOR_AMOUNT, STUTTER_MOTOR_AMOUNT);
        delay(100);
        SetMotors(-STUTTER_MOTOR_AMOUNT,-STUTTER_MOTOR_AMOUNT);
        delay(100);
    }
    SetMotors(0,0);   // OFF 
    
    /* Now Set Balance LED OFF. As the user moves the robot upright, 
     * increase the blink rate of the LED until within BALANCE_START.
     * Then turn LED ON and start loop.
     */
    // code here.....

#ifdef DEBUG_TERMINAL
    PRINTLN(F("***** Starting Balance loop"));
#endif
}

/* ----------------------------------------------------------------------
 * Main execution loop. Basic execution is as follows:
 *    Start timer for deltaTime calculation
 *    Get raw sensor data, accel x, y, x - gyro x, y, z
 *    Convert to pitch and roll
 *    Run through Kalman filter for kalmanPitchAngle kalmanRollAngle
 *    Run through PID to get motor drive pwmAmount
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

    /* Loop is approximately 6.05 msec ----- 
     * Once I have this running (or at least all coded), I will do a
     * more detailed assesment of the timing. If its under 18msec,
     * then I can use micros() instead of millis() to improve time
     * accuracy for the various filters and PID.
     */
    // NO!!!!! WE WANT TO KEEP TRACK OF TOTAL LOOP TIME!!
    // Not much difference though -- 
    startTime = millis();   // Start this pass timing

    /* get latest data from 9DOF sensor */
    sensor.getEvent(&accel, &mag, &gyro, &temp);

    /* Convert accel in degrees to pitch/roll angle. */
    pitch = atan(-accel.acceleration.x / 
        sqrt(accel.acceleration.y * accel.acceleration.y + 
             accel.acceleration.z * accel.acceleration.z)) * RADS_TO_DEG;
             
#ifdef USE_PITCH_AND_ROLL
    roll  = atan2(accel.acceleration.y, accel.acceleration.z) * RADS_TO_DEG;

    /* This fixes the transition problem when the accelerometer angle jumps between 
     * -180 and 180 degrees .  IS THIS NEEDED ???
     */
    if ( (roll < -90 && kalmanRollAngle > 90) || (roll > 90 && kalmanRollAngle < -90) ) {
        kalmanRoll.setAngle(roll);
        kalmanRollAngle = roll;
    } else
        kalmanRollAngle = kalmanRoll.getAngle(roll, gyro.gyro.x, SAMPLE_TIME);
  
    if ( abs(kalmanRollAngle) > 90 )
        gyro.gyro.y = -gyro.gyro.y; // Invert rate, so it fits the restriced accelerometer reading
#endif

    /* Now we run the pitch angle and pitch angle rate into the Kalman filter
     * to get a filtered answer that fuses accelerometer data in pitch degrees 
     * and gyro data in pitch degrees/sec based on the delta time - which is the
     * time from last time to this time.
     */
    kalmanPitchAngle = kalmanPitch.getAngle(pitch, gyro.gyro.y, SAMPLE_TIME) - LEVEL_ANGLE;

    if ( inStartup ) {
        /* If we're in startup, then determine when we are close enough to 
         * upright to take over autobalance.
         * THIS is where we need to give some visual / audible indication
         * that we're close enough
         */  
         if ( abs(kalmanPitchAngle) < ALMOST_UPRIGHT ) {
#ifdef DEBUG_TERMINAL
            PRINTLN(F("Robot upright. Starting autobalance"));
#endif
            inStartup = false;   
         }    
    } else {  // We are in autoBalance mode
        if ( abs(kalmanPitchAngle) > PITCH_TOO_GREAT ) {
            /* we are falling over. Can't compensate. */
            SetMotors(0,0);      // Turn off motors and fall
#ifdef DEBUG_TERMINAL
            PRINTLN(F("Pitch too great. Re-entering Startup"));
#endif
            /* Resetup variables, filters, etc to start over. */
            inStartup = true;
            delay(2000);        // let everything fall down.
        } else {   
            /* Compute error between kalmanPitchAngle and targetAngle. Return a 
             * pwmAmount in the range of -255 (full backwards) to 255 (full forwards)
             */
            myPID.Compute();
            /* How do we inject a turn or movement command into the PID? Break the 
             * PID into PD and PI calculations? To move, we need to set the 
             * angle of the robot to 'lean' into the movement.  Maybe that's how we
             * move. Program a lean angle to cause movement. -Check on this -----
             * If all well and good - set motors to PWM amounts
             */
            /* Should we watch for runaway conditions? What woulld that entail? */
            SetMotors(pwmAmount,pwmAmount);
        }
        
    } /* Done in AutobalanceMode */

#ifdef DEBUG_TERMINAL
    /* Update heading and low pass filter the result */
    heading = atan2(mag.magnetic.y,mag.magnetic.x);
    if ( heading < 0 ) heading += TWO_PI;
    heading *= RADS_TO_DEG;
    /* Need to add magnetic declination offset for our area */
    heading = lpf.NextValue(heading);
#endif

    /* Check for any procedures that are done periodically */

    if ( ++quarterSecondsCount == QUARTER_SEC_COUNT ) {
        quarterSecondsCount = 0;
        // Four times a second we need to check for switch presses and
        // update any serial displays
        CallFourTimesASecond();
    }

    if ( ++secondsCount == ONE_SEC_COUNT ) {
        secondsCount = 0;
        CallEverySecond();
    } 

    startTime = millis() - startTime;   // Loop complete - time it

#ifdef DEBUG_TERMINAL
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
#endif
    
    /* Check loop timing. Adjust delay to keep near LOOP_TIME_MSEC */
    if ( startTime < LOOP_TIME_MSEC )
        delay ( LOOP_TIME_MSEC - startTime );
    else {
        /* We're running slow. What to do if this continues?
         * Perhaps add an overRunCount variable that increments.
         */
#ifdef DEBUG_TERMINAL
        PRINTLN(F("LOOP OVERRUN"));
#endif
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
        analogWrite(RIGHT_WHEEL_AIA,rightPwmAmount);
    }
    else {
        // PORTB &= ~B00001000; 
        digitalWrite(RIGHT_WHEEL_AIA,LOW);
        analogWrite(RIGHT_WHEEL_AIB,-rightPwmAmount); 
    }                  
}

#if 0   
/* 
 * Simple PID --- may use this instead of PID_v1
 */
int UpdatePIDandReturnMotorPWM ( void ) {
    float deltaError, error;
    
    error = targetAngle - pitch;
    
    integratedError += error;
    if ( integratedError > INTEGRATION_GUARD )
        integratedError = INTEGRATION_GUARD;
    else if ( integratedError < -INTEGRATION_GUARD )
        integratedError = -INTEGRATION_GUARD;
  
    deltaError = error - lastError;
    lastError = error;
  
    return constrain(KpVal * error + 
                     KiVal * integratedError + 
                     KdVal * deltaError, -255, 255);   // PWM < 0 means go backwards
}
#endif

/*
 * Fuction called every 250 msec. Here we check rotaries and see if they
 * have changed. If so, update associated PID terms. Update any displays
 * attached to the robot.
 */
void CallFourTimesASecond ( void ) {, updated = false
    bool changed = false, updated = false;
    if ( pidEncoder.SwitchPressed() ) {
        if ( ++activeRotaryPid > KD_ROT ) activeRotaryPid = KP_ROT;
        pidEncoder.SetRotary(activeRotaryPid);
    }
    if ( pidEncoder.HasRotaryValueChanged(KP_ROT) ) {
        KpVal = (float)pidEncoder.GetRotaryValue(KP_ROT) * KVALUE_MULTIPLIER;
        changed = true; 
    }
    if ( pidEncoder.HasRotaryValueChanged(KI_ROT) ) {
        KiVal = (float)pidEncoder.GetRotaryValue(KI_ROT) * KVALUE_MULTIPLIER; 
        changed = true;       
    }
    if ( pidEncoder.HasRotaryValueChanged(KD_ROT) ) {
        KdVal = (float)pidEncoder.GetRotaryValue(KD_ROT) * KVALUE_MULTIPLIER;
        changed = true;        
    }

    if ( digitalRead(PID_EEPROM_WRITE) == LOW ) {
        /* Write PID values to EEPROM */
        EEPROM_writeInt(KP_EEPROM_ADD, (int)(KpVal / KVALUE_MULTIPLIER));
        EEPROM_writeInt(KI_EEPROM_ADD, (int)(KiVal / KVALUE_MULTIPLIER));
        EEPROM_writeInt(KD_EEPROM_ADD, (int)(KdVal / KVALUE_MULTIPLIER));
        updated = true;
        while ( digitalRead(PID_EEPROM_WRITE) == LOW ) ;
        delay(100);
    }

#ifdef OLED_SCREEN
    if ( changed ) {
#endif
        CLEAR_DISPLAY
        PRINT(F("Kp: ")); PRINT(KpVal); 
        PRINTLN(activeRotaryPid == KP_ROT ? SELECTED : NOT_SELECTED); 
        PRINT(F("Ki: ")); PRINT(KiVal); 
        PRINTLN(activeRotaryPid == KI_ROT ? SELECTED : NOT_SELECTED); 
        PRINT(F("Kd: ")); PRINT(KdVal);
        PRINTLN(activeRotaryPid == KD_ROT ? SELECTED : NOT_SELECTED); 
        SHOW_DISPLAY
        myPID.SetTunings(KpVal, KiVal, KdVal);
#ifdef OLED_SCREEN
    }
#endif
        // Once a second, update terminal if connected?
#ifdef DEBUG_TERMINAL
    PRINT(F("Pitch: ")); PRINTLN(kalmanPitchAngle);
#endif

#if defined ( FULL_DEBUG ) && defined ( DEBUG_TERMINAL )
    /* Update lastest Pitch and Roll data */
    PRINT(F("GYRO Pitchrate: ")); PRINT(gyro.gyro.y); PRINTLN(F(" deg/sec"));
    PRINT(F("Raw Pitch: ")); PRINT(pitch); 
    PRINT(F("GYRO Rollrate: ")); PRINT(gyro.gyro.x); PRINTLN(F(" deg/sec"));
    PRINT(F("Raw Roll: ")); PRINT(roll); PRINT(F("\tKalman: ")); PRINTLN(kalmanRollAngle);     
    PRINT(F("\tRoll: ")); PRINTLN(kalmanRollAngle);
    PRINT(F("PWM: ")); PRINTLN(pwmAmount);
#endif
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
    batteryReading = analogRead(BATTERY_12V);   // Resistor divider set for 13V max
    batteryReading = analogRead(SUPPLY_7_5V);            // Resistor divider set for 9V max

    /* Heading and temperature update every second */
#ifdef DEBUG_TERMINAL
    PRINT(F("Temp: ")); PRINT(temp.temperature); PRINTLN(F(" *C"));  
    PRINT(F("Heading: ")); PRINTLN(heading);
#endif

    /* Attach a servo with a pointer that points True North? */
}

/*
 * Functions to read / write integer data to EEPROM
 * Thanks http://forum.arduino.cc/index.php?topic=41497.0
 */
void EEPROM_writeInt ( int ee, int value )
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       EEPROM.write(ee++, *p++);
}

int EEPROM_readInt ( int ee )
{
   int value = 0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
       *p++ = EEPROM.read(ee++);
   return value;
}
