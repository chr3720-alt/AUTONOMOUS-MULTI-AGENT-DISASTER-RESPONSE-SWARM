#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//==================================================
// ESC Pins
//==================================================

#define ESC1 18
#define ESC2 19
#define ESC3 21
#define ESC4 22

//==================================================
// LoRa Pins
//==================================================

#define SS    5
#define RST   27
#define DIO0  26

//==================================================
// MPU6050 I2C
//==================================================

#define SDA_PIN 32
#define SCL_PIN 33

//==================================================
// PWM
//==================================================

#define PWM_FREQ 50
#define PWM_RESOLUTION 16

#define MIN_US 1000
#define MAX_US 1700

// If you want more headroom for punch-outs later, you can raise MAX_US
// toward 2000 once you've confirmed everything flies correctly at 1700.

//==================================================
// LEDC API compatibility (ESP32 core 2.x vs 3.x)
//==================================================
// ledcSetup()/ledcAttachPin() were removed in core 3.x in favor of
// ledcAttach(pin, freq, res). This block auto-selects the right API
// so the sketch compiles on either core version.

#if defined(ESP_ARDUINO_VERSION) && defined(ESP_ARDUINO_VERSION_VAL)
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)
    #define USE_NEW_LEDC_API
  #endif
#endif

//==================================================
// Receiver / Stick Mapping
//==================================================

#define STICK_CENTER 2048
#define STICK_DEADBAND 40

#define MAX_ROLL_ANGLE 20.0
#define MAX_PITCH_ANGLE 20.0
#define MAX_YAW_RATE 90.0     // deg/sec, yaw is rate-controlled (no heading reference)

#define EXPO 0.30

#define FAILSAFE_TIME 500     // ms without a valid packet before failsafe cuts motors

// Telemetry (drone -> transmitter) send interval
// Deliberately infrequent. Every telemetry send is time this radio
// spends transmitting instead of listening for your control packets -
// the control link (throttle/roll/pitch/yaw + failsafe) is what
// actually matters for flight safety, telemetry is just a nice-to-have
// status display. Sending it too often (this was previously set to
// 200ms) was starving control packet reception and causing the
// failsafe to force-disarm shortly after every arm. Do not lower this
// again without watching the drone's own Armed:/Thr: debug line for
// dropouts first.
#define TELEMETRY_INTERVAL 500  // ms

//==================================================

Adafruit_MPU6050 mpu;

//==================================================
// LoRa Inputs
//==================================================
// Packet format from transmitter: throttle,yaw,roll,pitch
// Transmitter pre-centers roll/yaw/pitch to 2048 at boot, so STICK_CENTER
// here will always match what's actually being sent.

int throttle = 0;
int yaw = STICK_CENTER;
int rollInput = STICK_CENTER;
int pitchInput = STICK_CENTER;

//==================================================
// Attitude
//==================================================

float roll = 0;
float pitch = 0;

float targetRoll = 0;
float targetPitch = 0;
float targetYawRate = 0;

//==================================================
// Gyro Offset
//==================================================

float gyroOffsetX = 0;
float gyroOffsetY = 0;
float gyroOffsetZ = 0;

//==================================================
// PID - Roll / Pitch (angle control)
//==================================================

float Kp = 1.5;
float Ki = 0.0;
float Kd = 0.05;

float rollIntegral = 0;
float pitchIntegral = 0;

float previousRollError = 0;
float previousPitchError = 0;

//==================================================
// PID - Yaw (rate control, P only - safe default)
//==================================================

float KpYaw = 2.0;

float rollPID = 0;
float pitchPID = 0;
float yawPID = 0;

//==================================================
// Motor Outputs
//==================================================

float m1 = MIN_US;
float m2 = MIN_US;
float m3 = MIN_US;
float m4 = MIN_US;

//==================================================
// Timing
//==================================================

unsigned long previousTime;
unsigned long lastPacketTime = 0;
unsigned long telemetryTimer = 0;   // single, top-level - not re-declared anywhere else

bool armed = false;

unsigned long armTimer = 0;
unsigned long disarmTimer = 0;

//==================================================

int usToDuty(int us)
{
    return map(us,1000,2000,3277,6553);
}

//==================================================
// Motor write wrapper (handles both LEDC APIs)
//==================================================

void writeMotor(int channel, int pin, int us)
{
    int duty = usToDuty(us);

#ifdef USE_NEW_LEDC_API
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

//==================================================

void stopMotors()
{
    writeMotor(0, ESC1, MIN_US);
    writeMotor(1, ESC2, MIN_US);
    writeMotor(2, ESC3, MIN_US);
    writeMotor(3, ESC4, MIN_US);

    m1 = MIN_US;
    m2 = MIN_US;
    m3 = MIN_US;
    m4 = MIN_US;
}

//==================================================

int applyDeadband(int value)
{
    if(abs(value-STICK_CENTER)<STICK_DEADBAND)
        return STICK_CENTER;

    return value;
}

//==================================================

float applyExpo(float value)
{
    return (1.0-EXPO)*value +
           EXPO*value*value*value;
}

//==================================================

void resetPID()
{
    rollIntegral = 0;
    pitchIntegral = 0;

    previousRollError = 0;
    previousPitchError = 0;
}

//==================================================
// Telemetry - sent back to the transmitter so it can show real
// link/arm status instead of just a local heartbeat.
//==================================================

void sendTelemetry()
{
    LoRa.beginPacket();
    LoRa.print(armed ? "ARMED" : "DISARMED");
    LoRa.endPacket();

    Serial.print("[TELEMETRY SENT] ");
    Serial.println(armed ? "ARMED" : "DISARMED");
}

//==================================================

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN,SCL_PIN);

    delay(1000);

    if(!mpu.begin())
    {
        Serial.println("MPU6050 Failed");
        while(1);
    }

    Serial.println("MPU6050 Connected");

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(2000);

    Serial.println("Calibrating Gyroscope...");
    Serial.println("Keep the drone completely still and level.");

    for(int i=0;i<2000;i++)
    {
        sensors_event_t accel,gyro,temp;

        mpu.getEvent(&accel,&gyro,&temp);

        gyroOffsetX += gyro.gyro.x;
        gyroOffsetY += gyro.gyro.y;
        gyroOffsetZ += gyro.gyro.z;

        delay(2);
    }

    gyroOffsetX/=2000;
    gyroOffsetY/=2000;
    gyroOffsetZ/=2000;

    Serial.println("Calibration Complete");

    //================ LoRa ==================

    SPI.begin(14,12,13,5);

    LoRa.setPins(SS,RST,DIO0);

    if(!LoRa.begin(433E6))
    {
        Serial.println("LoRa Failed");
        while(1);
    }

    LoRa.setSyncWord(0x12);

    Serial.println("LoRa Ready");

    //================ PWM ==================

#ifdef USE_NEW_LEDC_API
    ledcAttach(ESC1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ESC2, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ESC3, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ESC4, PWM_FREQ, PWM_RESOLUTION);
#else
    ledcSetup(0,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(1,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(2,PWM_FREQ,PWM_RESOLUTION);
    ledcSetup(3,PWM_FREQ,PWM_RESOLUTION);

    ledcAttachPin(ESC1,0);
    ledcAttachPin(ESC2,1);
    ledcAttachPin(ESC3,2);
    ledcAttachPin(ESC4,3);
#endif

    stopMotors();

    Serial.println("Arming ESCs...");
    delay(8000);

    Serial.println("Drone Ready");

    previousTime = micros();
}

//==================================================
// LOOP
//==================================================

void loop()
{
    //--------------------------------------------------
    // Read MPU6050
    //--------------------------------------------------

    sensors_event_t accel, gyro, temp;

    mpu.getEvent(&accel, &gyro, &temp);

    //--------------------------------------------------
    // Time
    //--------------------------------------------------

    unsigned long currentTime = micros();
    float dt = (currentTime - previousTime) / 1000000.0;
    previousTime = currentTime;

    if(dt <= 0)
        return;

    //--------------------------------------------------
    // Accelerometer Angles
    //--------------------------------------------------

    float accelRoll =
        atan2(accel.acceleration.y,
              accel.acceleration.z)
        *180.0/PI;

    float accelPitch =
        atan2(-accel.acceleration.x,
              sqrt(accel.acceleration.y*accel.acceleration.y +
                   accel.acceleration.z*accel.acceleration.z))
        *180.0/PI;

    //--------------------------------------------------
    // Gyroscope Rates
    //--------------------------------------------------

    float gyroRollRate =
        (gyro.gyro.x - gyroOffsetX)
        *180.0/PI;

    float gyroPitchRate =
        (gyro.gyro.y - gyroOffsetY)
        *180.0/PI;

    float gyroYawRate =
        (gyro.gyro.z - gyroOffsetZ)
        *180.0/PI;

    //--------------------------------------------------
    // Complementary Filter
    //--------------------------------------------------

    roll =
        0.98*(roll + gyroRollRate*dt)
        +0.02*accelRoll;

    pitch =
        0.98*(pitch + gyroPitchRate*dt)
        +0.02*accelPitch;

    //--------------------------------------------------
    // Read LoRa Packet
    //--------------------------------------------------

    int packetSize = LoRa.parsePacket();

    if(packetSize)
    {
        String data="";

        while(LoRa.available())
            data += (char)LoRa.read();

        int rThrottle, rYaw, rRoll, rPitch;

        if(sscanf(data.c_str(),
                  "%d,%d,%d,%d",
                  &rThrottle,
                  &rYaw,
                  &rRoll,
                  &rPitch)==4)
        {
            // Basic sanity check - reject obviously corrupted packets
            if(rThrottle>=0 && rThrottle<=4095 &&
               rYaw>=0     && rYaw<=4095 &&
               rRoll>=0    && rRoll<=4095 &&
               rPitch>=0   && rPitch<=4095)
            {
                throttle   = rThrottle;
                yaw        = rYaw;
                rollInput  = rRoll;
                pitchInput = rPitch;

                lastPacketTime = millis();
            }
        }
    }

    //--------------------------------------------------
    // ARM
    //--------------------------------------------------

    if(!armed)
    {
        if(throttle < 800 && yaw < 900)
        {
            if(armTimer==0)
                armTimer = millis();

            if(millis()-armTimer>=2000)
            {
                armed=true;
                armTimer=0;

                Serial.println("******** DRONE ARMED ********");
            }
        }
        else
        {
            armTimer=0;
        }
    }

    //--------------------------------------------------
    // DISARM
    //--------------------------------------------------

    if(armed)
    {
        if(throttle<250 && yaw>3800)
        {
            if(disarmTimer==0)
                disarmTimer=millis();

            if(millis()-disarmTimer>=2000)
            {
                armed=false;
                disarmTimer=0;

                stopMotors();
                resetPID();

                Serial.println("******** DRONE DISARMED ********");
            }
        }
        else
        {
            disarmTimer=0;
        }
    }

    //--------------------------------------------------
    // Failsafe
    //--------------------------------------------------

    if(millis()-lastPacketTime > FAILSAFE_TIME)
    {
        armed=false;

        stopMotors();
        resetPID();
    }

    rollPID = 0;
    pitchPID = 0;
    yawPID = 0;

    //--------------------------------------------------
    // Telemetry - runs every loop regardless of arm state,
    // so the transmitter always knows current status.
    //--------------------------------------------------

    if(millis() - telemetryTimer >= TELEMETRY_INTERVAL)
    {
        telemetryTimer = millis();
        sendTelemetry();
    }

    if(!armed)
    {
        return;
    }

    //--------------------------------------------------
    // Receiver Processing - Roll / Pitch (angle)
    //--------------------------------------------------

    int rollDB  = applyDeadband(rollInput);
    int pitchDB = applyDeadband(pitchInput);
    int yawDB   = applyDeadband(yaw);

    float rollStick =
        (rollDB - STICK_CENTER) / 2047.0;

    float pitchStick =
        (pitchDB - STICK_CENTER) / 2047.0;

    float yawStick =
        (yawDB - STICK_CENTER) / 2047.0;

    rollStick = applyExpo(rollStick);
    pitchStick = applyExpo(pitchStick);
    yawStick = applyExpo(yawStick);

    targetRoll  = rollStick * MAX_ROLL_ANGLE;
    targetPitch = pitchStick * MAX_PITCH_ANGLE;
    targetYawRate = yawStick * MAX_YAW_RATE;

    //--------------------------------------------------
    // PID Error - Roll / Pitch
    //--------------------------------------------------

    float rollError =
        targetRoll - roll;

    float pitchError =
        targetPitch - pitch;

    //--------------------------------------------------
    // Integral
    //--------------------------------------------------

    rollIntegral += rollError * dt;
    pitchIntegral += pitchError * dt;

    rollIntegral =
        constrain(rollIntegral, -50, 50);

    pitchIntegral =
        constrain(pitchIntegral, -50, 50);

    //--------------------------------------------------
    // Derivative
    //--------------------------------------------------

    float rollDerivative =
        (rollError - previousRollError) / dt;

    float pitchDerivative =
        (pitchError - previousPitchError) / dt;

    previousRollError = rollError;
    previousPitchError = pitchError;

    //--------------------------------------------------
    // PID Output - Roll / Pitch
    //--------------------------------------------------

    rollPID =
        Kp * rollError +
        Ki * rollIntegral +
        Kd * rollDerivative;

    pitchPID =
        Kp * pitchError +
        Ki * pitchIntegral +
        Kd * pitchDerivative;

    //--------------------------------------------------
    // PID Output - Yaw (rate control, P only)
    //--------------------------------------------------

    float yawRateError = targetYawRate - gyroYawRate;

    yawPID = KpYaw * yawRateError;

    //--------------------------------------------------
    // Motor Control
    //--------------------------------------------------

    if(throttle < 250)
    {
        stopMotors();
        resetPID();
    }
    else
    {
        //--------------------------------------------------
        // Base Throttle
        //--------------------------------------------------

        float baseThrottle =
            map(throttle,
                250,
                4095,
                MIN_US,
                MAX_US);

        //--------------------------------------------------
        // X-Frame Motor Mixing
        //
        // Motor Layout:
        //
        //        Front
        //
        //      M1(CCW)   M2(CW)
        //
        //      M3(CW)    M4(CCW)
        //
        // IMPORTANT: verify yaw direction on the bench (props off)
        // before flying. If yawing the frame by hand while armed
        // shows the correction fighting the wrong way, flip the
        // sign of every yawPID term below.
        //--------------------------------------------------

        m1 = baseThrottle + pitchPID - rollPID - yawPID;
        m2 = baseThrottle + pitchPID + rollPID + yawPID;
        m3 = baseThrottle - pitchPID - rollPID + yawPID;
        m4 = baseThrottle - pitchPID + rollPID - yawPID;

        //--------------------------------------------------
        // Limit Outputs
        //--------------------------------------------------

        m1 = constrain(m1, MIN_US, MAX_US);
        m2 = constrain(m2, MIN_US, MAX_US);
        m3 = constrain(m3, MIN_US, MAX_US);
        m4 = constrain(m4, MIN_US, MAX_US);

        //--------------------------------------------------
        // Send PWM
        //--------------------------------------------------

        writeMotor(0, ESC1, (int)m1);
        writeMotor(1, ESC2, (int)m2);
        writeMotor(2, ESC3, (int)m3);
        writeMotor(3, ESC4, (int)m4);
    }

    //--------------------------------------------------
    // Debug
    //--------------------------------------------------

    static unsigned long printTimer = 0;

    if(millis() - printTimer >= 100)
    {
        printTimer = millis();

        Serial.print("Roll:");
        Serial.print(roll,1);

        Serial.print(" Pitch:");
        Serial.print(pitch,1);

        Serial.print(" TR:");
        Serial.print(targetRoll,1);

        Serial.print(" TP:");
        Serial.print(targetPitch,1);

        Serial.print(" TYawRate:");
        Serial.print(targetYawRate,1);

        Serial.print(" PID_R:");
        Serial.print(rollPID,1);

        Serial.print(" PID_P:");
        Serial.print(pitchPID,1);

        Serial.print(" PID_Y:");
        Serial.print(yawPID,1);

        Serial.print(" Thr:");
        Serial.print(throttle);

        Serial.print(" Armed:");
        Serial.print(armed);

        Serial.print(" M1:");
        Serial.print((int)m1);

        Serial.print(" M2:");
        Serial.print((int)m2);

        Serial.print(" M3:");
        Serial.print((int)m3);

        Serial.print(" M4:");
        Serial.println((int)m4);
    }
}
