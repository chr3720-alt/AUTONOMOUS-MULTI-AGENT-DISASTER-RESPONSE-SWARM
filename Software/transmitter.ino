#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== LoRa Pins =====
#define SS    10
#define RST   3
#define DIO0  4

// ===== Joystick Pins =====
#define THROTTLE_PIN 1
#define YAW_PIN      2
#define ROLL_PIN     8
#define PITCH_PIN    9

// ===== OLED (I2C) =====
// Pins chosen to avoid conflicting with the LoRa SPI pins (10-13) and
// the joystick ADC pins (1,2,8,9) above. Double-check these are broken
// out and free on your specific ESP32-S3 supermini board - silkscreen
// labeling varies between vendors. Change if needed.
#define OLED_SDA 5
#define OLED_SCL 6

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C   // common default; try 0x3D if display stays blank

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Heartbeat pulses every time WE send a control packet - confirms the
// transmitter itself is alive and transmitting.
bool heartbeatOn = false;
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_INTERVAL 150  // ms, don't refresh every loop - I2C writes aren't free

// ===== Telemetry received FROM the drone =====
// The drone sends "ARMED"/"DISARMED" roughly every 500ms - deliberately
// infrequent, since telemetry sends compete with the drone's ability to
// receive your control packets over the same half-duplex radio. If we
// haven't heard anything in a while, that means the link itself is
// down (drone off, out of range, etc) - not the same thing as the
// drone being disarmed, so we show a distinct "NO LINK" state for that
// case. This timeout is set loose enough to ride out normal collision
// misses without flickering, while still catching a genuine loss of
// connection reasonably quickly. Cosmetic flicker here is an
// acceptable tradeoff - do NOT "fix" it by speeding up telemetry sends
// again, that compromises the actual control link.
String linkStatus = "NO LINK";
unsigned long lastTelemetryReceived = 0;
#define TELEMETRY_TIMEOUT 3000  // ms

// Interrupt-driven receive: instead of manually calling parsePacket()
// right after every send (which was fighting with beginPacket() for
// radio mode and silently breaking transmission - the actual cause of
// arming stopping working), we register a callback that fires
// automatically whenever a packet arrives. The main loop just checks
// a flag - no manual mode-juggling near the send path.
volatile bool telemetryFlag = false;
String pendingTelemetry = "";

void onLoRaReceive(int packetSize)
{
    if (packetSize == 0)
        return;

    String data = "";

    while (LoRa.available())
        data += (char)LoRa.read();

    pendingTelemetry = data;
    telemetryFlag = true;
}

// The receiver expects roll/yaw/pitch centered on 2048 with a small
// deadband around it. Physical joysticks/pots rarely sit exactly at
// 2048 at rest, so instead of hardcoding a center value here, we
// measure each axis's actual rest position at boot and apply an
// offset so the value we TRANSMIT is always centered on 2048.
// This keeps the transmitter and receiver in agreement regardless
// of your specific joystick hardware.

#define ADC_MAX 4095
#define TARGET_CENTER 2048
#define DEADBAND 20

int yawOffset   = 0;
int rollOffset  = 0;
int pitchOffset = 0;

//==================================================

int readCentered(int pin, int offset)
{
    int raw = analogRead(pin);
    int value = raw + offset;

    value = constrain(value, 0, ADC_MAX);

    return value;
}

//==================================================

int calibrateOffset(int pin)
{
    long sum = 0;
    const int samples = 200;

    for(int i=0;i<samples;i++)
    {
        sum += analogRead(pin);
        delay(2);
    }

    int avg = sum/samples;

    return TARGET_CENTER - avg;
}

//==================================================

void updateDisplay(int throttle, int yaw, int roll, int pitch, bool packetSent)
{
    if(millis() - lastDisplayUpdate < DISPLAY_INTERVAL)
        return;

    lastDisplayUpdate = millis();

    if(millis() - lastTelemetryReceived > TELEMETRY_TIMEOUT)
        linkStatus = "NO LINK";

    if(packetSent)
        heartbeatOn = !heartbeatOn;

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,0);
    display.print("RC TRANSMITTER");

    // Heartbeat dot, top-right corner - confirms WE are transmitting
    if(heartbeatOn)
        display.fillCircle(122, 3, 3, SSD1306_WHITE);
    else
        display.drawCircle(122, 3, 3, SSD1306_WHITE);

    display.drawLine(0, 11, 128, 11, SSD1306_WHITE);

    // Real status reported back by the drone
    display.setCursor(0, 14);
    display.print("Status: ");
    display.println(linkStatus);

    display.setCursor(0, 26);
    display.print("Throttle: ");
    display.println(throttle);

    display.setCursor(0, 37);
    display.print("Yaw:      ");
    display.println(yaw);

    display.setCursor(0, 48);
    display.print("Roll:     ");
    display.println(roll);

    display.setCursor(0, 56);
    display.print("Pitch:    ");
    display.println(pitch);
    // Layout note: 5 lines from y=14 to y=56 (last line occupies rows
    // 56-63) fits exactly within the 64px screen height. If you add
    // more lines later, keep spacing <= 10px/line measured from y=14.

    display.display();
}

//==================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("ESP32 Started");

    // ===== OLED Init =====
    Wire.begin(OLED_SDA, OLED_SCL);

    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("OLED init failed - check wiring/address");
        // Not halting here: flying without the screen is fine,
        // losing LoRa isn't. Continue on.
    }
    else
    {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0,0);
        display.println("Starting up...");
        display.display();
    }

    // SPI for SX1278
    SPI.begin(12, 13, 11, 10);

    LoRa.setPins(SS, RST, DIO0);

    Serial.println("Starting LoRa Transmitter...");

    if (!LoRa.begin(433E6))
    {
        Serial.println("LoRa init failed!");

        display.clearDisplay();
        display.setCursor(0,0);
        display.println("LoRa init FAILED");
        display.display();

        while (1);
    }

    // LoRa Settings
    LoRa.setTxPower(20);
    LoRa.setSpreadingFactor(7);
    LoRa.setSyncWord(0x12);

    LoRa.onReceive(onLoRaReceive);
    LoRa.receive();  // start listening immediately, even before first send

    Serial.println("LoRa Ready");

    // ===== Stick Calibration =====
    // Keep roll/pitch/yaw sticks centered (untouched) during boot.
    // Throttle is NOT calibrated this way since its rest position
    // is meant to be at the bottom, not centered.

    Serial.println("Calibrating stick centers - leave roll/pitch/yaw sticks untouched...");

    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Calibrating sticks...");
    display.println("Leave R/Y/P centered");
    display.display();

    yawOffset   = calibrateOffset(YAW_PIN);
    rollOffset  = calibrateOffset(ROLL_PIN);
    pitchOffset = calibrateOffset(PITCH_PIN);

    Serial.print("Yaw offset: ");
    Serial.println(yawOffset);

    Serial.print("Roll offset: ");
    Serial.println(rollOffset);

    Serial.print("Pitch offset: ");
    Serial.println(pitchOffset);

    Serial.println("Calibration complete.");

    display.clearDisplay();
    display.setCursor(0,0);
    display.println("Calibration done");
    display.println("Ready to fly");
    display.display();

    delay(1000);
}

//==================================================

void loop()
{
    int throttle = analogRead(THROTTLE_PIN);

    int yaw   = readCentered(YAW_PIN, yawOffset);
    int roll  = readCentered(ROLL_PIN, rollOffset);
    int pitch = readCentered(PITCH_PIN, pitchOffset);

    // Throttle cutoff
    if (throttle < 450)
        throttle = 0;

    // Deadband (applied here too, on top of receiver's own deadband,
    // just keeps the transmitted stream a bit cleaner)
    if (abs(yaw - TARGET_CENTER) < DEADBAND)
        yaw = TARGET_CENTER;

    if (abs(roll - TARGET_CENTER) < DEADBAND)
        roll = TARGET_CENTER;

    if (abs(pitch - TARGET_CENTER) < DEADBAND)
        pitch = TARGET_CENTER;

    // Send packet: throttle,yaw,roll,pitch
    LoRa.beginPacket();

    LoRa.print(throttle);
    LoRa.print(",");

    LoRa.print(yaw);
    LoRa.print(",");

    LoRa.print(roll);
    LoRa.print(",");

    LoRa.print(pitch);

    LoRa.endPacket();

    // Re-arm continuous receive mode after sending. This single call is
    // enough - the onLoRaReceive callback (interrupt-driven) handles
    // actually catching packets, so we don't manually call parsePacket()
    // here anymore (that redundant call right after receive() was
    // fighting with beginPacket() and silently broke transmission).
    LoRa.receive();

    //--------------------------------------------------
    // Check for telemetry coming back from the drone
    // (set asynchronously by the onLoRaReceive interrupt callback)
    //--------------------------------------------------

    if(telemetryFlag)
    {
        telemetryFlag = false;

        Serial.print("[RX] data='");
        Serial.print(pendingTelemetry);
        Serial.println("'");

        if(pendingTelemetry == "ARMED" || pendingTelemetry == "DISARMED")
        {
            linkStatus = pendingTelemetry;
            lastTelemetryReceived = millis();
        }
    }

    // Serial Monitor
    Serial.print("T:");
    Serial.print(throttle);

    Serial.print(" Y:");
    Serial.print(yaw);

    Serial.print(" R:");
    Serial.print(roll);

    Serial.print(" P:");
    Serial.println(pitch);

    // OLED update (throttled internally to DISPLAY_INTERVAL)
    updateDisplay(throttle, yaw, roll, pitch, true);

    delay(20);
}
