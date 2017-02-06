/*
  Giovanni Gentile
  February 2016
  Arduino 101 Pulse sensor
*/

#include <CurieBLE.h>
#include <PulseSensorBPM.h>

const boolean HAS_A_REF = false; //BUG? analogReference(EXTERNAL) causes a compile error on Arduino 101.
const int PIN_INPUT = A0;
const int PIN_BLINK = 13;        // Pin 13 is the on-board LED
const int PIN_FADE = 3;          // must be a pin that supports PWM.

const unsigned long MICROS_PER_READ = 2 * 1000L;
const boolean REPORT_JITTER_AND_HANG = false;
const long OFFSET_MICROS = 1L;  // NOTE: must be non-negative

unsigned long wantMicros;
long minJitterMicros;
long maxJitterMicros;
unsigned long lastReportMicros;
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 20;
// PWM steps per fade step.  More fades faster; less fades slower.
const int PWM_STEPS_PER_FADE = 12;
int fadePWM;
PulseSensorBPM pulseDetector(PIN_INPUT, MICROS_PER_READ / 1000L);

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)
BLEService heartRateService("180D"); // BLE Heart Rate Service

// BLE Heart Rate Measurement Characteristic"
BLECharacteristic heartRateChar("2A37",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                              // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.heart_rate_measurement.xml

int oldHeartRate = 0;  // last heart rate reading from analog input
long previousMillis = 0;  // last time the heart rate was checked, in ms

void setup() {
  Serial.begin(115200);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("GianniCuore");
  
  blePeripheral.setAdvertisedServiceUuid(heartRateService.uuid());  // add the service UUID

  blePeripheral.addAttribute(heartRateService);   // Add the BLE Heart Rate service
  blePeripheral.addAttribute(heartRateChar); // add the Heart Rate Measurement characteristic
  
  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
  if (HAS_A_REF) {
    //BUG? Causes a compile error on Arduino 101: analogReference(EXTERNAL);
  }
  // PIN_INPUT is set up by the pulseDetector constructor.
  pinMode(PIN_BLINK, OUTPUT);
  digitalWrite(PIN_BLINK, LOW);
  pinMode(PIN_FADE, OUTPUT);
  fadePWM = 0;
  analogWrite(PIN_FADE, fadePWM);   // sets PWM duty cycle

  // Setup our reporting and jitter measurement.
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
  lastReportMicros = 0L;
  resetJitter();

  // wait one sample interval before starting to search for pulses.
  wantMicros = micros() + MICROS_PER_READ;
}

void loop() {
  
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 2) {
        previousMillis = currentMillis;
        updateHeartRate();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  
}

void updateHeartRate() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the heart rate's measurement.
  */
   unsigned long nowMicros = micros();
  if ((long) (wantMicros - nowMicros) > 1000L) {
    return;  // we have time to do other things
  }
  if ((long) (wantMicros - nowMicros) > 3L + OFFSET_MICROS) {
    delayMicroseconds((unsigned int) (wantMicros - nowMicros) - OFFSET_MICROS);
    nowMicros = micros();    
  }
long jitterMicros = (long) (nowMicros - wantMicros);
  if (minJitterMicros > jitterMicros) {
    minJitterMicros = jitterMicros;
  }
  if (maxJitterMicros < jitterMicros) {
    maxJitterMicros = jitterMicros;
  }

  /*
   * If desired, after 60 seconds of running,
   * report our measured Jitter and hang.
   * 
   * NOTE: this mode won't work with the Processing Sketch.
   * It's designed for debug only.
   */
  if (REPORT_JITTER_AND_HANG
      && (long) (nowMicros - lastReportMicros) > 60000000L) {
    lastReportMicros = nowMicros;
    
    Serial.print(F("Jitter (min, max) = "));
    Serial.print(minJitterMicros);
    Serial.print(F(", "));
    Serial.print(maxJitterMicros);
    Serial.println();
    
    resetJitter();

    //hang because our prints are incompatible with the Processing Sketch
    for (;;) { }
  }
  
  wantMicros = nowMicros + MICROS_PER_READ;
  boolean QS = pulseDetector.readSensor();

  if (pulseDetector.isPulse()) {
    digitalWrite(PIN_BLINK, HIGH);
  } else {
    digitalWrite(PIN_BLINK, LOW);
  }

  if (QS) {
    fadePWM = 255;  // start fading on the start of each beat.
    analogWrite(PIN_FADE, fadePWM);
  }


  /*
   * Perform our Serial output. We don't worry about timing, because
   * the documentation for Serial says that "As of version 1.0,
   * serial transmission is asynchronous; Serial.print() will return
   * before any characters are transmitted."
   * 
   * The reader (the Processing Sketch) must read continuously
   * or else our app will block (stop temporarily).
   */

  /*
   * Every so often, send the latest Sample to the Processing Sketch.
   * We don't print every sample, because our baud rate
   * won't support that much I/O.
   */
  if (--samplesUntilReport == (byte) 0) {
    samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

    Serial.print('S');
    Serial.println(pulseDetector.getSignal());

    // Coincidentally, fade the LED a bit.
    fadePWM -= PWM_STEPS_PER_FADE;
    if (fadePWM < 0) {
      fadePWM = 0;
    }
    analogWrite(PIN_FADE, fadePWM);
    
  }

  // Every beat, report the heart rate and inter-beat-interval
  if (QS) {
    Serial.print('B');
    Serial.println(pulseDetector.getBPM());
    Serial.print('Q');
    Serial.println(pulseDetector.getIBI());
  }
  int heartRate = pulseDetector.getBPM();
  if (heartRate != oldHeartRate) {
    Serial.print("Heart Rate is now: "); // print it
    Serial.println(heartRate);
    const unsigned char heartRateCharArray[2] = { 0, (char)heartRate };
    heartRateChar.setValue(heartRateCharArray, 2);  // and update the heart rate measurement characteristic
    oldHeartRate = heartRate;           // save the level for next comparison
  }
}

void resetJitter() {
  // min = a number so large that any value will be smaller than it;
  // max = a number so small that any value will be larger than it.
  minJitterMicros = 60 * 1000L;
  maxJitterMicros = -1;
}
