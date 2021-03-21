// ShopSensor
//
// BMP280 and MCP9808 sensors
// with a push button door sensor
//

//#include <Wire.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MCP9808.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 915.0

//#if defined (__AVR_ATmega32U4__) // Feather 32u4 w/Radio
  #define RFM69_CS      8
  #define RFM69_INT     7
  #define RFM69_RST     4
  #define LED           13
//#endif

// Singleton instance of the radio driver
RH_RF69 rf69(RFM69_CS, RFM69_INT);

// Setup from the Sensor Code
#define BMP_SCK  (13)

// Create the BMP280 sensor object
Adafruit_BMP280 bmp; // I2C

// Create the MCP9808 temperature sensor object
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

// constants won't change. They're used here to set pin numbrers:
//const int iButtonPin = 12;    // the number of the pushbutton pin
//const int ledPin =  13;      // the number of the LED pin - defined above
const int iTempCheck = 5000;    // 5 seconds
const int iTXcheck = 5000;
const int iLoopspeed = 1000;    // 1 seconds

// variables will change:
int16_t ilPacketNum = 0;      // lPacket counter, we increment per transmission
//int iButtonState = 0;        // variable for reading the pushbutton status
int iTempCntrl;             // loop timer for temp control
int iTXcntrl;               // loop timer for tx control
String sRadioMsg;


void setup()
{
  Serial.begin(115200);

  // initialize the LED pin as an output:
  pinMode(LED, OUTPUT);
  // initialize the pushbutton pin as an input:
  //pinMode(iButtonPin, INPUT);
  // initialize the radio as an output:
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Shop Sensor ...");
  Serial.println();

  // SETUP the Radio
  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
    Serial.println("RFM69 radio init failed");
    while (1);
  }
  Serial.println("RFM69 radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    Serial.println("setFrequency failed");
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(14, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  // The encryption key has to be the same as the one in the server
  uint8_t key[] = { 0x00, 0x06, 0x01, 0x08, 0x01, 0x09, 0x07, 0x00,
                    0x00, 0x01, 0x02, 0x03, 0x01, 0x09, 0x07, 0x01};
  rf69.setEncryptionKey(key);

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");

  setup_bmp280();

  setup_mcp9808();
  tempsensor.wake();   // wake up MCP9808 - power consumption ~200 mikro Ampere

  iTempCntrl = iTempCheck;    // force the reading of temps the first time through
  iTXcntrl = iTempCheck;
  sRadioMsg = String();      // initialize the radio message string
}



void loop() {
  float fDegC;       // temperature in C
  long lPa;          // pressure in lPascals
  float fInHg;       // pressure in inches-mercury(HG)
  float fBackDegC;  // backroom temperature in C

  // timer management
  iTempCntrl = iTempCntrl + iLoopspeed;
  iTXcntrl = iTXcntrl + iLoopspeed;

  // Comment out the pushbutton stuff for now
  // read the state of the pushbutton value:
  //iButtonState = digitalRead(iButtonPin);

  // check if the pushbutton is pressed. If it is, the iButtonState is HIGH:
  //if (iButtonState == HIGH) {
    // turn LED on:
    //digitalWrite(LED, HIGH);
  //} else {
    // turn LED off:
    //digitalWrite(LED, LOW);
  //}

  if (iTempCntrl >= iTempCheck) {
    // reset temp_cntr1
    iTempCntrl = 0;

    // Read the BMP280 sensor (located in the main room)
    fDegC = bmp.readTemperature();

    lPa = bmp.readPressure();
    fInHg = lPa / 3386.39;

    // Read and print out the temperature, also shows the resolution mode used for reading.
    fBackDegC = tempsensor.readTempC();

    // Print out the results
    print_values(fDegC, fInHg, fBackDegC);
  }

  if (iTXcntrl >= iTXcheck) {
    // reset iTXcntrl
    iTXcntrl = 0;

    // Let's build the packet
    //sRadioMsg = temp, backtemp, InHg
    sRadioMsg = String(fDegC, 2) + "," + String(fBackDegC, 2) + "," + String(fInHg, 2) + String(" ");
    Serial.print("Sending: "); Serial.println(sRadioMsg);
    
    // Send a message!
    uint8_t data[25];
    sRadioMsg.toCharArray(data, sRadioMsg.length());
    rf69.send(data, strlen(data));
    rf69.waitPacketSent();

    // Now wait for a reply
    uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf69.waitAvailableTimeout(500))  {
      // Should be a reply message for us now
      if (rf69.recv(buf, &len)) {
        Serial.print("Got a reply: ");
        Serial.println((char*)buf);
        Blink(LED, 50, 3); //blink LED 3 times, 50ms between blinks
      } else {
        Serial.println("Receive failed");
      }
    } else {
      Serial.println("No reply, is another RFM69 listening?");
    }
  }

  delay(iLoopspeed);
}


void Blink(byte PIN, byte DELAY_MS, byte loops) {
  for (byte i=0; i<loops; i++)  {
    digitalWrite(PIN,HIGH);
    delay(DELAY_MS);
    digitalWrite(PIN,LOW);
    delay(DELAY_MS);
  }
}


void setup_bmp280() {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }

  Serial.println(F("Found BMP280"));

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.
}


void setup_mcp9808() {
  // Make sure the sensor is found, you can also lPass in a different i2c
  // address with tempsensor.begin(0x19) for example, also can be left in blank for default address use
  // Also there is a table with all addres possible for this sensor, you can connect multiple sensors
  // to the same i2c bus, just configure each sensor with a different address and define multiple objects for that
  //  A2 A1 A0 address
  //  0  0  0   0x18  this is the default address
  //  0  0  1   0x19
  //  0  1  0   0x1A
  //  0  1  1   0x1B
  //  1  0  0   0x1C
  //  1  0  1   0x1D
  //  1  1  0   0x1E
  //  1  1  1   0x1F
  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808! Check your connections and verify the address is correct.");
    while (1);
  }

  Serial.println(F("Found MCP9808!"));

  tempsensor.setResolution(1); // sets the resolution mode of reading, the modes are defined in the table bellow:
  // Mode Resolution SampleTime
  //  0    0.5°C       30 ms
  //  1    0.25°C      65 ms
  //  2    0.125°C     130 ms
  //  3    0.0625°C    250 ms
}


void print_values(float fDegC, float fInHg, float fBackDegC) {
  Serial.println(F("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));

  Serial.println(F("Temperature:"));
  Serial.print("\tFront: "); Serial.print(fDegC); Serial.print(" *C");
  Serial.print("\tBack: "); Serial.print(fBackDegC); Serial.println(" *C");

  Serial.println(F("\nPressure:"));
  Serial.print("\t"); Serial.print(fInHg); Serial.println(" Inches-Hg");
}
