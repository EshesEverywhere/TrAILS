
/*This is a sketch to control the TrAILS Camera device with a Raspberry Pi Pico
  microntroller. This code is specifically written to be used with the
  Arduino Mbed OS RP2040 Boards, Raspberry Pi Pico board manager. This board manager
  is key for integrating with the Arducam Mega Camera developer software. With all
  hardware peripherals prepared, this code can be uploaded to the Pi Pico to run the
  device. Further connectivity utilizes libraries provided for the Botletics SIM7000
  series of mcellular modem.

  Author: Benjamin Nelson, student at Case Western Reserve University

  Credit for Botletics Modem code and information regarding related function calls:
  Author: Timothy Woo (www.botletics.com)
  Github: https://github.com/botletics/SIM7000-LTE-Shield
*/


//**********Initialization************//

// For the calls to the Botletics SIM7000A cellular modem
#include "BotleticsSIM7000.h"  // https://github.com/botletics/Botletics-SIM7000/tree/main/src


// For dtostrf functionality // https://github.com/arduino/ArduinoCore-sam/tree/master/cores/arduino/avr
#include <avr/dtostrf.h>


// Initialization of the Modem related elements
#define SIMCOM_7000

// For botletics SIM7000 shield
#define PWRKEY 6
#define RST 7
#define TX 8  // Microcontroller RX
#define RX 9  // Microcontroller TX


// Further Pin assignments
// Definition of Interrupt Pins
const int cameraControl_Pin = 14;
const int SOS_Pin = 15;

// Pin for Arducam Power Switch
const int Arducam_Switch_Pin = 2;

// Pin for ADC
const int adcPin = 26;


// Parameters to manage states triggered on interrupts
bool SOSEvent = false;
bool inRest = false;
// Time of the device, to be used for SOS button on checking.
unsigned long buttonTime;

// Interrupt Variables/Flags
volatile bool takePictureStatus = false;
volatile bool SOSTriggerStatus = false;
volatile bool SOSOffTriggerStatus = false;
volatile bool checkNetworkStatus = false;


// Parameters for Low Power Detection
bool isBatteryLow = false;
// Battery level
int batteryLevel = 0;

// Parameters for a running average of battery level readings on the ADC
const int batteryAverageSampleSize = 10;
int batteryAverage[batteryAverageSampleSize];
int batteryPointer = 0;
int batteryAverageSum = 0;


// Initializw this large buffer for replies over transmissions
char replybuffer[255];


// Initialization of hardware serial
HardwareSerial* modemSerial = &Serial1;


// Definitions for Modem Operations
// Use this for 2G modules
#ifdef SIMCOM_2G
Botletics_modem modem = Botletics_modem(RST);

// Use this one for 3G modules
#elif defined(SIMCOM_3G)
Botletics_modem_3G modem = Botletics_modem_3G(RST);

// Use this one for LTE CAT-M/NB-IoT modules (like SIM7000)
// Notice how we don't include the reset pin because it's reserved for emergencies on the LTE module!
#elif defined(SIMCOM_7000) || defined(SIMCOM_7070) || defined(SIMCOM_7500) || defined(SIMCOM_7600)
Botletics_modem_LTE modem = Botletics_modem_LTE();
#endif


// MQTT PARAMETERS
#define MQTT_SERVER "ec2-52-23-7-21.compute-1.amazonaws.com"
#define MQTT_PORT 50001
#define MQTT_USERNAME "MQTT_USERNAME"
#define MQTT_PASSWORD "MQTT_PASSWORD"


// Set topic names to publish and subscribe to
#define GPS_TOPIC "location"
#define SOS_TOPIC "sos"


// Parameters for the formatting of data to transmit over MQTT
uint8_t readline(char* buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char imei[16] = { 0 };  // MUST use a 16 character buffer for IMEI!

// Parameters specifically for the GPS transmission message
float latitude, longitude, speed_kph, heading, altitude, gps_second;
uint16_t gps_year;
uint8_t gps_month, gps_day, gps_hour, gps_minute;
uint8_t counter = 0;
uint8_t GPS_Timeout_count = 0;
// NOTE: Keep the buffer sizes as small as possible, espeially on
// Arduino Uno which doesn't have much computing power to handle
// large buffers. On Arduino Mega you shouldn't have to worry much.

// Initialize character strings
char latBuff[12], longBuff[12], locBuff[50], speedBuff[12],
  headBuff[12], altBuff[12], tempBuff[12], battBuff[12];

// Set how many transmission failures in a row we're OK with before reset
uint8_t txfailures = 0;







//**********Methods**********/
// Interrupt Methods


/* This method is called upon SOS interrupt trigger. This method sets the status of the
SOS trigger state to be true if not already in an SOS state
*/
void SOSInterrupt() {
  // If SOS Event in progress, set Off trigger state
  if (SOSEvent) {
    SOSOffTriggerStatus = true;
  }
  // If SOS Event not in progress, set On trigger state
  else {
    SOSTriggerStatus = true;
  }
}


/* This method is called upon Camera interrupt trigger. This method sets the status of the
Camera trigger to be true, only if not in an SOS state
*/
void CameraInterrupt() {
  if (!SOSEvent) {
    if (!takePictureStatus) {
      takePictureStatus = true;
      digitalWrite(20, HIGH);
    }
  }
}


/* This method is called upon Network interrupt trigger. This method sets the status of the
Netowrk trigger to be true
*/
void NetworkInterrupt() {
  if (!checkNetworkStatus)
    checkNetworkStatus = true;
}


/* This method calls the setup of the device prior to the main loop. This initializes the states
of pins and parameters, establishes a serial connection, activates the start sequence of the modem,
performs initial battery measurement readings, and activates the triggers for the interrupt methods
*/
void setup() {

  // Pin setup for ADC input setup
  pinMode(adcPin, INPUT);
  adc_gpio_init(adcPin);
  adc_select_input(0);

  // Pin setup for the Arducam switch
  pinMode(Arducam_Switch_Pin, OUTPUT);
  digitalWrite(Arducam_Switch_Pin, LOW);

  // Pin setup for the Take picture LED
  pinMode(20, OUTPUT);
  digitalWrite(20, LOW);

  // Pin setup for the SOS mode LED
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);

  // Pin setup for the Network status LED
  pinMode(22, OUTPUT);
  digitalWrite(22, LOW);

  // Pin setup for the Low Battery Status LED
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);

  // Pin setup for the Modem trigger connections;
  pinMode(RST, OUTPUT);
  digitalWrite(RST, HIGH);  // Default state
  pinMode(PWRKEY, OUTPUT);


  // Start up sequence of the modem
  // Turn on the module by pulsing PWRKEY low for a little bit
  // This amount of time depends on the specific module that's used
  modem.powerOn(PWRKEY);  // Power on the module

  // Initialize Serial for status when connected via usb to computer
  Serial.begin(9600);
  Serial.println(F("Modem basic test"));
  Serial.println(F("Initializing....(May take several seconds)"));
  // SIM7000 takes about 3s to turn on
  // Press Arduino reset button if the module is still turning on and the board doesn't find it.
  // When the module is on it should communicate right after pressing reset


  // Begin the Hardware serial:
  modemSerial->begin(115200);  // Default SIM7000 baud rate

  if (!modem.begin(*modemSerial)) {
    DEBUG_PRINTLN(F("Couldn't find SIM7000"));
  }

  // Define parameters based off of the properties read from the modem
  type = modem.type();

  Serial.println(F("Modem is OK"));
  Serial.print(F("Found "));
  switch (type) {
    case SIM800L:
      Serial.println(F("SIM800L"));
      break;
    case SIM800H:
      Serial.println(F("SIM800H"));
      break;
    case SIM808_V1:
      Serial.println(F("SIM808 (v1)"));
      break;
    case SIM808_V2:
      Serial.println(F("SIM808 (v2)"));
      break;
    case SIM5320A:
      Serial.println(F("SIM5320A (American)"));
      break;
    case SIM5320E:
      Serial.println(F("SIM5320E (European)"));
      break;
    case SIM7000:
      Serial.println(F("SIM7000"));
      break;
    case SIM7070:
      Serial.println(F("SIM7070"));
      break;
    case SIM7500:
      Serial.println(F("SIM7500"));
      break;
    case SIM7600:
      Serial.println(F("SIM7600"));
      break;
    default:
      Serial.println(F("???"));
      break;
  }

  // Print module IMEI number.
  uint8_t imeiLen = modem.getIMEI(imei);
  if (imeiLen > 0) {
    Serial.print("Module IMEI: ");
    Serial.println(imei);
  }

  // Set modem to full functionality
  modem.setFunctionality(1);  // AT+CFUN=1

  // Configure SIM Card provider settings
  modem.setNetworkSettings(F("iot.1nce.net"));  //For 1NCE Sim Card

  // Set the network status LED blinking pattern while connected to a network (see AT+SLEDS command)
  modem.setNetLED(true, 2, 64, 3000);  // on/off, mode, timer_on, timer_off
  modem.setNetLED(false);              // Disable network status LED

  // Initialization of interrupts
  // Enable buttons after the modem is successfully connected to
  attachInterrupt(digitalPinToInterrupt(15), SOSInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(14), CameraInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(13), NetworkInterrupt, RISING);

  // Attempt initial connections prior to main loop
  ConnectToMQTT();
  CheckNetworkStatus();
  EnableCellularData();

  //Check initial state of battery
  InitializeBatteryReading();
  CheckBatteryLevel();

  //Initial Poll of the system states set by the interrupt triggers
  PollSystem();
}


/* This method is the Main Loop which governs the behavior of the system post-initialization.
The Modem is set to sleep for 60 seconds, and during each of those seconds a poll of the interrupts
is conducted to call the necessary behavior on user interaction with the buttons. Then the modem is
woken back up to read and publish GPS location data.
*/
void loop() {

  // For monitoring loop status
  Serial.print("Next Loop");

  // Set Modem to low power sleep mode
  inRest = true;
  SleepModem();

  // Poll for flags set by interrupts every second
  for (int i = 1; i < 60; i++) {
    PollSystem();
    sleep_ms(1000);
  }

  // Reactivate the modem to full functionality
  inRest = false;
  WakeModem();
  // Poll continually to ensure user inputs accomodated
  PollSystem();

  // Validate MQTT connection
  ConnectToMQTT();
  PollSystem();

  // Transmit GPS location data
  EnableCellularData();
  PollSystem();
  PublishGPSData();
  PollSystem();
}


/* This method polls the state of system by checking the flag variables that are set by the
interrupt method calls. It checks battery level, SOS, Picture, and Newtork statuses and performs
the necessary functionality for the state when activated.
*/
void PollSystem() {
  CheckBatteryLevel();
  SOSPoll();
  PicturePoll();
  NetworkPoll();
}


/* This method polls the SOS trigger from the SOS interrupt function call.
Then SOS functionality is performed.
*/
void SOSPoll() {

  // Checks if the SOS trigger status has been set to true
  if (SOSTriggerStatus) {

    // Checks if the SOS button has been held down continuously for 5 seconds
    bool heldDown = true;
    buttonTime = millis();
    while (millis() - buttonTime < 5000 && heldDown) {
      if (digitalRead(15) == LOW) {
        heldDown = false;
      }
    }
    if (heldDown) {
      // Sets the SOS event LED and transmits an SOS alert to the MQTT server
      digitalWrite(21, HIGH);
      SOSEvent = true;
      PublishSOSData();
    }
    // Resets the SOS trigger status
    SOSTriggerStatus = false;
  }
  // Checks if the SOS off trigger status has been set to true
  else if (SOSOffTriggerStatus) {

    // Checks if the SOS button has been held down continuously for 5 seconds
    bool heldDown = true;
    buttonTime = millis();
    while (millis() - buttonTime < 5000 && heldDown) {
      if (digitalRead(15) == LOW) {
        heldDown = false;
      }
    }
    if (heldDown) {
      // Sets the SOS event LED and transmits an SOS off alert to the MQTT server
      SOSEvent = false;
      PublishSOSData();
      digitalWrite(21, LOW);
    }
    // Resets the SOS off trigger status
    SOSOffTriggerStatus = false;
  }
}


/* This method polls the Take Picture trigger from the Take Picture interrupt call.
Then imaging functionality is performed.
*/
void PicturePoll() {
  // Checks if the take picture status is set to true
  if (takePictureStatus) {
    // Turn the Arducam Mega camera module on
    digitalWrite(Arducam_Switch_Pin, HIGH);

    /* This delay is a placeholder for a method to be added in the future that
    calls for an image to be taken and stored from the Arducam Mega camera.
    */
    delay(2000);

    // Turn off the Arducam Mega camera module and Take Picture LED
    digitalWrite(Arducam_Switch_Pin, LOW);
    digitalWrite(20, LOW);

    // Resets take a picture status. Performed following imaging to prevent unintentional repeat activation
    takePictureStatus = false;
  }
}


/* This method polls the check Network status from the Netowrk interrupt call.
Then network checking functionality is performed.
*/
void NetworkPoll() {
  // Checks if the check network status has been set to true
  if (checkNetworkStatus) {
    // Gets the network status from the modem
    uint8_t n = modem.getNetworkStatus();
    // A key for the network status meanings
    // (n == 0) "Not registered"
    // (n == 1) "Registered (home)"
    // (n == 2) "Not registered (searching)"
    // (n == 3) "Denied"
    // (n == 4) "Unknown"
    // (n == 5) "Registered roaming"
    Serial.println(n);

    // If the device finds a registered connection successfully
    if (n == 1 || n == 5) {
      // Set the blue network LED to be on for 3 seconds continuously
      digitalWrite(22, HIGH);
      delay(3000);
      digitalWrite(22, LOW);

    }
    // If the device is unable to connect to any network or is denied
    else if (n == 0 || n == 3 || n == 4) {
      // Set the blue network LED to be flash every 0.5 seconds for a total of 6 flashes
      for (int i = 0; i < 6; i++) {
        digitalWrite(22, HIGH);
        delay(500);
        digitalWrite(22, LOW);
        delay(500);
      }
    }
    // If the device is able to connect to a network and is searching
    else {
      for (int i = 0; i < 3; i++) {
        // Set the blue network LED to be flash every 1 second for a total of 3 flashes
        digitalWrite(22, HIGH);
        delay(1000);
        digitalWrite(22, LOW);
        delay(1000);
      }
    }

    // Resets the check network status
    checkNetworkStatus = false;
  }
}


/* This method calls for the modem to wake and exit low power mode.
*/
void WakeModem() {
  modem.enableSleepMode(false);
}


/* This method calls for the modem to sleep and enter low power mode.
*/
void SleepModem() {
  modem.enableSleepMode(true);
}


/* This method calls for the modem to enable cellular data transmissions.
*/
void EnableCellularData() {
#if defined(SIMCOM_7500) || defined(SIMCOM_7600)
  modem.enableGPRS(false);
#endif

  // enable data
  if (!modem.enableGPRS(true))
    Serial.println(F("Failed to turn on"));
}


/* This method calls for the modem to disable cellular data transmissions.
*/
void DisableCellularData() {
  if (!modem.enableGPRS(false))
    Serial.println(F("Failed to turn off"));
}


/* This method calls for the modem to connect to the MQTT server listed in the Initialization
section of the code.
*/
void ConnectToMQTT() {

  // If not already connected, connect to MQTT
  if (!modem.MQTT_connectionStatus()) {
    // Set up MQTT parameters (see MQTT app note for explanation of parameter values)
    modem.MQTT_setParameter("URL", MQTT_SERVER, MQTT_PORT);
    // Set up MQTT username and password if necessary
    modem.MQTT_setParameter("USERNAME", MQTT_USERNAME);
    modem.MQTT_setParameter("PASSWORD", MQTT_PASSWORD);
    //    modem.MQTTsetParameter("KEEPTIME", 30); // Time to connect to server, 60s by default

    Serial.println(F("Connecting to MQTT broker..."));
    if (!modem.MQTT_connect(true)) {
      Serial.println(F("Failed to connect to broker!"));
    }
  } else {
    Serial.println(F("Already connected to MQTT server!"));
  }
}


/* This method calls for the modem to enable the GPS functionality
*/
void TurnGPSOn() {
  // Initializes a variable to track timeout
  GPS_Timeout_count = 0;  // Total time is (GPS_Timeout_count * delay time)

  // Attempt to enable to GPS with a timeout count set in the while loop condition
  while (!modem.enableGPS(true) && GPS_Timeout_count < (5 - 1)) {
    delay(2000);  // Retry every 2s
    GPS_Timeout_count++;
    PollSystem();
  }
}


/* This method calls for the modem to disable the GPS functionality
*/
void TurnGPSOff() {
  if (!modem.enableGPS(false))
    Serial.println(F("Failed to turn off"));
}


/* This method calls for the modem to receive GPS data and store the information
in buffers for future MQTT transmissions
*/
void ReceiveGPSData() {

  // Initializes a variable to track timeout
  GPS_Timeout_count = 0;  // Total time is (GPS_Timeout_count * delay time)

  // Attempts to retrieve GPS data with a timeout count set in the while loop condition
  while (!modem.getGPS(&latitude, &longitude, &speed_kph, &heading, &altitude) && GPS_Timeout_count < (5 - 1)) {
    Serial.println(F("Failed to get GPS location, retrying..."));
    delay(2000);  // Retry every 2s
    GPS_Timeout_count++;
    PollSystem();
  }

  // Outputs the data received for viewing over serial
  Serial.println(F("Found Location"));
  Serial.println(F("---------------------"));
  Serial.print(F("Latitude: "));
  Serial.println(latitude, 6);
  Serial.print(F("Longitude: "));
  Serial.println(longitude, 6);
  Serial.print(F("Speed: "));
  Serial.println(speed_kph);
  Serial.print(F("Heading: "));
  Serial.println(heading);
  Serial.print(F("Altitude: "));
  Serial.println(altitude);
  /*
  // Uncomment this if you care about parsing UTC time
  Serial.print(F("Year: ")); Serial.println(year);
  Serial.print(F("Month: ")); Serial.println(gps_month);
  Serial.print(F("Day: ")); Serial.println(gps_day);
  Serial.print(F("Hour: ")); Serial.println(gps_hour);
  Serial.print(F("Minute: ")); Serial.println(gps_minute);
  Serial.print(F("Second: ")); Serial.println(gps_second);
  */
  Serial.println(F("---------------------"));


  // Formats the floating point numbers received from the modem into buffers for future MQTT transmission
  dtostrf(latitude, 1, 6, latBuff);  // float_val, min_width, digits_after_decimal, char_buffer
  dtostrf(longitude, 1, 6, longBuff);
  dtostrf(speed_kph, 1, 0, speedBuff);
  dtostrf(heading, 1, 0, headBuff);
  dtostrf(altitude, 1, 1, altBuff);

  // Also construct a combined, comma-separated location array
  // (many platforms require this for dashboards, like Adafruit IO):
  sprintf(locBuff, "%s,%s,%s,%s", speedBuff, latBuff, longBuff, altBuff);  // This could look like "10,33.123456,-85.123456,120.5"
}


/* This method publishes the SOS Data to the MQTT server
*/
void PublishSOSData() {
  // Prepare string for transmission
  char sos_message[1];
  // If publishing an SOS on alert
  if (SOSEvent) {
    sprintf(sos_message, "O");
  }
  // If publishing an SOS off alert
  else {
    sprintf(sos_message, "F");
  }

  // Wake modem for transmission if not already awake
  WakeModem();

  // Track time of transmission
  GetNetworkTime();

  // Publish the data to the MQTT server
  PublishToMQTT(SOS_TOPIC, sos_message, 1, 0);
  if (inRest) {
    SleepModem();
  }
}


/* This method publishes the GPS data to the MQTT server
*/
void PublishGPSData() {

  // Enables GPS if not already
  TurnGPSOn();
  PollSystem();

  // Receive the GPS location data
  ReceiveGPSData();
  PollSystem();

  //Get the time of transmission
  GetNetworkTime();

  // Publish the GPS data
  PublishToMQTT(GPS_TOPIC, locBuff, 1, 0);
  PollSystem();
}


/* This method publishes the input data with the provided status to the indicated topic
*/
// Parameters for MQTT_publish: Topic, message (0-512 bytes), message length, QoS (0-2), retain (0-1)
//Qos 0 at most once, 1 at least once, 2 exactly once
//retatin sets retain flag, to store previous submission
void PublishToMQTT(const char* topic, const char* message, int qos, int retain) {

  // Connect to the MQTT server if not already
  ConnectToMQTT();

  // Publish to the MQTT server
  if (!modem.MQTT_publish(topic, message, strlen(message), 1, 0)) {
    Serial.println(F("Failed to publish!"));
  }
}


/* This method calls the modem to retrieve the network status
*/
void CheckNetworkStatus() {

  // Checks the network status
  uint8_t n = modem.getNetworkStatus();

  // Outputs the network status over serial
  Serial.print(F("Network status "));
  Serial.print(n);
  Serial.print(F(": "));
  if (n == 0) Serial.println(F("Not registered"));
  if (n == 1) Serial.println(F("Registered (home)"));
  if (n == 2) Serial.println(F("Not registered (searching)"));
  if (n == 3) Serial.println(F("Denied"));
  if (n == 4) Serial.println(F("Unknown"));
  if (n == 5) Serial.println(F("Registered roaming"));
}


/* This method checks the current battery level
*/
void CheckBatteryLevel() {

  // Read the averaged battery level
  batteryLevel = ReadBattery();

  // The ADC is default configured to 10-bit reading mode where 1024 = V_REF ADC, which is 3.3V here
  // The power rail, which achieves a maximum of 4.2V in a resistive divider
  // containing 100k and 220k resistors, thus the current read ADC value is 1024*(11/16 of the rail voltage divided by 3.3V)
  // *Note the integers below were selected on an average value which included droops on the rail
  //  and so may not exactly line up with the results computed by this formula

  // 737 corresponds to 3.60 V averaged on the rail, activates low power indicator
  if (!isBatteryLow && batteryLevel < 737) {

    // Set low power state to be true and turn low power LED on
    isBatteryLow = true;
    digitalWrite(5, HIGH);
  }
  // 749 cooresponds to 3.65V averaged on the rail, deactivates low power indicator
  // Note the hysteresis to prevent the device from rapidly switching between normal and low power state
  else if (isBatteryLow && batteryLevel > 749) {

    // Set low power state to be false and turn low power LED off
    isBatteryLow = false;
    digitalWrite(5, LOW);
  }
}


/* This method reads the battery voltage from the ADC and outputs an averaged value to reduce effects of noise
and power supply droop as smome device elements switch on and off rapidly
*/
int ReadBattery() {

  // Store the most recent battery readings up to the sample size variable and compute the average to indicate battery level
  // Once the end of the array is reached, loop back to the start of the array and replace older measurements accordingly
  batteryAverage[batteryPointer] = analogRead(adcPin);
  if (batteryPointer < batteryAverageSampleSize - 1) {
    batteryPointer++;
  } else {
    batteryPointer = 0;
  }

  // Compute and return the average of array of most recent measured battery levels
  batteryAverageSum = 0;
  for (int sumPointer = 0; sumPointer < batteryAverageSampleSize; sumPointer++) {
    batteryAverageSum = batteryAverageSum + batteryAverage[sumPointer];
  }
  return (batteryAverageSum / batteryAverageSampleSize);
}


/* This method initializes the average battery array to a default state for use in the setup method
*/
void InitializeBatteryReading() {

  // Initialize battery array of size equal to the set sample size to all zeros
  for (int i = 0; i < batteryAverageSampleSize; i++) {
    batteryAverage[i] = 0;
  }

  // Perform an initial battery reading
  batteryLevel = ReadBattery();
}


/* This method calls the modem to get the current network time
*/
void GetNetworkTime() {
  // Get the network time and store it in a buffer
  char buffer[23];
  modem.getTime(buffer, 23);  // make sure replybuffer is at least 23 bytes!

  // Output the read time to serial
  Serial.print(F("Time = "));
  Serial.println(buffer);
}
