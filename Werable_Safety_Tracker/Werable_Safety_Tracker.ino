#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Wire.h>

#include <math.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11

// Used for hardware & software SPI
#define LIS3DH_CS 10

#define GPSSerial Serial1
#define GPSECHO false

#define BTN_PIN 5

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

Adafruit_GPS GPS(&GPSSerial);

unsigned long acc_timer = millis();
unsigned long hb_timer = millis();
unsigned long btn_timer = millis();
unsigned long GPS_timer = millis();

const int AccelerometerTaskDelay = 100;
const int HeartbeatTaskDelay = 2000;
const int EmergencyBtnTaskDelay = 250;
const int GPSTaskDelay = 3000;

bool emergencyBtn = 0;
bool falling = 0;

float gps_latitude = 0;
float gps_longitude = 0;
float gps_speed = 0;
float gps_altitude = 0;

int tmp_array[2] = {0, 0};

// TTN Configuration
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x73, 0x8B, 0x04, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0x77, 0xD4, 0x48, 0x31, 0x76, 0x5A, 0x57, 0xAE, 0x06, 0x37, 0xB0, 0xAD, 0x96, 0x82, 0x90, 0x1B };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

// payload to send to TTN gateway
static uint8_t payload[16];
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 7;

// Pin mapping for Adafruit Feather M0 LoRa
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

void setup() {
    delay(1000);
    while (! Serial);
    Serial.begin(115200);

    Wire.begin();
    lis.begin(0x19);
    
    pinMode(BTN_PIN, INPUT);

    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 
    GPS.sendCommand(PGCMD_ANTENNA);

    // LMIC init.
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // Disable link-check mode and ADR, because ADR tends to complicate testing.
    LMIC_setLinkCheckMode(0);
    // Set the data rate to Spreading Factor 7.  This is the fastest supported rate for 125 kHz channels, and it
    // minimizes air time and battery power. Set the transmission power to 14 dBi (25 mW).
    LMIC_setDrTxpow(DR_SF7,14);
    // in the US, with TTN, it saves join time if we start on subband 1 (channels 8-15). This will
    // get overridden after the join by parameters from the network. If working with other
    // networks or in other regions, this will need to be changed.
    //LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
    // of the things that will happen is callbacks for transmission complete or received messages. We also
    // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
    // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
    // will want to call `os_runloop_once()` every so often, to keep the radio running.
    os_runloop_once();

    if (millis() - acc_timer > AccelerometerTaskDelay) {
        readAccelerometerData();
    }

    if (millis() - hb_timer > HeartbeatTaskDelay) {
        readHeartbeat();
    }
    
    if (millis() - btn_timer > EmergencyBtnTaskDelay) {
        readEmergencyBtn();
    }
    
    //GPS Loop
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) 
        return;
    }
    if (millis() - GPS_timer > GPSTaskDelay) {
        readGPS();
    }
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:            
            Serial.println(F("Payload sent successfully"));
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            Serial.println(F("Starting new transmission"));
            break;
        default:
            Serial.println(F("ERROR: Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
        // prepare upstream data transmission at the next possible time.
        // transmit on port 1 (the first parameter); you can use any value from 1 to 223 (others are reserved).
        // don't request an ack (the last parameter, if not zero, requests an ack from the network).
        // Remember, acks consume a lot of network resources; don't ask for an ack unless you really need it.
        LMIC_setTxData2(1, payload, sizeof(payload)-1, 0);
        emergencyBtn = 0;
        falling = 0;
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void floatToTwoInt (int myArray[], float myValue){
    int firstPart = myValue;
    myArray[0] = firstPart;
    myValue -= firstPart;
    int secondPart = myValue * 10000;
    myArray[1] = secondPart;
}

void readHeartbeat() {
    if (millis() - hb_timer > 2000) {
        hb_timer = millis();
        Wire.requestFrom(0xA0 >> 1, 1);    // request 1 bytes from slave device
        Wire.available();         
        unsigned char hb = Wire.read();   // receive heart rate value (a byte)
        payload[0] = hb;
    }
}

void readEmergencyBtn() {
    btn_timer = millis();
    byte btn = digitalRead(BTN_PIN);
    if (btn == 1){
        emergencyBtn = 1;
    }
    payload[1] = emergencyBtn;
}

void readGPS() {
    GPS_timer = millis();
    uint8_t GPS_quality = GPS.fixquality;
    payload[13] = GPS_quality;
  
    if (GPS.fix) {
        gps_latitude = GPS.latitudeDegrees;
        gps_longitude = GPS.longitudeDegrees;
        gps_speed = GPS.speed;
        gps_altitude = GPS.altitude;
        uint8_t gps_satellites = GPS.satellites;

        floatToTwoInt(tmp_array, gps_latitude);
        
        payload[2] = tmp_array[0];
        payload[3] = lowByte(tmp_array[1]);
        payload[4] = highByte(tmp_array[1]);
        
        floatToTwoInt(tmp_array, gps_longitude);
        payload[5] = tmp_array[0];
        payload[6] = lowByte(tmp_array[1]);
        payload[7] = highByte(tmp_array[1]);

        uint16_t payload_speed = LMIC_f2sflt16(gps_speed/100);
        payload[8] = lowByte(payload_speed);
        payload[9] = highByte(payload_speed);

        uint16_t payload_alt = LMIC_f2sflt16(gps_altitude/10000);
        payload[10] = lowByte(payload_alt);
        payload[11] = highByte(payload_alt);

        payload[12] = gps_satellites;
    }
}

void readAccelerometerData() {
    acc_timer = millis();
    lis.read();
    sensors_event_t event;
    lis.getEvent(&event);
    double acc = sqrt(pow(event.acceleration.x,2) + pow(event.acceleration.y,2)+ pow(event.acceleration.z,2))-9.81;
    if (acc >= 18)
        falling = 1;
    payload[14] = falling;
}

void displayData(){
    Serial.print("BPM: ");
    Serial.println(payload[0]);

    Serial.print("Emergency: ");
    Serial.println(payload[1]);

    Serial.print("Latitude: ");
    Serial.print(gps_latitude);
    Serial.print(" | Longitude: ");
    Serial.print(gps_longitude);
    Serial.print(" | Speed (Knots): ");
    Serial.print(gps_speed);
    Serial.print(" | Altitude: ");
    Serial.print(gps_altitude);
    Serial.print(" | Satellites: ");
    Serial.print(payload[12]);
    Serial.print(" | Signal: ");
    Serial.println(payload[13]);

    Serial.print(" | Falling: ");
    Serial.println(payload[14]);

    Serial.println("-----------------");
}
