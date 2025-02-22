#include <NewPing.h> 
#include "LoRa_E220.h"

/*
  New Ping è una libreria che permette una gestione ottimizzata dei
  sensori di posizione. Fra le altre cose permette di utilizzare 
  un sensore che richiede due pin di digitali (trig e echo) usando
  solo un pin
*/
#define MAX_DISTANCE 200 // distanza massima pingabile in cm 
#define ACTIVATION_TRESH 70 // treshold di attivazione del sensore

// pin per il primo slot
#define PIN_SLOT_1_1 13
#define PIN_SLOT_1_2 12

// pin per il secondo slot
#define PIN_SLOT_2_1 11
#define PIN_SLOT_2_2 10

// pin per il terzo slot

#define PIN_SLOT_3_1 9
#define PIN_SLOT_3_2 8



#define DELTA_READING 100

const uint8_t led1 = 7;
const uint8_t led2 = 6;
const uint8_t led3 = 5;

uint8_t leds[3] = {led1, led2, led3};

#define ZONE 1 // hard coded zone

const uint64_t initRequestRate = 3000; // request rate for init phase
uint64_t stabilize[3] = {0, 0, 0};
bool tryInit = true; // to know when to send again init packet
bool to_stabilize[3] = {true, true, true};

uint64_t startMillis; // timer var for state changing

enum SensorState{
    off_,
    on_,
};

enum ParkingState{
    free_,
    taken_,
};

struct Sensor{
    NewPing sensor;
    SensorState state;

    Sensor(NewPing sens) : sensor(sens), state(off_) {}
};


struct Slot{
  uint8_t zone_; // parking zone
  uint8_t id_; // parking id
  double lat_; // parking latitude
  double lon_; // parking longitude
  Sensor s1_;
  Sensor s2_;
  ParkingState currentState;
  ParkingState futureState;

  /**
  * @brief Constructor for a slot.
  *
  * @param zone The zone that the slot belongs to.
  * @param id The unique id wrt zone.
  * @param lat Latitude of parking slot.
  * @param lon Longitude of parking slot.
  * @param s1 First ultrasonic sensor.
  * @param s2 Second ultrasonic sensor.
  * 
  *
  */
  Slot(uint8_t zone, uint8_t id, double lat, double lon, Sensor sensor1, Sensor sensor2) : 
    zone_(zone), id_(id), lat_(lat), lon_(lon), s1_(sensor1), s2_(sensor2), currentState(free_), futureState(free_) {}

};

struct packet{
  byte header;
  uint8_t id;
  byte footer;

  packet(byte head, uint8_t id_, byte foot):
    header(head), id(id_), footer(foot) {}
};


NewPing s1_1(PIN_SLOT_1_1, PIN_SLOT_1_1, MAX_DISTANCE);
NewPing s1_2(PIN_SLOT_1_2, PIN_SLOT_1_2, MAX_DISTANCE);

NewPing s2_1(PIN_SLOT_2_1, PIN_SLOT_2_1, MAX_DISTANCE);
NewPing s2_2(PIN_SLOT_2_2, PIN_SLOT_2_2, MAX_DISTANCE);

NewPing s3_1(PIN_SLOT_3_1, PIN_SLOT_3_1, MAX_DISTANCE);
NewPing s3_2(PIN_SLOT_3_2, PIN_SLOT_3_2, MAX_DISTANCE);


Sensor distanceSensor1_1(s1_1);
Sensor distanceSensor1_2(s1_2);

Sensor distanceSensor2_1(s2_1);
Sensor distanceSensor2_2(s2_2);

Sensor distanceSensor3_1(s3_1);
Sensor distanceSensor3_2(s3_2);

/*
LoRa definition

RXD refers to Arduino's RX pin
TXD refers to Arduino's TX pin

The comunication is Arduino_1 -> LoRa_1 -> LoRa_2 -> Arduino_2
*/
// pin E220-900T22D
#define RXD 2
#define TXD 3 
LoRa_E220 lora(RXD, TXD);

uint64_t delays[3] = {0, 0, 0};

/////////////////////////////////////////////////// test slots
Slot slots[3] = {
 Slot(ZONE, 1, 44.11, 10.11, distanceSensor1_1, distanceSensor1_2),
 Slot(ZONE, 2, 44.11, 10.11, distanceSensor2_1, distanceSensor2_2),
 Slot(ZONE, 3, 44.11, 10.11, distanceSensor3_1, distanceSensor3_2)
 };
///////////////////////////////////////////////////

bool timeout(uint64_t duration){
  return ((millis() - startMillis) > duration);
}


void setup() {
    Serial.begin(9600);
    while(!Serial){}

    Serial.println("Starting LoRa...");
    lora.begin();
    Serial.println("LoRa succesfully started!");
    ResponseStructContainer c;
    c = lora.getConfiguration();
    Serial.println("Get configuration effettuato");
	  // It's important get configuration pointer before all other operation
	  Configuration configuration = *(Configuration*) c.data;
    printParameters(configuration);

    
    // pinMode(2, OUTPUT);
    // pinMode(3, OUTPUT);
    

    pinMode(7, INPUT);
    pinMode(6, INPUT);
    pinMode(5, INPUT);

    Serial.println("Ora si entra nel loop!");

    delays[0] = millis();
    delays[1] = millis();
    delays[2] = millis();
}

void loop() {
  
    for(size_t i = 0; i < sizeof(slots) / sizeof(Slot); ++i){
        Slot& s = slots[i];

        if((millis() - delays[i]) > 50){
          delays[i] = millis();

          //////// TRANSMITION TEST ////////
          // ResponseStatus rs2 = lora.sendMessage("Hello World?");
          // packet msg(0xFF, s.id_, 0xFE);
          // ResponseStatus rs = lora.sendMessage(&msg, sizeof(struct packet));
          // Serial.println(rs.code);
          // Serial.println(rs.getResponseDescription());
          //////// TEST ////////
          
          int state1 = s.s1_.sensor.ping_cm();
          int state2 = s.s2_.sensor.ping_cm();

          //////// DEBUG PRINT ////////
          // if(s.id_ == 1){
          //   Serial.println(state1);
          //   Serial.println(state2);
          // }
          //////// DEBUG PRINT ////////
          //////// DEBUG PRINT ////////
          // if(state1 != 0){
          //   char str[50];
          //   sprintf(str, "ID: %d, sensore: 1, valore: %d", s.id_, state1);  // Formattiamo la stringa
          //   Serial.println(str);
          // }
          // if(state2 != 0){
          //   char str[50];
          //   sprintf(str, "ID: %d, sensore: 2, valore: %d", s.id_, state2);  // Formattiamo la stringa
          //   Serial.println(str);
          // }


          if(((state1 <= ACTIVATION_TRESH) && (state1 != 0)) && ((state2 <= ACTIVATION_TRESH) && (state2 != 0))){
            state1 = HIGH;
            state2 = HIGH;
          }

          if (state1 == HIGH) {
            s.s1_.state = on_;
            if (to_stabilize[i]){
              stabilize[i] = millis();
              to_stabilize[i] = false;
            }
          }
          else{
            s.s1_.state = off_;
            stabilize[i] = millis();
            to_stabilize[i] = true;
            
          }
          if (state2 == HIGH) {
            s.s2_.state = on_;
            if (to_stabilize[i]){
              stabilize[i] = millis();
              to_stabilize[i] = false;
            }
          }
          else{
            s.s2_.state = off_;
            stabilize[i] = millis();
            to_stabilize[i] = true;
          }
          

          // state change
          // default behaviour is staying in the current state
          s.futureState = s.currentState;
          if((s.currentState == free_) && s.s1_.state == on_ && s.s2_.state == on_ && ((millis() - stabilize[i]) > 1500)){
              s.futureState = taken_;
          }
          else if(s.currentState == taken_ && s.s1_.state == off_ && s.s2_.state == off_){
              s.futureState = free_;
          }

          // on-exit
          if(s.currentState != s.futureState){
            Serial.print("Il sensore ha cambiato stato passando da ");
            Serial.print(s.currentState);
            Serial.print(" a ");
            Serial.println(s.futureState);
            stabilize[i] = millis();
            to_stabilize[i] = true;

            packet msg(0xFF, s.id_, 0xFE);
            Serial.println("DEBUG: STO MANDNANDO IL MESSAGGIO");
            ResponseStatus rs = lora.sendMessage(&msg, sizeof(struct packet));
            Serial.println(rs.getResponseDescription());
          }
          
          // on-entry
          if(s.currentState != s.futureState){
              if(s.currentState == free_ && s.futureState == taken_){
                  Serial.println("Il parcheggio è occupato");
              }
              else if(s.currentState == taken_ && s.futureState == free_){
                  Serial.println("Il parcheggio è libero");

              }    
          }

          s.currentState = s.futureState;

          //output
          if(s.currentState == free_){
            digitalWrite(leds[s.id_], LOW);
          }
          else{
            digitalWrite(leds[s.id_], HIGH);
          }
        }
        // reading input
        


        
        //////////////////////////////////////////////////////////////////////////////
        ///DEBUG VERSION (SOLO 1 SENSORE) Da modificare
        //////////////////////////////////////////////////////////////////////////////

        /*
        int state1 = s.s1_.sensor.ping_cm();
        Serial.println(state1);

        if((state1 <= ACTIVATION_TRESH) && (state1 != 0)){
          state1 = HIGH;
        }

        if (state1 == HIGH) {
          s.s1_.state = on_;
          if (to_stabilize){
            stabilize[i] = millis();
            to_stabilize[i] = false;
          }
        }
        else{
          s.s1_.state = off_;
          stabilize[i] = millis();
          to_stabilize[i] = true;
          
        }

        s.futureState = s.currentState;
        if((s.currentState == free_) && s.s1_.state == on_ && ((millis() - stabilize[i]) > 1500)){
            s.futureState = taken_;
        }
        else if(s.currentState == taken_ && s.s1_.state == off_){
            s.futureState = free_;
        }


        if(s.currentState != s.futureState){
          Serial.print("Il sensore ha cambiato stato passando da ");
          Serial.print(s.currentState);
          Serial.print(" a ");
          Serial.println(s.futureState);
          stabilize[i] = millis();
          to_stabilize[i] = true;
          packet msg(0xFF, s.id_, 0xFE);
          //ResponseStatus rs = lora.sendMessage(String(s.id_));
          ResponseStatus rs = lora.sendMessage(&msg, sizeof(struct packet));
          //lora.sendMessage(String(s.id_).c_str());
          Serial.print(rs.getResponseDescription());
        }

        if(s.currentState != s.futureState){
            if(s.currentState == free_ && s.futureState == taken_){
                Serial.println("Il parcheggio è occupato");
            }
            else if(s.currentState == taken_ && s.futureState == free_){
                Serial.println("Il parcheggio è libero");

            }    
        }

        s.currentState = s.futureState;
        */
    }
}


void printParameters(struct Configuration configuration) {
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));  Serial.print(configuration.COMMAND, HEX);Serial.print(" ");Serial.print(configuration.STARTING_ADDRESS, HEX);Serial.print(" ");Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));  Serial.print(configuration.OPTION.subPacketSetting, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());


  Serial.println("----------------------------------------");
}
