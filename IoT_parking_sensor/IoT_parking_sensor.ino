#include <NewPing.h> 
#include "LoRa_E220.h"

/*
  New Ping è una libreria che permette una gestione ottimizzata dei
  sensori di posizione. Fra le altre cose permette di utilizzare 
  un sensore che richiede due pin di digitali (trig e echo) usando
  solo un pin
*/
#define MAX_DISTANCE 100 // distanza massima pingabile in cm 
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

// pin E220-900T22D
#define RXD 2
#define TXD 3 

#define DELTA_READING 100

const uint8_t led1 = 7;
const uint8_t led2 = 6;
const uint8_t led3 = 5;

uint8_t leds[3] = {led1, led2, led3};

#define ZONE 1 // hard coded zone

const uint64_t initRequestRate = 3000; // request rate for init phase
uint64_t stabilize = 0;
bool tryInit = true; // to know when to send again init packet
bool to_stabilize = true;

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

LoRa_E220 lora(RXD, TXD);

uint64_t delays[3] = {0, 0, 0};

/////////////////////////////////////////////////// test slots
Slot slots[1] = {
 Slot(ZONE, 1, 44.11, 10.11, distanceSensor1_1, distanceSensor1_2),
 //Slot(ZONE, 2, 44.11, 10.11, s2_1, s2_2),
 //Slot(ZONE, 3, 44.11, 10.11, s3_1, s3_2)
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

    /*
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    */

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
          
          int state1 = s.s1_.sensor.ping_cm();
          Serial.println(state1);
          int state2 = s.s2_.sensor.ping_cm();
          Serial.println(state2);

          if(((state1 <= ACTIVATION_TRESH) && (state1 != 0)) && ((state2 <= ACTIVATION_TRESH) && (state2 != 0))){
            state1 = HIGH;
            state2 = HIGH;
          }

          if (state1 == HIGH) {
            s.s1_.state = on_;
            if (to_stabilize){
              stabilize = millis();
              to_stabilize = false;
            }
          }
          else{
            s.s1_.state = off_;
            stabilize = millis();
            to_stabilize = true;
            
          }
          if (state2 == HIGH) {
            s.s2_.state = on_;
            if (to_stabilize){
              stabilize = millis();
              to_stabilize = false;
            }
          }
          else{
            s.s2_.state = off_;
            stabilize = millis();
            to_stabilize = true;
          }
          

          // state change
          // default behaviour is staying in the current state
          s.futureState = s.currentState;
          if((s.currentState == free_) && s.s1_.state == on_ && s.s2_.state == on_ && ((millis() - stabilize) > 1500)){
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
            stabilize = millis();
            to_stabilize = true;

            packet msg(0xFF, s.id_, 0xFE);
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
        ///DEBUG VERSION (SOLO 1 SENSORE)
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
            stabilize = millis();
            to_stabilize = false;
          }
        }
        else{
          s.s1_.state = off_;
          stabilize = millis();
          to_stabilize = true;
          
        }

        s.futureState = s.currentState;
        if((s.currentState == free_) && s.s1_.state == on_ && ((millis() - stabilize) > 1500)){
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
          stabilize = millis();
          to_stabilize = true;
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
