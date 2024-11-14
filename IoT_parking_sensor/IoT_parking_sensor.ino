#include <Arduino_JSON.h>
#include <LoRa.h>

/*
  New Ping è una libreria che permette una gestione ottimizzata dei
  sensori di posizione. Fra le altre cose permette di utilizzare 
  un sensore che richiede due pin di digitali (trig e echo) usando
  solo un pin
*/
#include <NewPing.h> 

#define MAX_DISTANCE 200 // distanza massima pingabile in cm 
#define ACTIVATION_TRESH 70 // treshold di attivazione del sensore

// pin per il primo slot
#define PIN_SLOT_1_1 12
#define PIN_SLOT_1_2 12

// pin per il secondo slot
#define PIN_SLOT_2_1 11
#define PIN_SLOT_2_2 10

// pin per il terzo slot

#define PIN_SLOT_3_1 9
#define PIN_SLOT_3_2 8

// pin E220-900T22D
#define RXD 2
#define TXD 1

#define DELTA_READING 100

const uint8_t ZONE = 1; // hard coded zone

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


NewPing s1_1(PIN_SLOT_1_1, PIN_SLOT_1_1, MAX_DISTANCE);
NewPing s1_2(PIN_SLOT_1_2, PIN_SLOT_1_2, MAX_DISTANCE);

NewPing s2_1(PIN_SLOT_2_1, PIN_SLOT_2_1, MAX_DISTANCE);
NewPing s2_2(PIN_SLOT_2_2, PIN_SLOT_2_2, MAX_DISTANCE);

NewPing s3_1(PIN_SLOT_3_1, PIN_SLOT_3_1, MAX_DISTANCE);
NewPing s3_2(PIN_SLOT_3_2, PIN_SLOT_3_2, MAX_DISTANCE);


Sensor distanceSensor1_1(s1_1);
Sensor distanceSensor1_2(s1_2);



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

// led rgb as actuator
uint8_t greenPin = 12;
uint8_t bluePin = 13;
uint8_t redPin = 11;

void output(uint8_t out){
  if(out == 'g'){
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, LOW);
  }
  else if(out == 'r'){
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
  }
}



void setup() {
    Serial.begin(115200);
    while(!Serial){}

    //LoRa.begin(868E6);
    

    pinMode(2, INPUT);
    pinMode(3, INPUT);

    Serial.println("Hello");
}

void loop() {
  
    for(size_t i = 0; i < sizeof(slots) / sizeof(Slot); ++i){
        Slot& s = slots[i];
        // reading input
        delay(50); // momentaneo
        int state1 = s.s1_.sensor.ping_cm();
        //Serial.println(state1);
        int state2 = s.s2_.sensor.ping_cm();

        if((state1 <= ACTIVATION_TRESH) && (state2 <= ACTIVATION_TRESH)){
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
          /*
          LoRa.beginPacket();
          LoRa.write(0xFF); // header
          LoRa.write(0x01); // size
          LoRa.write(byte(ZONE)); // zone
          LoRa.write(s.id_); // id
          LoRa.write(s.futureState); // state
          LoRa.write(0xFE); // footer
          LoRa.endPacket();
          */
          Serial.print("Il sensore ha cambiato stato passando da ");
          Serial.print(s.currentState);
          Serial.print(" a ");
          Serial.println(s.futureState);
          stabilize = millis();
          to_stabilize = true;
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

        // output
        /*
        if(s.currentState == free_){
          output('g');
        }
        if(s.currentState == taken_){
          output('r');
        }
        */

        //////////////////////////////////////////////////////////////////////////////
        ///DEBUG VERSION (SOLO 1 SENSORE)
        //////////////////////////////////////////////////////////////////////////////

        /*
        int state1 = s.s1_.sensor.ping_cm();
        Serial.println(state1);

        if((state1 <= ACTIVATION_TRESH)){
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
