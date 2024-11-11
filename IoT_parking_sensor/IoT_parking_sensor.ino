#include <Arduino_JSON.h>
#include <LoRa.h>

const uint8_t ZONE = 1; // hard coded zone

const uint64_t initRequestRate = 3000; // request rate for init phase
bool tryInit = true; // to know when to send again init packet

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
    uint8_t pin_;
    SensorState state;

    Sensor(uint8_t pin) : pin_(pin), state(off_) {}
};


struct Slot{
  uint8_t zone_; // parking zone
  uint8_t id_; // parking id
  double lat_; // parking latitude
  double lon_; // parking longitude
  Sensor s1_; // first sensor
  Sensor s2_; // second sensor
  ParkingState currentState;
  ParkingState futureState;

  /**
  * @brief Constructor for a slot.
  *
  * @param zone The zone that the slot belongs to.
  * @param id The unique id wrt zone.
  * @param lat Latitude of parking slot.
  * @param lon Longitude of parking slot.
  * @param pin1 The pin of the first sensor.
  * @param pin2 The pin of the second sensor.
  * 
  *
  */
  Slot(uint8_t zone, uint8_t id, double lat, double lon, uint8_t pin1, uint8_t pin2) : 
    zone_(zone), id_(id), lat_(lat), lon_(lon), s1_(pin1), s2_(pin2), currentState(free_), futureState(free_) {}

  // empty constructor
  Slot() : zone_(0), id_(0), lat_(0), lon_(0), s1_(0), s2_(0) {}
};


bool timeout(uint64_t duration){
  return ((millis() - startMillis) > duration);
}

// test slots
Slot slots[1] = {Slot(ZONE, 0, 44.0, 44.0, 2, 3)/*, Slot(1, 1, 44.0, 44.0, 4, 5), Slot(1, 2, 44.0, 44.0, 6, 7)*/};

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

bool recivedInit(){
  int packetSize = LoRa.parsePacket();
  if(packetSize){
    char buffer[255];
    uint8_t pointer = 0;
    while(LoRa.available()){
      buffer[pointer] = (char)LoRa.read();
      ++pointer;
    }
    if(buffer[0] == 0xFF){
      if(buffer[1] == "i"){
        // TODO: gestire il pacchetto
        return true;
      }
    }
  }
  return false;
}


void requestInput(uint8_t zone){
  uint64_t curMillis = millis();
  while(!recivedInit())
  {
    if(tryInit){
      LoRa.beginPacket();
      LoRa.write(0xFF); // header
      LoRa.write(0x01); // size
      LoRa.write(byte(zone)); // zone
      LoRa.write(0xFE); // footer
      LoRa.endPacket();
    }
    if((millis() - curMillis) > initRequestRate){
      tryInit = true;
      curMillis = millis();
    }
    else{
      tryInit = false;
    }
  }
}

void setup() {
    Serial.begin(9600);
    while(!Serial){}

    LoRa.begin(868E6);
    

    pinMode(2, INPUT);
    pinMode(3, INPUT);

    Serial.println("Hello");

    requestInput(ZONE);
    // i should retrive from the bridge the slots

}

void loop() {
  
    for(size_t i = 0; i < sizeof(slots) / sizeof(Slot); ++i){
        Slot& s = slots[i];
        // reading input
        int state1 = digitalRead(s.s1_.pin_);
        int state2 = digitalRead(s.s2_.pin_);

        // JUST FOR NOW
        state1 = LOW;
        state2 = LOW;

        if (state1 == HIGH) {
          s.s1_.state = on_;
        }
        else{
          s.s1_.state = off_;
        }

        if (state2 == HIGH) {
          s.s2_.state = on_;
        }
        else{
          s.s2_.state = off_;
        }

        // state change
        // default behaviour is staying in the current state
        s.futureState = s.currentState;
        if((s.currentState == free_) && s.s1_.state == on_ && s.s2_.state == on_ ){
            s.futureState = taken_;
        }
        else if(s.currentState == taken_ && s.s1_.state == off_ && s.s2_.state == off_){
            s.futureState = free_;
        }

        /*
        [i need to tell the bridge that i changed my state]
        */

        // on-exit
        if(s.currentState != s.futureState){
          LoRa.beginPacket();
          LoRa.write(0xFF); // header
          LoRa.write(0x01); // size
          LoRa.write(byte(zone)); // zone
          LoRa.write(s.id_); // id
          LoRa.write(s.futureState); // state
          LoRa.write(0xFE); // footer
          LoRa.endPacket();
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
        if(s.currentState == free_){
          output('g');
        }
        if(s.currentState == taken_){
          output('r');
        }
    }

}
