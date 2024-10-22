enum SensorState{
    off_,
    on_,
};

enum ParkingState{
    free_,
    requested_,
    taken_,
};

struct Sensor{
    uint8_t pin_;
    SensorState state;

    Sensor(uint8_t pin) : pin_(pin), state(off_) {}
};


struct Slot{
  uint8_t zone_;
  uint8_t id_;
  Sensor s1_;
  Sensor s2_;
  ParkingState currentState;
  ParkingState futureState;

  Slot(uint8_t zone, uint8_t id, uint8_t pin1, uint8_t pin2) : 
    zone_(zone), id_(id), s1_(pin1), s2_(pin2), currentState(free_), futureState(free_) {}
};

uint64_t startMillis;

bool timeout(uint64_t duration){
  return ((millis() - startMillis) > duration);
}

// test slots
Slot slots[3] = {Slot(1, 0, 2, 3), Slot(1, 1, 4, 5), Slot(1, 2, 6, 7)};

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
  else if(out == 'b'){
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
  }
  else if(out == 'r'){
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, LOW);
  }
}

void setup() {
    Serial.begin(9600);
}

void loop() {

    for(size_t i = 0; i < sizeof(slots) / sizeof(Slot); ++i){
        Slot s = slots[i];
        
        // reading input
        s.s1_.state = digitalRead(s.s1_.pin_);
        s.s2_.state = digitalRead(s.s2_.pin_);

        // state change
        // default behaviour is staying in the current state
        s.futureState = s.currentState;
        if((s.currentState == free_) && (s.s1_.state == on_ || s.s2_.state == on_) ){
            s.futureState = requested_;
        }
        else if(s.currentState == requested_ && s.s1_.state == off_ && s.s2_.state == off_){
            s.futureState = free_;
        }
        else if(s.currentState == requested_ && s.s1_.state == on_ && s.s2_.state == on_ && timeout(3000)){
            s.futureState = taken_;
        }
        else if(s.currentState == taken_ && s.s1_.state == off_ && s.s2_.state == off_){
            s.futureState = free_;
        }

        /*
        [i need to tell the bridge that i changed my state]
        */

        // on-exit

        // on-entry
        if(s.currentState != s.futureState){
            if(s.currentState == free_ && s.currentState == requested_){
                Serial.println("Il parcheggio è richiesto");
                startMillis = millis();
            }
            else if(s.currentState == requested_ && s.futureState == taken_){
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
        if(s.currentState == requested_){
          output('b');
        }
        if(s.currentState == taken_){
          output('r');
        }

        slots[i] = s;
    }

}
