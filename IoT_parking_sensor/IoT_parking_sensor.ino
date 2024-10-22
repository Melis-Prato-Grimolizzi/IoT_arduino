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

void setup() {
    Serial.begin(9600);
}

void loop() {

    for(size_t i = 0; i < sizeof(slots) / sizeof(Slot); ++i){
        Slot s = slots[i];

        // leggo gli input
        s.s1_.state = digitalRead(s.s1_.pin_);
        s.s2_.state = digitalRead(s.s2_.pin_);

        // transizione di stato
        // il comportamento di default è rimanere sullo stato corrente
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
        NB: in qualche modo devo comunicare il passaggio di stato al bridge
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
        slots[i] = s;
        // output
    }

}
