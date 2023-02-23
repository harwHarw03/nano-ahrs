#include <Arduino.h>
#include "Scheduler/Scheduler.h"
#include "Scheduler/Task.h"
#include "AHRS/AHRS.h"
#include "AHRS/Barometer.h"

AHRS *ahrs = AHRS::getInstance();

class IMUPDATE : public TimingTask
{
  public :
  IMUPDATE(uint32_t _rate):rate(_rate){updateTime(millis());}
  virtual void run(uint32_t now){
    ahrs->update();
    tick(rate);
  }
  private:
  uint32_t rate;
};

class Blinker : public TimingTask
{
  public:
    Blinker(uint32_t _rate):rate(_rate){updateTime(millis());}
    bool led_on = true;
    virtual void run(uint32_t now){
      if (led_on){digitalWrite(13, LOW); led_on = false;}
      else{digitalWrite(13, HIGH); led_on = true;}
      Serial.print(ahrs->get_roll());
      Serial.print(", ");
      Serial.print(ahrs->get_pitch());
      Serial.print(", ");
      Serial.print(ahrs->get_yaw());
      Serial.print(", ");
      Serial.println(ahrs->get_baro_altitude());
      tick(rate);
    } 
  private:
    uint32_t rate;
};

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
    ahrs->init();
  Blinker blinking(50);
  IMUPDATE imu_update(25);
  Task *tasks[] = {&imu_update, &blinking};
  Scheduler scheduler(tasks, NUM_TASKS(tasks));
  while(1){scheduler.runTasks();}
}

void loop() {}