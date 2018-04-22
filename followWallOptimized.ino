#include <Encoder.h>
#include <Motor.h>
#include <Sonar.h>
#include <NewPing.h>
#include <Robot.h>

#define SONAR_NUM 2 
#define DISTANCE_MAX 100
NewPing sonar[SONAR_NUM] = {
  NewPing(48,49, DISTANCE_MAX), // right
  NewPing(46,47, DISTANCE_MAX) // front-right
};
long pingTimer[SONAR_NUM];
long distance[SONAR_NUM];
int currentSonar = 0;

int tSampleRobot = 10;
int tSonar = 100;
Robot robot(tSampleRobot);

void setup() {
  Serial.begin(115200);
  delay(5000);
  pingTimer[0] = millis() + 75;
  for (int i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + tSonar;
}

void loop() {
  robot.countRobot();
  for (int i = 0; i < SONAR_NUM; i++)
    {
      if (millis() >= pingTimer[i])
      {
        //Serial.println("ok");
        pingTimer[i] += tSonar*SONAR_NUM;
        if (i == 0 && currentSonar == SONAR_NUM - 1)
          {
            //robot.printSonar(distance);
            robot.newFollowWallRight(distance, 20, 100);
          }
        sonar[currentSonar].timer_stop();
        currentSonar = i;
        distance[currentSonar] = 0;
        int inch_cm = 57;
        sonar[currentSonar].ping_timer(echoCheck);
        
      }
    }
}


void echoCheck(){
  if (sonar[currentSonar].check_timer())
        distance[currentSonar] = sonar[currentSonar].ping_result / US_ROUNDTRIP_CM;
}

