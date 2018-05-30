#include <Robot.h>
#include <Buffer.h>

#define SONAR_NUM 3
#define DISTANCE_MAX 200

#define STRAIGHT 1
#define TURN 2
#define SWEEP 3
#define STOPPED 4
#define GO_BASE 5

NewPing sonar[SONAR_NUM] = {
  NewPing(22, 22, DISTANCE_MAX), // left
  NewPing(23, 23, DISTANCE_MAX), // front-left
  NewPing(32, 33, DISTANCE_MAX)  // front
};
long pingTimer[SONAR_NUM];
double distance[SONAR_NUM];
int currentSonar = 0;

int tSampleRobot = 10;
int tSonar = 40;
Robot robot(tSampleRobot);

Buffer distanceBuffer(3);

void resetSonar()
{
  pingTimer[0] = millis() + 75;
  for (int i = 1; i < SONAR_NUM; i++)
    pingTimer[i] = pingTimer[i - 1] + tSonar;

  for (int count = 0; count < SONAR_NUM; ++count)
    distance[count] = 0;
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  resetSonar();

  robot.initialize();
  pinMode(2, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), interrupt, RISING);
}

// Forward declration of the function followWall
bool followWall(double limit = 0.0, double desiredAngle = 0.0);
void control(double desiredAngle = 0.0);


void resetAngle()
{
  robot.setOmega(0);
}

double stopDistance = 50.0;
double stopDistanceOld;
int state = STRAIGHT;
double omegaOld = 0.0;
double distanceFront = 0.0;
/////// MAIN LOOP ///////
void loop() {
  robot.countRobot();
  switch (state)
  {
    bool isReached;
    bool isTurned;
    bool isPuck;
    bool isDone;
    bool isOnBase;
    case STRAIGHT:
      isReached = followWall(stopDistance);
      if (isReached)
      {
        state = SWEEP;
        distanceFront = distance[2];
        omegaOld = robot.getOmega();
        robot.stop();
      }
      break;

    case SWEEP:
      isDone = false;
      isPuck = sweep(distanceFront, &isDone);
      if (isPuck)
        state = STRAIGHT;
      if (isDone)
        state = TURN;
      break;
  
    case TURN:
      isTurned = robot.turnRight90(0.0);
      if (isTurned)
      {
        state = STRAIGHT;
        robot.stop();
        resetSonar();
        resetAngle();
      }
      break;
    case GO_BASE:
      //isOnBase = robot.goToBase();
      if (isOnBase)
      {
        stopDistance = stopDistanceOld;
        state = STRAIGHT;
      }
      break;
    default:
      robot.stop();
      break;
  }
}

bool sweep(double distanceFront, bool *isDone)
{
  Serial.println("Is sweeping");
  actuateSonar();
  bool isReached;
  bool isPuck;
  isReached = robot.sweep();
  if (abs(distanceFront-distance[2])/distanceFront > 0.1
      && distance[2] > 1.0)
    isPuck = true;
  else
    isPuck = false;
  if (isReached) *isDone = true;
  return isPuck;
}

double dForwardOld = 0;
Buffer omegaBuffer(10);
bool puck = false;
double distanceFrontOld = 0.0;

bool followWall(double limit = 0.0, double desiredAngle = 0.0)
{
  /*  Actuate the value of the sonars
   *  When all the sonars are actuated, call 
   *  the control function
   *  If there is a wall in front of the robot
   *  return true
   */
  for (int i = 0; i < SONAR_NUM; i++)
  {
    if (millis() >= pingTimer[i])
    {
      //Serial.println("ok");
      pingTimer[i] += tSonar * SONAR_NUM;
      if (i == 0 && currentSonar == SONAR_NUM - 1)
      {
        control(desiredAngle);
        if (distance[2] < limit && distance[2] >= 1.0)
          return true;

      }
      sonar[currentSonar].timer_stop();
      currentSonar = i;
      distance[currentSonar] = 0;
      sonar[currentSonar].ping_timer(echoCheck);
    }
  }
  return false;
}

void control(double desiredAngle = 0.0)
{
  /* Compute the smallest distance from the wall, 
   * Call the function followWall of the robot class
   * Actuate the angle of the robot once the buffer 
   * of the angle value is full --> do an average
   */
  double actualDistance = 0;
  bool dismiss = false;
  if (distance[1] == 0)
  {
    if (distance[0] == 0) dismiss = true;
    else actualDistance = distance[0];
  }
  else if (distance[0] == 0)
  {
    actualDistance = distance[1];
  }
  else actualDistance = min(distance[0], distance[1]);

  // Add the computed distance to the buffer
  distanceBuffer.push(actualDistance);
  // Use a moving average in order to have clean data
  actualDistance = distanceBuffer.filter();
  //Serial.print(actualDistance);
  //Serial.print(' ');
  //robot.printSonar(distance); 
  double dForward = robot.getDForward();
  double deltaX = dForward - dForwardOld;
  dForwardOld = dForward;
  double deltaA = distanceBuffer[0]-distanceBuffer[1];
  double omega = robot.getOmega();

  // If the robot has moved, will compute
  // a value for its angle
  if (abs(deltaX) >= 0.001)
    omega = atan(-deltaA/deltaX);

  // Add the computed value to the buffer
  omegaBuffer.pushFilter(omega);

  // When the buffer is full, actuate the angle of the robot
  // based on an average on all the measurements. 
  // Then reset the buffer
  if (omegaBuffer.isFull())
  {
    double actualAngle = omegaBuffer.average();
    omegaBuffer.reset();
    robot.setOmega(actualAngle);
  }
  if ( !omegaBuffer.isFull() && distanceBuffer.isFull() )
    robot.newFollowWallLeft(-robot.getOmega(), desiredAngle, 100, 1);
  /*else
    robot.newFollowWallLeft(-robot.getOmega(), 0.1, 100, 1);
    */
  
  /*Serial.print(' ');
  Serial.print(robot.getOmega());
  Serial.print(' ');
  //Serial.print(omegaBuffer[0]);
  Serial.println();
  */
}


void echoCheck() {
  if (sonar[currentSonar].check_timer())
    distance[currentSonar] =
      static_cast<double>(sonar[currentSonar].ping_result) / 58.31;
}


bool actuateSonar()
{
  for (int i = 0; i < SONAR_NUM; i++)
  {
    if (millis() >= pingTimer[i])
    {
      //Serial.println("ok");
      pingTimer[i] += tSonar * SONAR_NUM;
      if (i == 0 && currentSonar == SONAR_NUM - 1)
      {
        return true;
      }
      sonar[currentSonar].timer_stop();
      currentSonar = i;
      distance[currentSonar] = 0;
      sonar[currentSonar].ping_timer(echoCheck);
    }
  }
  return false;
}  


void interrupt()
{
  CylinderRecuperation::s_isInterrupted = true;
  //Serial.println(digitalRead(2));
  robot.interrupted();
  //Serial.println("Trigger");
}

