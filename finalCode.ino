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
int state = STRAIGHT;
long pingTimer[SONAR_NUM];
double distance[SONAR_NUM];
int currentSonar = 0;

int tSampleRobot = 10;
int tSonar = 40;
Robot robot(tSampleRobot, &state);

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

int count = 0;

void resetAngle()
{
  robot.resetOmega();
}

double stopDistance = 30.0;
double stopDistanceOld;
double omegaOld = 0.0;
double distanceFront = 0.0;
int corner = 1;
bool isFull = false;
bool window = false;
bool ascend = true;

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
        count = 20; // so he will actuate the angle quickly
        ++corner;
        if (corner == 4 && !window)
        {
          if (ascend)
            stopDistance += 10.0;
          else
            stopDistance -= 10.0;

          if (stopDistance > 60.0)
            {
              ascend = false;
              robot.setFull();
            }
          if (stopDistance < 31.0)
          {
            ascend = true;
            robot.setFull();
          }
        }
        if (corner == 5) corner = 1;
        state = STRAIGHT;
        robot.stop();
        resetSonar();
        resetAngle();
      }
      break;
    case GO_BASE:
      stopDistanceOld = stopDistance;
      state = STRAIGHT;
      isFull = true;
      break;
    default:
      robot.stop();
      break;
  }
  if (isFull)
  {
    if (corner == 3)
    {
      stopDistance = 30.0;
      window = true;
    }
    if (corner == 1 && window)
    {
      robot.empty();
      stopDistance = stopDistanceOld;
      isFull = false;
      window = false;
    }
  }
}

bool sweep(double distanceFront, bool *isDone)
{
  //Serial.println("Is sweeping");
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


int countLimit = 30;
Buffer omegaBuffer(10);


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
  else actualDistance = distance[0];
  //else actualDistance = min(distance[0], distance[1]);
  if (distance[0] < 5.0)
  {
    robot.setOmega(0.05);
    count = 20;
  }
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
  {
    omega = atan(-deltaA/deltaX);

    // Add the computed value to the buffer
    omegaBuffer.pushFilter2(omega);
  }
  // When the buffer is full, actuate the angle of the robot
  // based on an average on all the measurements. 
  // Then reset the buffer
  if (count >= countLimit)
  {
    double actualAngle = omegaBuffer.average();
    //omegaBuffer.reset();
    robot.setOmega(omegaBuffer[0]);
    count = 0;
  }
  else
  {
    robot.newFollowWallLeft(-robot.getOmega(), desiredAngle, 100, 1);
    ++count;
  }
  /*else
    robot.newFollowWallLeft(-robot.getOmega(), 0.1, 100, 1);
    */
  /*
  Serial.print(' ');
  Serial.print(robot.getOmega());
  Serial.print(' ');
  Serial.print(omegaBuffer[0]);
  Serial.println();*/
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

