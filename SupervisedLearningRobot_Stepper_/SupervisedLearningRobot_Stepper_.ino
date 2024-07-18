#include<SoftwareSerial.h>
#include<QList.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float euler[3];
float ypr[3];
volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
SoftwareSerial BT(A0, A1);
int xs = 4, xd = 7, ys = 3, yd = 6, k = -1, d = 0;
float an, nn;
unsigned long int c = 0;
struct Node {
  unsigned long int steps;
  String cmd;
};
struct Paths
{
  QList<Node> a;
};
QList<Paths> P;
void Insert(unsigned long int steps, String cmd) {
  Node newNode;
  newNode.steps = steps;
  newNode.cmd = cmd;
  P[k].a.push_back(newNode);
}
void Initial() {
  for (int i = P[k].a.size() - 1; i >= 0; i--) {
    int st = P[k].a[i].steps;
    an = Angle();
    if (P[k].a[i].cmd == "Front")
    {
      while (st != 0) {
        Run(LOW, HIGH, 7000);
        d++;
        if (d == 20)
        {
          Check();
          d = 0;
          Check();
        }
        st--;
      }
      Check();
      d = 0;
    }
    else if (P[k].a[i].cmd == "Back")
    {
      while (st != 0) {
        Run(HIGH, LOW, 7000);
        d++;
        if (d == 20)
        {
          Check();
          d = 0;
          Check();
        }
        st--;
      }
      Check();
      d = 0;
    }
    else if (P[k].a[i].cmd == "Right")
    {
      int m = an - st;
      if (m < 0)
        m += 360;
      while (int(Angle()) != m) {
        Run(LOW, LOW, 9000);
      }
    }
    else if (P[k].a[i].cmd == "Left") {
      int m = an + st;
      if (m >= 360)
        m -= 360;
      while (int(Angle()) != m) {
        Run(HIGH, HIGH, 9000);
      }
    }
  }
}
void Execute() {
  for (int i = 0; i < P[k].a.size(); i++) {
    int st = P[k].a[i].steps;
    an = Angle();
    if (P[k].a[i].cmd == "Front")
    {
      while (st != 0) {
        Run(HIGH, LOW, 7000);
        d++;
        if (d == 20)
        {
          Check();
          d = 0;
          Check();
        }
        st--;
      }
      Check();
      d = 0;
    }
    else if (P[k].a[i].cmd == "Back")
    {
      while (st != 0) {
        Run(LOW, HIGH, 7000);
        d++;
        if (d == 20)
        {
          Check();
          d = 0;
          Check();
        }
        st--;
      }
      Check();
      d = 0;
    }
    else if (P[k].a[i].cmd == "Left")
    {
      int m = an - st;
      if (m < 0)
        m += 360;
      while (int(Angle()) != m) {
        Run(LOW, LOW, 9000);
      }
    }
    else if (P[k].a[i].cmd == "Right") {
      int m = an + st;
      if (m >= 360)
        m -= 360;
      while (int(Angle()) != m) {
        Run(HIGH, HIGH, 9000);
      }
    }
  }
  delay(5000);
  Initial();
}
void setup()
{
  pinMode(xs, OUTPUT);
  pinMode(xd, OUTPUT);
  pinMode(ys, OUTPUT);
  pinMode(yd, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(A1, OUTPUT);
  BT.begin(9600);
  Serial.begin(9600);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(500000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1858);
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  an = Angle(), nn = Angle();
}
float Angle()
{
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
#ifdef OUTPUT_READABLE_YAWPITCHROLL
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  if (ypr[0] < 0)
    ypr[0] += 2 * PI;
  if (ypr[0] > 2 * PI)
    ypr[0] -= 2 * PI;;
  return (ypr[0] * 180 / M_PI);
#endif
}
void Check()
{
  nn = Angle();
  int m;
  if (nn > an)
  {
    m = nn - an;
    if (m > (360 - nn) + an)
    {
      while (int(Angle()) != int(an))
        Run(HIGH, HIGH, 9000);
    }
    else
    {
      while (int(Angle()) != int(an))
        Run(LOW, LOW, 9000);
    }
  }
  else if (nn < an)
  {
    m = an - nn;
    if (m < (360 - an) + nn)
    {
      while (int(Angle()) != int(an))
        Run(HIGH, HIGH, 9000);
    }
    else
    {
      while (int(Angle()) != int(an))
        Run(LOW, LOW, 9000);
    }
  }
}
void Run(bool x, bool y, int d)
{
  digitalWrite(xd, x);
  digitalWrite(yd, y);
  digitalWrite(xs, HIGH);
  digitalWrite(ys, HIGH);
  delayMicroseconds(d);
  digitalWrite(ys, LOW);
  digitalWrite(xs, LOW);
  delayMicroseconds(d);
}
bool isInteger(const String &s) {
  for (char c : s)
    if (!isdigit(c)) return false;
  return true;
}
void loop()
{
  while (BT.available())
  {
    String t = BT.readString();
    if (t == "T_ON" && k != -1)
    {
      P[k].a.clear();
      while (t != "T_OFF" && t != "STOP")
      {
        an = Angle();
        if (BT.available())
          t = BT.readString();
        if (t == "Front")
        {
          while (t == "Front")
          {
            Run(HIGH, LOW, 7000);
            c++;
            d++;
            if (d > 19)
            {
              Check(); d = 0;
              Check();
            }
            if (BT.available())
              break;
          }
          Check(); d = 0;
          Insert(c, t);
        }
        else if (t == "Left")
        {
          while (t == "Left")
          {
            Run(LOW, LOW, 9000);
            if (BT.available())
              break;
          }
          nn = Angle();
          if (nn > an)
            c = 360 - nn + an;
          else
            c = an - nn;
          Insert(c, t);
        }
        else if (t == "Right")
        {
          while (t == "Right")
          {
            Run(HIGH, HIGH, 9000);
            if (BT.available())
              break;
          }
          nn = Angle();
          if (nn < an)
            c = 360 - an + nn;
          else
            c = nn - an;
          Insert(c, t);
        }
        else if (t == "Back")
        {
          while (t == "Back")
          {
            Run(LOW, HIGH, 7000);
            c++;
            d++;
            if (d > 19)
            {
              Check(); d = 0;
              Check();
            }
            if (BT.available())
              break;
          }
          Check(); d = 0;
          Insert(c, t);
        }
        c = 0;
      }
    }
    if (t == "T_OFF")
    {
      Initial();
      t = "";
      continue;
    }
    else if (isInteger(t))
    {
      k = t.toInt() - 1;
      continue;
    }
    else if (t == "Execute")
    {
      Execute();
      continue;
    }
    else if (t == "Add")
    {
      Paths z;
      P.push_back(z);
      k = -1;
      continue;
    }
    else if (t == "Delete")
    {
      P.pop_back();
      continue;
    }
    else if (t == "Clear All")
    {
      P.clear();
      continue;
    }
  }
}
