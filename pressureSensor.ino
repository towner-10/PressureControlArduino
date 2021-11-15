#include "NoDelay.h"

#define PUMP_DELAY 500

unsigned long controlLoopStartTime;
unsigned long controlLoopInterval = 180000;

unsigned long pumpDelayStartTime = 0;

unsigned long startTime;
unsigned long interval;

const byte pressurePin = A0;
const byte pumpPin = A1;

void outputPSI();
void controlLoop();

bool pumpRunning = false;

noDelay printPSI(500, outputPSI);
noDelay updateControlLoop(100, controlLoop, false);

void outputPSI()
{
  Serial.println(getPSI());
}

void startPump(long time)
{
  if (!pumpRunning && millis() - pumpDelayStartTime >= PUMP_DELAY)
  {
    interval = time;
    startTime = millis();
    pumpRunning = true;
    digitalWrite(pumpPin, LOW);
  }
}

float getPSI()
{
  int sensorVal = analogRead(pressurePin);
  float voltage = (sensorVal * 5.0) / 1024.0;
  float psi = (30 * ((voltage - 0.45) / (4.5 - 0.45)));

  return psi;
}

void controlLoop()
{
  if (millis() - controlLoopStartTime <= controlLoopInterval)
  {
    // Control Loop Code Here
    float targetPSI = (1.0 / 12.0) * ((float)(millis() - controlLoopStartTime) / 1000.0);
    float currentPSI = getPSI();
    float error = targetPSI - currentPSI;

    if (error > 0.4)
    {
      // activate pump for 1 second
      startPump(1000);
    }
  }
  else
  {
    updateControlLoop.stop();
    digitalWrite(pumpPin, HIGH);
    pumpRunning = false;
  }
}

void setup()
{
  pinMode(pumpPin, OUTPUT);
  Serial.begin(9600);
  digitalWrite(pumpPin, HIGH);
}

void loop()
{
  printPSI.update();
  updateControlLoop.update();

  if (pumpRunning)
  {
    if (millis() - startTime >= interval)
    {
      digitalWrite(pumpPin, HIGH);
      pumpRunning = false;
      pumpDelayStartTime = millis();
    }
  }
  if (Serial.available() > 0)
  {
    String msg = Serial.readString();
    msg.replace("\n", "");
    msg.toUpperCase();
    if (msg != "START" && !pumpRunning)
    {
      float val = msg.toFloat();
      if (val > 0.0 && val < 5.0)
      {
        pumpRunning = true;
        startTime = millis();
        interval = (val * 1000);
        digitalWrite(pumpPin, LOW);
      }
    }
    else
    {
      startPump(1000);
      controlLoopStartTime = millis();
      updateControlLoop.start();
    }
  }
}
