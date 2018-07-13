#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>

#define ON 0
#define OFF 1
#define loraIRSwitchMSG           "SW"
#define loraIRLatchMSG            "LA"
#define loraRetainSWMSG           "RRPS"
#define loraRetainLAMSG           "RRPL"
#define loraRetainRqMSG           "RRQ"
#define loraSWSendMSG             "SW%d%d"


const int sw[7] = {0, 18, 19, 20, 21, 3, 2};
const int device[7] = {0, 44, 42, 40, 38, 36, 34};
const int LED_RL[7] = {0, 32, 30, 28, 13, 12, 11};

char message[10];
boolean deviceState[7] = {0, OFF, OFF, OFF, OFF, OFF, OFF};
boolean isSendLora = false;
char buf[10];

#define CONHUB_ADDRESS 3
#define GATEWAY_ADDRESS 1

RH_RF95 driver;
RHReliableDatagram manager(driver, CONHUB_ADDRESS);

void setup()
{
  Serial.begin(9600);

  if (!manager.init())
    Serial.println("init failed");

  for (int i = 1; i <= 6; i++)
  {
    pinMode(sw[i], INPUT);
    pinMode(device[i], OUTPUT);
    digitalWrite(device[i], OFF);
    pinMode(LED_RL[i], OUTPUT);
    digitalWrite(LED_RL[i], OFF);
  }

  attachInterrupt(digitalPinToInterrupt(sw[1]), intr, RISING); //2
  attachInterrupt(digitalPinToInterrupt(sw[2]), intr, RISING); //3
  attachInterrupt(digitalPinToInterrupt(sw[3]), intr, RISING); //18
  attachInterrupt(digitalPinToInterrupt(sw[4]), intr, RISING); //19
  //  attachInterrupt(digitalPinToInterrupt(sw[5]), intr, RISING); //20
  //  attachInterrupt(digitalPinToInterrupt(sw6), intr_6, HIGH); //21

  delay(100);
  sendLora(loraRetainRqMSG);

}


void loop()
{
  if (isSendLora) sendLora(message);
  if (manager.available()) recvLora();
}

void intr()
{
  for (int i = 1; i <= 4; i++)
  {
    if (digitalRead(sw[i]))
    {
      deviceState[i] = !deviceState[i];
      digitalWrite(device[i], deviceState[i]);
      digitalWrite(LED_RL[i], deviceState[i]);
      sprintf(message, loraSWSendMSG, i, !deviceState[i]);
      isSendLora = true;
    }

  }
}

void checkLoraMSG(char* data)
{
  int pin = (int)data[2] - 48;
  if (strstr(data, loraIRSwitchMSG))    //control sw
  {
    deviceState[pin] = !((int)data[3] - 48);
    digitalWrite(device[pin], deviceState[pin]);
    digitalWrite(LED_RL[pin], deviceState[pin]);
  }
  if (strstr(data, loraIRLatchMSG))
  {
    int state = (int)data[3] - 48;
    Serial.println(state);
    if (state)
    {
      attachInterrupt(digitalPinToInterrupt(sw[pin]), intr, RISING);
    }
    else
    {
      detachInterrupt(digitalPinToInterrupt(sw[pin]));
      digitalWrite(device[pin], OFF);
      digitalWrite(LED_RL[pin], OFF);
    }
  }

  if (strstr(data, loraRetainSWMSG))
  {
    for (int i = 4; i <= 7; i++)
    {
      deviceState[i - 3] = !((int)data[i] - 48);
      digitalWrite(device[i - 3], deviceState[i - 3]);
      digitalWrite(LED_RL[i - 3], deviceState[i - 3]);
    }
  }

  if (strstr(data, loraRetainLAMSG))
  {
    for (int i = 4; i <= 7; i++)
    {
      int state = (int)data[i] - 48;
      if (state == 1)
      {
        attachInterrupt(digitalPinToInterrupt(sw[i - 3]), intr, RISING);
      }
      else
      {
        detachInterrupt(digitalPinToInterrupt(sw[i - 3]));
        deviceState[i - 3] = 1;
        digitalWrite(device[i - 3], OFF);
        digitalWrite(LED_RL[i - 3], OFF);
      }
    }
  }
}


void sendLora(uint8_t* data)
{
  Serial.print(manager.sendtoWait(data, 12, GATEWAY_ADDRESS));
  isSendLora = false;
  Serial.println((char*)data);
}

void recvLora()
{
  uint8_t len = sizeof(buf);
  uint8_t from;
  if (manager.recvfromAck(buf, &len, &from))
  {
    Serial.println((char*)buf);
    checkLoraMSG((char*)buf);
  }
}




