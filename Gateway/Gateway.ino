#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Arduino.h>
#include <IRremote.h>
#include <Wire.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#define mqtt_server "tapit.vn"
#define mqtt_port 1883
#define NumOfDevice 6

#define GATEWAY_ADDRESS 1
#define SENHUB_ADDRESS 2
#define CONHUB_ADDRESS 3

// Singleton instance of the radio driver
RH_RF95 driver(8, 2);
//RH_RF95 driver(5, 2); // Rocket Scream Mini Ultra Pro with the RFM95W

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, GATEWAY_ADDRESS);

//Led pin
#define ledPin 5

//For Senhub
#define topicSensorTemp "Room1/sensor/temp"
#define topicSensorLight "Room1/sensor/light"
#define topicSensorPir "Room1/sensor/pir"

//For IR
#define topicIRLearning       "Room1/IR/learning"
#define topicIRLearningACK    "Room1/IR/learning/ACK"
#define topicIRButtonList     "Room1/IR/buttonlist"
#define topicIRButtonListACK  "Room1/IR/buttonlist/ACK"
#define topicControlAC        "Room1/AC/button"
#define topicControlPR        "Room1/PR/button"

//For Lora message
#define loraIRSwitchMSG           "SW"
#define loraIRLatchMSG            "LA"
#define loraRetainRQ              "RRQ"
#define loraRetainRPSW            "RRPS"
#define loraRetainRPLA            "RRPL"
#define loraIRLearningMSG         "LN"
#define loraIRLearningMSGACK      "LNA"
#define loraIRButtonListMSG       "BL"
#define loraIRButtonListMSGACK    "BLA"
#define loraSensorTemp            "TE"
#define loraSensorLight           "LI"
#define loraSensorPir             "PI"


//Lora Retain Message
char loraSwitchRetain[] = "00000";
char loraLatchRetain[]  = "00000";


EthernetClient ethClient;
PubSubClient client(ethClient);
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 100);
const String topicControlSwitch  = "Room1/switch/device";
char topicControlSwitchChar[]     = "Room1/switch/device0";
const String topicLatchDevice  = "Room1/latch/device";
char topicLatchDeviceChar[]  = "Room1/latch/device0";
boolean isRemote = 1; //value control receive message
char loraBuffer[10];
String msgbuff = "";
boolean isWiFi = false;

void setup()
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  else Serial.println("init success");
  delay(1000);
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
//    Ethernet.begin(mac, ip);    
    digitalWrite(ledPin, HIGH);
  }
  else digitalWrite(ledPin, LOW);
  // print your local IP address:
  Serial.print("My IP address: ");
  ip = Ethernet.localIP();
}
void loop()
{  
  ethernetMode();
  if (!client.connected()) {
    digitalWrite(ledPin, HIGH);
    reconnect();
    delay(1000);
  }
  client.loop();
  delay(100);

  loraMode();
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(loraBuffer);
    uint8_t from;
    if (manager.recvfromAck(loraBuffer, &len, &from))
    {
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)loraBuffer);
      checkLoraMSG(loraBuffer);
    }
  }
  delay(100);
}

void callback(char* topic, byte* payload, unsigned int length) {
  loraMode();
  if (isRemote)
  {
    String topicState = topic;
    Serial.println(String("Topic: ") + topic);
    for (int i = 0; i < length; i++) {
      msgbuff += (char)payload[i];
    }
    Serial.println(String("Msg: ") + msgbuff);
    if (topicState.indexOf(topicControlSwitch) != -1) {
      int deviceNum = checkSwitch(topicState);
      loraSwitchRetain[deviceNum - 1] = msgbuff.toInt() + '0';
      msgbuff = loraIRSwitchMSG + String(deviceNum) + msgbuff;   //topicState.charAt(4) là stt phòng nếu cần
      char data[10];
      msgbuff.toCharArray(data, msgbuff.length() + 1);
      Serial.println(data);
      manager.sendtoWait(data, sizeof(data), CONHUB_ADDRESS);
    }
    if (topicState.indexOf(topicLatchDevice) != -1) {
      int deviceNum = checkSwitch(topicState);
      loraLatchRetain[deviceNum - 1] = msgbuff.toInt() + '0';
      msgbuff = loraIRLatchMSG + String(deviceNum) + msgbuff;  //topicState.charAt(4) là stt phòng nếu cần
      char data[10];
      msgbuff.toCharArray(data, msgbuff.length() + 1);
      Serial.println(data);
      manager.sendtoWait(data, sizeof(data), CONHUB_ADDRESS);
    }
    if (topicState.indexOf(topicIRLearning) != -1) {
      msgbuff = loraIRLearningMSG;
      char data[10];
      msgbuff.toCharArray(data, msgbuff.length() + 1);
      Serial.println(data);
      manager.sendtoWait(data, sizeof(data), SENHUB_ADDRESS);
    }
    if (topicState.indexOf(topicIRButtonList) != -1 || topicState.indexOf(topicControlAC) != -1 || topicState.indexOf(topicControlPR) != -1) {
      msgbuff = loraIRButtonListMSG + msgbuff;
      char data[10];
      msgbuff.toCharArray(data, msgbuff.length() + 1);
      Serial.println(data);
      manager.sendtoWait(data, sizeof(data), SENHUB_ADDRESS);
    }
    msgbuff = "";
    Serial.println();
  }
  else {
    isRemote = 1;
    msgbuff = "";
  }
}

unsigned int checkSwitch(String topicState) {
  int temp = topicState.length() - 1;
  return (int)topicState.charAt(temp) - 48; //change char to uint
}

void reconnect() {
  int i;
  char c;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("gatewayClient1", "vinhkhai", "vinhkhai")) {
      Serial.println("connected");
      // ... and resubscribe
      for (i = 1; i <= NumOfDevice - 2; i++) {
        c = (char)i + '0';    //change int to char
        topicControlSwitchChar[19] = c;
        client.subscribe(topicControlSwitchChar);
        client.loop();  //!important (avoid timeout)
      }
      client.loop();
      for (i = 1; i <= NumOfDevice - 2; i++) {
        c = (char)i + '0';    //change int to char
        topicLatchDeviceChar[18] = c;
        client.subscribe(topicLatchDeviceChar);
        client.loop();  //!important (avoid timeout)
      }
      client.subscribe(topicIRLearning);
      client.subscribe(topicIRButtonList);
      client.loop();
      client.subscribe(topicControlAC);
      client.subscribe(topicControlPR);
      client.loop();
      Serial.println("Subscribe Done");
      digitalWrite(ledPin, LOW);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void checkLoraMSG(char* data) {
  ethernetMode();
  if (strcmp(data, loraRetainRQ)==0) {
    loraMode();
    char _dataSW[15];
    strcpy(_dataSW, loraRetainRPSW);
    strcat(_dataSW, loraSwitchRetain);
    manager.sendtoWait(_dataSW, sizeof(_dataSW), CONHUB_ADDRESS);
    Serial.println(_dataSW);
    Serial.println("sent SW");
    delay(1000);
    char _dataLA[15];
    strcpy(_dataLA, loraRetainRPLA);
    strcat(_dataLA, loraLatchRetain);
    manager.sendtoWait(_dataLA, sizeof(_dataLA), CONHUB_ADDRESS);
    Serial.println(_dataLA);
    Serial.println("sent LA");
  }

  if (strstr(data, loraIRSwitchMSG)) {
    isRemote = 0;
    char _data[2];
    Serial.println("here");
    isRemote = 0;
    char deviceNum = (char)data[2];
    _data[0] = data[3];
    _data[1] = '\0';
    topicControlSwitchChar[19] = deviceNum;
    client.publish(topicControlSwitchChar, _data, true);
  }

  if (strstr(data, loraSensorTemp)) {
    char _data[6];
    int i;
    for (i = 0; i < sizeof(_data); i++) {
      _data[i] = data[i + 2];
    }
    _data[6] = '\0';
    client.publish(topicSensorTemp, _data);
  }


  if (strstr(data, loraSensorLight)) {
    char _data[2];
    _data[0] = loraBuffer[2];
    _data[1] = '\0';
    client.publish(topicSensorLight, _data);
  }

  if (strstr(data, loraSensorPir)) {
    char _data[2];
    _data[0] = loraBuffer[2];
    _data[1] = '\0';
    client.publish(topicSensorPir, _data);
  }

  if (strcmp(data, loraIRButtonListMSGACK) == 0) {
    client.publish(topicIRButtonListACK, "1");
  }
  if (strcmp(data, loraIRLearningMSGACK) == 0) {
    client.publish(topicIRLearningACK, "1");
  }
  resetLoRaBuffer();
}


//reset buf để tự động có được ký tự kết thúc khi xét chuỗi trong checkLoraMSG
void resetLoRaBuffer() {
  int i;
  for (i = 0; i < sizeof(loraBuffer); i++) {
    loraBuffer[i] = '\0';
  }
}
void ethernetMode() {
  digitalWrite(10, LOW);
  digitalWrite(8, HIGH);
}
void loraMode() {
  digitalWrite(10, HIGH);
  digitalWrite(8, LOW);
}

