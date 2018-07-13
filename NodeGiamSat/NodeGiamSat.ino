//getNewCode
//learningFlag
//websiteFlag

#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>
#include <Eeprom24Cxx.h>
#include <IRremote.h>

#define SENHUB_ADDRESS  2
#define GATEWAY_ADDRESS 1

RH_RF95 driver;
RHReliableDatagram manager(driver, SENHUB_ADDRESS);

//Pin define
#define lightPin 13
#define PIRPin 12

//For IR Adapter
#define maxLen 800
#define RECV_PIN 3             //D5
#define SEND_PIN 5             //D6
#define RED_PIN 22
#define GREEN_PIN 24
#define BLUE_PIN 26
#define BAUD_RATE 115200
#define CAPTURE_BUFFER_SIZE 1024
#define first_address 5         //Địa chỉ đầu tiên để lưu giá trị mảng
#define last_address 32767      //Địa chỉ cuối cùng để lưu giá trị con trỏ
#define next_array_address 0    //Ô số 0 chứa địa chỉ trống tiếp theo để lưu mảng mới
#define next_pointer_address 2  //Ô số 2 chứa địa chỉ con trỏ trống tiếp theo
#define number_of_array 4       //Ô số 4 chứa tổng số lượng mã đã lưu trong EEPROM
#if DECODE_AC
#define TIMEOUT 50U
#else
#define TIMEOUT 15U
#endif
#define MIN_UNKNOWN_SIZE 12
#define number_of_button 16
#define arduino_eeprom_size 512

//AC Button list
#define AC_ON  1
#define AC_OFF 2
#define AC_18  3
#define AC_20  4
#define AC_22  5
#define AC_24  6
#define AC_26  7
#define AC_28  8

//PR Button list
#define PR_POWER  9
#define PR_UP     10
#define PR_DOWN   11
#define PR_LEFT   12
#define PR_RIGHT  13
#define PR_OK     14
#define PR_MENU   15
#define PR_EXIT   16

//Lora Message
#define loraIRLearningMSG         "LN"
#define loraIRLearningMSGACK      "LNA"
#define loraIRButtonListMSG       "BL"
#define loraIRButtonListMSGACK    "BLA"

static Eeprom24C eeprom(256, 0x50);
boolean getNewCode = false;
boolean learningFlag = false;
boolean deleteEP = false;
boolean isTrueCode = true;
String inString;
char inChar;
unsigned int availableAddress;
unsigned int availablePointer;
boolean websiteFlag = false;
volatile unsigned int CorrectedRawLength = 0;
unsigned int irBuffer[maxLen];
char loraBuffer[10];
unsigned long sensorReadTime = millis();
char loraSensorData[10];
// Display the human readable state of an A/C message if we can.
IRsend irsend;

//Chế độ điều khiển
void Normal_Led() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);
}

//Chế độ đợi nhận mã hồng ngoại
void WaitCode_Led() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
}

//Chế độ đợi đồng bộ từ điện thoại và lưu EEPROM
void WaitSync_Led() {
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, LOW);
}

//Full bộ nhớ EEPROM
void Overflow_Led() {
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
}

void Error_Led() {
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  for (int i = 0; i < 10; i++) {
    digitalWrite(RED_PIN, LOW);
    delay(100);
    digitalWrite(RED_PIN, HIGH);
    delay(100);
  }
}

void setup()
{
  pinMode(lightPin, INPUT);
  pinMode(PIRPin, INPUT_PULLUP);
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  Normal_Led();
  Serial.begin(9600);
  while (!Serial) ; // Wait for serial port to be available
  if (!manager.init())
    Serial.println("init failed");
  deleteEEPROM();
  Serial.println("delete done");
}

void loop()
{  
  loraReceive();
  if (getNewCode) {
    WaitCode_Led();
    unsigned int* rawdata = learnFunction();
    if (CorrectedRawLength && isTrueCode) {
      WaitSync_Led();
      Serial.println(String("CorrectedRawLength=") + CorrectedRawLength);
      websiteFlag = true;
      if (!manager.init())
        Serial.println("re-init failed");
      else Serial.println("re-init success");
      manager.sendtoWait(loraIRLearningMSGACK, sizeof(loraIRLearningMSGACK), GATEWAY_ADDRESS);
      unsigned long WebsiteTime = millis();
      delay(100);
      Serial.println(String("time: ") + (millis() - WebsiteTime));
      while (millis() - WebsiteTime < 15000) {
        loraReceive();
        if (learningFlag) {
          Serial.println(String("learningFlag 2: ") + learningFlag);
          break;
        }
      }
      WebsiteTime = 0;
      if (learningFlag) {
        Serial.println("Saving to EEPROM");
        availableAddress = checkAvailableAddress(CorrectedRawLength);
        availablePointer = setPointer();
        Serial.println(availableAddress);
        Serial.println(availablePointer);

        if ((availableAddress != -1) && (availablePointer != -1 )) {
          Serial.println("Ready for saving");
          eeprom.write_IR_code_arduino(availableAddress, rawdata, CorrectedRawLength); //Write rawdata
          eeprom.write_2_byte(availablePointer, availableAddress);  //Write address of first element
          eeprom.write_2_byte(availablePointer + 2, CorrectedRawLength); //Write raw length
          setNextArrayAddress(availableAddress, CorrectedRawLength);
          setNextPointerAddress(availablePointer);
          setNumberOfArray();
          Normal_Led();
        }
        else {
          Serial.println("EEPROM overflow!!");
          Overflow_Led();
        }
        delay(10);
        Serial.println("Learn Successfull");
        manager.sendtoWait(loraIRButtonListMSGACK, sizeof(loraIRButtonListMSGACK), GATEWAY_ADDRESS);
      }
      else {
        Serial.println("Get Button Timeout!!!");
        Normal_Led();
      }
      learningFlag = false;
      CorrectedRawLength = 0;
      getNewCode = false;
    }
    else {
      Normal_Led();
      getNewCode = true;
    }
    if (!manager.init())
      Serial.println("re-init failed");
    else Serial.println("re-init success");
  }

  if (deleteEP) {
    deleteEEPROM();
    //      getNewCode = true;
    Serial.println("Deleted Done!!");
    Serial.println(String("Number of Array: ") + getNumberArray());
    deleteEP = false;
  }

  if ((sensorReadTime + 5000) < millis())
  {
    readTemp();
    delay(300);
    readPIR();
    delay(300);
    readLight();
    delay(300);
    sensorReadTime = millis();
  }
}

void loraReceive () {
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
}

void checkLoraMSG(char* data) {
  if (strcmp(data, loraIRLearningMSG) == 0) getNewCode = true;
  if (strstr(data, loraIRButtonListMSG)) {
    String buttonNum = "";
    int i = 2;
    while (data[i]) {
      buttonNum += data[i];
      i++;
    }
    Serial.println(buttonNum);
    if (websiteFlag == true) getButtonList(buttonNum.toInt());
    else controlIRDevice(buttonNum.toInt());
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

void readLight()
{
  boolean a = !digitalRead(lightPin);
  Serial.println(a);
  sprintf(loraSensorData, "LI%d", a);
  manager.sendtoWait(loraSensorData, sizeof(loraSensorData), GATEWAY_ADDRESS);
  Serial.println(loraSensorData);
}

void readTemp()
{
  float reading;
  reading = analogRead(A0);
  float temp = reading * 4.5 / 1023.0 * 100;
  char m_message[6];
  dtostrf(temp, 4, 2, m_message);
  strcpy(loraSensorData, "TE");
  strcat(loraSensorData, m_message);
  manager.sendtoWait(loraSensorData, sizeof(loraSensorData), GATEWAY_ADDRESS);
  Serial.println(loraSensorData);
}

void readPIR()
{
  boolean a = digitalRead(PIRPin);
  sprintf(loraSensorData, "PI%d", a);
  manager.sendtoWait(loraSensorData, sizeof(loraSensorData), GATEWAY_ADDRESS);
  Serial.println(loraSensorData);
}


void rxIR_Interrupt_Handler()
{
  if (CorrectedRawLength > maxLen) return 0; //ignore if irBuffer is already full
  irBuffer[CorrectedRawLength++] = micros(); //just continually record the time-stamp of signal transitions
}

unsigned int* learnFunction() {
  unsigned long learnTimeout = millis();
  Serial.println(String("before attach: ") + digitalRead(RECV_PIN));
  attachInterrupt(digitalPinToInterrupt(RECV_PIN), rxIR_Interrupt_Handler, CHANGE);
  Serial.println(String("attach: ") + digitalRead(RECV_PIN));
  Serial.println(String("CorrectedRawLength first = ") + CorrectedRawLength);
  while (getNewCode)
  {
    if (millis() - learnTimeout >= 11000) {
      Serial.println("IR Received timeout!!");
      detachInterrupt(digitalPinToInterrupt(RECV_PIN));
      Normal_Led();
      break;
    }
    if (CorrectedRawLength) {
      delay(1000);  //waiting for IR received finish
      Serial.println();
      if (CorrectedRawLength >= 30) {   //Right code?
        Serial.println();
        Serial.print(F("Raw:(")); //dump raw header format - for library
        Serial.print(CorrectedRawLength - 1);
        Serial.print(F(") "));
        detachInterrupt(digitalPinToInterrupt(RECV_PIN));//stop interrupts & capture until finshed here
        Serial.println(String("detach: ") + digitalRead(RECV_PIN));
        for (int i = 1; i < CorrectedRawLength; i++)
        {
          Serial.print(irBuffer[i] - irBuffer[i - 1]);
          Serial.print(F(", "));
        }
        CorrectedRawLength -= 1;
        isTrueCode = true;
      }
      else {
        Serial.println(String("CorrectedRawLength = ") + CorrectedRawLength);
        detachInterrupt(digitalPinToInterrupt(RECV_PIN));//stop interrupts & capture until finshed here
        CorrectedRawLength = 0;
        isTrueCode = false;
        Error_Led();
        Serial.println("ERROR Code");
      }
      getNewCode = false;
    }
  }
  return irBuffer;
}

void setNextArrayAddress(unsigned int availableAdrress, uint16_t raw_length) {
  unsigned int i = availableAddress + raw_length * 2;
  eeprom.write_2_byte(next_array_address, i);
}

void setNextPointerAddress(unsigned int availablePointer) {
  unsigned int i = availablePointer - 4;
  eeprom.write_2_byte(next_pointer_address, i);
}

unsigned int checkAvailableAddress(uint16_t raw_length) {
  unsigned int address = 0, address_available = 0;
  unsigned int i = eeprom.read_2_byte(next_array_address); //read recent array address
  while (i <= last_address) {
    if (eeprom.read_2_byte(i) == 0) {
      address_available++;
      if (address_available == 1)  address = i;
    }
    else  address_available = 0;
    if (address_available >= (raw_length + 4)) return address;
    i += 2;
  }
  return -1;
}

unsigned int setPointer() {
  unsigned int i = eeprom.read_2_byte(next_pointer_address);  //read recent pointer address
  Serial.println(String("next_poiner_address: ") + i);
  while (i > 0) {
    if (eeprom.read_4_byte(i) == 0) return i;
    i -= 4;
  }
  return -1;
}


void setNumberOfArray() {
  byte new_number = eeprom.read_1_byte(number_of_array) + 1;
  eeprom.write_1_byte(number_of_array, new_number);
}

void send_IR_code_By_Pointer(unsigned int pointer) {
  unsigned int address_pointer = 32768 - pointer * 4;
  unsigned int length_pointer = address_pointer + 2;
  uint16_t array_address = eeprom.read_2_byte(address_pointer);
  uint16_t raw_length = eeprom.read_2_byte(length_pointer);
  Serial.println(String("array address: ") + array_address);
  Serial.println(String("raw length: ") + raw_length);
  uint16_t raw_array[raw_length];
  eeprom.read_IR_code(array_address, raw_length, raw_array);
  irsend.sendRaw(raw_array, raw_length, 38);
  Serial.println("sent done");
}

void deleteEEPROM() {
  unsigned int next_array = eeprom.read_2_byte(next_array_address);
  unsigned int next_pointer = eeprom.read_2_byte(next_pointer_address);
  Serial.println(next_array);
  Serial.println(next_pointer);
  //first_address - 1 để xóa luôn ô số 4 chứa số lượng mảng
  eeprom.delete_eeprom_forward(first_address - 1, next_array);
  eeprom.delete_eeprom_backward(last_address, next_pointer);

  eeprom.write_2_byte(next_array_address, first_address);
  eeprom.write_2_byte(next_pointer_address, last_address - 3);

  for (int x = 1; x <= number_of_button; x++) EEPROM.write(x, 0);
  for (int y = arduino_eeprom_size - number_of_button; y < arduino_eeprom_size; y++) EEPROM.write(y, 0);
}

byte getNumberArray() {
  return eeprom.read_1_byte(number_of_array);
}

void getButtonList(int inMenu) {
  EEPROM.write(inMenu, 1);
  EEPROM.write(arduino_eeprom_size - inMenu, getNumberArray() + 1);
  websiteFlag = false;
  learningFlag = true;
}

void controlIRDevice(int button) {
  if (EEPROM.read(button) == 1)  {
    int buttonAddress = EEPROM.read(arduino_eeprom_size - button);
    Serial.println(String("button address: ") + buttonAddress);
    send_IR_code_By_Pointer(buttonAddress);
  }
}
