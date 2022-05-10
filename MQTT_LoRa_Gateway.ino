#include "RH_RF95.h"            //RJ getting errors on MKR1400 V11
#include "RTCZero.h"
#include <MKRGSM.h>             //usefull to work with an MKR board
#include <MQTT.h>
#include "Gateway_settings.h"
#include <NTPClient.h>


String datetime;
String gatewayStatus;
const int FW_VERSION = FIRMWARE_VERSION;
const int debug = IS_DEBUG;
const int BATT_POLL_INT_M = POLL_BATT_MINUTE_INTERVAL;
String gateway = GATEWAY_NAME;
String gatewayTopic = "Gateway/" + gateway;
String topic = "Nerds";
uint16_t count = 0;
unsigned long lastBattMeas = 0;
float battV = 0;
static const int RXPin = 4, TXPin = 3;
uint8_t rfbuf[RH_RF95_MAX_MESSAGE_LEN];
bool connected = false;
String LoRaPacketBuff[PACKET_BUFF_LIMIT] = {};
int RSSIBuff[PACKET_BUFF_LIMIT] = {};
int buffCounter = 0;
String server = MQTT_SERVER_ADDR;
String deploymentID = G_SHEET; //Packenham google sheet

RH_RF95 rf95(12, 6);
RTCZero rtc;              //Real Time clock
GSM gsmAccess(false);            //access to the network, true if you want to have all messages from the GSM chip
GSMClient net;
GSMUDP timeUDP;
GPRS gprs;                //to the date
MQTTClient client(256);   //increase buffer size to 256
NTPClient timeClient(timeUDP,"pool.ntp.org");

void setup() {
  SerialUSB.begin(9600);
  rtc.begin();
  setupWDT( WATCHDOG_TIMER_MS );
  pinMode(LED, OUTPUT);
  while (!rf95.init()) {
    delay(100);
  }
  rf95.setTxPower(23, false);
  rf95.setFrequency(LORA_FREQUENCY);
  client.begin((char*)server.c_str(), 1883, net);
  client.setKeepAlive(3600);
  sendLora("LoRa to MQTT gateway starting up");
  ConnectToTheWorld();
  timeClient.begin();
  gatewayStatusPing();
}

void loop() {
  while (debug) {
    resetWDT();
    SerialUSB.println("Hello this is a test");
    quickSendMQTT("Hello this is a test");
    SerialUSB.println("Hello this is a test has been sent over MQTT");
    delay(5000);
  }

  uint8_t len;
  resetWDT();
  updateLastWill();
  while (rf95.available()) {
    len = sizeof(rfbuf);
    if (rf95.recv(rfbuf, &len)) {
      char temp[len];
      char* key;
      strcpy(temp, (char*)rfbuf);
      key = strtok(temp, ":");    //Add after melton. SET DEFAULT AS NULL
      if(strcmp(key,"PKT") == 0||strcmp(key,"DBG") == 0){  //Include if collecting too much crap
      LoRaPacketBuff[buffCounter] = (char*)rfbuf;
      RSSIBuff[buffCounter] = rf95.lastRssi();
      buffCounter++;
      buffFullCheck();
    }
    }
  }
  if (selfCheck()) {
    resetWDT();
    //buffFullCheck();
    gatewayStatusPing();
  }
}

void sendLora(String msg) {
  char *ppmsg;
  ppmsg = (char*)msg.c_str();
  rf95.send((uint8_t *)ppmsg, strlen(ppmsg) + 1);
  rf95.waitPacketSent();
}

void ConnectToTheWorld() {
  resetWDT();
  gsmAccess.setTimeout(10 * 1000); //stop gsm after 'timeot' seconds if cannot connect
  gprs.setTimeout(10 * 1000); //stop gsm after 'timeot' seconds if cannot connect
  gsmAccess.shutdown();
  connected = false;  // connection state
  int trials = 0;     //limit for the numbers of connection trials...
  //SerialUSB.println("Connecting to GSM");
  while (!connected  && (trials <= NETWORK_ATTEMPTS )) {
    resetWDT();
    if ((gsmAccess.begin() == GSM_READY)) {
      connected = true;
      //SerialUSB.println("GSM: Success");
    } else {
      //SerialUSB.println("GSM: Failed");
      delay(500);
    }
    trials++;
  }
  trials = 0;     //limit for the numbers of connection trials...
  connected = false;  // connection state
  while (!connected  && (trials <= NETWORK_ATTEMPTS )) {
    resetWDT();
    if (gprs.attachGPRS(GPRS_APN, GPRS_LOGIN, GPRS_PASSWORD) == GPRS_READY) {
      connected = true;
      //SerialUSB.println("GPRS: Success");
    } else {
      //SerialUSB.println("GPRS: Failed");
      delay(500);
    }
    trials++;
  }
}

void setupWDT( uint8_t period) {
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(5) | GCLK_GENDIV_DIV(8);  //gendiv 4=16s, gendiv 5=32s,6=1 min, 7=2min, 8= 4min
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(5) |
                      GCLK_GENCTRL_GENEN |                      
                      GCLK_GENCTRL_SRC_OSCULP32K |
                      GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);  // Syncronize write to GENCTRL reg.
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT |
                      GCLK_CLKCTRL_CLKEN |
                      GCLK_CLKCTRL_GEN_GCLK5;
  WDT->CTRL.reg = 0; // disable watchdog
  WDTsync(); // sync is required
  WDT->CONFIG.reg = period; // see Table 17-5 Timeout Period (valid values 0-11)
  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable watchdog
  WDTsync();
}

void systemReset() {  // use the WDT watchdog timer to force a system reset.
  WDT->CLEAR.reg = 0x00; // system reset via WDT
  WDTsync();
}

void resetWDT() {
  WDT->CLEAR.reg = 0xA5; // reset the WDT
  WDTsync();
}

void disableWDT() {
  WDT->CTRL.reg = 0;
}

static void WDTsync() {
  while (WDT->STATUS.bit.SYNCBUSY == 1); //Just wait till WDT is free
}

bool selfCheck() {
  if ((millis() - lastBattMeas) > (BATT_POLL_INT_M * 60 * 1000)) {
    battV = analogRead(32) * 0.003 + 1.083;
    //    LoRaPacketBuff[buffCounter] = "NULL,AA,GWAY," + String(FW_VERSION) + "," + String(count++) + ",NULL,NULL," + String(battV) + ",NULL,NULL,NULL,NULL,";
    //    RSSIBuff[buffCounter] = 0;
    //    buffCounter++;
    lastBattMeas = millis();
    return true;
  }
  return false;
}

void buffFullCheck() {
  if (buffCounter >= PACKET_BUFF_LIMIT) {
    SerialUSB.println("Buffer full, sending over MQTT");
    buffCounter = 0;
    SerialUSB.println("Buffer full, connected");
    MQTTPublish();  //If we aren't connected we crash here
  }
}

void quickSendMQTT(String packt) {
  MQTTPublishQuick(packt);
}

void MQTTPublishQuick(String pkt) {  //***SEND MESSAGE TO GOOGLE SPREADSHEET
  String JSONPKT;
  resetWDT();
  JSONPKT = "{\"String\":\"" + pkt + "\",\"URL\":\"" + deploymentID + "\"}";
    if (!connected) {
    ConnectToTheWorld();
  }
  if (!client.connected()) {
    connectMQTT();
  }
  if (client.connected()) {
    client.publish(topic, JSONPKT);
  }
  client.disconnect();
}

void MQTTPublish() {
  String dataString;
  String JSONPKT;
  resetWDT();
    if (!connected) {
    ConnectToTheWorld();
  }
  if (!client.connected()) {
    connectMQTT();
  }
  for (int i = 0; i < PACKET_BUFF_LIMIT ; i++) {
    SerialUSB.println("Buffer emptying");
    char* token;
    char temp[LoRaPacketBuff[i].length()];
    char nodeTopic[10];
    LoRaPacketBuff[i].toCharArray(temp,LoRaPacketBuff[i].length());
    token = strtok (temp,",");
    token = strtok (NULL,",");
    strcpy(nodeTopic,token);
    token = strtok (NULL,",");
    strcat(nodeTopic,"/");
    strcat(nodeTopic,token);
    dataString = LoRaPacketBuff[i] +","+ String(RSSIBuff[i],DEC);
    JSONPKT = "{\"String\":\"" + String(dataString) + "\",\"URL\":\"" + deploymentID + "\"}";
    if (client.connected()) {
      client.publish(nodeTopic, JSONPKT);
    }
    delay(100); //not sure about this
  }
  // client.close();
  client.disconnect();
}

int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

String nowthatIcanread() { //fix
  return print2digits(rtc.getDay())+"/"+print2digits(rtc.getMonth())+"/"+print2digits(rtc.getYear())+" "+print2digits(rtc.getHours())+":"+print2digits(rtc.getMinutes())+":"+print2digits(rtc.getSeconds());
}
String print2digits(int number) { //print correctly dates
  String numback="";
  if (number < 10) { numback="0";} // print a 0 before if the number is < than 10
  numback=numback+String(number);
  return numback;
}

void gatewayStatusPing() {
  SerialUSB.println("Publishing Status to Topic:Gateway");
  resetWDT();
  if (!connected) {
    ConnectToTheWorld();
  }
  if (!client.connected()) {
    connectMQTT();
  }
  if (client.connected()) {
  getInternetTime();
  datetime = nowthatIcanread();
  gatewayStatus = "{\"Gateway_Name\":\"" + gateway + "\",\"Firmware_Version\":\"" + String(FW_VERSION) + "\",\"Battery_Voltage\":\"" + String(battV) + "\",\"Uptime\":\"" + String(millis()/1000/60) + "\",\"Last_Update\":\"" + datetime + "\",\"Status\":\"Alive\"}";
    SerialUSB.println("Published: " + gatewayStatus);// To ensure status gets updated
      client.publish(gatewayTopic.c_str(), gatewayStatus.c_str(), gatewayStatus.length(), true, 1); //Retain latest message from gateway
  }
  SerialUSB.println("Finished publishing Status to Topic:Gateway");
  client.disconnect();
  delay(500);
  gsmAccess.shutdown();
  connected = false;
}

void updateLastWill() {
  datetime = nowthatIcanread();
  gatewayStatus = "{\"Gateway_Name\":\"" + gateway + "\",\"Firmware_Version\":\"" + String(FW_VERSION) + "\",\"Battery_Voltage\":\"" + String(battV) + "\",\"Uptime\":\"" + String(millis()/1000/60) + "\",\"Last_Update\":\"" + datetime + "\",\"Status\":\"Down\"}";
  client.setWill(gatewayTopic.c_str(), gatewayStatus.c_str(), true, 1);
  }
  
void connectMQTT() {
    if (connected) {
    resetWDT();
    int trials = 0;
    while (!client.connected() && (trials <= NETWORK_ATTEMPTS )) {
      client.connect("arduino", "public", "public");
      //SerialUSB.print(".");
       delay(500);
       trials++;
    }
    //client.subscribe(topic);
    //SerialUSB.println("Connected to GSM and Subbed to topics");
  } else {
    //SerialUSB.println("GSM not connected. Won't sub to topics");
  }
}

void getInternetTime(){
  timeClient.update();
  rtc.setEpoch(timeClient.getEpochTime() + 36000);
  }
