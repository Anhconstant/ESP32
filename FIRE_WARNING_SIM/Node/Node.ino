/*-------------------------------------------------------------------------------*/
/*
NODE 1  : MAC Address this device : 0xC8, 0x2E, 0x18, 0xF1, 0x88, 0x8C
NODE 2  : MAC Address this device : 0xC8, 0x2E, 0x18, 0xF1, 0x49, 0x7C
GATEWAY : MAC Address this device : 0x08, 0xB6, 0x1F, 0xB8, 0x5D, 0xF8
ID: 01:01:01
Tầng 1: Phòng 1: Thiết bị 1: reason and command
*/
/*-------------------------------------------------------------------------------*/
/*-----------------------------------Include-------------------------------------*/

#include "WiFi.h"
#include "esp_now.h"
#include "string.h"

/*-----------------------------------Define Constant-----------------------------*/
#define default_floor   1
#define default_room    1
#define default_device  1

#define Period    5000
#define RES_REF   9600
#define COMMAND_SET     1    // COMMAND_SET : send warning from node to gateway
#define COMMAND_ON      2    // COMMAND_ON : send warning from gateway to node
#define COMMAND_OFF     3    // COMMAND_OFF : send warning from gateway to node
#define COMMAND_RESET   4    // COMMAND_RESET : send reset from gateway to node

/*-----------------------------------Define Sensor Pin---------------------------*/

#define SensorNTC1        36 
#define SensorNTC2        39
#define SensorMQ2analog   32
#define SensorMQ2digital  33

/*-----------------------------------Define Button Pin---------------------------*/

#define ButtonWaring      22
#define ButtonBoot0       0
//#define ButtonBoot2       2

/*-----------------------------------Define LED Pin------------------------------*/

#define LedMode           21
#define ActuatorSig       19  
#define Relay             18

/*-----------------------------------Define SIM Pin------------------------------*/

#define SimRST            5
#define SimTX             17
#define SimRX             16

String Phone = "0xxxxxxxx";
/*-----------------------------------Declare variable----------------------------*/

int adc_temp1         =   0 ;   //    pin 36
int adc_temp2         =   0 ;   //    pin 39
int SmokeAnalog       =   0 ;   //    pin 32
int SmokeDigital      =   0 ;   //    pin 33
uint8_t   BoardcastAddress[] = { 0x08, 0xB6, 0x1F, 0xB8, 0x5D, 0xF8 }  ;    

uint32_t  Data2send = 0 ;

uint8_t   Data2send_4[4] = {0x00, 0x00 , 0x00, 0x00 } ;
 
uint8_t   Data2reiceive_4[4] = {0x00, 0x00 , 0x00, 0x00 } ;

esp_now_peer_info_t peerInfo;

// STATUS   
int relay_status = 0;
int speaker_status = 0 ;
int sendsms_status = 0 ;
int sendgateway_status = 0 ;
int warning = 0  ;
int wait_reset = 0 ;
int reason = 0 ;       // =1 : gas // =2 : smoke // =3 : Temp  // == 4: another node // ==5 : button
int old_reason = 4 ;
String sms_Text = "";

int time_button = 0 ;

/*-----------------------------------Define function-----------------------------*/

#define READ_BYTE( DATA, BYTE )     ( DATA >> ( 8*BYTE ) & ( 0 | 0xFF ) )
#define GET_FLOOR( DATA )           ((int)READ_BYTE(  DATA, 0 ))
#define GET_ROOM( DATA )            ((int)READ_BYTE(  DATA, 1 ))
#define GET_DEVICE( DATA )          ((int)READ_BYTE(  DATA, 2 ))
#define GET_COMMAND( DATA )         ((int)READ_BYTE(  DATA, 3 ))

void setupdata(uint8_t data[] , uint8_t  floor, uint8_t  room, uint8_t  devide, uint8_t  command, uint8_t  reason ){
  data[0] = floor ;
  data[1] = room  ;
  data[2] = devide  ;
  data[3] = reason << 4 ;
  data[3] |= command ;
}

int sms_setup( int reason){
  int check = 0 ;
  sms_Text = "";

  sms_Text += "CANH BAO CHAY, PHAT HIEN " ;
  switch(reason){
    case 1:
      sms_Text = sms_Text + "NONG DO CO2 CAO TAI:" + "TANG 1 PHONG 1 THIET BI 1" ;
      check = 1 ;
      break;
    case 2:
      sms_Text = sms_Text + "KHOI TAI:" + "TANG 1 PHONG 1 THIET BI 1"  ;
      check = 1 ;
      break;
    case 3:
      sms_Text = sms_Text + "NHIET DO CAO TAI:" + "TANG 1 PHONG 1 THIET BI 1" ;
      check = 1 ;
      break;
    case 4:
      sms_Text = sms_Text + "TAI THIET BI KHAC:"+ "TANG "+ String(Data2reiceive_4[0]) +"PHONG "+ String(Data2reiceive_4[1]) + "THIET BI " + String(Data2reiceive_4[2]) ;
      check = 1 ;
      break;
    case 5:
      sms_Text = sms_Text + "BANG NUT BAM TAI:" + "TANG 1 PHONG 1 THIET BI 1" ;
      check = 1 ;
      break;
    default:
      check = 0;
      break;
  }
  Serial.println(sms_Text);

  return check;
}

void ButtonCallback(){
  if(millis() - time_button < 10*1000 ) return;
  Serial.println("With button");
  time_button = millis();

  if(warning == 1){
    warning = 0 ;
    wait_reset = 1 ;
  }
  else if(warning == 0){
    warning = 1 ;
    
    reason = 5 ;
    digitalWrite(ActuatorSig , 1) ;
  }
}

void clear_data(){
  Data2send = 0 ;
  Data2send_4[0] = 0 ;
  Data2send_4[1] = 0 ;
  Data2send_4[2] = 0 ;
  Data2send_4[3] = 0 ;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("\r\nLast Packet Send Status:\t");
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.println("===================");
}

void SentData(){
  esp_err_t result = esp_now_send(BoardcastAddress, (uint8_t *) &Data2send_4, sizeof(Data2send_4));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Data2reiceive_4, incomingData, sizeof(Data2reiceive_4));
  Serial.print("Bytes received: ");
  Serial.println(Data2reiceive_4[0]);
  Serial.println(Data2reiceive_4[1]);
  Serial.println(Data2reiceive_4[2]);
  Serial.println(Data2reiceive_4[3]);
  Serial.println(Data2reiceive_4[3] & 0x0F);

  if((Data2reiceive_4[3] & 0x0F) ==  2){
    warning  = 1;
    reason = 4;
  }
  if(((Data2reiceive_4[3] & 0x0F) ==  3 ) && (reason == 4)){
    warning  = 0;
    wait_reset = 1 ;
  }
}

void Call2Sim(){
  Serial.println("Calling to "+Phone);
  Serial2.print("ATD08xxxxxx;\r");
}

/*-----------Declare function----------- */
void ReadSensor(){
  int temp1 = 1 ;
  int temp2 = 1 ;
  //temp1       =   analogRead(SensorNTC1)        ;
  temp2       =   analogRead(SensorNTC2)        ;
  //adc_temp1   =   RES_REF * ( ( 4095 - temp1)*1.0 / temp1 )  ;     // RES_REF = 9600 Ohm : hiệu chỉnh , thiết kế ban đầu là 10k Ohm
  adc_temp1 = 9000;
  adc_temp2   =   RES_REF * ( ( 4095 - temp2)*1.0 / temp2 )  ;
  if(millis() > 3*60*1000){
    SmokeAnalog     =   analogRead(SensorMQ2analog)   ;
  }
  else SmokeAnalog = 500 ;
  //SmokeAnalog = 1400;
  
  Serial.print("Smoke: ") ; Serial.println(SmokeAnalog);
  
  Serial.print("NTC_1: "); Serial.println(adc_temp1);
  Serial.print("NTC_2: "); Serial.println(adc_temp2);
}

int CheckWarningStatus(int temp1 ,int temp2,int Smoke){
  int check = 0;
  if( Smoke < 100){
    reason = 1 ;
    check = 1 ;
  }
  else if(Smoke > 1000 ){
    reason = 2 ;
    check = 1 ;
  }

  if( ( temp1 < 2000 ) || ( temp2 < 2000 ) ) {
    reason = 3 ;
    check = 1 ;
  }
  return check ;
}

void Send2Sim(String * buffer){
  
  Serial.println("Sending sms to "+Phone);
  Serial2.print("AT+CMGF=1\r");
  delay(500);
  Serial2.print("AT+CMGS=\""+Phone+"\"\r");
  delay(500);
  Serial2.print(sms_Text);
  delay(100);
  Serial2.write(0x1A); //ascii code for ctrl-26 
  delay(1000);
  Serial.println("SMS Sent Successfully.");

}

void reset_status(){
  speaker_status = 0  ;
  sendsms_status = 0  ;
  sendgateway_status = 0 ;
  old_reason = 4  ;
  relay_status = 0;
  Serial.println("RESET_STATUS...!! ");
}

/*-----------Declare variable flag----------- */
void init_pin(){
  pinMode(SensorNTC1,INPUT);
  pinMode(SensorNTC2,INPUT);
  pinMode(SensorMQ2analog,INPUT);
  pinMode(SensorMQ2digital,INPUT);
  pinMode(ButtonWaring,INPUT);

  pinMode(Relay,OUTPUT);
  pinMode(LedMode,OUTPUT);
  pinMode(ActuatorSig,OUTPUT);
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//


void setup() {
/*-----------Configure Pin -----------------*/
  init_pin();
  attachInterrupt(ButtonWaring, ButtonCallback, FALLING);

/*-----------Configure Wifi -----------------*/
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register gateway peer  
  memcpy(peerInfo.peer_addr, BoardcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

/*---------------------------------------------*/
  

  Serial2.begin(9600);
  Serial2.println("Serial2");
  
  Serial.begin(115200);

  Serial.println("Start SETUP");
  Serial.println("Bandrate UART0:115200");

  Serial.println("End SETUP - >>> Time config: 30s !!!");
  delay(30000);
  Serial.println("START.........................!!!");
  digitalWrite(LedMode,HIGH);
}


//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//

void loop() {
  if( warning == 0){
    ReadSensor() ;
    delay(100);
    digitalWrite(ActuatorSig , 0) ;
    digitalWrite(Relay, 0) ;
    warning = CheckWarningStatus( adc_temp1 , adc_temp2, SmokeAnalog ) ;
    Serial.print("CHECK WARNING !! : ") ;
    Serial.println(warning) ;
  }

  if( (warning == 1 ) && ( wait_reset != 1 ) ){
    ReadSensor() ;
    Serial.println("IN WARNING !!") ;
    if( (old_reason) == 4 && (reason !=4) ){
      old_reason = reason ;
      if(sendgateway_status >1 ){
        setupdata(Data2send_4, default_floor, default_room, default_device, COMMAND_SET, reason);
        SentData();
      }
      
      if(sendsms_status){
      sms_setup(reason);
      Send2Sim(&sms_Text);
      }

    }
    if( sendsms_status == 0 ){
      Serial.println("SENT SMS !!") ;
      sms_setup(reason);
      Send2Sim(&sms_Text);
      delay(1000);
      Call2Sim(); 
      sendsms_status = 1;
      
    }
    if( speaker_status == 0 ){
      Serial.println("WARNING ON !!");
      digitalWrite(ActuatorSig , 1) ;
      speaker_status = 1;
    }
    
    if( sendgateway_status < 2 ){
      if(reason != 4 ){
      Serial.println("SENT DATA GATEWAY !!") ;
      setupdata(Data2send_4, default_floor, default_room, default_device, COMMAND_SET, reason);
      //Serial.println(Data2send);
      sendgateway_status++ ;
      SentData();
      }
    }
    if( relay_status == 0 ){
      delay(1000);
      Serial.println("relay ON !!");
      digitalWrite(Relay,1);
      relay_status = 1;
    }

  }
  else {
    reset_status() ;
  }

  if(wait_reset){
    wait_reset = 0 ;
  }


  delay(5000) ;
}
