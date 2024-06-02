const String PHONE = "YOUR-PHONE";
String sms_text = "test";
/*  
use : Hareware Serial (Serial2)
      TX - pin 17 --> RX SIM800L
      RX - pin 16 --> TX SIM800L
*/

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  delay(10000);   /*  Waiting Initialization Module SIM800L   */
  Serial2.begin(9600);
}

void loop() {
  Serial.println("Sending sms to"+PHONE);
  Serial2.print("AT+CMGF=1\r");
  delay(500);
  Serial2.print("AT+CMGS=\""+PHONE+"\"\r");
  delay(500);
  Serial2.print(sms_text);
  delay(100);
  Serial2.write(0x1A); //ascii code for ctrl-26 
  delay(1000);
  Serial.println("SMS Sent Successfully.");
  delay(60*1000); //send 1sms/min
}
