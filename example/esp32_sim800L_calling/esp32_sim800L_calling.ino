const String PHONE = "YOUR-PHONE";
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
  Serial.println("Calling to"+PHONE);
  Serial2.print("ATD"+PHONE+";\r");
  Serial.println("delay");
  delay(10000);
}
