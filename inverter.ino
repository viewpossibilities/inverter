
#include "EasyNextionLibrary.h"  // Include EasyNextionLibrary
                                 // Download the latest version https://github.com/Seithan/EasyNextionLibrary
                                 // or from Arduino's IDE Library Manager

EasyNex myNex(Serial1);  // Create an object of EasyNex class with the name < myNex >
                         // Set as parameter the Hardware Serial you are going to use

const int REFRESH_TIME = 1000;           // time to refresh the Nextion data every 1000 ms
unsigned long refresh_timer = millis();  // timer for refreshing Nextion's page

String stringFromNextion;

int relayPin = 13;


void setup() {
  myNex.begin(115200);  // Begin the object with a baud rate of 9600
                      // If no parameter was given in the begin(), the default baud rate of 9600 will be used
  Serial1.begin(115200);
  Serial.begin(115200);

    pinMode(relayPin, OUTPUT);
}

void loop() {
  myNex.NextionListen();  // WARNING: This function must be called repeatedly to response touch events
                          // from Nextion touch panel. Actually, you should place it in your loop function.
  Nextion_display();

  

 relay();


}

void Nextion_display() {
  if ((millis() - refresh_timer) > REFRESH_TIME) {  //IMPORTANT do not have serial print commands in the loop without a delay
                                                    // or an if statement with a timer condition like this one.

    power_display();
    voltage_display();
    current_display();
    phase_power_display();
    inputvoltage_display();
    mosfet_temp_display();
    trans_temp_display();
    power_button();






    refresh_timer = millis();  // Set the timer equal to millis, create a time stamp to start over the "delay"
  }
}

void power_display() {
  myNex.writeStr("pu.txt", "1.76");
  myNex.writeStr("pv.txt", String(1.94));
  myNex.writeStr("pw.txt", String(2.19));
}

void voltage_display() {
  myNex.writeStr("vu.txt", "220");
  myNex.writeStr("vv.txt", String(221));
  myNex.writeStr("vw.txt", String(219));
}

void current_display() {
  myNex.writeStr("cu.txt", "10");
  myNex.writeStr("cv.txt", String(11));
  myNex.writeStr("cw.txt", String(10));
}

void phase_power_display() {
  myNex.writeStr("pkva.txt", "6.89");
  myNex.writeStr("pkw.txt", String(5.5));
}

void inputvoltage_display() {
  myNex.writeStr("ivolt.txt", "72");
}

void mosfet_temp_display() {
  myNex.writeStr("ulow.txt", "32");
  myNex.writeStr("uhigh.txt", "30");
  myNex.writeStr("vlow.txt", "33");
  myNex.writeStr("vhigh.txt", "34");
  myNex.writeStr("wlow.txt", "33");
  myNex.writeStr("whigh.txt", "34");
}
void trans_temp_display() {
  myNex.writeStr("xpow.txt", "30");
}



String power_button(){
 int  power= myNex.readNumber("bt0.val");

    String state;

  if (power == 1) {
    //Serial.println("ON");
    state = "ON";
    //return "ON";

  } else if (power == 0) {
   // Serial.println("OFF");
    state = "OFF";
  }
  else if (power < 0){
     state = "OFF";
  }
  return state;

}

void relay(){
  if(power_button() == "ON"){
    digitalWrite(relayPin, HIGH); 
      myNex.writeStr("g0.txt", "Inverter Is Running"); 
  }
  else if(power_button() == "OFF"){
    digitalWrite(relayPin, LOW);
    myNex.writeStr("g0.txt", "Inverter Is Off");   
  }

}