
// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include "EmonLib.h"                   // Include Emon Library

EnergyMonitor emonU;                   // Create an instance for phase U
EnergyMonitor emonV;                   // Create an instance for phase V
EnergyMonitor emonW;                   // Create an instance for phase W

#define Buzzer 6                       // Pin assignment for the buzzer

#define voltageB A6                    // pin assignment for battery voltage sensor

#define voltageU  A3                   // Pin assignment for phase U voltage sensor
#define voltageV  A4                   // Pin assignment for phase V voltage sensor
#define voltageW  A5                   // Pin assignment for phase W voltage sensor

#define mosfetU_Pin  A7                   // Pin assignment for phase U mosfet temperature sensor
#define mosfetV_Pin  A8                   // Pin assignment for phase V mosfet temperature sensor
#define mosfetW_Pin  A9                   // Pin assignment for phase W mosfet temperature sensor

#define contactor 7                    // pin assignment for the contactor 
#define fan 8

double voltageU_val = 0.0;
double voltageV_val = 0.0;
double voltageW_val = 0.0;



float ResistorVal = 10000;               // value of Resistor used in the voltage devider circuit for the mosfet temperature sensors

float  U_val, V_val, W_val;                       // Variables to hold resistance value of each temperature sensor for phase U,V,W

float logU_val, logV_val, logW_val;             // Variables to hold log of the resistance value of each temperature sensor for phase U,V,W

float TempU, TempV, TempW;            // Variables to hold raw temperature value of each temperature sensor for phase U,V,W

float TempC_U, TempC_V, TempC_W;      // Variables to hold Temp value in degree centigrade of each temperature sensor for phase U,V,W

float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07; // sensor calibration constrants according to datasheet


void setup()
{  
  Serial.begin(9600);
  
  emonU.current(A0, 111.1);             // Current: input pin, calibration for phase U.
  emonV.current(A1, 111.1);             // Current: input pin, calibration for phase V.
  emonW.current(A2, 111.1);             // Current: input pin, calibration for phase W.

  pinMode(Buzzer, OUTPUT);
  pinMode(fan, OUTPUT);
}

/*
Here is the function for measuring 
the output voltage for the three phases
*/
void get_output_voltage(){

double valU = (analogRead(voltageU) * 5) / 1024.0;
voltageU_val = valU / (2200 / (4700 + 2200));         //calculating the phase_U output voltage based on the voltage devider circuit 
Serial.println(voltageU_val);


double valV = (analogRead(voltageV) * 5) / 1024.0;
voltageV_val = valV / (2200 / (4700 + 2200));         //calculating the phase_V output voltage based on the voltage devider circuit 
Serial.println(voltageV_val);

double valW = (analogRead(voltageW) * 5) / 1024.0;
voltageW_val = valW / (2200 / (4700 + 2200));         //calculating the phase_W output voltage based on the voltage devider circuit 
Serial.println(voltageW_val);
  
}

/*
Here is the function for measuring 
the output current and aparent power 
for the three phases
*/

void get_current_and_Power(){

  double IrmsU = emonU.calcIrms(1480);  // Calculate Irms only for phase U
  double IrmsV = emonV.calcIrms(1480);  // Calculate Irms only for phase V
  double IrmsW = emonW.calcIrms(1480);  // Calculate Irms only for phase W
  
  Serial.println(IrmsU);		         // Irms for phase U
  Serial.print(IrmsU*voltageU_val);	       // Apparent power for phase U

  Serial.println(IrmsV);		         // Irms for phase V
  Serial.print(IrmsV*voltageV_val);	       // Apparent power for phase V
  
  Serial.println(IrmsW);		         // Irms for phase W
  Serial.print(IrmsW*voltageW_val);	       // Apparent power for phase W 
}

/*
Here is the function for the battery voltage 
monitoring and under voltage shutdown protection
*/

void get_battery_voltage (){
  double valB = (analogRead(voltageB) * 5) / 1024.0;
  double voltageB_val = valB / (217 / (4560 + 217));         //calculating the battery output voltage based on the voltage devider circuit 

 
 if (voltageB_val <= 68 && voltageB_val > 63 ){       // evaluaute and trigger low battery warning

  digitalWrite (Buzzer, HIGH);
 }
  else if (voltageB_val <= 63 ){                      //  Low battery automatic shutdown

  digitalWrite (Buzzer, LOW);
  delay(500);
  digitalWrite (contactor, HIGH);
 }
 else {        
  digitalWrite (Buzzer, LOW);
  digitalWrite (contactor, LOW);
 }
}
/*
Here is the function for the 
temperature monitoring, 
fan control and emagency shutdown 
*/

void get_mosfet_Temp (){

//mosfet temperature monitor for phase U

float sensorval_U = analogRead(mosfetU_Pin);
  U_val = 10000 * (1023.0 / sensorval_U - 1.0);
  logU_val = log(U_val);

  TempU = (1.0 / (c1 + c2*logU_val + c3*logU_val*logU_val*logU_val));
  TempC_U = TempU - 273.15;
  Serial.print (TempU);
  Serial.print("Phase U Temperature: "); 
  Serial.print(TempC_U);
  Serial.println(" C");

//mosfet temperature monitor for phase V

float sensorval_V = analogRead(mosfetV_Pin);
  V_val = 10000 * (1023.0 / sensorval_V - 1.0);
  logV_val = log(V_val);

  TempV = (1.0 / (c1 + c2*logV_val + c3*logV_val*logV_val*logV_val));
  TempC_V = TempV - 273.15;
  Serial.print (TempV);
  Serial.print("Phase V Temperature: "); 
  Serial.print(TempC_V);
  Serial.println(" C");

//mosfet temperature monitor for phase W

float sensorval_W = analogRead(mosfetW_Pin);
  W_val = 10000 * (1023.0 / sensorval_W - 1.0);
  logW_val = log(W_val);

  TempW = (1.0 / (c1 + c2*logW_val + c3*logW_val*logW_val*logW_val));
  TempC_W = TempW - 273.15;
  Serial.print (TempW);
  Serial.print("Phase W Temperature: "); 
  Serial.print(TempC_W);
  Serial.println(" C");
  
  // Mosfet Temperature colling fan control

  // condition for turning on the fan
  if ((TempC_U >= 45 && TempC_U <65) || (TempC_V >= 45 && TempC_V <65) || (TempC_W >= 45 && TempC_W <65)){

    digitalWrite(fan, HIGH);

  }
  // condition for turning off the fan
    else if ((TempC_U  <40) && (TempC_V <40) && (TempC_W <40)){

    digitalWrite(fan, LOW);

  }
  // condition for mosfet overheating shutdown 
    else if ((TempC_U >65) || (TempC_V >65) || (TempC_W >65)){

    digitalWrite(fan, LOW);
    delay(100);
    digitalWrite (contactor, HIGH);
  }

}
void loop()
{
get_output_voltage();
get_current_and_Power();
get_battery_voltage ();
get_mosfet_Temp();
}