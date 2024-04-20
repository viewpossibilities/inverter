
#include "EasyNextionLibrary.h"

EasyNex myNex(Serial1);

const int REFRESH_TIME = 1000;           // time to refresh the Nextion data every 1000 ms
unsigned long refresh_timer = millis();  // timer for refreshing Nextion's page

String stringFromNextion;


#include "EmonLib.h"  // Include Emon Library

EnergyMonitor emonU;  // Create an instance for phase U
EnergyMonitor emonV;  // Create an instance for phase V
EnergyMonitor emonW;  // Create an instance for phase W
const double currentCalibFactor = 111.1;

#define Buzzer 6  // Pin assignment for the buzzer

#define voltageB A6  // pin assignment for battery voltage sensor

#define voltageU A3  // Pin assignment for phase U voltage sensor
#define voltageV A4  // Pin assignment for phase V voltage sensor
#define voltageW A5  // Pin assignment for phase W voltage sensor

#define mosfetU_Pin A7  // Pin assignment for phase U mosfet temperature sensor
#define mosfetV_Pin A8  // Pin assignment for phase V mosfet temperature sensor
#define mosfetW_Pin A9  // Pin assignment for phase W mosfet temperature sensor

#define contactor 7  // pin assignment for the contactor
#define fan 8



float pf = 0.8;  // power factor

float ResistorVal = 10000;  // value of Resistor used in the voltage devider circuit for the mosfet temperature sensors

float mosfetTemp_U, mosfetTemp_V, mosfetTemp_W;  // Variables to hold Temp value in degree centigrade of each temperature sensor for phase U,V,W

float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;  // sensor calibration constrants according to datasheet

//ACouput voltages
float U_AcOutputVoltage, V_AcOutputVoltage, W_AcOutputVoltage;

//output current
float U_Current, V_Current, W_Current;

//AC Power
float U_Power, V_Power, W_Power;

//Dc battery
float batteryVoltage;


void setup() {
  myNex.begin(115200);  // Begin the object with a baud rate of 9600
                        // If no parameter was given in the begin(), the default baud rate of 9600 will be used
  Serial1.begin(115200);
  Serial.begin(115200);

  //pinMode(relayPin, OUTPUT);
  emonU.current(A0, currentCalibFactor);  // Current: input pin, calibration for phase U.
  emonV.current(A1, currentCalibFactor);  // Current: input pin, calibration for phase V.
  emonW.current(A2, currentCalibFactor);  // Current: input pin, calibration for phase W.

  pinMode(Buzzer, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(contactor, OUTPUT);
}

void loop() {
  //saves the AC output voltages in variables for each phase
  U_AcOutputVoltage = AC_outputVoltage(voltageU);
  V_AcOutputVoltage = AC_outputVoltage(voltageV);
  W_AcOutputVoltage = AC_outputVoltage(voltageW);


  //saves the AC current in variables for each phase
  U_Current = U_getCurrent();
  V_Current = V_getCurrent();
  W_Current = W_getCurrent();

  //saves the AC power in variables for each phase
  U_Power = U_getPower();
  V_Power = V_getPower();
  W_Power = W_getPower();

  //saves the DC input battery in a variable
  batteryVoltage = dcInputVoltage(voltageB);

  //check battery voltages
  checkBattery(batteryVoltage);

  //saves the MOSFET temperature in variables for each phase
  mosfetTemp_U = getMosfetTemperature(mosfetU_Pin);
  mosfetTemp_V = getMosfetTemperature(mosfetV_Pin);
  mosfetTemp_W = getMosfetTemperature(mosfetW_Pin);


  //check mosfet temperatues
  temperatureCheck(mosfetTemp_U, mosfetTemp_V, mosfetTemp_W);

  //inverter power switch
  Power_Inverter(batteryVoltage, mosfetTemp_U, mosfetTemp_V, mosfetTemp_W);

  myNex.NextionListen();  // WARNING: This function must be called repeatedly to response touch events
                          // from Nextion touch panel. Actually, you should place it in your loop function.

  //displays value to nextion screen
  Nextion_display();
}

void Nextion_display() {
  if ((millis() - refresh_timer) > REFRESH_TIME) {  //IMPORTANT do not have serial print commands in the loop without a delay
                                                    // or an if statement with a timer condition like this one.

    power_display(U_Power, V_Power, W_Power);
    voltage_display(U_AcOutputVoltage, V_AcOutputVoltage, W_AcOutputVoltage);
    current_display(U_Current, V_Current, W_Current);
    totalphase_power_display(U_Power, V_Power, W_Power);
    dcInputVoltage_display(batteryVoltage);
    mosfet_temp_display(mosfetTemp_U, mosfetTemp_V, mosfetTemp_W);
    transformer_temp_display();
    refresh_timer = millis();  // Set the timer equal to millis, create a time stamp to start over the "delay"
  }
}

//function to calculate power for u, v, w
float U_getPower() {
  return U_AcOutputVoltage * U_Current;
}
float V_getPower() {
  return V_AcOutputVoltage * V_Current;
}
float W_getPower() {
  return W_AcOutputVoltage * W_Current;
}

void totalphase_power_display(float u, float v, float w) {
  float kva = (u + v + w) / 3;
  float kw = ((u * pf) + (v * pf) + (w * pf)) / 3;
  myNex.writeStr("pkva.txt", String(kva));
  myNex.writeStr("pkw.txt", String(kw));
}

void power_display(float upower, float vpower, float wpower) {
  myNex.writeStr("pu.txt", String(upower));
  myNex.writeStr("pv.txt", String(vpower));
  myNex.writeStr("pw.txt", String(wpower));
}

//Function to read AC output voltage
float AC_outputVoltage(int sensorPin_ACVoltage) {
  //stepdowntransformer is 220/6V
  int sensorValue = analogRead(sensorPin_ACVoltage);
  float voltage = sensorValue * (5.0 / 1023.0);
  float voltage_ = voltage / 0.5;
  // voltage_ += 0.75;
  voltage_ *= 37.5;
  //Serial.println(voltage_);
  if (voltage_ < 60) {
    voltage_ = 0.0;
  }
  return voltage_;
}

//display voltage to screen
void voltage_display(float U_AcOutputVoltage, float V_AcOutputVoltage, float W_AcOutputVoltage) {


  myNex.writeStr("vu.txt", String(U_AcOutputVoltage));


  myNex.writeStr("vv.txt", String(V_AcOutputVoltage));


  myNex.writeStr("vw.txt", String(W_AcOutputVoltage));
}

//Current for u, v and w

float U_getCurrent() {
  return emonU.calcIrms(1480);  // Calculate Irms only for phase U
}

float V_getCurrent() {
  return emonV.calcIrms(1480);  // Calculate Irms only for phase U
}
float W_getCurrent() {
  return emonW.calcIrms(1480);  // Calculate Irms only for phase U
}

//display current
void current_display(float ucurrent, float vcurrent, float wcurrent) {
  myNex.writeStr("cu.txt", String(ucurrent));
  myNex.writeStr("cv.txt", String(vcurrent));
  myNex.writeStr("cw.txt", String(wcurrent));
}

//battery voltage
float dcInputVoltage(int voltagepin_) {
  float valB = (analogRead(voltagepin_) * 5) / 1024.0;
  float voltageB_val = valB / (217 / (4560 + 217));  //calculating the battery output voltage based on the voltage devider circuit
  return voltageB_val;
}

//display battery voltage to screen
void dcInputVoltage_display(float batVolt) {
  myNex.writeStr("ivolt.txt", String(batVolt));
}

//battery check to send warning at 68V and shut down at 63V

void checkBattery(float voltageB_val) {
  if (voltageB_val > 63.0 && voltageB_val <= 68.0) {  // evaluaute and trigger low battery warning

    digitalWrite(Buzzer, HIGH);
  } else if (voltageB_val <= 63) {  //  Low battery automatic shutdown

    digitalWrite(Buzzer, LOW);
    digitalWrite(contactor, HIGH);
  } else {
  }
}

//function that gets MOSFET temperature
float getMosfetTemperature(int mosfet_phase_Pin) {
  float inputValue = analogRead(mosfet_phase_Pin);
  float value = 10000 * (1023.0 / inputValue - 1.0);
  float logValue = log(value);
  float Temp = (1.0 / (c1 + c2 * logValue + c3 * logValue * logValue * logValue));
  return Temp - 273.15;
}

//function that displays the MOSFET temperature
void mosfet_temp_display(float mosU, float mosV, float mosW) {
  myNex.writeStr("ulow.txt", "-");
  myNex.writeStr("uhigh.txt", String(mosU));
  myNex.writeStr("vlow.txt", "-");
  myNex.writeStr("vhigh.txt", String(mosV));
  myNex.writeStr("wlow.txt", "-");
  myNex.writeStr("whigh.txt", String(mosW));
}

//function to check system temperature
void temperatureCheck(float TempC_U, float TempC_V, float TempC_W) {
  // condition for turning on the fan
  if ((TempC_U >= 45.0 && TempC_U < 65.0) || (TempC_V >= 45.0 && TempC_V < 65.0) || (TempC_W >= 45.0 && TempC_W < 65.0)) {

    digitalWrite(fan, HIGH);

  }
  // condition for turning off the fan
  else if ((TempC_U < 40.0) && (TempC_V < 40.0) && (TempC_W < 40.0)) {

    digitalWrite(fan, LOW);

  }
  // condition for mosfet overheating shutdown
  else if ((TempC_U > 65.0) || (TempC_V > 65.0) || (TempC_W > 65.0)) {

    digitalWrite(fan, LOW);
    delay(100);
    digitalWrite(contactor, HIGH);
  }
}

void transformer_temp_display() {
  myNex.writeStr("xpow.txt", "-");
}


//function that receives state from the power button on the screen
String power_button() {
  int power = myNex.readNumber("bt0.val");

  String state;

  if (power == 1) {

    state = "ON";

  } else if (power == 0) {

    state = "OFF";
  } else if (power < 0) {
    state = "OFF";
  }
  return state;
}


//function that power on the inveter after meeting certain condition and can power it off
void Power_Inverter(float batVoltage, float TempC_U, float TempC_V, float TempC_W) {
  if (power_button() == "ON") {
    if (batVoltage > 63.0) {
      if ((TempC_U < 65.0) || (TempC_V < 65.0) || (TempC_W < 65.0)) {
        digitalWrite(contactor, LOW);
        myNex.writeStr("g0.txt", "Inverter Is Running");

      } else {
        myNex.writeStr("g0.txt", "Temperature Is too High");
      }


    } else {
      myNex.writeStr("g0.txt", "Battery Is Low, Please Charge");
    }
  } else if (power_button() == "OFF") {
    digitalWrite(contactor, HIGH);
    myNex.writeStr("g0.txt", "Inverter Is Off");
  }
}