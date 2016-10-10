/*
 * Inputs ADC Value from Thermistor and outputs Temperature in Celsius
 *  requires: include <math.h>
 * Utilizes the Steinhart-Hart Thermistor Equation:
 *    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]3}
 *    where A = 0.001129148, B = 0.000234125 and C = 8.76741E-08
 *
 * These coefficients seem to work fairly universally, which is a bit of a
 * surprise.
 *
 * Schematic:
 *   [Ground] -- [10k-pad-resistor] -- | -- [thermistor] --[Vcc (5 or 3.3v)]
 *                                               |
 *                                          Analog Pin 0
 *
 * In case it isn't obvious (as it wasn't to me until I thought about it), the analog ports
 * measure the voltage between 0v -> Vcc which for an Arduino is a nominal 5v, but for (say)
 * a JeeNode, is a nominal 3.3v.
 *
 * The resistance calculation uses the ratio of the two resistors, so the voltage
 * specified above is really only required for the debugging that is commented out below
 *
 * Resistance = PadResistor * (1024/ADC -1)  
 *
 * I have used this successfully with some CH Pipe Sensors (http://www.atcsemitec.co.uk/pdfdocs/ch.pdf)
 * which be obtained from http://www.rapidonline.co.uk.
 *
 */
//preparation for temp-measurement
#include <math.h>

#define ThermistorPIN 1                 // Analog Pin 1
#

float vcc = 4.91;                       // only used for display purposes, if used
                                        // set to the measured Vcc.
float pad = 9850;                       // balance/pad resistor value, set this to
                                        // the measured resistance of your pad resistor
float thermr = 10000;                   // thermistor nominal resistance

float Thermistor(int RawADC) {
  long Resistance;  
  float Temp;  // Dual-Purpose variable to save space.

  Resistance=pad*((1024.0 / RawADC) - 1);
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.001129148 + (0.000234125 * Temp) + (0.0000000876741 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      

  // BEGIN- Remove these lines for the function not to display anything
  //Serial.print("ADC: ");
  //Serial.print(RawADC);
  //Serial.print("/1024");                           // Print out RAW ADC Number
  //Serial.print(", vcc: ");
  //Serial.print(vcc,2);
  //Serial.print(", pad: ");
  //Serial.print(pad/1000,3);
  //Serial.print(" Kohms, Volts: ");
  //Serial.print(((RawADC*vcc)/1024.0),3);  
  //Serial.print(", Resistance: ");
  //Serial.print(Resistance);
  //Serial.print(" ohms, ");
  // END- Remove these lines for the function not to display anything

  // Uncomment this line for the function to return Fahrenheit instead.
  //temp = (Temp * 9.0)/ 5.0 + 32.0;                  // Convert to Fahrenheit
  return Temp;                                      // Return the Temperature
}
//preparation for lcd hield
#include <LiquidCrystal.h>

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);        // select the pins used on the LCD panel

unsigned long tepTimer ;  
unsigned long currTime;


int heatPin = 3;                 // Heating connected to digital pin 3

void setup() {
  pinMode(heatPin, OUTPUT);      // sets the digital pin as output
  Serial.begin(115200);
  lcd.begin(16, 2);                       // start the library
}

//TODO: Command: Insert Batch. notFlipped=1; notDone=1;
//TODO: calculate totalTime 
//TODO: set targetTemp;

float hystHight=5; // 5°C Hystersis to reduce the switching on/off of the relay
float targetTemp=75; //Target sensor readout temp which equals the optimum temp of the frying pan
long totalTime=180000;  // complete time for one pancake is 180s
long flipTime=80000;  //time until flip is 1m 20s
/*int SollZeit=360;
int Wenden=180;
int IstZeit=0;*/

boolean notFlipped=1;
boolean notDone=1;
boolean control=0;

boolean calculateControl(float temp,boolean oldControl){
  // Serial.println(temp);
  //  Serial.println(targetTemp);
  if (temp<targetTemp)
    {control=1;
    digitalWrite(heatPin, HIGH);   // switches the heating on
    Serial.println("Heizen!");  //tells to heat via the serial port
    }
 /* else if ((temp-hystHight)<targetTemp && oldControl)
    control=1;*/
  else 
  {
    control=0;
    digitalWrite(heatPin, LOW);    // turns the heating off
    Serial.println("Kuehlen!");    // tells to turn the heating off via serial port
  }
  
  Serial.println(control);  
}

/*void setControl(boolean control){
  if (control==1) {
      digitalWrite(heatPin, HIGH);   // sets the heating on
      Serial.println("Heizen!");   
  }
  else 
  {  
      digitalWrite(heatPin, LOW);    // sets the heating off
      Serial.println("Kuehlen!");   
  }
}
*/
void loop() {
  
  float temp;
  temp=Thermistor(analogRead(ThermistorPIN));    // read ADC and  convert it to Celsius

/*  
  do  {
   temp=Thermistor(analogRead(ThermistorPIN));    // read ADC and  convert it to Celsius
   delay(1000);
 } while (temp<targetTemp);*/

  currTime=millis();
  if ((currTime>flipTime) && notFlipped==1) { // Tell user when its time to flip
  //der ganze prozess soll eh nur einmal durchlaufen werden, ich würde die variable notFlipped weglassen und nur abfragen wann currTime
    Serial.println("Flip it!");
    lcd.setCursor(2,1);
    lcd.print("Flip it!");
    delay(1000);
    notFlipped=0;
  }
  if ((currTime>totalTime) && notDone==1) { // Tell user when pancake is done
    Serial.println("Enjoy ;)");
    lcd.setCursor(2,1);
    lcd.print("Enjoy ;)");
    digitalWrite(heatPin, LOW);    // sets the heating off
    delay(5000000);
    notDone=0;
  }
  delay(100);
//setControl(control);
  control=calculateControl(temp, control);
  lcd.setCursor(0, 0);                   // set the LCD cursor   position 
  Serial.print("Celsius: ");
  Serial.print(temp,1);   
  //to do: Plausibilitätsprüfung um Messfehler wie 300°C etc rauszufiltern. (würde nur einmal verkehrt heizen/kühlen)

  Serial.println("");                                  
  delay(1000);                                      // Delay a bit...
//  IstZeit=IstZeit+1;
  lcd.print("T: ");               
  lcd.print(temp);             
  lcd.print("C");  
}



