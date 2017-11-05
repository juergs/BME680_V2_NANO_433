/*====================================================================================================================================
*    Arduino Nano Project: BME680_V2_NANO_433.ino
*    Author: juergs for FHEM  https://forum.fhem.de/index.php/topic,78619.45.html
*    Release-Date: 1.11.2017
*    Version V2.1 for arduino nano 16Mhz
*
*    ***********************************************************************************************************************************
*
*    sensor-tec-BSEC-Software: https://www.bosch-sensortec.com/bst/products/all_products/BSEC
*    From Basic.ino using      https://github.com/vicatcu/BME680_Breakout/blob/master/BME680_Library/examples/Basic/Basic.ino
*    Luftdruck-Kalibrierung:   https://github.com/DFRobot/DFRobot_BME680/blob/master/examples/DFRobot_BME680_I2C/DFRobot_BME680_I2C.ino
*    Arduino Plotter:          http://www.instructables.com/id/Ultimate-Guide-to-Adruino-Serial-Plotter/
*    powerful Data-Viewer(java): TelemetryViewer_v0.4
*
==================================================================================================================================== */
#include <Wire.h>
#include <TimerOne\TimerOne.h>  
#include ".\utility\BME680_Library.h"
#include ".\utility\movingAvg.h" // uint32
#include ".\utility\movingAvgFloat.h" // float
#include ".\utility\LaCrosse.h"

#define LACROSSE_SENSOR_ID_BASIS        90   //--- results in 2 Lacrosse instances with two cannles each for for measument data
#define DELAY_PAUSE_PERIOD              10000  // im ms = 30s 
#define DELAY_PAUSE_PERIOD_SEC          30     // im s = 10s 
#define DELAY_TIMER_CYCLES              10     // im s = 10s 
#define PIN_ENABLE_TIMESTAMP_IN_OUTPUT  10     // jumper nach Masse schaltet Timestamp in der Ausgabe aus.  

// #define ESP_CONFIG_ENABLE true
#define CALIBRATE_PRESSURE

//--- Vars
float seaLevel;
movingAvg IAQ; 
movingAvgFloat tquer;
movingAvgFloat pquer;
movingAvgFloat hquer;

BME680_Library bme680;
bool isBme680_initialized = false; 

volatile float bme680_temperature = 0.0;
volatile float bme680_humidity = 0.0;
volatile float bme680_pressure = 0.0;
volatile float bme680_gas = 0.0;


//----------------------------------------------------------------------------------------------
int64_t get_timestamp_us()
{
    //int64_t system_current_time = 0;
    // ...
    // Please insert system specific function to retrieve a timestamp (in microseconds)
    // ...
    return (int64_t)millis() * 1000;
}
void print_timestamp()
{
    //--- a jumper to GND on D10 does the job! 
    if (digitalRead(PIN_ENABLE_TIMESTAMP_IN_OUTPUT) == LOW)
    {
        Serial.print("[");
        Serial.print(get_timestamp_us() / 1e6);
        Serial.print("] ");
    }
}
void executeLaCrosseSend()
{
    static uint8_t counter = 0; 

    counter++; 
    
    if (digitalRead(PIN_ENABLE_TIMESTAMP_IN_OUTPUT) == LOW && counter > DELAY_TIMER_CYCLES)
    {
        print_timestamp();
        Serial.println(F("Timer-Interrupt!"));
    }

    if (isBme680_initialized && counter > DELAY_TIMER_CYCLES)
    {
        counter = 0; 

        //*********************************************
        //--- LaCrosse-Protocol for 433 MHz-TX 
        //--- transfer measured values through LaCrosse-TX2-instance
        //--- LaCrosse-object declared as static
        LaCrosse.bSensorId = LACROSSE_SENSOR_ID_BASIS;
        LaCrosse.t = bme680_temperature;         //--- alias temperature;  
        LaCrosse.h = bme680_humidity;    //--- alias humidity;  
        LaCrosse.sendTemperature();
        LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */
        LaCrosse.sendHumidity();
        
        LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */

        //--- zweite ID liefert die CO2-Gaswerte + den Luftdruck/10 
        LaCrosse.bSensorId = LACROSSE_SENSOR_ID_BASIS + 1;
        LaCrosse.t = (float) bme680_gas / 10000;
        LaCrosse.sendTemperature();
        
        LaCrosse.sleep(1);        /* 1 second, no power-reduction! see impact on powersave */

        //LaCrosse.bSensorId = LACROSSE_SENSOR_ID_BASIS + 2;
        LaCrosse.t = bme680_pressure / 10.0;        //--- alias humidity;
        LaCrosse.sendTemperature();
        
        LaCrosse.sleep(1);        /* 1 second delay, no power-reduction! */
    }
}
//----------------------------------------------------------------------------------------------
void setup(void) 
{
    Serial.begin(115200);
    Serial.println("*Setup...");

    pinMode(PIN_ENABLE_TIMESTAMP_IN_OUTPUT, INPUT_PULLUP);

    //Timer1.initialize((unsigned long) DELAY_PAUSE_PERIOD_SEC * 1000 * 1000); // yS in Sekunden
    Timer1.initialize((unsigned long)30000000); // yS in Sekunden
    Timer1.attachInterrupt(executeLaCrosseSend);

    //--- preset SensorId & TX instance 
    LaCrosse.bSensorId = 100;       // Basis: es werden 2 LaCrosseIds verwendet. Id2 = Id1+1!
    LaCrosse.setTxPinMode(OUTPUT);
  
    Wire.begin();

    print_timestamp(); 
    Serial.print("* BME Initialization...");
    if (bme680.begin()) 
    {
        print_timestamp();
        Serial.print("* Succeeded!");
    }
    else 
    {
        print_timestamp();
        Serial.print("* Failed!");
        for (;;); // spin forever
    }
    Serial.println();

    print_timestamp();
    Serial.print("* Configuring Forced Mode...");
    if (bme680.configureForcedMode()) 
    {
        print_timestamp();
        Serial.print("* Succeeded!");
    }
    else 
    {
        print_timestamp();
        Serial.print("* Failed!");
        for (;;); // spin forever
    }

    #ifdef CALIBRATE_PRESSURE
        //bme680.startConvert();
        //delay(1000);
        //bme680.update();
        ///*You can use an accurate altitude to calibrate sea level air pressure.
        //*And then use this calibrated sea level pressure as a reference to obtain the calibrated altitude.
        //*In this case, 184.0m is  Karlsruhe, Germany altitude.
        //*/
        //seaLevel = bme680.readSeaLevel(184.0);
        //Serial.print(F("seaLevel :"));
        //Serial.println(seaLevel);
    #endif


    Serial.println();

    print_timestamp();
    Serial.println(F("Temperature(degC),Relative_Humidity(%),Pressure(hPa),Gas_Resistance(Ohms), Gas_Resistance_Average (Ohms)"));
}
//----------------------------------------------------------------------------------------------
void loop(void) 
{
    bme680.configureForcedMode(); // otherwise you get BME680_W_NO_NEW_DATA warning code?
    if (bme680.read()) 
    {
        isBme680_initialized = true; 

        tquer.reading(bme680.getTemperature());
        pquer.reading(bme680.getBarometricPressure());
        hquer.reading(bme680.getRelativeHumidity());
        IAQ.reading(bme680.getGasResistance());

        bme680_temperature = tquer.getAvg(); 
        bme680_humidity = hquer.getAvg(); 
        bme680_pressure = pquer.getAvg(); 
        bme680_gas = IAQ.getAvg(); 

        print_timestamp();         
        Serial.print(bme680_temperature, 2);
        Serial.print(F(","));
        Serial.print(bme680_humidity, 2);
        Serial.print(F(","));
        Serial.print(bme680_pressure, 2);
        Serial.print(F(","));
        Serial.print(bme680.getGasResistance());  // raw-Wert
        Serial.print(F(","));
        Serial.print(IAQ.getAvg());  // int
        Serial.println();        
    }
    else 
    {
        isBme680_initialized = false; 
        Serial.println("BME680 Read Failed! No TX!");
    }

    delay(1000);

}
//----------------------------------------------------------------------------------------------
