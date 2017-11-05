# BME680_V2_NANO_433

Introducing BOSCH sensortec BME680 sensor to FHEM (https://fhem.de/commandref.html#intro).

This sensor includes 4 measument Temperatur, Humidity, Air-Pressure and CO2 relative resitance.

Datasheet(s):
https://download.mikroe.com/documents/datasheets/BME680.pdf
https://www.mikrocontroller.net/attachment/249626/Bosch_Sensortec_Flyer_BME680.pdf


Indoor Air Quality - Handbook:
http://www.tsi.com/uploadedFiles/_Site_Root/Products/Literature/Handbooks/IAQ_Handbook_2011_A4_5001020-web.pdf


See code for more detail upon sensor-tx-ID etc. and used Arduino Pins.

Remark:

The calculated IAQ-Indoor-Air-Quality Index is not calculated inside the Sensor but delivered
as a precompiled library  at them Moment fpr three 32-bit processor architectures: ARM, ESP, RL.

The iAQ-Calculation is therefore kept as "secret", not making it possible to adapt these resistance-values to IAQ or PPM.  

BME-specific code may be found at: https://github.com/BoschSensortec

Breakout-Board:
https://www.amazon.de/BlueDot-BME680-Sensor-f%C3%BCr-Arduino/dp/B075Y21H24

