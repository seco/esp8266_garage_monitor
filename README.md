# esp8266_garage_monitor

/* Garage monitor and door controller: 17 Apr 2016 

  This controls an ESP8266 in the garage wich monitors 
  temperature, humidity and garage door position.  All items
  are reported back via MQTT to a Raspberry Pi running OpenHAB.
  
  There is also a relay attached that can be triggered 
  remotely to open or close the garage door.
  
  Tested and installed this afternoon (17 Apr 2016).
  
  Modified 1 May 2016: Updated DHT sensor to DHT22 and removed serial output.
  Modified 16 May 2016 to add pressure and additional temp measure from BMP180.
  Modified 30 May 2016 to add an automatic reset for DHT22
  
  DO NOT FORGET TO INITIALIZE NON-STANDARD SDA/SCL PIN SETUP IN Wire.begin(sda, scl).

*/
