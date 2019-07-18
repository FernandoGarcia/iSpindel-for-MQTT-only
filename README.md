# iSpindel - *DIY tilt hydrometer*  [![Build Status](https://travis-ci.com/FernandoGarcia/iSpindel-for-MQTT-only.svg?branch=master)](https://travis-ci.com/FernandoGarcia/iSpindel-for-MQTT-only)

It's a modified version of iSpindel 6.0.0 by Samuel Lang available at: <https://github.com/universam1/iSpindel>

It was modified to support only MQTT.

It's configured to connect directly to my broker at: <https://ferduino.com>

*The board used for this build is an ESP-WROOM-02 motherboard. This board has high power consumption and is not viable to battery powered project. So is needed find a way to make it more efficient.*

Bellow you can find more details about this type of issue:

https://tinker.yeoman.com.au/2016/05/29/running-nodemcu-on-a-battery-esp8266-low-power-consumption-revisited/

https://jasiek.me/2016/04/24/running-wemos-nodemcu-off-a-18650-battery.html

Pins used are:

D2 - D1 for I2C;

D7 for DS18B20;

A0 for battery meter.

The parameters bellow can be changed via MQTT message in JSON format.

Upload interval: {"interval": 120}. Range: 11 ~ 3599 S;

Battery factor: {"factor": 191.8}. Range: any value;

Temperature unit: {"unit": "c"}. Options: "f" to Fahrenheit and "c" to Celsius.

Daylight saving time: {"summer": "false"}. Options: "true" to enable and "false" to disable;

Timezone: {"timezone": -3}. Range: -12 ~ +12

Important: You can change just a parameter per message.

Assigned topics:

Log: tilt/log/username/password

Configuration: tilt/config/username/password

Response: tilt/response/username/password

## 3D printed parts
Sled: http://www.thingiverse.com/thing:3075683

Mold to cast lead counterweight: http://www.thingiverse.com/thing:3082397

## Build details

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/1.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/2.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/3.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/4.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/5.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/6.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/7.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/8.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/9.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/10.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/11.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/12.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/13.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/14.jpg">

<img width="50%" src="https://github.com/FernandoGarcia/iSpindel-for-MQTT-only/blob/master/Build%20details/15.jpg">
