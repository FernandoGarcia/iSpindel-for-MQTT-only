It's a modified version of iSpindel 6.0.0 by Samuel Lang available at: <https://github.com/universam1/iSpindel>

It was modified to support only MQTT.

It's configured to connect directly to my broker at: <https://ferduino.com>

You should select the better NTP server for your region and place the url in global variable `NTP_SERVER`
You can find more informations here: <https://ntppool.org>

Defaut board is an ESP-WROOM-02 motherboard.

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
