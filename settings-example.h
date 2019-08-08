// MQTT settings
#define MQTT_TOPIC "demosensor"
#define MQTT_SERVER "mqtt.example.org"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt_user_with_read-write_permission_to_topic"
#define MQTT_PASSWORD "mqtt_password"
#define MQTT_MAX_PACKET_SIZE 256 // For pubsubclient, default is 128 B

#define SENSOR_SEND_MAX_DELAY 60000 // milliseconds, after this send data anyway even it has not changed at all
#define STATUS_SEND_DELAY 60000

// Sensor settings

// Buttons (digital HIGH / LOW)
#define PUSHBUTTON_1 D7
#define PUSHBUTTON_2 D8
#define PUSHBUTTON_SEND_DELAY 100

#define BME680_HEATING_TIME 150 // milliseconds
#define BME680_SEND_DELAY 30000 // milliseconds

#define BME280_SEND_DELAY 30000 // milliseconds

#define SDS011_SEND_DELAY 1000 // milliseconds
#define SDS011_RXPIN 14
#define SDS011_TXPIN 12

// I2C settings
#define SDA     4
#define SCL     5
