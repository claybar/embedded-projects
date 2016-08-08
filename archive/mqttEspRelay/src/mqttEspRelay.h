void setup_wifi();

void serialMQTTRelay();
void serialUnrecognized(const char *command);

void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttReconnect();
