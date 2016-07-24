int serial_putchar(char c, FILE* f);
byte readI2CRegister(byte i2c_address, byte reg);

void lightingOff();
void lightingAmbient();
void lightingBright();

boolean mqttConnect();
void mqttSubscribe(const char* name);
void mqttPublish(const char* name, const char* payload);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
