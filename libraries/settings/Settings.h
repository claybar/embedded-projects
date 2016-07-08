unsigned int version;

struct commonSettings0_t
{
  const uint8_t version = 0;
  char deviceName[12];  // Also used as mqttClientID
  char mqttTopicBase[12];
  char mqttWillTopic[16];
  char mqttWillMessage[16];
};

struct bikeshedControllerSettings0_t
{
  const uint8_t version = 0;
  unsigned long outsideLightAfterMotionTime;  // milliseconds
};