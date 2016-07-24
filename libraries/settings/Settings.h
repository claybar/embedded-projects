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
  uint16_t sunlightThreshold;  // level of sunlight to turn lights on
  uint8_t insideLightsBrightness;
  uint8_t outsideLightsBrightness;
};

struct frontstepsControllerSettings0_t
{
  const uint8_t version = 0;
  unsigned long lightingAfterMotionTime;  // milliseconds
  uint8_t lightingLevelOff; // percentage
  uint8_t lightingLevelAmbient; // percentage
  uint8_t lightingLevelBright;  // percentage
  int lightingChangeTime;  // milliseconds
};
