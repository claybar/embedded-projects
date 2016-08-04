unsigned int version;

#define ADDR_COM_SETTINGS_OFFSET      (0)     // settings starting address
struct commonSettings0_t
{
  const uint8_t version = 0;
  char deviceName[12];  // Also used as mqttClientID
  char mqttTopicBase[20];
  char mqttWillTopic[20];
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

#define ADDR_FS_SETTINGS_OFFSET   (512)   // settings starting address
struct frontstepsControllerSettings0_t
{
  const uint8_t version = 0;
  unsigned long lightingAfterMotionTime;  // milliseconds
  uint8_t lightingLevelOff; // percentage
  uint8_t lightingLevelAmbient; // percentage
  uint8_t lightingLevelBright;  // percentage
};
