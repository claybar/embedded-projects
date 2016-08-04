unsigned int version;

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

/** EEPROM data addresses */
#define FRONTSTEPS_SETTINGS_VERSION         0
#define ADDR_FS_SETTINGS_OFFSET             (512)   // settings starting address
#define ADDR_FS_VERSION                     (ADDR_FS_SETTINGS_OFFSET)                                     // uint8_t
#define ADDR_FS_LIGHTING_AFTER_MOTION_TIME  (ADDR_FS_VERSION + sizeof(uint8_t))                           // unsigned long
#define ADDR_FS_LIGHTING_OFF_PERCENTAGE     (ADDR_FS_LIGHTING_AFTER_MOTION_TIME + sizeof(unsigned long))  // uint8_t
#define ADDR_FS_LIGHTING_AMBIENT_PERCENTAGE (ADDR_FS_LIGHTING_OFF_PERCENTAGE + sizeof(uint8_t))           // uint8_t
#define ADDR_FS_LIGHTING_BRIGHT_PERCENTAGE  (ADDR_FS_LIGHTING_AMBIENT_PERCENTAGE + sizeof(uint8_t))       // uint8_t

/*
struct frontstepsControllerSettings0_t
{
  const uint8_t version = 0;
  unsigned long lightingAfterMotionTime;  // milliseconds
  uint8_t lightingLevelOff; // percentage
  uint8_t lightingLevelAmbient; // percentage
  uint8_t lightingLevelBright;  // percentage

};
*/
