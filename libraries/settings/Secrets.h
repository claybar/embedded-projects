// WiFi setup
#define AP_SSID "xxxxxx"
#define AP_PASSWORD "xxxxxx"

// MQTT setup
#define MQTT_SERVER_IP { 000, 000, 000, 000 }
#define MQTT_USERNAME "USERNAME"
#define MQTT_PASSWORD "PASSWORD"

// Time keeping
#define NTP_SERVER_IP { 000, 000, 000, 000 }

//US Eastern Time Zone (New York, Detroit)
TimeChangeRule myDST = {"EDT", Second, Sun, Mar, 2, -240};    //Daylight time = UTC - 4 hours
TimeChangeRule mySTD = {"EST", First, Sun, Nov, 2, -300};     //Standard time = UTC - 5 hours


//const int timeZone = 1;     // Central European Time
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

