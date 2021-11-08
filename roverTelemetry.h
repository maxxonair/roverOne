
// MQTT
const char* mqtt_server       = "192.168.0.198";  
const char* distance_topic    = "home/marvMellow/distance";
const char* mqtt_username     = "box"; // MQTT username
const char* mqtt_password     = "box"; // MQTT password
const char* clientID          = "client_roomBox_01"; // MQTT client ID
/*String containing Hostname, Device Id & Device Key in the format:                         */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
static const char* connectionString = "HostName=192.168.0.198;DeviceId=marvin;SharedAccessKey=marvinInTheHouse";

const int port = 1883;

const String serverIP = "192.168.0.198";

Telemetry telemetry;
