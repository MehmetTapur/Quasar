/*
 * References:
 * 1-https://www.electronicshub.org/esp32-ble-tutorial/
 * 2-https://randomnerdtutorials.com/esp32-mqtt-publish-subscribe-arduino-ide/
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MQTT_Config.h"
#include "BLEDevice.h"
#include <Dictionary.h>

#define SIZE 200


//dictionary settings
#define _DICT_KEYLEN 40
#define _DICT_VALLEN 254

#define MQTTsubQos          1  
#define LEDdefined          0

#if LEDdefined

  #define MQTT_LED 2
  #define BLE_LED 12
  #define WAR_LED 27


#endif
TimerHandle_t One_Shot_Timer;
Dictionary &dict = *(new Dictionary());

/*MAC addres*/
const char MacAddr_mr[]= "df:33:66:23:98:01";
                   
char UUID_Value_mr[]= "D1410000-67F5-479E-8711-B3B99198CE6C";

const char *MacAddr[15]= {"ff:33:66:23:98:01","ff:33:66:23:98:02","ff:33:66:23:98:03","ff:33:66:23:98:04", \
                    "ff:33:66:23:98:05","ff:33:66:23:98:06","ff:33:66:23:98:07","ff:33:66:23:98:08", \
                    "ff:33:66:23:98:09","ff:33:66:23:98:10","ff:33:66:23:98:11","ff:33:66:23:98:12", \
                    "ff:33:66:23:98:13","ff:33:66:23:98:14","ff:33:66:23:98:15" \
                    };
                   
                   
char *UUID_Value[15]= { "E1410000-67F5-479E-8711-B3B99198CE6C","E2410000-67F5-479E-8711-B3B99198CE6C","E3410000-67F5-479E-8711-B3B99198CE6C","E4410000-67F5-479E-8711-B3B99198CE6C", \
                  "E5410000-67F5-479E-8711-B3B99198CE6C","E6410000-67F5-479E-8711-B3B99198CE6C","E7410000-67F5-479E-8711-B3B99198CE6C","E8410000-67F5-479E-8711-B3B99198CE6C", \
                  "E9410000-67F5-479E-8711-B3B99198CE6C","EA410000-67F5-479E-8711-B3B99198CE6C","EB410000-67F5-479E-8711-B3B99198CE6C","EC410000-67F5-479E-8711-B3B99198CE6C", \
                  "ED410000-67F5-479E-8711-B3B99198CE6C","EE410000-67F5-479E-8711-B3B99198CE6C","EF410000-67F5-479E-8711-B3B99198CE6C" \
                };


/*Task Handle*/
TaskHandle_t MQTTTask_Handle;
TaskHandle_t BLETask_Handle;
/*Semaphore Handle*/
static SemaphoreHandle_t barrier;

/*
 * Pivate variables
 */

/* Specify the Service UUID of Server */
static BLEUUID deviceUUID("");
/* Specify the Service UUID of Server */
static BLEUUID serviceUUID("");
/* Specify the Characteristic UUID of Server */
static BLEUUID charUUID("");


/*boolean variables*/
static boolean  flagmr_t = false;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static boolean smr_flag = false;
static boolean  flag_t = false;

bool rst_flag = false;

bool s_flag = false;
size_t i = 0;

bool topic_flag = false; //0 = smartdesk / 1 = smartroom
/*ble variables*/
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;
BLEClient*  pClient;
BLERemoteService* pRemoteService;


String id_val;
String uuid_val;
String uuid_event;
String event_str;



/*private structure*/
typedef struct event {
  char device_uuid_val[37];
  char service_uuid_val[37];
  char char_uuid_val[37];
  char event_status_val[37];
  char event_time_val[37];
  char event_name_val[37];
  char event_dataToBe_sent[80];
} event_t;

event_t evt;

typedef struct message {
  char device_uuid_val[37];
  char service_uuid_val[37];
  char char_uuid_val[37];
  char employee_id_val[55];
  uint8_t d_number;
  char d_numberchar[2];
} message_t;

message_t msg;


WiFiClient espClient;
PubSubClient mqttClient(espClient);

/*Function Prtototypes*/
void BLE_Task(void *param);
void MQTT_Task(void *param);
void connectToWiFi(void);
void setupMQTT(void);
void callback(char* topic, byte* payload, unsigned int length) ;
void reconnect(void);
static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
bool connectToServer();



static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.println((char*)pData);
}

class MyClientCallback : public BLEClientCallbacks 
{
  
  void onConnect(BLEClient* pclient)
  {   
  }

  void onDisconnect(BLEClient* pclient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};
void OneShotTimerCallback(TimerHandle_t xTimer)
{
  mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
  mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
  delay(1000);

 // Serial.println(" geldi");
    ESP.restart();
    
   // CurrentTime = xTaskGetTickCount();
    //sprintf(string,"One-Shot Timer: %d\n",CurrentTime);
   // Uart_Print(string);
}
bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");
  xTimerStart(One_Shot_Timer,0);
  pClient->setClientCallbacks(new MyClientCallback());

    /* Connect to the remote BLE Server */
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

    /* Obtain a reference to the service we are after in the remote BLE server */
   pRemoteService = pClient->getService(serviceUUID);
   xTimerStop( One_Shot_Timer, 0 );
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  /* Obtain a reference to the characteristic in the service of the remote BLE server */
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  /* Read the value of the characteristic */
  /* Initial value is 'Hello, World!' */
  if(pRemoteCharacteristic->canRead())
  {
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if(pRemoteCharacteristic->canNotify())
  {
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  }

    connected = true;
    return true;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
 /* Called for each advertising BLE server. */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    /* We have found a device, let us now see if it contains the service we are looking for. */
    /*buraya mac addresini de ekleyebilirsin*/

    if(topic_flag)
    {
      if ((advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(deviceUUID))&&(strncmp(advertisedDevice.getAddress().toString().c_str(),MacAddr_mr,strlen(MacAddr_mr))==0))
      {
        Serial.print("===Found ESL with matching device UUID==");
        Serial.println(MacAddr_mr);
                doConnect = true;
        doScan = true;
        BLEDevice::getScan()->stop();
       myDevice = new BLEAdvertisedDevice(advertisedDevice); 

        doConnect = true;
        doScan = true;

        delay(100);

  
      }
    }
    else{
       if ((advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(deviceUUID))&&(strncmp(advertisedDevice.getAddress().toString().c_str(),MacAddr[msg.d_number-1],strlen(MacAddr[msg.d_number-1]))==0))
       {
        Serial.print("===Found ESL with matching device UUID==");
        Serial.println(MacAddr[msg.d_number-1]);
       doConnect = true;
        doScan = true;
        BLEDevice::getScan()->stop();
        myDevice = new BLEAdvertisedDevice(advertisedDevice); 
        doConnect = true;
        doScan = true;
  
      }
      
    }

  }

};


void reconnect(void)
{
  int nofConTried=0;
    // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(CLIENT_ID, MQTT_USER_NAME, MQTT_PASSWORD)) 
    {
      Serial.println("MQTT Broker Connected.");
      #if LEDdefined
      digitalWrite(MQTT_LED, HIGH);
      #endif
     // digitalWrite(MQTT_LED, HIGH);
      //subscribe to topic
      mqttClient.subscribe("/o1/m/esl",MQTTsubQos);
      mqttClient.subscribe("/o1/esl",MQTTsubQos);
      mqttClient.subscribe("/o1/esl-controller",0);
    }
    else {
      //MQTT Could not reconnect, wifi/esp32 error
      #if LEDdefined
      digitalWrite(MQTT_LED, LOW);
      #endif
      nofConTried++;
      Serial.print("Connection failed, rc=");
     // digitalWrite(MQTT_LED, LOW);
      Serial.println(mqttClient.state());
      Serial.println("Retrying in 1 second\n");
      
      if (nofConTried > 5) {
        Serial.print("Rebooting the device...");
        ESP.restart();
      }
    }
    delay(1000);
  
  }
}




void setupMQTT(void)
{
  mqttClient.setServer(MQTT_SERVER_NAME, MQTT_PORT);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive(60);
  
}

void connectToWiFi()
{
  
  delay(10);
  uint8_t i =0;
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    i++;
   Serial.print(".");
   if (i > 10) {
        Serial.println("Rebooting the device...");
        i=0;
        ESP.restart();      
      }
    
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
}
void callback(char* topic, byte* payload, unsigned int length)
{

  char desk_number[2]={0};
  const char *delimeter = ";";
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char message[SIZE];
  
  for (size_t i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message[i]= (char)payload[i];
  }
  message[length]= '\0';
  Serial.println();

   if (strcmp(topic, "/o1/m/esl") == 0)//meeting room
  {
  //  int desk_n = 0;
    topic_flag =true;
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);

    Serial.println();
    memset(&evt,0,sizeof(evt));
    event_str.clear();
    
    strcpy(evt.device_uuid_val, UUID_Value_mr);
    strcpy(evt.service_uuid_val, evt.device_uuid_val);
    evt.service_uuid_val[7] = '1';
    strcpy(evt.char_uuid_val, evt.device_uuid_val);
    evt.char_uuid_val[7] = '2';


    
    strcpy(evt.event_status_val, strtok(message, delimeter));
    strcpy(evt.event_time_val, strtok(NULL, delimeter));
    strcpy(evt.event_name_val, strtok(NULL, delimeter));

      Serial.print("Device uuid: "); Serial.println(evt.device_uuid_val);
      Serial.print("Service uuid: "); Serial.println(evt.service_uuid_val);
      Serial.print("Characteristic uuid: "); Serial.println(evt.char_uuid_val);
      Serial.print("Event Status: "); Serial.println(evt.event_status_val);
      Serial.print("Event Time: "); Serial.println(evt.event_time_val);
      Serial.print("Name: "); Serial.println(evt.event_name_val);
      uuid_event = evt.device_uuid_val;
      
      strcpy(evt.event_dataToBe_sent,evt.event_status_val);
      strcat(evt.event_dataToBe_sent, "^");
      strcat(evt.event_dataToBe_sent, evt.event_time_val);    
      strcat(evt.event_dataToBe_sent, "^");
      event_str = strcat(evt.event_dataToBe_sent, evt.event_name_val);
      event_str.replace(" ","^");
      if (dict[uuid_event] != event_str) {
        // Transmit the message data to queue.
        flagmr_t = true;
      } else {
        Serial.println("The charactheristic was the same... Doing nothing...");
      }
  }
  else if (strcmp(topic, "/o1/esl") == 0)//smart desk
  {
    int desk_n = 0;
    topic_flag =false;
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);

    Serial.println();
   /* for (size_t i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message[i] = (char)payload[i];
    }*/
    memset(&msg,0,sizeof(msg));
    strcpy(desk_number, strtok(message,delimeter));
     strcpy(msg.d_numberchar, desk_number);
    desk_n = atoi(desk_number);
    msg.d_number= (uint8_t) desk_n;
    strcpy(msg.device_uuid_val, UUID_Value[desk_n-1]);
    strcpy(msg.service_uuid_val, msg.device_uuid_val);
    msg.service_uuid_val[7] = '1';
    strcpy(msg.char_uuid_val, msg.device_uuid_val);
    msg.char_uuid_val[7] = '2';
    strcpy(msg.employee_id_val, strtok(NULL, delimeter));

      Serial.print("Device uuid: "); Serial.println(msg.device_uuid_val);
      Serial.print("Service uuid: "); Serial.println(msg.service_uuid_val);
      Serial.print("Characteristic uuid: "); Serial.println(msg.char_uuid_val);
      Serial.print("Employee ID: "); Serial.println(msg.employee_id_val);

      uuid_val = msg.device_uuid_val;
      id_val = msg.employee_id_val;
      if (dict[uuid_val] != id_val) {
        // Transmit the message data to queue.
        flagmr_t = true;
      } else {
        Serial.println("The charactheristic was the same... Doing nothing...");
      }
    }
    else
    {
        if(strncmp(message,"esl-controller reset",20)==0)
        {
          id_val = message;
           ESP.restart();
        }
         
      
    }

    
}
  
/*
*@brief This task always listen MQTT topic. When a message received, semaphore gives
*/
void MQTT_Task(void *param)
{
  Serial.println("MQTT_Task init running...");
   BaseType_t rc;
  int count = 0;
  while(1)
  {
    if(!mqttClient.connected()) {
      Serial.println("Reconnecting to the broker..");
      reconnect();
    }
    mqttClient.loop();
    if(flagmr_t == true)
    {
      flagmr_t = false;
      Serial.println("Released a semaphore");
      rc = xSemaphoreGive(barrier);
      vTaskSuspend( MQTTTask_Handle );
    }
      
  }
  
}

void BLE_Task(void *param)
{

  BaseType_t rc;
  static int cnt = 0;
  static int fail_cnt = 0;
  while(1)
  {
    rc=xSemaphoreTake(barrier, portMAX_DELAY);   
    Serial.println("Obtained a semaphore");
    /*for meeting room*/
     if (topic_flag) {
      deviceUUID= BLEUUID(evt.device_uuid_val);
      serviceUUID = BLEUUID(evt.service_uuid_val);
      charUUID = BLEUUID(evt.char_uuid_val);
    }
    else {
      deviceUUID = BLEUUID(msg.device_uuid_val);
      serviceUUID = BLEUUID(msg.service_uuid_val);
      charUUID = BLEUUID(msg.char_uuid_val);
    }
  //  Serial.println("1");
    // Scan the BLE devices.
    BLEScan* pBLEScan = BLEDevice::getScan();
   //  Serial.println("2");
    // Connect to ESL according to the UUID value in the Queue.
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  //   Serial.println("3");
    delay(1000);
     //Serial.println("4");
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(45, false);

   if (doConnect == true)
   {
      if(connectToServer()){
        Serial.println("We are now connected to the BLE Server.");
        fail_cnt = 0;
        #if LEDdefined

        digitalWrite(BLE_LED, HIGH);

        #endif
      }
      else{
        Serial.println("We have failed to connect to the server; there is nothin more we will do.1");
        fail_cnt++;
        if(fail_cnt<3)
        {
          smr_flag =false;
          s_flag=false;
          xSemaphoreGive(barrier);
        }
          
        else
        {
        fail_cnt=0;
        if(topic_flag)
        {
          smr_flag =false;
          mqttClient.publish("/o1/esl/warning", "!");
          mqttClient.publish("/o1/esl/warning", "!");
          delay(50);//new
        } 
        else
        {
          s_flag=false;
          mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
          mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
          delay(50);
        }
          
        dict(uuid_val, "error");
        
        #if LEDdefined
        digitalWrite(BLE_LED, LOW);
        digitalWrite(WAR_LED, HIGH);
        delay(250);
        digitalWrite(WAR_LED, LOW);
        delay(250);
        digitalWrite(WAR_LED, HIGH);
        delay(250);
        digitalWrite(WAR_LED, LOW);
        #endif
        
        vTaskResume( MQTTTask_Handle );
        }

      }
      
      doConnect = false;
   }else{
       Serial.println("We have failed to connect to the server; there is nothin more we will do.2");
        fail_cnt++;
       if(fail_cnt<3)
        {
          smr_flag =false;
          s_flag=false;
          xSemaphoreGive(barrier);
        }
          
        else
        {
          fail_cnt=0;
          if(topic_flag)
          {
            smr_flag =false;
            mqttClient.publish("/o1/esl/warning", "!");
            mqttClient.publish("/o1/esl/warning", "!");
            delay(50);
          } 
          else
          {
            s_flag =false;
            mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
            mqttClient.publish("/o1/esl/warning", msg.d_numberchar);
             delay(50);
          }
         dict(uuid_val, "error");
  
          #if LEDdefined
  
          digitalWrite(BLE_LED, LOW);
          digitalWrite(WAR_LED, HIGH);
          delay(500);
          digitalWrite(WAR_LED, LOW);
  
          #endif
         vTaskResume( MQTTTask_Handle );
        }
   } 
  if(connected)
   {
    String newString = "";
    if(topic_flag)
    {
        if(strlen(evt.event_dataToBe_sent)>20){
        char temp[21]={0};
        if(smr_flag == false)
        {
          strncpy(temp, evt.event_dataToBe_sent, 20);
          temp[19] = '.';
          newString = temp;
          newString.replace(" ", "^");
          smr_flag = true;
          rst_flag = true;
          xSemaphoreGive(barrier);
        }
        else{
          memset(temp,'\0',sizeof(temp));//do not need!!!
          strncpy(temp, evt.event_dataToBe_sent+19, 20);
          newString = temp;
          newString.replace(" ", "^");
          smr_flag = false;
          rst_flag = false;
          cnt +=2;
        }
        
      }
      else {
       newString = evt.event_dataToBe_sent;
       newString.replace(" ", "^");
       smr_flag=false;
       rst_flag=false;
       cnt ++;
       }
          newString.replace("ğ", "g");
          newString.replace("ç", "c");
          newString.replace("ü", "u");
          newString.replace("ö", "o");
          newString.replace("ş", "s");
          newString.replace("Ğ", "G");
          newString.replace("Ç", "C");
          newString.replace("Ü", "U");
          newString.replace("Ö", "O");
          newString.replace("Ş", "S");
          Serial.println("xxxxxxxx");
          Serial.println(newString);
          Serial.println("xxxxxxxx");
          dict(uuid_event, event_str);
          pRemoteCharacteristic->writeValue(newString.c_str(), newString.length());
          
          pClient->disconnect();
            delay(100);
          #if LEDdefined
          digitalWrite(BLE_LED, LOW);
          #endif
         if(smr_flag == false)
         {
            if(cnt >= 1 && (rst_flag == false))
            {
              cnt = 0; // :))))
              ESP.restart();
            }
           vTaskResume( MQTTTask_Handle );
         }
            
        
     
    }
    else{
        if(strlen(msg.employee_id_val)>19){
       // Serial.println("!!!!!!!!employee_id_val");
        char temp[21]={0};
        if(s_flag == false)
        {
          strncpy(temp, msg.employee_id_val, 20);
          temp[19] = '.';
          newString = temp;
          newString.replace(" ", "^");
          s_flag = true;
         // Serial.println("!s_flag == false");
         rst_flag = true;
          xSemaphoreGive(barrier);
        }
        else{
          strncpy(temp, msg.employee_id_val+19, 20);
          strcat(temp,"-");
          newString = temp;
          newString.replace(" ", "^");
          s_flag = false;
          rst_flag = false;
          cnt +=2;
          
          
        }
        
      }
      else {
       char temp[21]={0};
       strcpy(temp, msg.employee_id_val);
       strcat(temp,"&");
       newString = temp;
       newString.replace(" ", "^");
       s_flag=false;
       rst_flag=false;
       cnt ++;
       }
          newString.replace("ğ", "g");
          newString.replace("ç", "c");
          newString.replace("ü", "u");
          newString.replace("ö", "o");
          newString.replace("ş", "s");
          newString.replace("Ğ", "G");
          newString.replace("Ç", "C");
          newString.replace("Ü", "U");
          newString.replace("Ö", "O");
          newString.replace("Ş", "S");
          Serial.println("xxxxxxxx");
          Serial.println(newString);
          Serial.println("xxxxxxxx");
          dict(uuid_val, id_val);
          pRemoteCharacteristic->writeValue(newString.c_str(), newString.length());
          
          pClient->disconnect();
            delay(100);
          #if LEDdefined
          digitalWrite(BLE_LED, LOW);
          #endif
          if(s_flag == false)
          {
            if(cnt >= 1 && (rst_flag == false))
            {
              cnt = 0; // :))))
              ESP.restart();
            }  
            vTaskResume( MQTTTask_Handle );
          }
            
       
        
    }

   }   
  }
}



void setup() {
  Serial.begin(115200);
  One_Shot_Timer = xTimerCreate("One-Shot",             // Software timer's name
                                   pdMS_TO_TICKS(20000),   // Period
                                   pdFALSE,               // One-shot mode
                                   0,                     // Timer id
                                   OneShotTimerCallback); // Callback function

  #if LEDdefined

    pinMode(MQTT_LED, OUTPUT);
    digitalWrite(MQTT_LED, LOW);
    pinMode(BLE_LED, OUTPUT);
    digitalWrite(BLE_LED, LOW);
    pinMode(WAR_LED, OUTPUT);
    digitalWrite(WAR_LED, LOW);

  
  #endif

  int app_cpu = xPortGetCoreID();
  BaseType_t rc;
  connectToWiFi();
  setupMQTT();

  BLEDevice::init("");

  delay(2000);
  barrier = xSemaphoreCreateBinary();
  assert(barrier);

  
  rc = xTaskCreatePinnedToCore(
         MQTT_Task,
         "MQTTTask",
         10000, // Stack Size
         NULL,
         1, // Priortiy
         &MQTTTask_Handle,// Task Handle
         app_cpu // CPU
       );
  assert(rc == pdPASS);
  assert(MQTTTask_Handle);

  rc = xTaskCreatePinnedToCore(
         BLE_Task,
         "BLETask",
         10000, // Stack Size
         NULL,
         1, // Priortiy
         &BLETask_Handle, // Task Handle
         app_cpu  // CPU
       );
  assert(rc == pdPASS);
  assert(BLETask_Handle);

  Serial.println("setup running...");
  


  
  
}


void loop() {

  vTaskDelete(nullptr);
}
