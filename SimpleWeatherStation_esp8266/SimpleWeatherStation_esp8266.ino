/*
   BME280とNJL7502Lを使った温度・湿度・気圧+照度ロガー
   NJL7502LをADコンバーターMCP3002で読む。
   RTCモジュールを使わず、deepSleep直前の時刻とdeepSleep時間から
   deepSleep後の時刻を計算する

   立ち上がったらRTCメモリーからRTCメモリーからdeepSleep直前の時刻と
   deepSleep時間を読み、現在時刻を計算してシステムに設定(setTime)
   BME280の値を読む
   restartの値により処理を分ける
   restart==0  wi-fi onで起動。valid==trueならデーターを送信。
   ++restartし、バッファーに時刻とBME280の値を書く
*/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <arduino.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <ESP8266WiFi.h>
//#include "WiFi.h"
//#include <WebServer.h>
//#include <DNSServer.h>
//#include <WiFiManagerESP32.h>
#include <WiFiManager.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include "MCP3002.h"
#include "Totem.h"
#include "zach_aes.h"
//#include "esp_deep_sleep.h"
#include "NTPClient.h"
#include <WiFiUdp.h>
#include "Event.h"
#include "DHT.h"
//#include "driver/rtc_io.h"

// ###### TFT ######
#define TFT_CS     15
#define TFT_RST    5  // you can also connect this to the Arduino reset
// in which case, set this #define pin to -1!
#define TFT_DC     4

// For 1.44" and 1.8" TFT with ST7735 use
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
#define TFT_SCLK 13   // set these to be whatever pins you like!
#define TFT_MOSI 11   // set these to be whatever pins you like!

void TFTdrawString(int x, int y, char *text) {
  tft.setCursor(x, 3+y * 8);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextWrap(false);
  tft.print("                  ");
  tft.setCursor(x, 3+y * 8);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.print(text);
}
//############  AM2320   ############
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

Adafruit_AM2320 am2320 = Adafruit_AM2320();

//############  EventManager   ############
EventManager evtManager;

//############  Include Example   ############
extern "C" {
#include "user_interface.h"
  //#include "sntp.h"
}
//############  Zach   ############
unsigned int channelId = 4824;
int sendDataToAmbient(bool valid);
void sendEmail(const char *a, const char *b, const char *c);

Totem ambient;
Totem zachmail;
//############  Wifi   ############
char writeKey[100] = "86fe0a8edf672236";
char accountID[100] = "1";

//char ssid[64] = "GAIN";
char ssid[100] = "";
//char password[64] = "tinylake85WpQm09W8";
char password[100] = "";
IPAddress ip(192, 168, 2, 7);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient client;
/** WiFiUDP class for NTP server */
WiFiUDP ntpUDP;
/** NTP client class */
NTPClient timeClient(ntpUDP);
//############  SNTP   ############
time_t getSntpTime(void);

//############  CRC   ############
uint32_t calcCRC32(const uint8_t *data, size_t length);

//############  GPIO   ############
#define RESTART_BUTTON      0
#define RSET_BUTTON         12
#define LED_PIN             2
#define DHT22_PIN           5
#define DHTTYPE DHT22
#define LED 4
#define MCP_CS 5
#define BME_CS 15

//############  AES   ############
#define ENCFLG 1
#define DECFLG 2
int enc(char *plane,  int len);
void DbgPrint(char *str);

//############  Debug   ############
#define _DEBUG 1
#if _DEBUG
#define DBG(...) { Serial.print(__VA_ARGS__); }
#define DBGLED(...) { digitalWrite(__VA_ARGS__); }
#else
#define DBG(...)
#define DBGLED(...)
#endif /* _DBG */

void DbgPrint(char *str)
{
  DBG(str);
}
//############  Restart   ############
#define MaxRestart 6
#define PERIOD (60)
#define COMP 0.978

//############  Data   ############
#pragma pack(push)
#pragma pack(4)
struct OneData_t {
  time_t created;
  double temp;
  double humid;
  double pressure;
  double illumi;
};

#define MaxDataArea  (512 - sizeof(uint32_t) - sizeof(int) - sizeof(time_t))
#define MaxBlocks    (MaxDataArea / sizeof(OneData_t))

struct Data {
  uint32_t crc32;
  int    restart;
  time_t tbefore;
  time_t sleeptime;
  struct OneData_t blocks[MaxBlocks];
};

union {
  struct Data data;
  byte buffer[512];
} rtcData;

bool state = 0;

#pragma pack(pop)

//############  DHT   ############

DHT DHTam2302(DHT22_PIN, DHTTYPE);

double DHTReadTemperature( DHT _useDHT )
{
  int ii = 0;
  float num = _useDHT.readTemperature(true);
  for (ii = 0; ii < 5 && isnan(num); ii++) {
    delay(1000);
    num = _useDHT.readTemperature(true);
  }

  double temp  = 5.0 / 9.0 * (num - 32.0); // fahrenheit -> celsius
  return temp;
}
double DHTReadHumidity( DHT _useDHT )
{
  int ii = 0;
  float num = _useDHT.readHumidity(true);
  for (ii = 0; ii < 5 && isnan(num); ii++) {
    delay(1000);
    num = _useDHT.readHumidity(true);
  }
  return num;
}
/**
   Initialize NTP client
*/
void initNTP() {
  // Start NTP listener
  timeClient.begin();
  timeClient.setTimeOffset(9 * 3600);
  // Force update of time from NTP server
  timeClient.forceUpdate();
}

//############  Interrupt   ############
volatile int interruptCounter = 0;
volatile int interruptResetCounter = 0;
int reset_counter = 0;
void handleInterrupt() {
  interruptCounter = 1;
  digitalWrite(LED_PIN, HIGH);
}
void handleResetInterrupt() {
  interruptResetCounter = 1;
  digitalWrite(LED_PIN, HIGH);
}
//############  EEPROM   ############
//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback (WiFiManager *myWiFiManager) {
  DBG("Should save config");
  shouldSaveConfig = true;
  
        TFTdrawString(0, 3, "Now AP mode      ");
}
//############  IP   ############
String ipToString(IPAddress ip) {
  String s = "";
  for (int i = 0; i < 4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}
//############  MainTask   ############
struct MainTask : public EventTask
{
  using EventTask::execute;

  void execute(Event evt)
  {
    bool valid;  //  RTCメモリーの値が有効かどうかのフラグ
    int restart;  //  何回目の起動かのカウンター。0の時はWi-Fiオンで起動されている
    time_t currenttime;
    double sleeptime;
    time_t sntptime;
    char temp128[128];
    initNTP();
    int t = millis();

    //mcp3002.begin(MCP_CS);
    // RTCメモリーからデーターを読む
    ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData));

    uint32_t crcOfData = calcCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);

    if ((crcOfData == rtcData.data.crc32) && 0 <= rtcData.data.restart && rtcData.data.restart < MaxRestart) {
      restart = rtcData.data.restart;
      valid = true;
      sleeptime = (double)rtcData.data.sleeptime * COMP;
      currenttime = rtcData.data.tbefore + (time_t)sleeptime;
    } else {
      restart = 0;
      valid = false;
      sleeptime = 0;
      currenttime = 0;
    }
    TFTdrawString(0, 3, (char *)"start          ");

    setTime(currenttime);
#ifdef _DEBUG
    Serial.begin(115200);
    delay(10);
#endif
    DBG("\r\nStart\r\n");
    if (valid) {
      DBG("sleeptime: "); DBG(rtcData.data.sleeptime);
      DBG(", tbefore: "); DBG(rtcData.data.tbefore); DBG("\r\n");
    }
    //  Wi-Fi接続より先にセンサーを読んでしまう

    double temp = 0, humid = 0, pressure = 0, illumi = 0;
    double temp2 = 0, humid2 = 0;
    double temp3 = 0, humid3 = 0;
    int temp_count = 0, humid_count = 0;

    DHTam2302.begin();    //  Wi-Fi接続には時間がかかり、測定周期がぶれるので、

    TFTdrawString(0, 3, (char *)"readTemperature");
    temp =  DHTReadTemperature( DHTam2302 );
    DBG(temp); DBG("\n");
    if ( isnan(temp) ) {
      temp = 0;
    } else {
      temp_count++;
    }
    if ( temp_count ) {
      temp = (temp) / temp_count;
    }

    TFTdrawString(0, 3, "readHumidity   ");
    humid = DHTReadHumidity(DHTam2302);
    DBG(humid); DBG("\n");

    if ( isnan(humid) ) {
      humid = 0;
    } else {
      humid_count++;
    }
    if ( humid_count ) {
      humid = (humid) / humid_count;
    }

    int indx = restart - 1;
    if (indx < 0) {
      indx = MaxRestart - 1;
    }

    rtcData.data.blocks[indx].created = now();
    rtcData.data.blocks[indx].temp = temp;
    rtcData.data.blocks[indx].humid = humid;
    rtcData.data.blocks[indx].pressure = pressure;
    rtcData.data.blocks[indx].illumi = illumi;

    DisplayLCD_temphum(temp, humid);

    DBG("currentTime: "); DBG(currenttime);
    DBG(", temp: "); DBG(temp);
    DBG(" DegC,  humid: "); DBG(humid);
    DBG(" %, pressure: "); DBG(pressure);
    //DBG(" hPa, illumi: "); DBG(illumi);
    DBG(" Lux\r\n");
    delay(200);

    DisplayLCD_restart(restart);

    DBG("valid: "); DBG(valid);
    DBG(", restart: "); DBG(restart); DBG("\r\n");

    if (restart == 0 ) {
      //##### read configuration from FS json #####
      Serial.println("mounting FS...");

      if (SPIFFS.begin()) {
        Serial.println("mounted file system");
        if (SPIFFS.exists("/config.json")) {
          //file exists, reading and loading
          Serial.println("reading config file");
          File configFile = SPIFFS.open("/config.json", "r");
          if (configFile) {
            Serial.println("opened config file");
            size_t size = configFile.size();
            // Allocate a buffer to store contents of the file.
            std::unique_ptr<char[]> buf(new char[size]);

            configFile.readBytes(buf.get(), size);
            DynamicJsonBuffer jsonBuffer;
            JsonObject& json = jsonBuffer.parseObject(buf.get());
            json.printTo(Serial);
            if (json.success()) {
              Serial.println("\nparsed json");

              const char* tempssid = json["mqtt_ssid"];
              strcpy( ssid, tempssid);

              const char* temppassword = json["mqtt_password"];
              strcpy( password, temppassword);

              const char* tempwriteKey = json["mqtt_writeKey"];
              strcpy( writeKey, tempwriteKey);
              //json["mqtt_writeKey"].printTo((char*)writeKey, json["mqtt_writeKey"].measureLength() + 1);
              const char* tempaccountID = json["mqtt_accountID"];
              strcpy( accountID, tempaccountID);
              //json["mqtt_accountID"].printTo((char*)accountID, json["mqtt_accountID"].measureLength() + 1);

            } else {
              Serial.println("failed to load json config");
            }
          }
        }
      } else {
        Serial.println("failed to mount FS");
      }
      DBG("\n### Connect To ###\n");
      DBG("ssid = ");
      DBG(ssid);
      DBG("   password = ");
      DBG(password);
      DBG("\n");
      //end read
      WiFi.mode(WIFI_STA);
      //WiFi.begin("","");
      WiFi.begin(ssid, password);
      //接続が確立するまで、・・・を表示
      int ii = 0;
      //接続が確立するまで、・・・を表示
      sprintf(&(temp128[0]), "now connecting");
      TFTdrawString(0, 3, temp128);
      //delay(500);

      int connect_count = 0;
      while ((WiFi.status() != WL_CONNECTED) && (connect_count <= 10)) {
        connect_count++;
        delay(1000);
        DBG(".");
        memset(temp128, 0x20, sizeof(temp128));
        for (ii = 0; ii < connect_count; ii++) {
          sprintf(&(temp128[ii]), ".");
        }
        TFTdrawString(0, 3, temp128);
      }

      //if (WiFi.status() != WL_CONNECTED) {
      sprintf(&(temp128[0]), "now connecting");
      TFTdrawString(0, 3, temp128);
      //##### Wifi Parameter #####
      WiFiManagerParameter custom_mqtt_writeKey("writeKey", "mqtt writeKey", writeKey, 62);
      WiFiManagerParameter custom_mqtt_accountID("accountID", "mqtt accountID", accountID, 62);

      //WiFiManager
      WiFiManager wifiManager;
      //Local intialization. Once its business is done, there is no need to keep it around
      wifiManager.setConnectTimeout(5);


      //set config save notify callback
      wifiManager.setAPCallback(saveConfigCallback);

      //add all your parameters here
      wifiManager.addParameter(&custom_mqtt_writeKey);
      wifiManager.addParameter(&custom_mqtt_accountID);

      if ( !wifiManager.autoConnect("ZachConnectAP") ) {
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(1000);
      }

      DBG( wifiManager.getSSID() );
      String str;
      str = wifiManager.getSSID();
      str.toCharArray(ssid, str.length()+1); 

      DBG( wifiManager.getPassword() );
      str = wifiManager.getPassword();
      str.toCharArray(password, str.length()+1); 

      /*
                if (!wifiManager.startConfigPortal("ZachConnectAP")) {
                Serial.println("failed to connect and hit timeout");
                delay(3000);
                //reset and try again, or maybe put it to deep sleep
                ESP.restart();
                delay(5000);
                }
      */
      sprintf(&(temp128[0]), "connected     ");
      TFTdrawString(0, 3, temp128);

      //exit after config instead of connecting
      //wifiManager.setBreakAfterConfig(true);

      //set static ip
      //wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 2, 99), IPAddress(192, 168, 2, 99), IPAddress(255, 255, 255, 0));



      //if you get here you have connected to the WiFi
      Serial.println("connected...yeey :)");

      //read updated parameters
      strcpy(writeKey , custom_mqtt_writeKey.getValue());
      strcpy(accountID, custom_mqtt_accountID.getValue());

      Serial.println("saved");
      //save the custom parameters to FS
      if (shouldSaveConfig) {
        shouldSaveConfig = 0;
        Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["mqtt_ssid"] = ssid;
        json["mqtt_password"] = password;
        json["mqtt_writeKey"] = writeKey;
        json["mqtt_accountID"] = accountID;

        File configFile = SPIFFS.open("/config.json", "w");
        if (!configFile) {
          Serial.println("failed to open config file for writing");
        }

        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        //end save
        DBG("\n");
      }
      //} else {
      //  sprintf(&(temp128[0]), "connected      ");
      //  TFTdrawString(0, 3, temp128);
      //}


      /*
            WiFi.mode(WIFI_STA);
            //WiFiを繋ぐ前に、WiFi状態をシリアルに出力
            //WiFi.printDiag(Serial);
            WiFi.begin(ssid, password);  //  Wi-Fi APに接続
            delay(100);
            WiFi.reconnect();
      */
      DBG("autoConnect DONE\n");
      //接続が確立するまで、・・・を表示
      DBG("WiFi connected\r\nIP address: ");
      DBG(WiFi.localIP()); DBG("\r\n");

      sprintf(&(temp128[0]), "connected       ");
      TFTdrawString(0, 3, temp128);

      time_t sntptime;  //  SNTPで時刻取得
      do {
        myprintJST();
        sntptime = getSntpTime();  //  SNTPで時刻取得
        delay(100);
      } while ( year() < 2018 );
      DisplayLCD_IP(ipToString(WiFi.localIP()));
    }
    myprintJST();

    if (restart == 0 && valid) {
      DBG("Send to Zach\n");
      int sret = sendDataToAmbient(valid);
      if ( sret == -1 ) {
        const char email_subject[] = "エラー：SimpleWeatherStation";
        const char email_BCC[] = "";
        const char email_body[] = "fail to send.";
        sendEmail(email_subject, email_BCC, email_body);
      }
      WiFi.disconnect();
    }

    if (++restart >= MaxRestart) {
      restart = 0;
    }

    rtcData.data.restart = restart;
    rtcData.data.tbefore = now();

    t = millis() - t; // t: リスタート直後からの経過時間(ミリ秒)
    t = (t < PERIOD * 1000) ? (PERIOD * 1000 - t) : 1;  // sleeptime(ミリ秒)
    DBG("sleep time = ");
    DBG(t);
    DBG("\n");

    rtcData.data.sleeptime = (time_t)((double)t / 1000.0 / COMP);
    rtcData.data.crc32 = calcCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);

    ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData));
    //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_AUTO);
    //gpio_pullup_en(RESTART_BUTTON);    // use pullup on GPIO
    //gpio_pulldown_dis(RESTART_BUTTON); // not use pulldown on GPIO

    //if (esp_deep_sleep_enable_timer_wakeup(t * 1000 / COMP) == ESP_OK) {
    //  esp_deep_sleep_enable_ext0_wakeup(RESTART_BUTTON, 0); // 指定したGPIOがLowの時に起動
    //  DBG("esp_deep_sleep_enable_ext0_wakeup done.\n");
    //}

    //if (esp_sleep_enable_timer_wakeup(t * 1000 / COMP) == ESP_OK) {
    //  esp_sleep_enable_ext0_wakeup(RESTART_BUTTON, 0); // 指定したGPIOがLowの時に起動
    //  DBG("esp_sleep_enable_ext0_wakeup done.\n");
    //}
    //delay(t / 1000.0 / COMP / 10);

    //esp_deep_sleep_start(); // スリープモード実行
    //if (esp_light_sleep_start() != ESP_OK) { // ESP_ERR_INVALID_STATE for WiFi or BT is not stopped
    //  DBG("deepsleep error\n");
    ///}
    DBG("Loop DONE\n");
    TFTdrawString(0, 3, "Done           ");
  }
} MainTask;

void setup() {
  //rtc_gpio_init(LED_PIN);//< 初期化
  //rtc_gpio_set_direction(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);//< 出力に設定
  //rtc_gpio_hold_en(LED_PIN); //< 持続を有効化
  pinMode(LED_PIN, OUTPUT);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 0.96" 180x60 TFT
  //tft.initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display

  tft.fillScreen(ST77XX_BLACK);

  TFTdrawString(0, 3, "mcp3002.begin   ");

  //timer = timerBegin(0, 80, true);
  //timerAttachInterrupt(timer, &RTCDisp, true);
  //timerAlarmWrite(timer, 1 * 1000, true);
  //timerAlarmEnable(timer);

  pinMode(RESTART_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESTART_BUTTON), handleInterrupt, FALLING);

  pinMode(RSET_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RSET_BUTTON), handleResetInterrupt, FALLING);

  /*
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RESTART_BUTTON, handleInterrupt, (void*) RESTART_BUTTON);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(RSET_BUTTON, handleResetInterrupt, (void*) RSET_BUTTON);
  */
  evtManager.subscribe(Subscriber("event.keepAlive", &MainTask));
  Event keepAlive = Event("event.keepAlive");
  evtManager.triggerInterval(TimedTask(PERIOD * 1000, keepAlive));

}

void DisplayLCD_time()
{
  char temp128[128];
  sprintf(&(temp128[0]), "%02d/%02d/%02d    ",  year(), month(), day());
  TFTdrawString(0, 4, temp128);
  sprintf(&(temp128[0]), "%02d:%02d:%02d    ", hour(), minute(), second());
  TFTdrawString(0, 5, temp128);
}
void DisplayLCD_IP(String IP)
{
  TFTdrawString(0, 6, "               ");
  DBG("DisplayLCD_IP(String IP)");
  DBG(IP);
  TFTdrawString(0, 6, (char *)IP.c_str());
}

void DisplayLCD_temphum(double temp, double humid)
{
  char temp128[128];
  sprintf(&(temp128[0]), "temp  = %3.1f", temp);
  TFTdrawString(0, 0, temp128);
  sprintf(&(temp128[0]), "humid = %3.1f", humid);
  TFTdrawString(0, 1, temp128);
}

void DisplayLCD_restart(int restart)
{
  char temp128[128];
  sprintf(&(temp128[0]), "restart = %d", restart);
  TFTdrawString(0, 2, temp128);
}

void DisplayLCD_delay(int ltime)
{
  char temp128[128];
  sprintf(&(temp128[0]), "time=%02d  rst=%02d", ltime, reset_counter);
  TFTdrawString(0, 7, temp128);
}


void loop()
{
  int skipflg = 0;
  int resetflg = 0;
  DisplayLCD_time();
  if (interruptCounter == 1) {
    interruptCounter = 0;
    skipflg = 1;
  }

  if ( 1 == interruptResetCounter ) {
    interruptResetCounter = 0;
    resetflg = 1;
  }

  if ( 1 == resetflg  ) {
    reset_counter++;
    DBG("!!!!! RESET counter ++ !!!!!");
    DBG(reset_counter);
    if ( reset_counter >= 3 ) {
      DBG("!!!!! RESET !!!!!");
      TFTdrawString(0, 2, "reset         ");
      WiFi.disconnect();
      WiFi.begin("0", "0");
      delay(100);
    }
  }

  digitalWrite(LED_PIN, LOW);

  int ltime = evtManager.tick(skipflg);
  DisplayLCD_delay(ltime / 1000);
}

uint32_t calcCRC32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void myprintJST() {
  DBG("JST is ");
  DBG(year()); DBG('/');
  DBG(month()); DBG('/');
  DBG(day()); DBG(' ');
  DBG(hour()); DBG(':');
  DBG(minute()); DBG(':');
  DBG(second());
  DBG("\r\n");
}

time_t getSntpTime() {
  configTime(9 * 3600, 0, "ntp.nict.jp", NULL, NULL);
  if (timeClient.update()) {
    setTime(timeClient.getEpochTime());
  }
  time_t timeInfo;
  localtime(&timeInfo);
  return timeInfo;
}

#define BUFSIZE 1024

char buffer[BUFSIZE];
char destbuffer[BUFSIZE];
char tempbuffer[BUFSIZE];

int sendDataToAmbient(bool valid) {
  int i;
  time_t created;
  int vb;
  char vbbuf[12];

  DBG("Ambient.send() to: "); DBG(channelId); DBG("\r\n");

  DBG(", accountID: "); DBG(accountID);
  DBG(" , writeKey: "); DBG(writeKey); DBG("\r\n");
  ambient.begin(60000, writeKey, &client);

  sprintf(&(buffer[0]), "{\"accountID\":\"%s\",\"writeKey\":\"%s\",\"data\":[", accountID , writeKey);
  //DBG("buffer: " ); DBG(buffer); DBG("\r\n");
  if (valid) { // もしRTCデーターがvalidなら全部のデーターを送信
    i = 0;
  } else { // そうでないなら[MaxRestart - 1]のスロットだけ送信
    i = MaxRestart - 1;
  }
  for (; i < MaxRestart; i++) {
    created = rtcData.data.blocks[i].created;
    sprintf(&buffer[strlen(buffer)], "{\"created\":%d,\"time\":1,\"d1\":%3.1f,\"d2\":%3.1f,\"d3\":%3.1f,\"d4\":%3.1f,\"d5\":%3.1f,\"d6\":%3.1f,\"d7\":%3.1f,\"d8\":%3.1f,\"d9\":%3.1f},",
            created,
            rtcData.data.blocks[i].temp,    // d1
            rtcData.data.blocks[i].humid,   // d2
            rtcData.data.blocks[i].pressure,// d3
            rtcData.data.blocks[i].illumi,  // d4
            0.0, // d5
            0.0, // d6
            0.0, // d7
            0.0, // d8
            0.0  // d9
           );

    //DBG("buffer: " ); DBG(buffer); DBG("\r\n");
  }

  buffer[strlen(buffer) - 2] = '\0';
  sprintf(&buffer[strlen(buffer)], "}]}", vbbuf);

  DBG("buf: "); DBG(strlen(buffer)); DBG(" bytes\r\n");

  int n = ambient.bulk_send(buffer, strlen(buffer));
  DBG("sent: "); DBG(n); DBG("\r\n");
  DBG(buffer); DBG("\r\n");
  return n;
}

void sendEmail(const char *email_subject, const char *email_BCC, const char *email_body) {
  int i;
  const char emailsendpass[16 + 1] = "zaker578d/23ej78";
  unsigned char emailsendpassvelify[32];
  memset(emailsendpassvelify, 0x00, 32);
  //sha1(emailsendpass, 16, emailsendpassvelify);
  memcpy((void *)emailsendpass, (const void*)"\x01\x02\x13\x24\x75\x56\x07\x38\x09\x2A\x2B\x1C\x7D\x2E\x3F\x10\x4\x3\x5\x4\x5\x4\x3\x2\x8\x9\x0\x8\x0\x9\x0\x2\x0\x9", (size_t)32);
  const char email_fromaddr[] = "tktomaru.fs@gmail.com";
  const char email_password[] = "pass";
  const char email_toaddr[] = "tktomaru@gmail.com";
  //const char email_subject[] = "ErrMail";
  //const char email_BCC[] = "";
  //const char email_body[] = "test mail";

  DBG("zachmail.send() to: "); DBG(50000); DBG("\r\n");
  DBG("pass: "); DBG(emailsendpass); DBG("\r\n");
  DBG("email_fromaddr: "); DBG(email_fromaddr);
  DBG("email_toaddr: "); DBG(email_toaddr); DBG("\r\n");
  zachmail.begin(50001, writeKey, &client);

  memset(buffer, 0x0, sizeof(buffer));
  for (i = 0; i < 16; i++) {
    sprintf(&(buffer[strlen(buffer)]), "%02x", emailsendpassvelify[i]);
  }
  sprintf(&(buffer[strlen(buffer)]), "{");
  //sprintf(&(buffer[0]), "{\"p\":\"");
  //for (i = 0; i < 16; i++) {
  //  sprintf(&(buffer[strlen(buffer)]), "%02x", emailsendpassvelify[i]);
  //}
  sprintf(&(buffer[strlen(buffer)]), "\"t\":\"%d\",\"accountID\":\"%s\",\"pass\":\"%s\",\"data\":[", now(), accountID , emailsendpass);
  i = 0;
  for (; i < 1; i++) {
    sprintf(&buffer[strlen(buffer)], "{\"from_addr\":\"%s\",\"password\":\"%s\",\"to_addr\":\"%s\",\"subject\":\"%s\",\"BCC\":\"%s\",\"body\":\"%s\"},",
            email_fromaddr,
            email_password,
            email_toaddr,
            email_subject,
            email_BCC,
            email_body
           );
  }
  buffer[strlen(buffer) - 2] = '\0';
  sprintf(&buffer[strlen(buffer)], "}]}");
  int enclen = strlen(buffer);
  int paddingsize = 16 - (enclen % 16);
  for (i = 0; i < paddingsize; i++) {
    sprintf(&buffer[strlen(buffer)], " ");
  }

  DBG(buffer); DBG("\r\n");

  DBG("buf: "); DBG(enclen); DBG(" bytes\r\n");

  unsigned char key[16 + 1] = "z0cH3D-2/D1kT6V3";
  unsigned char iv[16 + 1] =  "13D93j=^/2D34Dff";
  dump( "key", (unsigned char*)key, 16 );
  dump( "iv", (unsigned char*)iv, 16 );
  memset(destbuffer, 0x0, sizeof(destbuffer));
  dump( "buffer", (unsigned char*)buffer, enclen + paddingsize );
  DBG("paddingsize: "); DBG( paddingsize); DBG(" bytes\r\n");
  DBG("buffer: "); DBG(enclen + paddingsize); DBG(" bytes\r\n");
  enc_dec(ENCFLG, key, iv,  (unsigned char*)buffer,  (unsigned char*)destbuffer, enclen + paddingsize);
  //dump( "ENCRYPTED", (unsigned char*)destbuffer, enclen + paddingsize );

  int n = zachmail.bulk_send(destbuffer, enclen + paddingsize);
  DBG("sent: "); DBG(n); DBG("\r\n");
  dump( "sent buffer", (unsigned char*)destbuffer, n );
}
void dump( char *label, unsigned char *block , int len)
{
  int i;

  memset(&(tempbuffer[0]), 0x01, sizeof(tempbuffer));
  sprintf(&(tempbuffer[0]), "%s : " , label);

  for ( i = 0; i < len; i++ ) {
    sprintf(&tempbuffer[strlen(tempbuffer)],  "%02x", block[i]);
  }
  DBG(tempbuffer); DBG("\r\n");
}

int enc_dec(int flg, unsigned char *key, unsigned  char *iv, unsigned  char *plane, unsigned  char *dest, int len)
{
  aes_context ctx[1];

  aes_set_key_zach( (unsigned char const*)key, 16, ctx );
  if (flg == ENCFLG) {
    aes_cbc_encrypt_zach( (const unsigned char *)plane, (unsigned char *) & (dest[0]),
                          len / 16, (unsigned char *)iv, (const aes_context*) ctx );
  } else {
    aes_cbc_decrypt_zach( (const unsigned char *)plane, (unsigned char *) & (dest[0]),
                          len / 16, (unsigned char *)iv, (const aes_context*) ctx );
  }

  return ( 0 );
}

int encTest()
{
  unsigned char key[16 + 1] = "0123456789ABCDEF";
  unsigned char data[16 + 1] = "abcdefghijklmnop";
  aes_context ctx[1];
  unsigned char encrypted[N_BLOCK], decrypted[N_BLOCK];

  unsigned char iv[16 + 1] = "\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0";
  unsigned char lbuffer[16 + 1] = "\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0";
  unsigned char lbuffer2[16 + 1] = "\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0\x0";

  dump( "key", (unsigned char*)key, 16 );

  aes_set_key_zach( (unsigned char const*)key, 16, ctx );
  dump( "DATA", (unsigned char*)data, 16 );

  aes_encrypt_zach(  (unsigned char const*)data, encrypted, ctx );
  dump( "ENCRYPTED", encrypted, 16 );

  aes_decrypt_zach( (unsigned char const*)encrypted, decrypted, ctx );
  dump( "DECRYPTED", (unsigned char*)decrypted, 16 );

  aes_cbc_encrypt_zach( (const unsigned char *)data, (unsigned char *) & (lbuffer[0]),
                        sizeof(lbuffer) / 16, (unsigned char *)iv, (const aes_context*) ctx );

  dump( "ENCRYPTED CBC", (unsigned char*)lbuffer, 16 );

  memset( iv, 0x00, sizeof(iv) );
  aes_cbc_decrypt_zach( (const unsigned char *)lbuffer, (unsigned char *) & (lbuffer2[0]),
                        sizeof(lbuffer) / 16, (unsigned char *)iv, (const aes_context*) ctx );

  return ( 0 );
}
