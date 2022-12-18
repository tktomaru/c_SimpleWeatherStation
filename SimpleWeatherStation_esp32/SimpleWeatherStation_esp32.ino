/*
   AM2320を使った温度・湿度ロガー
   RTCモジュールを使わず、deepSleep直前の時刻とdeepSleep時間から
   deepSleep後の時刻を計算する

   立ち上がったらFlashメモリーからdeepSleep直前の時刻と
   deepSleep時間を読み、現在時刻を計算してシステムに設定(setTime)
   AM2320の値を読む
   restartの値により処理を分ける
   restart==0  valid==trueならデーターを送信。
   ++restartし、バッファーに時刻とAM2320の値を書く
*/
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <SPIFFS.h>
#include <arduino.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManagerESP32.h>
#include <SPI.h>
#include <Time.h>
#include <TimeLib.h>
#include "Zackert.h"
#include "zach_aes.h"
#include "esp_deep_sleep.h"
#include <U8x8lib.h>
#include "NTPClient.h"
#include <WiFiUdp.h>
#include "Event.h"
#include "mbedtls/md.h"

#include <Wire.h>
#include <Adafruit_SHT31.h>


void DisplayLCD_time();
void DisplayLCD_IP(String IP);
void DisplayLCD_temp(double temp);
void DisplayLCD_hum(double humid);
void DisplayLCD_restart(int restart);
void DisplayLCD_delay(int ltime);

#define debug_zach 0 // 0 運用   1 ローカルテスト
// ###### the OLED used ######
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
int LCD_PRINT = 1;

//############  EventManager   ############
EventManager evtManager;

//############  Include Example   ############
extern "C" {
#include "CalcCRC.h"
#include "CTime.h"
}

//############  Zach   ############
int sendDataTozach(bool valid);
void sendEmail(const char *a, const char *b, const char *c);

Zackert zach;
Zackert zachmail;
//############  Wifi   ############
char writeKey[100] = "input writeKey";
char accountID[100] = "input accountID";

char ssid[100] = "input ssid";
char password[100] = "input ssid password";
IPAddress ip(192, 168, 2, 8);
IPAddress gateway(192, 168, 2, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiClient client;
/** WiFiUDP class for NTP server */
WiFiUDP ntpUDP;
/** NTP client class */
//NTPClient timeClient(ntpUDP);
NTPClient timeClient(ntpUDP, "ntp.nict.jp", 0);
//############  SNTP   ############
unsigned long getSntpTime(void);

//############  GPIO   ############
#define HARD_RESET          GPIO_NUM_25
#define LCD_BUTTON          GPIO_NUM_26 // blue
#define RESTART_BUTTON      GPIO_NUM_27 // green
#define LED_PIN             GPIO_NUM_14 // red
#define RSET_BUTTON         GPIO_NUM_12 // black

#define LED 4
#define MCP_CS 5
#define BME_CS 15

//############  AES   ############
#define ENCFLG 1
#define DECFLG 2
int enc(char *plane,  int len);
void DbgPrint(char *str);

//############  Debug   ############
#define SerialSpeed 115200
#define _DEBUG 1
#if _DEBUG
#define DBG(...) { Serial.print(__VA_ARGS__); }
#define DBGLED(...) { digitalWrite(__VA_ARGS__); }

//###############

Adafruit_SHT31 sht31 = Adafruit_SHT31();

int SHT31begin()
{
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
    Serial.println("Couldn't find SHT31");
    while (1) delay(1);
  }
}

double readTemperature()
{
  int ii = 0;
  double temp = 0;
  u8x8.drawString(0, 3, "readTemperature");
  temp = sht31.readTemperature();
  DBG(temp); DBG("\n");
  for ( ii = 0; (ii < 3) && (isnan(temp)); ii++ ) {
    delay(1000);
    temp = sht31.readTemperature();
    DBG(temp); DBG("\n");
    if (isnan(temp)) {
      digitalWrite(HARD_RESET, LOW);
      delay(1000);
      digitalWrite(HARD_RESET, HIGH);
    }
  }
  if (isnan(temp)) {
    temp = 0;
  }
  return temp;
}

double readHumidity()
{
  int ii = 0;
  double humid = 0;
  u8x8.drawString(0, 3, "readHumidity   ");
  humid = sht31.readHumidity();
  DBG(humid); DBG("\n");
  for ( ii = 0; (ii < 3) && (isnan(humid)); ii++ ) {
    delay(1000);
    humid = sht31.readHumidity();
    DBG(humid); DBG("\n");
    if (isnan(humid)) {
      digitalWrite(HARD_RESET, LOW);
      delay(1000);
      digitalWrite(HARD_RESET, HIGH);
    }
  }
  if (isnan(humid)) {
    humid = 0;
  }
  return humid;
}
void DbgPrint( const char *format, ... )
{
  va_list ap;
  char chbuff[512];

  // 可変長引数を１個の変数にまとめる
  va_start( ap, format );
  // まとめられた変数で処理する
  vsprintf( chbuff, format, ap );
  va_end( ap );
  //シリアル出力
  DBG(chbuff);
  // 戻り値省略
}
#else
#define DBG(...)
#define DBGLED(...)
void DbgPrint( const char *format, ... )
{
}
#endif /* _DBG */

void DbgPrint(char *str)
{
  DBG(str);
}
//############  Restart   ############
#define MaxRestart 5
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
int    wifi_fail_count = 0;

static RTC_RODATA_ATTR union {
  struct Data data;
  byte buffer[512];
} rtcData;

static union {
  struct Data data;
  byte buffer[512];
} rtcRamData;

static RTC_RODATA_ATTR bool state = 0;

#pragma pack(pop)

//############  IP   ############
String ipToString(IPAddress ip) {
  String s = "";
  for (int i = 0; i < 4; i++)
    s += i  ? "." + String(ip[i]) : String(ip[i]);
  return s;
}

//############  NTP   ############
/**
   Initialize NTP client
*/
void initNTP() {
  // Start NTP listener
  timeClient.begin();
  //timeClient.setTimeOffset(9 * 3600);
  // Force update of time from NTP server
  //timeClient.forceUpdate();
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

void TimeSync()
{
  int ii = 0;
  unsigned long sntptime;  //  SNTPで時刻取得
  time_t t_before;
  struct tm *tm_pbefore;
  struct tm tm_before;
  struct tm *tm_pafter;
  struct tm tm_after;
  t_before = now();
  tm_before = unix_time_to_date(t_before);
  do {
    sntptime = getSntpTime();  //  SNTPで時刻取得
    DBG(sntptime);
    tm_after = unix_time_to_date(sntptime);
    //tm_pafter = localtime((time_t *)&sntptime);
    //memcpy( &tm_after, tm_pafter, sizeof(struct tm));
    //DBG(sntptime);
    ii = ii + 1;
    delay(200);
    //　整時後が2018年前か（200回）、または、整時前と整時後が2018年前の間くりかえし（初回時に整時をする）
  } while ( ((ii <= 200) && (tm_after.tm_year < 2018)) || ((tm_pbefore->tm_year < 2018) && (tm_after.tm_year < 2018)) );

  if (tm_after.tm_year >= 2018) {
    setTime(sntptime);
  }
  myprintJST();
  DisplayLCD_IP(ipToString(WiFi.localIP()));
}

//############  Interrupt   ############
hw_timer_t * timer = NULL;
unsigned long before_interrupt_time = 0; // 前回の割り込み時刻

volatile int interruptCounter = 0;
volatile int interruptResetCounter = 0;
int reset_counter = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE mux_reset = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR handleInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  interruptCounter = 1;
  portEXIT_CRITICAL_ISR(&mux);
  digitalWrite(LED_PIN, HIGH);
}

void IRAM_ATTR handleResetInterrupt() {
  portENTER_CRITICAL_ISR(&mux_reset);
  interruptResetCounter = 1;
  portEXIT_CRITICAL_ISR(&mux_reset);
  digitalWrite(LED_PIN, HIGH);
}

bool isDisplayOutput = 1;
void IRAM_ATTR handleLCDInterrupt() {
  portENTER_CRITICAL_ISR(&mux_reset);
  if( isDisplayOutput ) {
    u8x8.setPowerSave( 0 );
  } else {
    u8x8.setPowerSave( 1 );
  }
  portEXIT_CRITICAL_ISR(&mux_reset);
}


//############  CallBack   ############
//flag for saving data
bool shouldSaveConfig = false;
//callback notifying us of the need to save config
void saveConfigCallback (WiFiManagerESP32 *myWiFiManager) {
  DBG("Should save config");
  shouldSaveConfig = true;
}

//############  FS   ############
void deleteFile() {
  if (SPIFFS.remove("/config.json")) {
    //Serial.println("- file deleted");
  } else {
    //Serial.println("- delete failed");
  }
}

void ReadFS()
{
  DBG("mounting FS...");

  if (SPIFFS.begin()) {
    DBG("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      DBG("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        DBG("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        //DynamicJsonBuffer jsonBuffer;
        DynamicJsonDocument json(1024);
        //JsonObject& json = jsonBuffer.parseObject(buf.get());
        DeserializationError error = deserializeJson(json, buf.get());
        //json.printTo(Serial);
        serializeJson(json, Serial);
        //if (json.success()) {
        if (!error) {
          DBG("\nparsed json");

          const char* tempssid = json["mqtt_ssid"];
          strcpy( ssid, tempssid);
          const char* temppassword = json["mqtt_password"];
          strcpy( password, temppassword);
          const char* tempwriteKey = json["mqtt_writeKey"];
          strcpy( writeKey, tempwriteKey);
          const char* tempaccountID = json["mqtt_accountID"];
          strcpy( accountID, tempaccountID);
        } else {
          DBG("failed to load json config");
        }
      }
    }
  } else {
    DBG("failed to mount FS");
  }
}

//############  WiFi   ############
void ConnectWiFi()
{
WIFI_RESTART:
  WiFiManagerESP32 wifiManager;
  char temp128[128];
  sprintf(&(temp128[0]), "now connecting");
  u8x8.drawString(0, 3, temp128);
  //##### Wifi Parameter #####
  WiFiManagerESP32Parameter custom_mqtt_writeKey("writeKey", "mqtt writeKey", writeKey, 62);
  WiFiManagerESP32Parameter custom_mqtt_accountID("accountID", "mqtt accountID", accountID, 62);

  //Local intialization. Once its business is done, there is no need to keep it around
  wifiManager.setConnectTimeout(20);

  //set config save notify callback
  wifiManager.setAPCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_writeKey);
  wifiManager.addParameter(&custom_mqtt_accountID);

  WiFi.mode(WIFI_STA);
  //WiFiを繋ぐ前に、WiFi状態をシリアルに出力
  //WiFi.printDiag(Serial);
  WiFi.begin(ssid, password);  //  Wi-Fi APに接続

  //接続が確立するまで、・・・を表示
  int one_wifi_fail_count = 0;
  int WIFI_ONEFAIL_MAX = 10;
  memset( temp128, 0x00, sizeof(temp128) );
  while ((one_wifi_fail_count < WIFI_ONEFAIL_MAX) && (WiFi.status() != WL_CONNECTED)) {
    delay(1000);
    DBG(".");
    sprintf(&(temp128[one_wifi_fail_count]), ".");
    u8x8.drawString(0, 3, temp128);
    one_wifi_fail_count = one_wifi_fail_count + 1;
  }
  DBG("IP address: ");
  DBG(WiFi.localIP()); DBG("\r\n");
  int WIFI_FAIL_MAX = 3;
  if ( one_wifi_fail_count >=  WIFI_ONEFAIL_MAX) {
    wifi_fail_count = wifi_fail_count + 1;
    if ( wifi_fail_count >= WIFI_FAIL_MAX ) {
      sprintf(&(temp128[0]), "ap start      ");
      u8x8.drawString(0, 3, temp128);
      if ( !wifiManager.autoConnect("ZachConnectAP") ) {
        //reset and try again, or maybe put it to deep sleep
        ESP.restart();
        delay(1000);
      }
    } else {
      // WIFI FAIL
      WiFi.disconnect();
      goto WIFI_RESTART;
    }
  }
  wifi_fail_count = 0;
  sprintf(&(temp128[0]), "connected     ");
  u8x8.drawString(0, 3, temp128);

  //if you get here you have connected to the WiFi
  DBG("connected...yeey :)\n");


  //save the custom parameters to FS
  if (shouldSaveConfig) {
    //read updated parameters
    strcpy(writeKey , custom_mqtt_writeKey.getValue());
    strcpy(accountID, custom_mqtt_accountID.getValue());
    shouldSaveConfig = 0;
    DBG("saving config");
    //DynamicJsonBuffer jsonBuffer;
    DynamicJsonDocument json(1024);
    //JsonObject& json = jsonBuffer.createObject();
    json["mqtt_ssid"] = wifiManager.getSSID();
    json["mqtt_password"] = wifiManager.getPassword();
    json["mqtt_writeKey"] = writeKey;
    json["mqtt_accountID"] = accountID;
    DBG(wifiManager.getSSID());
    DBG(wifiManager.getPassword());

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      DBG("failed to open config file for writing");
    }

    //json.printTo(Serial);
    serializeJson(json, Serial);
    //json.printTo(configFile);
    serializeJson(json, configFile);
    configFile.close();
    //end save
    DBG("\n");
  }

  DBG("autoConnect DONE\n");
  DBG("WiFi connected\r\nIP address: ");
  DBG(WiFi.localIP()); DBG("\r\n");

  //############  NTP   ############
  u8x8.drawString(0, 3, "NTP init start ");
  initNTP();
  u8x8.drawString(0, 3, "NTP init end   ");
  // Force update of time from NTP server
  timeClient.forceUpdate();

  sprintf(&(temp128[0]), "connected       ");
  u8x8.drawString(0, 3, temp128);
}

void ConnectAndSettime(int restart)
{
  //##### read configuration from FS json #####
  ReadFS();

  DBG("\n### Connect To ###\n");
  DBG("ssid = ");
  DBG(ssid);
  DBG("   password = ");
  DBG(password);
  DBG("\n");
  //接続が確立するまで、now connectingを表示

  //###### WiFi接続と設定値保存 ######
  //WiFiManager
  ConnectWiFi();

  //###### 整時 ######
  u8x8.drawString(0, 3, "NTP start       ");
  TimeSync();
  u8x8.drawString(0, 3, "NTP end         ");
  if ((restart == 0)) {
    WiFi.disconnect();
  }
}
//############  MainTask   ############
struct MainTask : public EventTask
{
  using EventTask::execute;

  void execute(Event evt)
  {
    bool valid;  //  Flashメモリーの値が有効かどうかのフラグ
    int restart;  //  何回目の起動かのカウンター。0の時はWi-Fiオンで起動されている
    time_t currenttime;// 現在時刻
    double sleeptime;   // sleepタイム
    time_t sntptime;// 整時用変数
    char temp128[128]; // LCD出力用

    DBG("\r\nStart\r\n");
    int t = millis();

    //############  RTCメモリーからデーターを読む   ############
    uint32_t crcOfData = 0;
    u8x8.drawString(0, 3, "Read RTC          ");
    memcpy((unsigned char*)&rtcRamData, (unsigned char*)&rtcData, sizeof(rtcData));
    u8x8.drawString(0, 3, "calc RTC          ");
    //crcOfData = (uint32_t)calcCRC16(0, (((unsigned char *)(&rtcRamData)) + 4), sizeof(rtcRamData) - 4);
    crcOfData = calcCRC32(((uint8_t*) &rtcRamData) + 4, sizeof(rtcRamData) - 4);

    if ((crcOfData == rtcRamData.data.crc32) && 0 <= rtcRamData.data.restart && rtcRamData.data.restart <= MaxRestart) {
      restart = rtcRamData.data.restart;
      valid = true;
      sleeptime = (double)rtcRamData.data.sleeptime * COMP;
      currenttime = rtcRamData.data.tbefore;// + (time_t)sleeptime;
    } else {
      restart = 0;
      valid = false;
      sleeptime = 0;
      currenttime = 0;
    }
    //setTime(currenttime);
    if (valid) {
      DBG("sleeptime: "); DBG(rtcRamData.data.sleeptime);
      DBG(", tbefore: "); DBG(rtcRamData.data.tbefore); DBG("\r\n");
    }

    //############  Wi-Fi接続より先にセンサーを読んでしまう   ############
    double temp = 0, humid = 0;
    u8x8.drawString(0, 3, "SHT begin           ");
    SHT31begin();
    temp = readTemperature();
    DisplayLCD_temp(temp);
    humid = readHumidity();
    DisplayLCD_hum(humid);

    //###### センサー値保存 ######
    int indx = restart;
    unsigned long tik = now();
    DBG("created = ");
    DBG(tik);
    DBG("\n");
    DBG("indx = ");
    DBG(indx);
    DBG("\n");
    struct tm tm_before;
    tm_before = unix_time_to_date(tik);
    DbgPrint("%d/", tm_before.tm_year);
    DbgPrint("%d/", tm_before.tm_mon);
    DbgPrint("%d ", tm_before.tm_yday);
    DbgPrint("%d:", tm_before.tm_hour);
    DbgPrint("%d:", tm_before.tm_min);
    DbgPrint("%d", tm_before.tm_sec);
    DBG("\n");
    rtcData.data.blocks[indx].created = tik;
    rtcData.data.blocks[indx].temp = temp;
    rtcData.data.blocks[indx].humid = humid;
    rtcData.data.blocks[indx].pressure = 0;
    rtcData.data.blocks[indx].illumi = 0;

    DBG("currentTime: "); DBG(currenttime);
    DBG(", temp: "); DBG(temp);
    DBG(" DegC,  humid: "); DBG(humid);
    delay(200);
    DisplayLCD_restart(restart);

    DBG("valid: "); DBG(valid);
    DBG(", restart: "); DBG(restart); DBG("\r\n");

    if ((restart == MaxRestart)) {
      ConnectAndSettime(restart);
    }

    //###### Zachへ送信 ######
    u8x8.drawString(0, 3, "send zach start ");
    if (restart == MaxRestart && valid) {
      DBG("Send to Zach\n");
      int sret = sendDataTozach(valid);
      if ( sret == -1 ) {
        const char email_subject[] = "エラー：SimpleWeatherStation";
        const char email_BCC[] = "";
        const char email_body[] = "fail to send.";
        sendEmail(email_subject, email_BCC, email_body);
      }
      //memset( (unsigned char*)&rtcData.data, 0x00, sizeof(rtcData.data) * (MaxRestart -1 ));
      WiFi.disconnect();
    }
    u8x8.drawString(0, 3, "send zach end   ");

    if (++restart > MaxRestart) {
      restart = 0;
    }

    //###### RTCメモリへ保存 ######
    rtcData.data.restart = restart;
    rtcData.data.tbefore = now();

    t = millis() - t; // t: リスタート直後からの経過時間(ミリ秒)
    t = (t < PERIOD * 1000) ? (PERIOD * 1000 - t) : 1;  // sleeptime(ミリ秒)
    DBG("sleep time = ");
    DBG(t);
    DBG("\n");

    rtcData.data.sleeptime = (time_t)((double)t / 1000.0 / COMP);
    rtcData.data.crc32 = 0;
    //rtcData.data.crc32 = (uint32_t)calcCRC16(0, (unsigned char *)((unsigned char *)(&rtcData)) + 4, sizeof(rtcData) - 4);
    rtcData.data.crc32 = calcCRC32(((uint8_t*) &rtcData) + 4, sizeof(rtcData) - 4);

    DBG("Loop DONE\n");
    u8x8.drawString(0, 3, "Done           ");
  }
} MainTask;

//############  LCD   ############
void DisplayLCD_time()
{
  char temp128[128];
  struct tm tm_after;
  tm_after = unix_time_to_date(now());
  //tm_pafter = localtime((time_t *)&sntptime);
  //memcpy( &tm_after, tm_pafter, sizeof(struct tm));
  sprintf(&(temp128[0]), "%02d/%02d/%02d    ",  tm_after.tm_year, tm_after.tm_mon,  tm_after.tm_yday);
  u8x8.drawString(0, 4, temp128);
  sprintf(&(temp128[0]), "%02d:%02d:%02d    ", tm_after.tm_hour, tm_after.tm_min, tm_after.tm_sec);
  u8x8.drawString(0, 5, temp128);
}
void DisplayLCD_IP(String IP)
{
  u8x8.drawString(0, 6, "               ");
  u8x8.drawString(0, 6, IP.c_str());
}

void DisplayLCD_temp(double temp)
{
  char temp128[128];
  sprintf(&(temp128[0]), "temp  = %3.1f", temp);
  u8x8.drawString(0, 0, temp128);
}

void DisplayLCD_hum(double humid)
{
  char temp128[128];
  sprintf(&(temp128[0]), "humid = %3.1f", humid);
  u8x8.drawString(0, 1, temp128);
}

void DisplayLCD_restart(int restart)
{
  char temp128[128];
  sprintf(&(temp128[0]), "restart = %d", restart);
  u8x8.drawString(0, 2, temp128);
}

void DisplayLCD_delay(int ltime)
{
  char temp128[128];
  sprintf(&(temp128[0]), "time=%02d  rst=%02d", ltime, reset_counter);
  u8x8.drawString(0, 7, temp128);
}

unsigned long getSntpTime() {
  unsigned long t = 0 ;
  configTime(9 * 3600 , 0, "ntp.nict.jp", NULL, NULL);
  if (timeClient.update()) {
    t = timeClient.getEpochTime();
  }
  return t;
}

#define BUFSIZE 1024

char buffer[BUFSIZE];
char destbuffer[BUFSIZE];
char tempbuffer[BUFSIZE];

int sendDataTozach(bool valid) {
  int i;
  time_t created;
  int vb;
  char vbbuf[12];

  DBG("zach.send() to: accountID = ");  DBG(accountID); DBG("\r\n");
  DBG("writeKey: "); DBG(writeKey); DBG("\r\n");
  zach.begin(60000, writeKey, &client, debug_zach);

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
            0.0 // d9
           );

    //DBG("buffer: " ); DBG(buffer); DBG("\r\n");
  }

  buffer[strlen(buffer) - 2] = '\0';
  sprintf(&buffer[strlen(buffer)], "}]}", vbbuf);

  DBG("buf: "); DBG(strlen(buffer)); DBG(" bytes\r\n");

  int n = zach.bulk_send(buffer, strlen(buffer));
  DBG("sent: "); DBG(n); DBG("\r\n");
  DBG(buffer); DBG("\r\n");
  return n;
}

void sendEmail(const char *email_subject, const char *email_BCC, const char *email_body) {
  int i;
  const char emailsendpass[16 + 1] = "zaker578d/23ej78";
  unsigned char emailsendpassvelify[32];
  const char email_fromaddr[] = "tktomaru.fs@gmail.com";
  const char email_password[] = "pass";
  const char email_toaddr[] = "tktomaru@gmail.com";

  // 暗号化後のデータを解析しにくくするために先頭にSHA256を配置する
  mbedtls_md_context_t ctx;
  mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
  const size_t payloadLength = strlen(emailsendpass);
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
  mbedtls_md_starts(&ctx);
  mbedtls_md_update(&ctx, (const unsigned char *) emailsendpass, payloadLength);
  mbedtls_md_finish(&ctx, emailsendpassvelify);
  mbedtls_md_free(&ctx);

  DBG("zachmail.send() to: "); DBG(50000); DBG("\r\n");
  DBG("pass: "); DBG(emailsendpass); DBG("\r\n");
  DBG("email_fromaddr: "); DBG(email_fromaddr);
  DBG("email_toaddr: "); DBG(email_toaddr); DBG("\r\n");
  zachmail.begin(50001, writeKey, &client, debug_zach);

  memset(buffer, 0x0, sizeof(buffer));
  for (i = 0; i < 16; i++) {
    sprintf(&(buffer[strlen(buffer)]), "%02x", emailsendpassvelify[i]);
  }
  sprintf(&(buffer[strlen(buffer)]), "{");
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
void loop()
{
  int skipflg = 0;
  int resetflg = 0;
  unsigned long now_time;

  //############  NTP   ############
  //initNTP();

  //#ifdef _DEBUG
  //  Serial.begin(SerialSpeed);
  //  delay(10);
  //#endif

  DisplayLCD_time();
  // 現在時刻取得
  now_time = millis() / 1000;

  // 現在時刻が、秒単位で異なっている場合のみ実行する
  if ( (now_time  != before_interrupt_time) && (now_time  != (before_interrupt_time + 1) % 60) ) {
    before_interrupt_time = now_time;

    portENTER_CRITICAL(&mux);
    if (interruptCounter == 1) {
      interruptCounter = 0;
      skipflg = 1;
      if ( 1 == LCD_PRINT ) {
        LCD_PRINT = 0;
      } else {
        LCD_PRINT = 1;
      }
    }
    portEXIT_CRITICAL(&mux);

    portENTER_CRITICAL(&mux_reset);
    if ( 1 == interruptResetCounter ) {
      interruptResetCounter = 0;
      resetflg = 1;
    }
    portEXIT_CRITICAL(&mux_reset);
  }

  if ( 1 == resetflg  ) {
    reset_counter++;
    DBG("!!!!! RESET counter ++ !!!!!");
    DBG(reset_counter);
    if ( reset_counter >= 3 ) {
      DBG("!!!!! RESET !!!!!");
      u8x8.drawString(0, 2, "reset         ");
      deleteFile();
      delay(100);
    }
  }

  digitalWrite(LED_PIN, LOW);

  int ltime = evtManager.tick(skipflg);
  DisplayLCD_delay(ltime / 1000);
}
void setup() {

#ifdef _DEBUG
  Serial.begin(SerialSpeed);
  delay(10);
#endif
  pinMode(LED_PIN, OUTPUT);
  u8x8.begin();
  u8x8.setFont(u8x8_font_artossans8_r);

  u8x8.drawString(0, 1, "booting     ");


  u8x8.drawString(0, 3, "attachInterrupt");
  
  pinMode(RESTART_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESTART_BUTTON), handleInterrupt, FALLING);

  pinMode(LCD_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RESTART_BUTTON), handleLCDInterrupt, FALLING);
  
  pinMode(HARD_RESET, OUTPUT);
  digitalWrite(HARD_RESET, HIGH);

  u8x8.drawString(0, 3, "handleResetInt  ");
  pinMode(RSET_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RSET_BUTTON), handleResetInterrupt, FALLING);
  //
  ConnectAndSettime(0);

  evtManager.subscribe(Subscriber("event.keepAlive", &MainTask));
  Event keepAlive = Event("event.keepAlive");
  evtManager.triggerInterval(TimedTask(PERIOD * 1000, keepAlive));
}
