/*
  29元手冲称改造计划
  BUTTON1 2 //input
  BUTTON2 3 //tare
  BUTTON3 4 //timer
  BUTTON4 5 //power
  HX711_sda_ A2
  HX711_scl_ A3
  clock 13
  data  11
  cs    10
  dc    9
  reset 8
  EEPROM
  0-3 byte float cal value
  4-7 byte int samples

  2021-08-01 开机清零校准，按住时间开机查看版本，按住录入开机进行校准 v1.0
  2021-08-02 Sample设定，存储（EEPROM） v1.01
  2021-08-25 单独的称重模式
*/

#include "stdlib.h"
#include <Arduino.h>
#define HW_I2C //oled连接方式
#define THREE_BUTTON
//#define THREE_BUTTON
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <HX711_ADC.h>
#include <AceButton.h>
using namespace ace_button;
#include <StopWatch.h>

//针脚定义
#define BUZZER 12
const int OLED_SCL = 6;
const int OLED_SDA = 7;
const int OLED_RST = 8;
const int OLED_DC = 9;
const int OLED_CS = 10;

const int BUTTON_SET = 3;
const int BUTTON_TIME = 4;
const int BUTTON_TARE = 5;

const int HX711_SDA =  A2;
const int HX711_SCL =  A3;

void(* resetFunc) (void) = 0;//重启函数


//电子秤校准参数
float calibrationValue;
int sampleNumber = 3; //0-7
const int sample[] = {1, 2, 4, 8, 16, 32, 64, 128};
const char* sampleText[] = {"    1   >", "<   2   >", "<   4   >", "<   8   >", "<   16   >", "<   32   >", "<   64   >", "<  128  "};

//HX711模数转换初始化
HX711_ADC scale(HX711_SDA, HX711_SCL);

//模式标识
bool boolPourOver = false; //手冲模式标识
bool boolCalibration = false; //校准标识
bool boolSetSample = false;
bool boolShowInfo = false;

//EEPROM地址列表
int address_calibrationValue = 0;
int address_sampleNumber = sizeof(calibrationValue);

unsigned long t_cal = 0;
int button_cal_status = 0;
float GRIND_COFFEE_WEIGHT = 0.0; //咖啡粉重量
char coffee[10]; //咖啡重量

const int Margin_Top = 0;
const int Margin_Bottom = 0;
const int Margin_Left = 0;
const int Margin_Right = 0;

StopWatch stopWatch;
const char minsec[] = "00:00";
char *sec2minsec(int n) {
  int minute = 0;
  int second = 0;
  if (n < 99 * 60 + 60) {
    if (n >= 60) {
      minute = n / 60;
      n = n % 60;
    }
    second = n;
  } else {
    minute = 99;
    second = 59;
  }
  snprintf(minsec, 6, "%02d:%02d", minute, second);
  return minsec;
}

//电子秤参数和计时点
bool fixWeightZero = false;
char wu[10];
char r[10];
unsigned long t0 = 0;               //开始萃取打点
unsigned long t1 = 0;               //下液第一滴打点
unsigned long t2 = 0;               //下液结束打点
unsigned long t_ReadyToBrew = 0;    //准备好冲煮的时刻
float w0 = 0.0; //咖啡粉重（g）
float w1 = 0.0;                 //下液重量（g）
float r0 = 0.0;   //粉水比 w1/w0
int tareCounter = 0; //不稳计数器
int defaultPowderWeight = 16;//默认咖啡粉重量

float aWeight = 0;          //稳定状态比对值（g）
float aWeightDiff = 0.05;    //稳定停止波动值（g）
float atWeight = 0;         //自动归零比对值（g）
float atWeightDiff = 0.3;   //自动归零波动值（g）
float asWeight = 0;         //下液停止比对值（g）
float asWeightDiff = 0.1;   //下液停止波动值（g）
float rawWeight = 0.0;      //原始读出值（g）

unsigned long scaleStableMarker = 0;    //稳定状态打点
unsigned long timeOutMarker = 0;        //超时打点
unsigned long t = 0;                  //最后一次重量输出打点
unsigned long oledRefreshMarker = 0;   //最后一次oled刷新打点
const int autoTareInterval = 500;       //自动归零检测间隔（毫秒）
const int autoStopInterval = 400;       //下液停止检测间隔（毫秒）
const int scaleStableInterval = 100;   //稳定状态监测间隔（毫秒）
const int timeOutInterval = 10000;      //超时检测间隔（毫秒）
int oledPrintInterval = 0;     //oled刷新间隔（毫秒）
const int serialPrintInterval = 0;  //称重输出间隔（毫秒）
bool readyToBrew = false; //准备好冲煮标志

//按钮配置
#include <AceButton.h>
using namespace ace_button;
ButtonConfig config1;
AceButton buttonSet(&config1);
AceButton buttonTime(&config1);
AceButton buttonTare(&config1);

//显示屏初始化 https://github.com/olikraus/u8g2/wiki/u8g2reference
//设置字体 https://github.com/olikraus/u8g2/wiki/fntlistall
#define FONT_L u8g2_font_logisoso24_tn
#define FONT_M u8g2_font_fub14_tr //单称2行
#define FONT_S u8g2_font_helvR12_tr
#define FONT_BATTERY u8g2_font_battery19_tn
char* c_battery = "0";//电池字符 0-5有显示
char* c_batteryTemp = "0";
unsigned long t_battery = 0; //电池充电循环判断打点
int i_battery = 0; //电池充电循环变量
int batteryRefreshTareInterval = 3 * 1000; //10秒刷新一次电量显示
unsigned long t_batteryRefresh = 0; //电池充电循环判断打点
float vRef = 4.72;

int FONT_M_HEIGHT;
int FONT_S_HEIGHT;
int FONT_L_HEIGHT;
//电子秤外壳对屏幕的遮盖像素补偿

#include <U8g2lib.h>
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
#ifdef HW_I2C
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
#endif
//U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
#ifdef SW_SPI
U8G2_SSD1306_128X64_NONAME_1_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* cs=*/ OLED_CS, /* dc=*/ OLED_DC, /* reset=*/ OLED_RST);
#endif
#ifdef SW_I2C
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ OLED_SCL, /* data=*/ OLED_SDA, /* reset=*/ U8X8_PIN_NONE);
#endif


//文本对齐 AC居中 AR右对齐 AL左对齐 T为要显示的文本
#define LCDWidth  u8g2.getDisplayWidth()
#define LCDHeight u8g2.getDisplayHeight()
#define AC(T)     ((LCDWidth - (u8g2.getStrWidth(T))) / 2 - Margin_Left - Margin_Right)
#define AR(T)     (LCDWidth -  u8g2.getStrWidth(T) - Margin_Right)
#define AL(T)     (u8g2.getStrWidth(T) + Margin_Left)

#define AM()     ((LCDHeight + u8g2.getMaxCharHeight()) / 2 - Margin_Top - Margin_Bottom)
#define AB()     (LCDHeight -  u8g2.getMaxCharHeight() - Margin_Bottom)
#define AT()     (u8g2.getMaxCharHeight() + Margin_Top)

//自定义trim消除空格
char *ltrim(char *s) {
  while (isspace(*s)) s++;
  return s;
}

char *rtrim(char *s) {
  char* back = s + strlen(s);
  while (isspace(*--back));
  *(back + 1) = '\0';
  return s;
}

char *trim(char *s) {
  return rtrim(ltrim(s));
}

void button_init() {
  pinMode(BUTTON_SET, INPUT_PULLUP);
#ifdef THREE_BUTTON
  pinMode(BUTTON_TIME, INPUT_PULLUP);
#endif
  pinMode(BUTTON_TARE, INPUT_PULLUP);
  buttonSet.init(BUTTON_SET);
#ifdef THREE_BUTTON
  buttonTime.init(BUTTON_TIME);
#endif
  buttonTare.init(BUTTON_TARE);
  config1.setEventHandler(handleEvent1);
  config1.setFeature(ButtonConfig::kFeatureClick);
  config1.setFeature(ButtonConfig::kFeatureLongPress);
  config1.setRepeatPressInterval(10);
}

void handleEvent1(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  int pin = button->getPin();
  switch (eventType) {
    case AceButton::kEventPressed:
      switch (pin) {
        case BUTTON_SET:
          beep(1, 100);
#ifdef THREE_BUTTON
          if (boolSetSample)
            sampleNumber--;
#endif
#ifdef TWO_BUTTON
          if (boolSetSample)
            sampleNumber++;
#endif
          else if (boolShowInfo)
            boolShowInfo = false;
          else
            buttonSet_Clicked();
          break;
#ifdef THREE_BUTTON
        case BUTTON_TIME:
          beep(1, 100);
          if (boolSetSample)
            setSample(1);
          else if (boolShowInfo)
            boolShowInfo = false;
          else {
            buttonTime_Clicked();
          }
          break;
#endif
        case BUTTON_TARE:
          beep(1, 100);
          if (boolCalibration)
            button_cal_status++;
          else if (boolShowInfo)
            boolShowInfo = false;
#ifdef THREE_BUTTON
          else if (boolSetSample)
            sampleNumber++;
#endif
#ifdef TWO_BUTTON
          else if (boolSetSample)
            setSample(1);
#endif
          else {
            buttonTare_Clicked();
          }
          break;
      }
      break;
    case AceButton::kEventLongPressed:
      switch (pin) {
        case BUTTON_SET:
          beep(1, 200);
          if (boolPourOver == true) //手冲模式下 长按回归普通模式
            boolPourOver = !boolPourOver;
          break;
        case BUTTON_TARE:
          beep(1, 200);
          if (boolCalibration)
            resetFunc();
          break;
      }
  }
}

void buttonSet_Clicked() {
  if (boolPourOver) {
    if (stopWatch.isRunning() == false) {
      //无论何时都可以操作的时钟
      if (stopWatch.elapsed() == 0)
      {
        //初始态 时钟开始
        //ESP.wdtFeed();
        delay(500);
        stopWatch.start();
      }
      if (stopWatch.elapsed() > 0)
      {
        //停止态 时钟清零
        stopWatch.reset();
      }
    }
    else {
      //在计时中 按一下则结束计时 停止冲煮 （固定参数回头再说）
      stopWatch.stop();
    }
  }
  else {
    //普通录入 按一下Set切换到手冲模式
    if (rawWeight > 0)
    {
      if (rawWeight < 3)
        GRIND_COFFEE_WEIGHT = defaultPowderWeight; //不足3g 录入为默认值（20g）
      else
        GRIND_COFFEE_WEIGHT = rawWeight;
      initPourOver();
      //fixWeightZero = true;
      stopWatch.stop();
      stopWatch.reset();
      //scale.tareNoDelay();
      //readyToBrew = true;
      //t_ReadyToBrew = millis();
    }
    boolPourOver = true;
  }
}

void initPourOver() {
  t0 = 0;               //开始萃取打点
  t1 = 0;               //下液第一滴打点
  t2 = 0;               //下液结束打点
  tareCounter = 0; //不稳计数器
  scaleStableMarker = 0;    //稳定状态打点
  timeOutMarker = 0;        //超时打点
  t = 0;                  //最后一次重量输出打点
}

void buttonTime_Clicked() { //时间
  if (stopWatch.isRunning() == false) {
    //无论何时都可以操作时钟的上按键
    if (stopWatch.elapsed() == 0)
    {
      //初始态 时钟开始
      delay(500);
      stopWatch.start();
    }
    if (stopWatch.elapsed() > 0)
    {
      //停止态 时钟清零
      stopWatch.reset();
    }
  }
  else {
    //在计时中 按一下则结束计时 停止冲煮 （固定参数回头再说）
    stopWatch.stop();
  }
}

void buttonTare_Clicked() { //清零
  if (stopWatch.isRunning() == false) {
    //不在计时中 也就是说没有冲煮 因此可以重量归零 时间归零
    fixWeightZero = true;
    if (!boolPourOver)
      GRIND_COFFEE_WEIGHT = 0;
    stopWatch.reset();
    t0 = 0;
    t1 = 0;
    t2 = 0;
    scale.tareNoDelay();
  }
  else {
    //在计时中 按一下则结束计时 停止冲煮 （固定参数回头再说）
    stopWatch.stop();
  }
}

void setSample(int input) {
  static const char* sampleText[] = {"1", "2", "4", "8", "16", "32", "64", "128"};
  while (boolSetSample) {
    buttonSet.check();//3按钮左移动 2按钮右移动
#ifdef THREE_BUTTON
    buttonTime.check();//3按钮确认 //右移动
#endif
    buttonTare.check();
#ifdef THREE_BUTTON
    if (sampleNumber < 0)
      sampleNumber = 0;
    if (sampleNumber > 7)
      sampleNumber = 7;
#endif
#ifdef TWO_BUTTON
    if (sampleNumber > 7)
      sampleNumber = 0;
#endif
    switch (input) {
      case 0: //左右移动
        refreshOLED("Set Sample", sampleText[sampleNumber]);
        break;
      case 1: //保存
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(address_sampleNumber, sampleNumber);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        refreshOLED("Saved", sampleText[sampleNumber]);
        delay(1000);
        boolSetSample = false;
        break;
    }
  }
  Serial.print(F("sampleNumber: "));
  Serial.print(sampleNumber);
  Serial.print(F(" sample set to"));
  Serial.println(sample[sampleNumber]);
}


void cal() {
  char* calval = "";
  if (button_cal_status == 1) {
    scale.setSamplesInUse(16);
    Serial.println(F("***"));
    Serial.println(F("Start calibration:"));
    Serial.println(F("Place the load cell an a level stable surface."));
    Serial.println(F("Remove any load applied to the load cell."));
    Serial.println(F("Press Button to set the tare offset."));
    refreshOLED("Press", "Tare Button");

    boolean _resume = false;
    while (_resume == false) {
      scale.update();
      buttonTare.check();
      if (button_cal_status == 2) {
        Serial.println(F("Hands off Taring...3"));
        refreshOLED("Hands off", "Tare in...3");
        delay(1000);
        refreshOLED("Hands off", "Tare in...2");
        delay(1000);
        refreshOLED("Hands off", "Tare in...1");
        delay(1000);
        refreshOLED("Taring...");
        scale.tare();
        Serial.println(F("Tare done"));
        refreshOLED("Tare done");
        delay(1000);
        _resume = true;
      }
    }
    Serial.println(F("Now, place your known mass on the loadcell."));
    Serial.println(F("Then send the weight of this mass (i.e. 100.0) from serial monitor."));
    refreshOLED("Put 100g");

    float known_mass = 0;
    _resume = false;
    while (_resume == false) {
      scale.update();
      buttonTare.check();
      if (button_cal_status == 3) {
        known_mass = 100.0;
        if (known_mass != 0) {
          Serial.print(F("Known mass is: "));
          Serial.println(known_mass);
          refreshOLED("Hands off", "Cal in 3");
          delay(1000);
          refreshOLED("Hands off", "Cal in 2");
          delay(1000);
          refreshOLED("Hands off", "Cal in 1");
          delay(1000);
          refreshOLED("Calibrating...");
          _resume = true;
        }
      }
    }
    scale.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
    calibrationValue = scale.getNewCalibration(known_mass); //get the new calibration value
    Serial.print(F("New calibration value: "));
    Serial.println(calibrationValue);
#if defined(ESP8266)|| defined(ESP32)
    EEPROM.begin(512);
#endif
    EEPROM.put(0, calibrationValue);
#if defined(ESP8266)|| defined(ESP32)
    EEPROM.commit();
#endif
    dtostrf(calibrationValue, 10, 2, calval);
    refreshOLED("Cal value", trim(calval));
    delay(1000);
  }
  boolCalibration = false;
}

void refreshOLED(char* input) {
  u8g2.firstPage();
  u8g2.setFont(FONT_M);
  do {
    //1行
    //FONT_M = u8g2_font_fub14_tn;
    u8g2.drawStr(AC(input), AM() - 5, input);
  } while ( u8g2.nextPage() );
}

void refreshOLED(char* input1, char* input2) {
  u8g2.firstPage();
  u8g2.setFont(FONT_M);
  do {
    //2行
    //FONT_M = u8g2_font_fub14_tn;
    u8g2.drawStr(AC(input1), FONT_M_HEIGHT, input1);
    u8g2.drawStr(AC(input2), LCDHeight - 5, input2);
  } while ( u8g2.nextPage() );
}

void refreshOLED(char* input1, char* input2, char* input3) {
  u8g2.firstPage();
  u8g2.setFont(FONT_S);
  do {
    //2行
    //FONT_M = u8g2_font_fub14_tn;
    u8g2.drawStr(AC(input1), FONT_S_HEIGHT, input1);
    u8g2.drawStr(AC(input2), AM(), input2);
    u8g2.drawStr(AC(input3), LCDHeight, input3);
  } while ( u8g2.nextPage() );
}

void checkBrew() {
  //下液超过1g自动开始计时 && 准备冲煮的标志为真 则开始计时
  //因为已经开冲了 所以准备标志应当复位为假 这样秒表开始也只按一次

  if (rawWeight > 1 && readyToBrew == true && fixWeightZero == false && (millis() - t_ReadyToBrew > 3000)) {
    stopWatch.start();
    beep(1, 100);
    readyToBrew = false;
  }
}

void setup() {
  delay(50); //有些单片机会重启两次
  Serial.begin(115200);
  while (!Serial); //等待串口就绪

  button_init();
  pinMode(BUZZER, OUTPUT);
  u8g2.begin();
  u8g2.setFont(FONT_S);
  FONT_S_HEIGHT = u8g2.getMaxCharHeight();
  u8g2.setFont(FONT_L);
  FONT_L_HEIGHT = u8g2.getMaxCharHeight();
  u8g2.setFont(FONT_M);
  FONT_M_HEIGHT = u8g2.getMaxCharHeight();
  beep(1, 100);
  refreshOLED("soso P.R.S.");
  stopWatch.setResolution(StopWatch::SECONDS);
  stopWatch.start();
  stopWatch.reset();

  button_cal_status = 1;//校准状态归1
  unsigned long stabilizingtime = 500;  //去皮时间(毫秒)，增加可以提高去皮精确度
  boolean _tare = true;                  //电子秤初始化去皮，如果不想去皮则设为false
  scale.begin();
  scale.start(stabilizingtime, _tare);

  //检查校准值合法性
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif
  EEPROM.get(0, calibrationValue);
  Serial.print(F("readEeprom(0, calibrationValue);"));
  Serial.println(calibrationValue);
  if (isnan(calibrationValue) || calibrationValue <= 0) {
    boolCalibration = true; //让按钮进入校准状态3
    cal(); //无有效读取，进入校准模式
  }
  else
    scale.setCalFactor(calibrationValue);  //设定校准值

  //检查sample值合法性
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif
  EEPROM.get(address_sampleNumber, sampleNumber);
  Serial.print(F("eepromSampleNumber: "));
  Serial.println(sampleNumber);
  if (isnan(sampleNumber)) {
    Serial.println("nan sample number");
    boolSetSample = true;
    sampleNumber = 3;//读取失败 默认值为3 对应sample为8
    setSample(0);
  }

#ifdef TWO_BUTTON
  if (digitalRead(BUTTON_TARE) == LOW && digitalRead(BUTTON_SET) == LOW) {
    boolShowInfo = true;
    showInfo();
    delay(2000);
  }
#endif

  if (digitalRead(BUTTON_SET) == LOW) {
    boolSetSample = true;
    setSample(0);
  }

#ifdef THREE_BUTTON
  if (digitalRead(BUTTON_TIME) == LOW) {
    boolShowInfo = true;
    showInfo();
    delay(2000);
  }
#endif

  if (digitalRead(BUTTON_TARE) == LOW) {
    boolCalibration = true; //让按钮进入校准状态3
    cal(); //无有效读取，进入校准模式
  }
  scale.setCalFactor(calibrationValue);
  scale.setSamplesInUse(sample[sampleNumber]);
}

void showInfo() {
  char* info[] = {/*版本号*/ "Version: 1.1", /*编译日期*/ "Build: 20210827", /*序列号*/ "S/N: PRSU001"}; //序列号
  //prsh hero
  //prs 手冲
  //prsu usb

  refreshOLED(info[0], info[1], info[2]);
  while (boolShowInfo) {
    buttonTare.check();
#ifdef THREE_BUTTON
    buttonTime.check();
#endif
    buttonSet.check();
  }
}

void pourOverScale() {
  if (GRIND_COFFEE_WEIGHT == 0)
    GRIND_COFFEE_WEIGHT = defaultPowderWeight;
  checkBrew();
  static boolean newDataReady = 0;
  static boolean scaleStable = 0;
  if (scale.update()) newDataReady = true;
  if (newDataReady) {
    rawWeight = scale.getData();
    newDataReady = 0;
    //-0.0 -> 0.0 正负符号稳定
    if (rawWeight >= -0.15 && rawWeight <= 0)
      rawWeight = 0.0;
    dtostrf(rawWeight, 7, 1, wu);
    Serial.print(trim(wu));
  }

  //记录咖啡粉时，将重量固定为0
  if (scale.getTareStatus()) {
    //完成清零
    beep(2, 50);
    readyToBrew = true;
    t_ReadyToBrew = millis();
    fixWeightZero = false;
  }
  if (fixWeightZero)
    rawWeight = 0.0;

  dtostrf(rawWeight, 7, 1, wu);

  float ratio_temp = rawWeight / GRIND_COFFEE_WEIGHT;
  if (ratio_temp < 0)
    ratio_temp = 0.0;
  if (GRIND_COFFEE_WEIGHT < 0.1)
    ratio_temp = 0.0;
  dtostrf(ratio_temp, 7, 1, r);
}

void pureScale() {
  static boolean newDataReady = 0;
  static boolean scaleStable = 0;
  if (scale.update()) newDataReady = true;

  if (newDataReady) {
    rawWeight = scale.getData();
    newDataReady = 0;
    if (rawWeight >= -0.15 && rawWeight <= 0)
      rawWeight = 0.0;
    //dtostrf(rawWeight, 7, decimalPrecision, wu);
    //Serial.print(trim(wu));
  }

  //记录咖啡粉时，将重量固定为0
  if (scale.getTareStatus()) {
    //完成清零
    beep(2, 50);
    fixWeightZero = false;
  }
  if (fixWeightZero)
    rawWeight = 0.0;
  dtostrf(rawWeight, 7, 1, wu);


  float ratio_temp = rawWeight / GRIND_COFFEE_WEIGHT;
  if (ratio_temp < 0)
    ratio_temp = 0.0;
  if (GRIND_COFFEE_WEIGHT < 0.1)
    ratio_temp = 0.0;
  dtostrf(ratio_temp, 7, 1, r);
}

void checkBattery() {
  float batteryVoltage = analogRead(A0) * vRef;
  float usbVoltage = analogRead(A1) * vRef;
  float perc = map(batteryVoltage, 3.6 * 1023, 4.1 * 1023, 0, 100);
  int i_icon = map(perc, 0, 100, 0, 5);
  switch (i_icon) {
    case 0:
      c_battery = "0";
      break;
    case 1:
      c_battery = "1";
      break;
    case 2:
      c_battery = "2";
      break;
    case 3:
      c_battery = "3";
      break;
    case 4:
      c_battery = "4";
      break;
    case 5:
      c_battery = "5";
      break;
    default:
      c_battery = "5";
      break;
  }
  if (usbVoltage > 4.3 * 1023) {
    c_battery = "c";
  }
#ifdef DEBUG
  Serial.print(c_battery);
  Serial.println(batteryVoltage);
#endif
}

void loop() {
  buttonSet.check();
#ifdef THREE_BUTTON
  buttonTime.check();
#endif
  buttonTare.check();
  checkBattery();

  if (boolPourOver) {
    pourOverScale();
  }
  else {
    pureScale();
  }
  Serial.println(trim(wu));
  if (millis() > oledRefreshMarker + oledPrintInterval)
  {
    //达到设定的oled刷新频率后进行刷新
    oledRefreshMarker = millis();
    int x = 0;
    int y = 0;
    char ratio[30];
    sprintf(ratio, "1:%s", trim(r));
    char coffeepowder[30];
    dtostrf(GRIND_COFFEE_WEIGHT, 7, 1, coffeepowder);
    u8g2.firstPage();
    if (boolPourOver) {
      do {
        u8g2.setFontDirection(0);
        //单称2行
        u8g2.setDisplayRotation(U8G2_R0);
        u8g2.setFont(FONT_M);
        x = Margin_Left;
        y = FONT_M_HEIGHT + Margin_Top - 5;
        u8g2.drawStr(x, y, trim(wu));
        u8g2.drawStr(AR(sec2minsec(stopWatch.elapsed())), y, sec2minsec(stopWatch.elapsed()));
        //u8g2.drawStr(AR(sec2sec(stopWatch.elapsed())), y, sec2sec(stopWatch.elapsed()));

        y = y + FONT_M_HEIGHT;
        //u8g2.setFont(FONT_S);
        u8g2.drawStr(AC("PourOver"), y + 1, "PourOver");

        u8g2.setFont(FONT_M);
        x = Margin_Left;
        y = LCDHeight - Margin_Bottom;
        u8g2.drawStr(x, y, trim(coffeepowder));
        u8g2.drawStr(AR(trim(ratio)), y, trim(ratio));

        u8g2.setFontDirection(1);
        u8g2.setFont(FONT_BATTERY);
        if (c_battery == "c") {
          if (i_battery == 6)
            i_battery = 0;
          if (millis() > t_battery + 500) {
            i_battery++;
            t_battery = millis();
          }
          String(i_battery).toCharArray(c_battery, 2);
          u8g2.drawStr(108, 29, c_battery);
        }
        else {
          if (millis() > t_batteryRefresh + batteryRefreshTareInterval) {
            c_batteryTemp = c_battery;
            t_batteryRefresh = millis();
          }
          u8g2.drawStr(108, 29, c_batteryTemp);
        }
        //单称4行
        //      u8g2.setFont(FONT_M);
        //      x = Margin_Left;
        //      y =  FONT_S_HEIGHT + FONT_M_HEIGHT + Margin_Top;
        //      u8g2.drawStr(x, y, trim(wu));
        //      u8g2.drawStr(AR(sec2minsec(stopWatch.elapsed())), y, sec2minsec(stopWatch.elapsed()));
        //
        //      y = LCDHeight - FONT_S_HEIGHT - Margin_Bottom;
        //      u8g2.drawStr(x, y, trim(coffeepowder));
        //      u8g2.drawStr(AR(trim(ratio)), y, trim(ratio));
        //
        //      u8g2.setFont(FONT_S);
        //      y = FONT_S_HEIGHT + Margin_Top;
        //      u8g2.drawStr(x, y, "WEIGHT");
        //      u8g2.drawStr(AR("TIME"), y, "TIME");
        //
        //      u8g2.setFont(FONT_S);
        //      y = LCDHeight - Margin_Bottom;
        //      u8g2.drawStr(x, y, "COFFEE");
        //      u8g2.drawStr(AR("RATIO"), y, "RATIO");


        //双称
        //      int LINE_INTERVAL = 4;
        //      int WORD_INTERVAL = 5;
        //      x = Margin_Left;
        //      u8g2.setFont(FONT_S);
        //      y = FONT_M_HEIGHT + Margin_Top;
        //      u8g2.drawStr(x, y, "WEIGHT");
        //
        //      y = FONT_M_HEIGHT * 2 + Margin_Top + LINE_INTERVAL;
        //      u8g2.drawStr(x, y, "LOWER");
        //
        //      y = FONT_M_HEIGHT * 3 + Margin_Top + LINE_INTERVAL * 2;
        //      u8g2.drawStr(x, y, "TOTAL");
        //
        //      x = AL("") + WORD_INTERVAL;
        //      u8g2.setFont(FONT_M);
        //      y = FONT_M_HEIGHT + Margin_Top;
        //      u8g2.drawStr(x, y, trim(wu));
        //
        //      y = FONT_M_HEIGHT * 2 + Margin_Top + LINE_INTERVAL;
        //      u8g2.drawStr(x, y, trim(wl));
        //
        //      y = FONT_M_HEIGHT * 3 + Margin_Top + LINE_INTERVAL * 2;
        //      u8g2.drawStr(x, y, trim(wuwl));
        //
        //
        //      u8g2.setFont(FONT_M);
        //      y = FONT_M_HEIGHT + Margin_Top;
        //      u8g2.drawStr(AR(trim(coffeepowder)), y, trim(coffeepowder));
        //      y = FONT_M_HEIGHT * 2 + Margin_Top + LINE_INTERVAL;
        //      u8g2.drawStr(AR(sec2minsec(stopWatch.elapsed())), y, sec2minsec(stopWatch.elapsed()));
        //      y = FONT_M_HEIGHT * 3 + Margin_Top + LINE_INTERVAL * 2;
        //      u8g2.drawStr(AR(trim(ratio)), y, trim(ratio));

      } while ( u8g2.nextPage() );
    }
    else  {
      //纯称重
      do {
        //ESP.wdtFeed();
        u8g2.setDisplayRotation(U8G2_R0);
        u8g2.setFont(FONT_L);
        FONT_L_HEIGHT = u8g2.getMaxCharHeight();
        x = AC(trim(wu));
        y = AM();
        u8g2.drawStr(x, y - 5, trim(wu));
        u8g2.setFontDirection(1);
        u8g2.setFont(FONT_BATTERY);
        if (c_battery == "c") {
          if (i_battery == 6)
            i_battery = 0;
          if (millis() > t_battery + 500) {
            i_battery++;
            t_battery = millis();
          }
          String(i_battery).toCharArray(c_battery, 2);
          u8g2.drawStr(108, 7, c_battery);
        }
        else {
          if (millis() > t_batteryRefresh + batteryRefreshTareInterval) {
            c_batteryTemp = c_battery;
            t_batteryRefresh = millis();
          }
          u8g2.drawStr(108, 7, c_batteryTemp);
        }
        u8g2.setFontDirection(0);
#ifdef DEBUG
        u8g2.setFont(FONT_S);
        float batteryVoltage = analogRead(A0) * vRef / 1023;
        char c_votage[10];
        String(batteryVoltage).toCharArray(c_votage, 10);
        u8g2.drawStr(AR(c_votage), 64, trim(c_votage));
        Serial.print("v");
        Serial.println(c_votage);
#endif
      } while ( u8g2.nextPage() );
    }
  }
}

void beep(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER, HIGH);
    delay(duration);
    digitalWrite(BUZZER, LOW);
    delay(50);
  }
}
