#include <lvgl.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include <AccelStepper.h>
#include "Preferences.h"

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9488   _panel_instance;
  lgfx::Bus_SPI         _bus_instance;  
  lgfx::Light_PWM       _light_instance;
  lgfx::Touch_XPT2046   _touch_instance;

public:
  LGFX(void) {
    {
      auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。

      cfg.spi_host = VSPI_HOST;     // 使用するSPIを選択  (VSPI_HOST or HSPI_HOST)
      cfg.spi_mode = 0;             // SPI通信モードを設定 (0 ~ 3)
      cfg.freq_write = 40000000;    // 送信時のSPIクロック (最大80MHz, 80MHzを整数で割った値に丸められます)
      cfg.freq_read  = 16000000;    // 受信時のSPIクロック
      cfg.spi_3wire  = false;        // 受信をMOSIピンで行う場合はtrueを設定
      cfg.use_lock   = true;        // トランザクションロックを使用する場合はtrueを設定
      cfg.dma_channel = 1;          // Set the DMA channel (1 or 2. 0=disable)   使用するDMAチャンネルを設定 (0=DMA不使用)
      cfg.pin_sclk = 18;            // SPIのSCLKピン番号を設定
      cfg.pin_mosi = 23;            // SPIのMOSIピン番号を設定
      cfg.pin_miso = 19;            // SPIのMISOピン番号を設定 (-1 = disable)
      cfg.pin_dc   = 16;            // SPIのD/Cピン番号を設定  (-1 = disable)
      _bus_instance.config(cfg);    // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
    }

    {
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。

      cfg.pin_cs           =    5;  // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst          =    17;  // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy         =    -1;  // BUSYが接続されているピン番号 (-1 = disable)

      cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
      cfg.memory_height    =   320;  // ドライバICがサポートしている最大の高さ
      cfg.panel_width      =   240;  // 実際に表示可能な幅
      cfg.panel_height     =   320;  // 実際に表示可能な高さ
      cfg.offset_x         =     0;  // パネルのX方向オフセット量
      cfg.offset_y         =     0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         = true;  // データ読出しが可能な場合 trueに設定
      cfg.invert           = false;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false;  // データ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared       = true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

      _panel_instance.config(cfg);
    }

    {
      auto cfg = _light_instance.config();    // バックライト設定用の構造体を取得します。

      cfg.pin_bl = 21;              // バックライトが接続されているピン番号
      cfg.invert = false;           // バックライトの輝度を反転させる場合 true
      cfg.freq   = 44100;           // バックライトのPWM周波数
      cfg.pwm_channel = 7;          // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセットします。
    }

    {
      auto cfg = _touch_instance.config();

      cfg.x_min      = 0;    // タッチスクリーンから得られる最小のX値(生の値)
      cfg.x_max      = 319;  // タッチスクリーンから得られる最大のX値(生の値)
      cfg.y_min      = 0;    // タッチスクリーンから得られる最小のY値(生の値)
      cfg.y_max      = 479;  // タッチスクリーンから得られる最大のY値(生の値)
      cfg.pin_int    = -1;   // INTが接続されているピン番号
      cfg.bus_shared = true; // 画面と共通のバスを使用している場合 trueを設定
      cfg.offset_rotation = 0;// 表示とタッチの向きのが一致しない場合の調整 0~7の値で設定

      cfg.spi_host = VSPI_HOST;// 使用するSPIを選択 (HSPI_HOST or VSPI_HOST)
      cfg.freq = 1000000;     // SPIクロックを設定
      cfg.pin_sclk = 18;     // SCLKが接続されているピン番号
      cfg.pin_mosi = 23;     // MOSIが接続されているピン番号
      cfg.pin_miso = 19;     // MISOが接続されているピン番号
      cfg.pin_cs   = 22;     //   CSが接続されているピン番号

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  // タッチスクリーンをパネルにセットします。
    }

    setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};

LGFX tft;

#define stopMotor 0
#define pauseMotor 1
#define runMotor 2

#define syrine1 0
#define syrine2 1
#define syrine3 2

#define Syringe1diameter 0
#define Syringe1value 1
#define Syringe2diameter 2
#define Syringe2value 3
#define Syringe3diameter 4
#define Syringe3value 5
#define proccessValue 6
#define proccessSpeed 7
#define calibration 8

#define startBtn 0
#define moveBtnP 1
#define moveBtnN 2
#define pauseBtn 3
#define stopBtn 4

#define secMode 0
#define minMode 1
#define hrMode 2
#define ulMode 3
#define mlMode 4

#define uLsecMode 0
#define uLminMode 1
#define mLsecMode 2
#define mLminMode 3
#define mLhrMode 4

#define motorConstSpeed 0
#define motorMaxSpeed 1
#define motorAccel 2
#define motorDestination 3

#define LV_STATE_UNCHECKED 0

/*Change to your screen resolution*/
static const uint32_t screenWidth  = 320;
static const uint32_t screenHeight = 260;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

static lv_style_t style_bg;
static lv_style_t style_indic;
static lv_style_t btn_style_1;
static lv_style_t btn_style_2;
  
lv_obj_t * barHandler;
lv_obj_t * btn[5];
lv_obj_t * checkBox[3];
lv_obj_t * keyBoard;
lv_obj_t * tv1;
lv_obj_t * tile1;
lv_obj_t * tile2;
lv_obj_t * starterSheet;
lv_obj_t * finalSheet;
lv_obj_t * dropDown[2];
lv_obj_t * textArea[9];

bool syringeChecked[] = {LV_STATE_UNCHECKED, LV_STATE_UNCHECKED, LV_STATE_UNCHECKED};
bool syringFlag = false;

unsigned long int startTime = 0;
uint8_t  motorState = stopMotor;
float residueValue = 0;

float textAreaInput[8] = {0, 0, 0, 0, 0, 0, 0, 1};
float syringParam[2] = {0, 0};
bool textAreaCheck[8] = {false, false, false, false, false, false, false, false};
bool textAreaFlag = false;

String dropDownValue[2] = {"NULL", "NULL"};
int dropDownValueInt[2] = {0, 0};
bool dropDownCheck[] = {false, false};
bool dropDownFlag = false;
const char * timeinputUnit[5] = {"second", "minute", "Hour", "uL", "mL"};
const char * timeOpt = "second\n"
                       "minute\n"
                       "Hour\n"
                       "uL\n"
                       "mL\n";
const char * speedinputUnit[5] = {"uL/Sec", "uL/min", "mL/sec", "mL/min", "mL/hr"};
const char * speedOpt = "uL/Sec\n"
                        "uL/min\n"
                        "mL/sec\n"
                        "mL/min\n"
                        "mL/hr\n";                  

int motorMode = secMode;
int speedMode = uLsecMode;
float motorData[4] = {0, 0, 0, 0};
bool ccwMotor = false;
bool setMotorPosition = false;

const int dirPin = 2;
const int stepPin = 25;
const int enPin = 26;
#define motorInterfaceType 1

const char *textArea_addr[9] = {"iiiiiiiiiiii", 
                                "aaaaaaaaaaaa",
                                "hhhhhhfffffdddd", 
                                "cccccccccccc", 
                                "dddddddddddd", 
                                "eeeeeeeeeeee", 
                                "ffffffffffff", 
                                "gggggggggggg", 
                                "hhhhhhhhhhhh", };
  
const char *checkBox_addr[3] = {"checkBoxx_addr", 
                                "bdghcbth",
                                "dki5437"};

const char *dropDown_addr[2] = {"lllllllllll", 
                                "ttttttttttt"};

struct Button {
  const uint8_t PIN;
  bool pressed;
};

Button button1 = {32, false};
Button button2 = {33, false};
bool switchHold = false;

unsigned long switchTime = 0;


AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);
TaskHandle_t Task1;
Preferences NVS;


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p ) {
  uint32_t w = ( area->x2 - area->x1 + 1 );
  uint32_t h = ( area->y2 - area->y1 + 1 );

  tft.startWrite();
  tft.setAddrWindow( area->x1, area->y1, w, h );
  // tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
  tft.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);
  tft.endWrite();

  lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data ) {
  uint16_t touchX, touchY;
  bool touched = tft.getTouch( &touchX, &touchY);
  if( !touched ) {
    data->state = LV_INDEV_STATE_REL;
  }
  else {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;

    // Serial.print( "Data x " );
    // Serial.println( touchX );

    // Serial.print( "Data y " );
    // Serial.println( touchY );
  }
}

void setup() {
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  while (!Serial);
  
  pinMode(button1.PIN ,INPUT_PULLUP);
  pinMode(button2.PIN ,INPUT_PULLUP);
    
  NVS.begin("SM1001", false);
  // ReStore_Param();

  syringeChecked[syrine1] = NVS.getBool(checkBox_addr[syrine1]); 
  Serial.println(syringeChecked[syrine1]);

  syringeChecked[syrine2] = NVS.getBool(checkBox_addr[syrine2]); 
  Serial.println(syringeChecked[syrine2]);
  
  syringeChecked[syrine3] = NVS.getBool(checkBox_addr[syrine3]); 
  Serial.println(syringeChecked[syrine3]);


  textAreaInput[Syringe1diameter] = NVS.getFloat(textArea_addr[Syringe1diameter]); 
  Serial.println(textAreaInput[Syringe1diameter]);
  
  textAreaInput[Syringe1value] = NVS.getFloat(textArea_addr[Syringe1value]); 
  Serial.println(textAreaInput[Syringe1value]);
  
  textAreaInput[Syringe2diameter] = NVS.getFloat(textArea_addr[Syringe2diameter]); 
  Serial.println(textAreaInput[Syringe2diameter]);

  textAreaInput[Syringe2value] = NVS.getFloat(textArea_addr[Syringe2value]); 
  Serial.println(textAreaInput[Syringe2value]);
  
  textAreaInput[Syringe3diameter] = NVS.getFloat(textArea_addr[Syringe3diameter]); 
  Serial.println(textAreaInput[Syringe3diameter]);
  
  textAreaInput[Syringe3value] = NVS.getFloat(textArea_addr[Syringe3value]); 
  Serial.println(textAreaInput[Syringe3value]);

  textAreaInput[calibration] = NVS.getFloat(textArea_addr[calibration]); 
  Serial.println(textAreaInput[calibration]);  
  
  if (syringeChecked[syrine1]) {
    syringParam[0] = textAreaInput[Syringe1diameter] / 2;
    syringParam[1] = textAreaInput[Syringe1value];
  } else if (syringeChecked[syrine2]) {
    syringParam[0] = textAreaInput[Syringe2diameter] / 2;
    syringParam[1] = textAreaInput[Syringe2value];
  } else if (syringeChecked[syrine3]) {
    syringParam[0] = textAreaInput[Syringe3diameter] / 2;
    syringParam[1] = textAreaInput[Syringe3value];
  }
  
  dropDownValueInt[0] = NVS.getInt(dropDown_addr[0]);
  Serial.println(dropDownValueInt[0]);

  dropDownValueInt[1] = NVS.getInt(dropDown_addr[1]);
  Serial.println(dropDownValueInt[1]);


  textAreaInput[proccessValue] = NVS.getFloat(textArea_addr[proccessValue]);
  if (dropDownValueInt[0] == secMode) {
    motorMode = secMode;
    // textAreaInput[proccessValue] = textAreaInput[proccessValue];
  } else if (dropDownValueInt[0] == minMode) {
    motorMode = minMode;
    textAreaInput[proccessValue] = textAreaInput[proccessValue] * (float)60;
  } else if (dropDownValueInt[0] == hrMode) {
    motorMode = hrMode;
    textAreaInput[proccessValue] = textAreaInput[proccessValue] * (float)60 * (float)60;
  } else if (dropDownValueInt[0] == ulMode) {
    motorMode = ulMode;
    textAreaInput[proccessValue] = textAreaInput[proccessValue] / (float)1000;
  } else if (dropDownValueInt[0] == mlMode) {
    motorMode = mlMode;
    textAreaInput[proccessValue] = textAreaInput[proccessValue];
  }
  Serial.println(textAreaInput[proccessValue], 4);
  
  textAreaInput[proccessSpeed] = NVS.getFloat(textArea_addr[proccessSpeed]);
  if (dropDownValueInt[1] == uLsecMode) {
    speedMode = uLsecMode;
    textAreaInput[proccessSpeed] = textAreaInput[proccessSpeed]  / (float)1000;
  } else if (dropDownValueInt[1] == uLminMode) {
    speedMode = uLminMode;
    textAreaInput[proccessSpeed] = textAreaInput[proccessSpeed]  / (float)1000 / (float)60;
  } else if (dropDownValueInt[1] == mLsecMode) {
    speedMode = mLsecMode;
    // textAreaInput[proccessSpeed] = textAreaInput[proccessSpeed] ;
  } else if (dropDownValueInt[1] == mLminMode) {
    speedMode = mLminMode;
    textAreaInput[proccessSpeed] = textAreaInput[proccessSpeed]  / (float)60;
  } else if (dropDownValueInt[1] == mLhrMode) {
    speedMode = mLhrMode;
    textAreaInput[proccessSpeed] = textAreaInput[proccessSpeed]  / (float)60 / (float)60;
  }
  Serial.println(textAreaInput[proccessSpeed], 4);
  
  myStepper.setPinsInverted(true ,false ,true);
  myStepper.setEnablePin(enPin);
  myStepper.disableOutputs();
  
  xTaskCreatePinnedToCore(
                    dispalyCore,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */              
  delay(500);   

  switchTime = millis();
}

void loop() {

  // // Handle motor to go to 0 position
  // if ((myStepper.distanceToGo() == 0) && (motorState == runMotor) && (ccwMotor == false) && !setMotorPosition) {
  //   Serial.println("Motor Going to 0 pos.");
  //   Serial.println();
  //   Serial.println();
  //   myStepper.moveTo(0);    
  //   ccwMotor = true;
  // }

  // // Change Sheet after complate proccess
  // if ((myStepper.distanceToGo() == 0) && (ccwMotor == true)) {
  //   lv_obj_add_flag(finalSheet, LV_OBJ_FLAG_HIDDEN);
  //   lv_obj_clear_flag(starterSheet, LV_OBJ_FLAG_HIDDEN);
  //   myStepper.disableOutputs();
  //   motorState = stopMotor;  
  //   ccwMotor = false;
  // }

  // Change Sheet after complate proccess
  if ((myStepper.distanceToGo() == 0) && (motorState == runMotor) && !setMotorPosition) {
    lv_obj_add_flag(finalSheet, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(starterSheet, LV_OBJ_FLAG_HIDDEN);
    myStepper.disableOutputs();
    motorState = stopMotor;  
    // ccwMotor = false;
  }

  // Handle Stop Of Move+/Move-
  if ((myStepper.distanceToGo() == 0) && setMotorPosition) {
    myStepper.disableOutputs();
    motorState = stopMotor;      
    setMotorPosition = false;
  }
  
  // Check Start/End mcroSwitch
  if((!digitalRead(button1.PIN) || !digitalRead(button2.PIN)) && ((millis() - switchTime) > 3000)) {
    Serial.println("button_callBack --> END.");
    myStepper.moveTo(0);    
    // ccwMotor = true;
    switchHold = false;
    switchHold = false;
    switchTime = millis();
  }

  // Handle Motor Movement
  if (myStepper.distanceToGo() != 0) {
    myStepper.run();
  }
}

void dispalyCore( void * pvParameters ){
  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */
  tft.setBrightness(255);
  uint16_t calData[] = { 217, 3899, 158, 355, 3892, 3883, 3855, 369 };
  tft.setTouchCalibrate( calData );

  lv_init();

  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );

  lv_init_style();
  starterSheetConf();
  motorSheetConf();  
  
  lv_obj_add_flag(finalSheet, LV_OBJ_FLAG_HIDDEN);
  startTime = millis();
  
  for(;;) {
    lv_timer_handler(); /* let the GUI do its work */

    if((millis() - startTime > 500) && ((motorState == runMotor) || (motorState == pauseMotor))) {
      
      residueValue = ( myStepper.currentPosition() / motorData[motorDestination] ) * 100;

      Serial.print("targetPosition is : ");   Serial.println(myStepper.targetPosition());
      Serial.print("currentPosition is : ");   Serial.println(myStepper.currentPosition());
      Serial.print("residueValue is : ");   Serial.println(residueValue ,4);
      Serial.println();

      am_set_bar_value(barHandler, (int32_t)residueValue);
      startTime = millis();   
    }
   
    vTaskDelay(5);
  } 
}


/* CallBack Function*/
/********************************************************************************************************/
static void button_callBack(lv_event_t * e) {
  // Serial.println("button_callBack --> In CallBack");
  
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_CLICKED) {
    if(obj == btn[startBtn] && (motorState == stopMotor)) {
      Serial.println("button_callBack --> start Btn Pressed.");

      Serial.print("button_callBack --> proccessSpeed is : ");   Serial.println(textAreaInput[proccessSpeed], 4);  
      Serial.print("button_callBack --> radios is : ");   Serial.println(syringParam[0], 4); 
      Serial.print("button_callBack --> proccessValue is : ");   Serial.println(textAreaInput[proccessValue], 4); 
      Serial.print("button_callBack --> calibration is : ");   Serial.println(textAreaInput[calibration]); 
    
      if ((motorMode == secMode) || (motorMode == minMode) || (motorMode == hrMode)) {
        motorData[motorConstSpeed] = (textAreaInput[calibration] * (float)3200 * ( textAreaInput[proccessSpeed] / (3.14 * syringParam[0] * syringParam[0]) ) );
        motorData[motorDestination] = textAreaInput[proccessValue] * motorData[motorConstSpeed]; 
        motorData[motorMaxSpeed] = motorData[motorConstSpeed]; 
        motorData[motorAccel] = motorData[motorConstSpeed];
      } else {
        motorData[motorConstSpeed] = (textAreaInput[calibration] * (float)3200 * ( textAreaInput[proccessSpeed] / (3.14 * syringParam[0] * syringParam[0]) ) );
        motorData[motorDestination] = (textAreaInput[calibration] * (float)3200 * ( textAreaInput[proccessValue] / (3.14 * syringParam[0] * syringParam[0]) ) ); 
        motorData[motorMaxSpeed] = motorData[motorConstSpeed]; 
        motorData[motorAccel] = motorData[motorConstSpeed];
      }

      Serial.print("button_callBack --> motorConstSpeed is : ");   Serial.println(motorData[motorConstSpeed], 4);  
      Serial.print("button_callBack --> motorMaxSpeed is : ");   Serial.println(motorData[motorMaxSpeed], 4); 
      Serial.print("button_callBack --> motorAccel is : ");   Serial.println(motorData[motorAccel], 4); 
      Serial.print("button_callBack --> motorDestination is : ");   Serial.println((long) motorData[motorDestination]); 

      myStepper.enableOutputs();
      myStepper.setCurrentPosition(0);
      myStepper.setMaxSpeed(motorData[motorMaxSpeed]);
      myStepper.setAcceleration(motorData[motorAccel]);
      myStepper.setSpeed(motorData[motorConstSpeed]);    
      myStepper.moveTo((long) motorData[motorDestination]);      
       
      lv_obj_add_flag(starterSheet, LV_OBJ_FLAG_HIDDEN);
      lv_obj_clear_flag(finalSheet, LV_OBJ_FLAG_HIDDEN);
      motorState = runMotor;
    }

    if(obj == btn[pauseBtn]) {
      Serial.println("button_callBack --> Pause Btn Pressed.");    
      lv_obj_t * label = lv_obj_get_child(obj, 0);
      
      if(motorState == runMotor) {
        Serial.println("button_callBack --> Motor State Changed to Pause.");
        lv_label_set_text(label, "Resume");  
        motorState = pauseMotor;
        myStepper.stop(); 
      } else if (motorState == pauseMotor) {
        Serial.println("button_callBack --> Motor State Changed to Run.");
        lv_label_set_text(label, "Pause");  
        // myStepper.setCurrentPosition();
        myStepper.moveTo(motorData[motorDestination]);
        motorState = runMotor;          
      }   
    } 
    
    if(obj == btn[stopBtn]) {
      Serial.println("button_callBack --> Stop Btn Pressed.");
      myStepper.stop();    
      // ccwMotor = true;
    }

    if(obj == btn[moveBtnP] && (motorState == stopMotor) && (switchHold == false)) {
      Serial.println("button_callBack --> move P Btn Pressed.");
      myStepper.enableOutputs();
      myStepper.setCurrentPosition(0);
      myStepper.setMaxSpeed(6400);
      myStepper.setSpeed(3200);
      myStepper.setAcceleration(3200);  
      myStepper.moveTo(6400); 
      setMotorPosition = true;
      motorState = runMotor; 
    }

    if(obj == btn[moveBtnN] && (motorState == stopMotor) && (switchHold == false)) {
      Serial.println("button_callBack --> move N Btn Pressed.");
      myStepper.enableOutputs();
      myStepper.setCurrentPosition(0);
      myStepper.setMaxSpeed(6400);
      myStepper.setSpeed(6400);  
      myStepper.setAcceleration(3200);
      myStepper.moveTo(-6400); 
      setMotorPosition = true;
      motorState = runMotor; 
    }
  } 
}

static void checkBox_callBack(lv_event_t * e) {
  // Serial.println("checkBox_callBack --> In CallBack");  
  
  char buf[64];
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_VALUE_CHANGED) {
    // const char * txt = lv_checkbox_get_text(obj);
    // const char * state = lv_obj_get_state(obj) & LV_STATE_CHECKED ? "Checked" : "Unchecked";

    if(obj == checkBox[syrine1]) {
      Serial.println("checkBox_callBack --> Syringe 1 Checked Pressed.");
      syringeChecked[syrine1] = LV_STATE_CHECKED;
      syringeChecked[syrine2] = LV_STATE_UNCHECKED;
      syringeChecked[syrine3] = LV_STATE_UNCHECKED;

      lv_obj_add_state(checkBox[syrine1], syringeChecked[syrine1]);
      lv_obj_clear_state(checkBox[syrine2], LV_STATE_CHECKED);
      lv_obj_clear_state(checkBox[syrine3], LV_STATE_CHECKED);

      NVS.putBool(checkBox_addr[syrine1], syringeChecked[syrine1]);  
      NVS.putBool(checkBox_addr[syrine2], syringeChecked[syrine2]);   
      NVS.putBool(checkBox_addr[syrine3], syringeChecked[syrine3]); 
      
      syringParam[0] = textAreaInput[Syringe1diameter] / 2;
      syringParam[1] = textAreaInput[Syringe1value];
      syringFlag = true;
    } 
    
    if(obj == checkBox[syrine2]) {
      Serial.println("checkBox_callBack --> Syringe 2 Checked Pressed.");
      syringeChecked[syrine1] = LV_STATE_UNCHECKED;
      syringeChecked[syrine2] = LV_STATE_CHECKED;
      syringeChecked[syrine3] = LV_STATE_UNCHECKED;
      
      lv_obj_clear_state(checkBox[syrine1], LV_STATE_CHECKED);
      lv_obj_add_state(checkBox[syrine2], syringeChecked[syrine2]);
      lv_obj_clear_state(checkBox[syrine3], LV_STATE_CHECKED);
      
      NVS.putBool(checkBox_addr[syrine1], syringeChecked[syrine1]);  
      NVS.putBool(checkBox_addr[syrine2], syringeChecked[syrine2]);   
      NVS.putBool(checkBox_addr[syrine3], syringeChecked[syrine3]);  
      
      syringParam[0] = textAreaInput[Syringe2diameter] / 2;
      syringParam[1] = textAreaInput[Syringe2value];
      syringFlag = true;
    } 

    if(obj == checkBox[syrine3]) {
      Serial.println("checkBox_callBack --> Syringe 3 Checked Pressed.");
      syringeChecked[syrine1] = LV_STATE_UNCHECKED;
      syringeChecked[syrine2] = LV_STATE_UNCHECKED;
      syringeChecked[syrine3] = LV_STATE_CHECKED;

      lv_obj_clear_state(checkBox[syrine1], LV_STATE_CHECKED); 
      lv_obj_clear_state(checkBox[syrine2], LV_STATE_CHECKED);   
      lv_obj_add_state(checkBox[syrine3], syringeChecked[syrine2]);

      NVS.putBool(checkBox_addr[syrine1], syringeChecked[syrine1]);  
      NVS.putBool(checkBox_addr[syrine2], syringeChecked[syrine2]);   
      NVS.putBool(checkBox_addr[syrine3], syringeChecked[syrine3]); 
      
      syringParam[0] = textAreaInput[Syringe3diameter] / 2;
      syringParam[1] = textAreaInput[Syringe3value];
      syringFlag = true; 
    }
  }
}

static void textArea_callBack(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
    lv_obj_t * kb = (lv_obj_t *) lv_event_get_user_data(e);
    
    if(code == LV_EVENT_FOCUSED) {
      lv_keyboard_set_textarea(kb, ta);
      lv_obj_clear_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }

    if(code == LV_EVENT_DEFOCUSED) {
      lv_keyboard_set_textarea(kb, NULL);
      lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }

    if(code == LV_EVENT_READY ) {
      const char * txt = lv_textarea_get_text(ta);
      if (ta == textArea[0]) {
        textAreaInput[Syringe1diameter] = atof(txt); 
        textAreaCheck[Syringe1diameter] = true;
        Serial.print("Syringe1diameter changed to : ");   Serial.println(textAreaInput[Syringe1diameter], 4);
        NVS.putFloat(textArea_addr[Syringe1diameter], textAreaInput[Syringe1diameter]);
        textAreaFlag = true;  

        syringParam[0] = textAreaInput[Syringe1diameter] / 2;
        syringParam[1] = textAreaInput[Syringe1value];
      }

      if (ta == textArea[1]) {
        textAreaInput[Syringe1value] = atof(txt); 
        textAreaCheck[Syringe1value] = true;
        Serial.print("Syringe1value changed to : ");   Serial.println(textAreaInput[Syringe1value], 4);
        NVS.putFloat(textArea_addr[Syringe1value], textAreaInput[Syringe1value]);
        textAreaFlag = true;

        syringParam[0] = textAreaInput[Syringe1diameter] / 2;
        syringParam[1] = textAreaInput[Syringe1value];
      }

      if (ta == textArea[2]) {
        textAreaInput[Syringe2diameter] = atof(txt); 
        textAreaCheck[Syringe2diameter] = true;
        Serial.print("Syringe2diameter changed to : ");   Serial.println(textAreaInput[Syringe2diameter], 4);
        NVS.putFloat(textArea_addr[Syringe2diameter], textAreaInput[Syringe2diameter]);
        textAreaFlag = true;

        syringParam[0] = textAreaInput[Syringe2diameter] / 2;
      syringParam[1] = textAreaInput[Syringe2value];
      }

      if (ta == textArea[3]) {
        textAreaInput[Syringe2value] = atof(txt); 
        textAreaCheck[Syringe2value] = true;
        NVS.putFloat(textArea_addr[Syringe2value], textAreaInput[Syringe2value]);
        textAreaFlag = true;

        syringParam[0] = textAreaInput[Syringe2diameter] / 2;
        syringParam[1] = textAreaInput[Syringe2value];
      }

      if (ta == textArea[4]) {
        textAreaInput[Syringe3diameter] = atof(txt); 
        textAreaCheck[Syringe3diameter] = true;
        Serial.print("Syringe3diameter changed to : ");   Serial.println(textAreaInput[Syringe3diameter], 4);
        NVS.putFloat(textArea_addr[Syringe3diameter], textAreaInput[Syringe3diameter]);
        textAreaFlag = true;

        syringParam[0] = textAreaInput[Syringe3diameter] / 2;
        syringParam[1] = textAreaInput[Syringe3value];
      }

      if (ta == textArea[5]) {
        textAreaInput[Syringe3value] = atof(txt); 
        textAreaCheck[Syringe3value] = true;
        Serial.print("Syringe3value changed to : ");   Serial.println(textAreaInput[Syringe3value], 4);
        NVS.putFloat(textArea_addr[Syringe3value], textAreaInput[Syringe3value]);
        textAreaFlag = true;

        syringParam[0] = textAreaInput[Syringe3diameter] / 2;
        syringParam[1] = textAreaInput[Syringe3value];
      }

      if (ta == textArea[6]) {
        if (motorMode == secMode) {
          textAreaInput[proccessValue] = atof(txt);
        } else if (motorMode == minMode) {
          textAreaInput[proccessValue] = atof(txt) * (float)60;
        } else if (motorMode == hrMode) {
          textAreaInput[proccessValue] = atof(txt) * (float)60 * (float)60;
        } else if (motorMode == ulMode) {
          textAreaInput[proccessValue] = atof(txt) / (float)1000;
        } else if (motorMode == mlMode) {
          textAreaInput[proccessValue] = atof(txt);
        }

        NVS.putFloat(textArea_addr[proccessValue], atof(txt));
          
        textAreaCheck[proccessValue] = true;
        Serial.print("Value changed to : ");   Serial.println(textAreaInput[proccessValue], 4);
        textAreaFlag = true;
      }

      if (ta == textArea[7]) {
        if (speedMode == uLsecMode) {
          textAreaInput[proccessSpeed] = atof(txt) / (float)1000;
        } else if (speedMode == uLminMode) {
          textAreaInput[proccessSpeed] = atof(txt) / (float)1000 / (float)60;
        } else if (speedMode == mLsecMode) {
          textAreaInput[proccessSpeed] = atof(txt);
        } else if (speedMode == mLminMode) {
          textAreaInput[proccessSpeed] = atof(txt) / (float)60;
        } else if (speedMode == mLhrMode) {
          textAreaInput[proccessSpeed] = atof(txt) / (float)60 / (float)60;
        }

        NVS.putFloat(textArea_addr[proccessSpeed], atof(txt));
 
        textAreaCheck[proccessSpeed] = true;
        Serial.print("proccess Speed changed to : ");   Serial.println(textAreaInput[proccessSpeed], 4);
        textAreaFlag = true;
      }

      if (ta == textArea[8]) {
        textAreaInput[calibration] = atof(txt); 
        textAreaCheck[calibration] = true;
        Serial.print("calibration changed to : ");   Serial.println(textAreaInput[calibration], 4);
        NVS.putFloat(textArea_addr[calibration], textAreaInput[calibration]);
        textAreaFlag = true;
      }

      lv_keyboard_set_textarea(kb, NULL);
      lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
    }
}

static void dropDown_callBack(lv_event_t * e) {
  lv_event_code_t code = lv_event_get_code(e);
  lv_obj_t * obj = lv_event_get_target(e);
  
  if(code == LV_EVENT_VALUE_CHANGED) {
    char buf[32];
    lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
    
    if (obj == dropDown[0]) {
      if (String(buf) == String(timeinputUnit[0])) {
        dropDownValue[0] = String(timeinputUnit[0]);
        dropDownValueInt[0] = 0;   
        dropDownCheck[0] = true;      
        Serial.println("Motor Mode Changed To : " + String(dropDownValueInt[0]));
        NVS.putInt(dropDown_addr[0], dropDownValueInt[0]);
        motorMode = secMode;
      }

      if (String(buf) == String(timeinputUnit[1])) {
        dropDownValue[0] = String(timeinputUnit[1]);
        dropDownValueInt[0] = 1;
        dropDownCheck[0] = true;      
        Serial.println("Motor Mode Changed To : " + String(dropDownValueInt[0]));
        NVS.putInt(dropDown_addr[0], dropDownValueInt[0]);
        motorMode = minMode;
      }

      if (String(buf) == String(timeinputUnit[2])) {
        dropDownValue[0] = String(timeinputUnit[2]);
        dropDownValueInt[0] = 2;
        dropDownCheck[0] = true;      
        Serial.println("Motor Mode Changed To : " + String(dropDownValueInt[0]));
        NVS.putInt(dropDown_addr[0], dropDownValueInt[0]);
        motorMode = hrMode;
      }

      if (String(buf) == String(timeinputUnit[3])) {
        dropDownValue[0] = String(timeinputUnit[3]);
        dropDownValueInt[0] = 3;
        dropDownCheck[0] = true;      
        Serial.println("Motor Mode Changed To : " + String(dropDownValueInt[0]));
        NVS.putInt(dropDown_addr[0], dropDownValueInt[0]);
        motorMode = ulMode;
      }

      if (String(buf) == String(timeinputUnit[4])) {
        dropDownValue[0] = String(timeinputUnit[4]);
        dropDownValueInt[0] = 4;
        dropDownCheck[0] = true;      
        Serial.println("Motor Mode Changed To : " + String(dropDownValueInt[0]));
        NVS.putInt(dropDown_addr[0], dropDownValueInt[0]);
        motorMode = mlMode;
      }
      
      dropDownFlag = true;
    }

    if (obj == dropDown[1]) {
      if (String(buf) == String(speedinputUnit[0])) {
        dropDownValue[1] = String(speedinputUnit[0]);
        dropDownValueInt[1] = 0;
        dropDownCheck[1] = true;        
        Serial.println("Speed Mode Changed To : " + String(dropDownValueInt[1]));
        NVS.putInt(dropDown_addr[1], dropDownValueInt[1]);
        speedMode = uLsecMode;
      }

      if (String(buf) == String(speedinputUnit[1])) {
        dropDownValue[1] = String(speedinputUnit[1]);
        dropDownValueInt[1] = 1;
        dropDownCheck[1] = true;        
        Serial.println("Speed Mode Changed To : " + String(dropDownValueInt[1]));
        NVS.putInt(dropDown_addr[1], dropDownValueInt[1]);
        speedMode = uLminMode;
      }

      if (String(buf) == String(speedinputUnit[2])) {
        dropDownValue[1] = String(speedinputUnit[2]);
        dropDownValueInt[1] = 2;
        dropDownCheck[1] = true;        
        Serial.println("Speed Mode Changed To : " + String(dropDownValueInt[1]));
        NVS.putInt(dropDown_addr[1], dropDownValueInt[1]);
        speedMode = mLsecMode;
      }

      if (String(buf) == String(speedinputUnit[3])) {
        dropDownValue[1] = String(speedinputUnit[3]);
        dropDownValueInt[1] = 3;
        dropDownCheck[1] = true;        
        Serial.println("Speed Mode Changed To : " + String(dropDownValueInt[1]));
        NVS.putInt(dropDown_addr[1], dropDownValueInt[1]);
        speedMode = mLminMode;
      }

      if (String(buf) == String(speedinputUnit[4])) {
        dropDownValue[1] = String(speedinputUnit[4]);
        dropDownValueInt[1] = 4;
        dropDownCheck[1] = true;        
        Serial.println("Speed Mode Changed To : " + String(dropDownValueInt[1]));
        NVS.putInt(dropDown_addr[1], dropDownValueInt[1]);
        speedMode = mLhrMode;
      }

      dropDownFlag = true;
    }
  }
}

/* Sheet Design Function*/
/********************************************************************************************************/

void lv_init_style(void) {
  lv_style_init(&style_bg);
  lv_style_set_border_color(&style_bg, lv_palette_main(LV_PALETTE_RED));
  lv_style_set_border_width(&style_bg, 2);
  lv_style_set_pad_all(&style_bg, 6); /*To make the indicator smaller*/
  lv_style_set_radius(&style_bg, 6);
  lv_style_set_anim_time(&style_bg, 300);

  /********************************************************************************************************/

  lv_style_init(&style_indic);
  lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
  lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_radius(&style_indic, 3);

  /********************************************************************************************************/
  
  /*Init the style for the default state*/
  lv_style_init(&btn_style_1);

  lv_style_set_radius(&btn_style_1, 3);

  lv_style_set_bg_opa(&btn_style_1, LV_OPA_100);
  lv_style_set_bg_color(&btn_style_1, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_bg_grad_color(&btn_style_1, lv_palette_darken(LV_PALETTE_BLUE, 2));
  lv_style_set_bg_grad_dir(&btn_style_1, LV_GRAD_DIR_VER);

  lv_style_set_border_opa(&btn_style_1, LV_OPA_40);
  lv_style_set_border_width(&btn_style_1, 2);
  lv_style_set_border_color(&btn_style_1, lv_palette_main(LV_PALETTE_BLUE));

  // lv_style_set_shadow_width(&btn_style_1, 2);
  // lv_style_set_shadow_color(&btn_style_1, lv_palette_main(LV_PALETTE_BLUE));
  // lv_style_set_shadow_ofs_y(&btn_style_1, 2);

  lv_style_set_outline_opa(&btn_style_1, LV_OPA_COVER);
  lv_style_set_outline_color(&btn_style_1, lv_palette_main(LV_PALETTE_BLUE));

  lv_style_set_text_color(&btn_style_1, lv_color_white());
  lv_style_set_pad_all(&btn_style_1, 10);

  /********************************************************************************************************/

  /*Init the pressed style*/
  lv_style_init(&btn_style_2);

  /*Ad a large outline when pressed*/
  lv_style_set_outline_width(&btn_style_2, 30);
  lv_style_set_outline_opa(&btn_style_2, LV_OPA_TRANSP);

  lv_style_set_translate_y(&btn_style_2, 5);
  lv_style_set_shadow_ofs_y(&btn_style_2, 3);
  lv_style_set_bg_color(&btn_style_2, lv_palette_darken(LV_PALETTE_RED, 2));
  lv_style_set_bg_grad_color(&btn_style_2, lv_palette_darken(LV_PALETTE_RED, 4));

  /********************************************************************************************************/
  
}

void motorSheetConf(void) {
  lv_obj_t * label;

  finalSheet = lv_obj_create(lv_scr_act());
  lv_obj_set_size(finalSheet, 350, 250);
  lv_obj_set_scrollbar_mode(finalSheet, LV_SCROLLBAR_MODE_OFF);
  lv_obj_align(finalSheet, LV_ALIGN_TOP_MID, -5, -5);

  barHandler = lv_bar_create(finalSheet);
  lv_obj_remove_style_all(barHandler);  /*To have a clean start*/
  lv_obj_add_style(barHandler, &style_bg, 0);
  lv_obj_add_style(barHandler, &style_indic, LV_PART_INDICATOR);
  lv_obj_set_size(barHandler, 200, 20);
  lv_bar_set_value(barHandler, residueValue, LV_ANIM_ON);
  lv_obj_align(barHandler, LV_ALIGN_CENTER, 0, -50);

  label = lv_label_create(finalSheet);
  lv_label_set_text(label, "Syringe residue :");
  lv_obj_align_to(label, barHandler, LV_ALIGN_OUT_TOP_MID, 0, -10);

  btn[pauseBtn] = lv_btn_create(finalSheet);
  lv_obj_remove_style_all(btn[pauseBtn]);                          /*Remove the style coming from the theme*/
  lv_obj_add_style(btn[pauseBtn], &btn_style_1, 0);
  lv_obj_add_style(btn[pauseBtn], &btn_style_2, LV_STATE_PRESSED);
  lv_obj_set_size(btn[pauseBtn], 100, 40);
  lv_obj_align(btn[pauseBtn], LV_ALIGN_CENTER, 0, +10);
  lv_obj_add_event_cb(btn[pauseBtn], button_callBack, LV_EVENT_ALL, NULL);

  label = lv_label_create(btn[pauseBtn]);
  lv_label_set_text(label, "Pause");
  lv_obj_center(label);

  btn[stopBtn] = lv_btn_create(finalSheet);
  lv_obj_remove_style_all(btn[stopBtn]);                          /*Remove the style coming from the theme*/
  lv_obj_add_style(btn[stopBtn], &btn_style_1, 0);
  lv_obj_add_style(btn[stopBtn], &btn_style_2, LV_STATE_PRESSED);
  lv_obj_set_size(btn[stopBtn], 100, 40);
  lv_obj_align(btn[stopBtn], LV_ALIGN_CENTER, 0, +60);
  lv_obj_add_event_cb(btn[stopBtn], button_callBack, LV_EVENT_ALL, NULL);

  label = lv_label_create(btn[stopBtn]);
  lv_label_set_text(label, "Stop");
  lv_obj_center(label);
}

void starterSheetConf(void) {
  lv_obj_t * label;
  starterSheet = lv_obj_create(lv_scr_act());
  lv_obj_set_size(starterSheet, 350, 250);
  lv_obj_set_scrollbar_mode(starterSheet, LV_SCROLLBAR_MODE_OFF);
  lv_obj_align(starterSheet, LV_ALIGN_TOP_MID, -5, -5);
  lv_obj_t *tv1 = lv_tileview_create(starterSheet);

  /* Tile 1 : Start Sheet */
  /********************************************************************************************************/
 
  tile1 = lv_tileview_add_tile(tv1, 0, 0, LV_DIR_RIGHT);  

  lv_obj_t * panel = lv_obj_create(tile1);
  lv_obj_set_size(panel, 316, 130);
  lv_obj_set_scrollbar_mode(panel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_align(panel, LV_ALIGN_TOP_MID, 0, 2);

  checkBox[syrine1] = lv_checkbox_create(panel);
  lv_checkbox_set_text(checkBox[syrine1], "Syringe 1 :");
  lv_obj_add_state(checkBox[syrine1], syringeChecked[syrine1]);
  lv_obj_add_event_cb(checkBox[syrine1], checkBox_callBack, LV_EVENT_ALL, NULL);
  lv_obj_align(checkBox[syrine1], LV_ALIGN_TOP_LEFT, 10, 40);
  
  checkBox[syrine2] = lv_checkbox_create(panel);
  lv_checkbox_set_text(checkBox[syrine2], "Syringe 2 :");
  lv_obj_add_state(checkBox[syrine2], syringeChecked[syrine2]);
  lv_obj_add_event_cb(checkBox[syrine2], checkBox_callBack, LV_EVENT_ALL, NULL);
  lv_obj_align(checkBox[syrine2], LV_ALIGN_TOP_LEFT, 10, 170);

  checkBox[syrine3] = lv_checkbox_create(panel);
  lv_checkbox_set_text(checkBox[syrine3], "Syringe 3 :");
  lv_obj_add_state(checkBox[syrine3], syringeChecked[syrine3]);
  lv_obj_add_event_cb(checkBox[syrine3], checkBox_callBack, LV_EVENT_ALL, NULL);
  lv_obj_align(checkBox[syrine3], LV_ALIGN_TOP_LEFT, 10, 300);

  /*Create an array for the points of the line*/
  static lv_point_t line_points1[] = { {5, 5}, {315, 5}, {315, 235}, {5, 235}, {5, 5} };
  static lv_point_t line_points2[] = { {13, 110}, {13, 130}, {307, 130}, {307, 110} };

  /*Create style*/
  static lv_style_t style_line;
  lv_style_init(&style_line);
  lv_style_set_line_width(&style_line, 2);
  lv_style_set_line_color(&style_line, lv_palette_main(LV_PALETTE_BLUE));
  lv_style_set_line_rounded(&style_line, true);

  /*Create a line and apply the new style*/
  lv_obj_t * line1;
  line1 = lv_line_create(tile1);
  lv_line_set_points(line1, line_points1, 5);     /*Set the points*/
  lv_obj_add_style(line1, &style_line, 0);
  
  line1 = lv_line_create(tile1);
  lv_line_set_points(line1, line_points2, 4);     /*Set the points*/
  lv_obj_add_style(line1, &style_line, 0);

  /*Create a keyboard to use it with an of the text areas*/
  keyBoard = lv_keyboard_create(lv_scr_act());

  char buffer[10];
  /*Create a text area. The keyboard will write here*/
  textArea[0] = lv_textarea_create(panel);
  lv_obj_align(textArea[0], LV_ALIGN_TOP_LEFT, 135, 7);
  lv_obj_add_event_cb(textArea[0], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[0], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe1diameter]);
  lv_textarea_add_text(textArea[0], buffer);
  lv_obj_set_size(textArea[0], 100, 40);

  textArea[1] = lv_textarea_create(panel);
  lv_obj_align(textArea[1], LV_ALIGN_TOP_LEFT, 135, 57);
  lv_obj_add_event_cb(textArea[1], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[1], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe1value]);
  lv_textarea_add_text(textArea[1], buffer);
  lv_obj_set_size(textArea[1], 100, 40);

  textArea[2] = lv_textarea_create(panel);
  lv_obj_align(textArea[2], LV_ALIGN_TOP_LEFT, 135, 137);
  lv_obj_add_event_cb(textArea[2], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[2], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe2diameter]);
  lv_textarea_add_text(textArea[2], buffer);
  lv_obj_set_size(textArea[2], 100, 40);

  textArea[3] = lv_textarea_create(panel);
  lv_obj_align(textArea[3], LV_ALIGN_TOP_LEFT, 135, 187);
  lv_obj_add_event_cb(textArea[3], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[3], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe2value]);
  lv_textarea_add_text(textArea[3], buffer);
  lv_obj_set_size(textArea[3], 100, 40);

  textArea[4] = lv_textarea_create(panel);
  lv_obj_align(textArea[4], LV_ALIGN_TOP_LEFT, 135, 267);
  lv_obj_add_event_cb(textArea[4], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[4], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe3diameter]);
  lv_textarea_add_text(textArea[4], buffer);
  lv_obj_set_size(textArea[4], 100, 40);

  textArea[5] = lv_textarea_create(panel);
  lv_obj_align(textArea[5], LV_ALIGN_TOP_LEFT, 135, 317);
  lv_obj_add_event_cb(textArea[5], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[5], " ");
  sprintf(buffer, "%0.4f", textAreaInput[Syringe3value]);
  lv_textarea_add_text(textArea[5], buffer);
  lv_obj_set_size(textArea[5], 100, 40);  

  lv_keyboard_set_textarea(keyBoard, textArea[0]);
  lv_keyboard_set_textarea(keyBoard, textArea[1]);
  lv_keyboard_set_textarea(keyBoard, textArea[2]);
  lv_keyboard_set_textarea(keyBoard, textArea[3]);
  lv_keyboard_set_textarea(keyBoard, textArea[4]);
  lv_keyboard_set_textarea(keyBoard, textArea[5]);
  
  btn[startBtn] = lv_btn_create(tile1);
  lv_obj_remove_style_all(btn[startBtn]);                          /*Remove the style coming from the theme*/
  lv_obj_add_style(btn[startBtn], &btn_style_1, 0);
  lv_obj_add_style(btn[startBtn], &btn_style_2, LV_STATE_PRESSED);
  lv_obj_set_size(btn[startBtn], 100, 40);
  lv_obj_align(btn[startBtn], LV_ALIGN_CENTER, 0, +60);
  lv_obj_add_event_cb(btn[startBtn], button_callBack, LV_EVENT_ALL, NULL);

  label = lv_label_create(btn[startBtn]);
  lv_label_set_text(label, "Start");
  lv_obj_center(label);

  /* Tile 2 : Configuration Sheet */
  /********************************************************************************************************/
  tile2 = lv_tileview_add_tile(tv1, 1, 0, LV_DIR_LEFT);
  
  lv_obj_t * confPanel = lv_obj_create(tile2);
  
  lv_obj_set_size(confPanel, 316, 130);
  lv_obj_set_scrollbar_mode(confPanel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_align(confPanel, LV_ALIGN_TOP_MID, 0, 2);
  
  line1 = lv_line_create(tile2);
  lv_line_set_points(line1, line_points1, 5);   
  lv_obj_add_style(line1, &style_line, 0);

  line1 = lv_line_create(tile2);
  lv_line_set_points(line1, line_points2, 4);     /*Set the points*/
  lv_obj_add_style(line1, &style_line, 0);

  label = lv_label_create(confPanel);
  lv_label_set_text(label, "Volume :");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 40);
  
  label = lv_label_create(confPanel);
  lv_label_set_text(label, "Speed :");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 160);
    
  label = lv_label_create(confPanel);
  lv_label_set_text(label, "Calibration :");
  lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 290);

  textArea[6] = lv_textarea_create(confPanel);
  lv_obj_align(textArea[6], LV_ALIGN_TOP_LEFT, 135, 7);
  lv_obj_add_event_cb(textArea[6], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[6], " ");
  sprintf(buffer, "%0.4f", textAreaInput[proccessValue]);
  lv_textarea_add_text(textArea[6], buffer);
  lv_obj_set_size(textArea[6], 100, 40);  

  textArea[7] = lv_textarea_create(confPanel);
  lv_obj_align(textArea[7], LV_ALIGN_TOP_LEFT, 135, 127);
  lv_obj_add_event_cb(textArea[7], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[7], " ");
  sprintf(buffer, "%0.4f", textAreaInput[proccessSpeed]);
  lv_textarea_add_text(textArea[7], buffer);
  lv_obj_set_size(textArea[7], 100, 40); 
  
  textArea[8] = lv_textarea_create(confPanel);
  lv_obj_align(textArea[8], LV_ALIGN_TOP_LEFT, 135, 280);
  lv_obj_add_event_cb(textArea[8], textArea_callBack, LV_EVENT_ALL, keyBoard);
  lv_textarea_set_placeholder_text(textArea[8], " ");
  sprintf(buffer, "%0.4f", textAreaInput[calibration]);
  lv_textarea_add_text(textArea[8], buffer);
  lv_obj_set_size(textArea[8], 100, 40); 

  lv_keyboard_set_textarea(keyBoard, textArea[6]);
  lv_keyboard_set_textarea(keyBoard, textArea[7]);
  lv_keyboard_set_textarea(keyBoard, textArea[8]);
  lv_keyboard_set_mode(keyBoard, LV_KEYBOARD_MODE_NUMBER);
  lv_obj_add_flag(keyBoard, LV_OBJ_FLAG_HIDDEN);
  lv_obj_align(keyBoard, LV_ALIGN_BOTTOM_MID, 0, -20);

  dropDown[0] = lv_dropdown_create(confPanel);
  lv_dropdown_set_options_static(dropDown[0], timeOpt);
  lv_obj_set_size(dropDown[0], 100, 40);
  lv_obj_align(dropDown[0], LV_ALIGN_TOP_LEFT, 135, 58);
  lv_dropdown_set_selected(dropDown[0], dropDownValueInt[0]);
  lv_dropdown_set_selected_highlight(dropDown[0], true);
  lv_obj_add_event_cb(dropDown[0], dropDown_callBack, LV_EVENT_ALL, NULL);

  dropDown[1] = lv_dropdown_create(confPanel);
  lv_dropdown_set_options_static(dropDown[1], speedOpt);
  lv_obj_set_size(dropDown[1], 100, 40);
  lv_obj_align(dropDown[1], LV_ALIGN_TOP_LEFT, 135, 178);
  lv_dropdown_set_selected(dropDown[1], dropDownValueInt[1]);
  lv_dropdown_set_selected_highlight(dropDown[1], true);
  lv_obj_add_event_cb(dropDown[1], dropDown_callBack, LV_EVENT_ALL, NULL);

  btn[moveBtnP] = lv_btn_create(tile2);
  lv_obj_remove_style_all(btn[moveBtnP]);                          /*Remove the style coming from the theme*/
  lv_obj_add_style(btn[moveBtnP], &btn_style_1, 0);
  lv_obj_add_style(btn[moveBtnP], &btn_style_2, LV_STATE_PRESSED);
  lv_obj_set_size(btn[moveBtnP], 100, 40);
  lv_obj_align(btn[moveBtnP], LV_ALIGN_CENTER, -60, +60);
  lv_obj_add_event_cb(btn[moveBtnP], button_callBack, LV_EVENT_ALL, NULL);

  label = lv_label_create(btn[moveBtnP]);
  lv_label_set_text(label, "Move +");
  lv_obj_center(label);

  btn[moveBtnN] = lv_btn_create(tile2);
  lv_obj_remove_style_all(btn[moveBtnN]);                          /*Remove the style coming from the theme*/
  lv_obj_add_style(btn[moveBtnN], &btn_style_1, 0);
  lv_obj_add_style(btn[moveBtnN], &btn_style_2, LV_STATE_PRESSED);
  lv_obj_set_size(btn[moveBtnN], 100, 40);
  lv_obj_align(btn[moveBtnN], LV_ALIGN_CENTER, +60, +60);
  lv_obj_add_event_cb(btn[moveBtnN], button_callBack, LV_EVENT_ALL, NULL);

  label = lv_label_create(btn[moveBtnN]);
  lv_label_set_text(label, "Move -");
  lv_obj_center(label);
}

/* Graphic Function*/
/********************************************************************************************************/

void am_set_bar_value(lv_obj_t * handler, int32_t value) {
  lv_bar_set_value(handler, residueValue, LV_ANIM_ON);  
}
