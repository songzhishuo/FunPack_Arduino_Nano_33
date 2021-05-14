//任务目的：利用NANO-33 BLE的传感器，搭建一个小型环境监测站用于监测户外环境。待监测的参数包括：
//
//· 周边环境温度（精度：±0.1°C, ±0.1°F）     //HTS221
//· 周边环境湿度（精度：±1%）
//· 大气压强（精度：±0.1kPa, ±0.1psi）      //LPS22HB
//· 日照强度（用于判断白天/夜晚）
//· 周边平均噪声（精度：±1dB）

#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_APDS9960.h>

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#include <PDM.h>

#define SERIAL_DEBUG 0
#define DISPLAY_PAGE_MAX 3
void vGet_Temp_Hum_Val(float *tem, float *hum); //温湿度的读取
void vGet_Press(float *press);                  //获取气压
void onPDMdata();
/*OLED初始化 U8G2*/
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED


/*音频相关参数*/
boolean isPDMReady = false;
short pdmBuffer[512];
volatile int pdmSize;



void setup()
{
    // put your setup code here, to run once:

    //初始化声音传感器
    //PDM.onReceive(onPDMdata);
    //PDM.begin(1, 16000);

    //初始化串口
    Serial.begin(9600);
    while (!Serial)
        ;

    //麦克风音频初始化
    vPDM_init();

    //初始化温度湿度传感器
    if (!HTS.begin())
    { //HTS温湿度传感器初始化
#if SERIAL_DEBUG   
        Serial.println("Failed to initialize humidity temperature sensor!");
#endif // SERIAL_DEBUG  
        while (1)
            ;
    }
    //初始化气压传感器
    if (!BARO.begin())
    {
#if SERIAL_DEBUG          
        Serial.println("Failed to initialize pressure sensor!");
#endif
        while (1)
            ;
    }
    //初始化颜色传感器
    if (!APDS.begin())
    {
#if SERIAL_DEBUG          
        Serial.println("Error initializing APDS9960 sensor.");
#endif
    }

    u8g2.begin();
    u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
}

float f_tem = 0;
float f_hum = 0;
float f_press = 0;

unsigned int ui_press = 0;
unsigned int ui_decibel = 0;
unsigned char uc_page = 0,sec_flag;
void loop()
{
    // put your main code here, to run repeatedly:

    vGet_Temp_Hum_Val(&f_tem, &f_hum); //临时温湿度的读取
    vGet_Press(&f_press);              //临时气压值读取
    ui_press = uiGet_Bright();         //临时亮度读取 
    //vDisplay_test(); //显示函数
    ui_decibel = uiGet_PCM_data();     //临时分贝获取

    //vDisplay_normal(float tem, float hum, float press, unsigned int bright, int decibel)
    vDisplay_normal(uc_page, f_tem, f_hum, f_press, ui_press, ui_decibel);
    //vRGB2Bright(int r_val, int g_val, int b_val)
    // wait 1 second to print again

    sec_flag++;
    if(sec_flag >= 3)               //
    {
        sec_flag = 0;
        uc_page++;
        if(uc_page >= DISPLAY_PAGE_MAX+1)
        {
            uc_page = 0;
        }
    }
    delay(1000);
}

unsigned int uiGet_Bright() //获取亮度值
{
    int r, g, b;
    unsigned int ret = 0;
    while (!APDS.colorAvailable())
    {
        delay(5);
    }
    APDS.readColor(r, g, b);

#if SERIAL_DEBUG  
    // print the values
    Serial.print("r = ");
    Serial.println(r);
    Serial.print("g = ");
    Serial.println(g);
    Serial.print("b = ");
    Serial.println(b);
    Serial.println();
#endif
    ret = uiRGB2Bright(r, g, b);
#if SERIAL_DEBUG      
    Serial.print("bright = ");
    Serial.println(ret);
#endif    
    return ret;
}

void vGet_Temp_Hum_Val(float *tem, float *hum)
{
    float temperature = HTS.readTemperature();
    float humidity = HTS.readHumidity();

    *tem = temperature;
    *hum = humidity;
#if SERIAL_DEBUG  
    // print each of the sensor values
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °F");

    Serial.print("Humidity    = ");
    Serial.print(humidity);
    Serial.println(" %");

    // print an empty line
    Serial.println();
#endif    
}

void vGet_Press(float *press)
{
    float pressure = BARO.readPressure(PSI);

    *press = pressure;
#if SERIAL_DEBUG  
    // print the sensor value
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" psi");

    // print an empty line
    Serial.println();
#endif
}

unsigned int uiRGB2Bright(int r_val, int g_val, int b_val)
{
    int Y;
    Y = ((r_val * 299) + (g_val * 587) + (b_val * 114)) / 1000;
    return Y;
}

void vDisplay_test()
{
    u8g2.clearBuffer();                  // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);  // choose a suitable font
    u8g2.drawStr(0, 10, "Hello World!"); // write something to the internal memory
    u8g2.sendBuffer();                   // transfer internal memory to the display
}


/**
 * @brief: OLED显示屏显示
 * @param:[IN] page:显示页面
 * @param:[IN] tem:温度数据
 * @param:[IN] hum:温度数据
 * @param:[IN] press:压强数据
 * @param:[IN] bright:亮度数据
 * @param:[IN] decibel:声音平均
 * @retval: none
 */ 
void vDisplay_normal(unsigned char page,float tem, float hum, float press, unsigned int bright, int decibel)
{
    char line0_buf[128] = {0};
    char line1_buf[128] = {0};
    char line2_buf[128] = {0};
    if(page == 0)                                   //tem  hum
    {
        sprintf(line0_buf,"tem: %.2f ",tem);
        sprintf(line1_buf,"hum: %.2f ",hum);
    }
    else if(page == 1)
    {
        sprintf(line0_buf,"press:");
        sprintf(line1_buf,"              %.2f ",press);        
    }
    else if(page == 2)
    {
        sprintf(line0_buf,"bright:");
        sprintf(line1_buf,"                   %d ",bright);        
    }
    else if(page == 3)
    {
        sprintf(line0_buf,"decibel:");
        sprintf(line1_buf,"                   %d ",decibel);        
    }

    u8g2.clearBuffer();                  // clear the internal memory
    
    u8g2.setFont(u8g2_font_tenfatguys_tf);  // choose a suitable font  //https://github.com/olikraus/u8g2/wiki/fntlistall

    u8g2.drawStr(0, 15, line0_buf); // write something to the internal memory
    u8g2.drawStr(0, 30, line1_buf);
    u8g2.sendBuffer();                   // transfer internal memory to the display
}

/**
 * @brief: 音频初始化
 * @param: none
 * @retval: none
 */ 
void vPDM_init()
{
  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and -10 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(1, 16000)) {
#if SERIAL_DEBUG        
    Serial.println("Failed to start PDM!");
#endif    
  }
}

/**
 * @brief: 获取音频数据
 * @param: none
 * @retval: 音频分贝
 */ 
unsigned int uiGet_PCM_data()
{
    unsigned int data = 0;
    if (pdmSize) {
    //   Serial.println(PCM2DecibelRMS(pdmBuffer, pdmSize));
        data = PCM2Decibel(pdmBuffer, pdmSize);
#if SERIAL_DEBUG          
        Serial.print("PCM data:");
        Serial.println(data);
#endif        
        pdmSize = 0;
    }
    else
    {
        data = 0;
    }

    return data;
}


void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(pdmBuffer, bytesAvailable);
  pdmSize = bytesAvailable / 2; // 16-bit, 2 bytes per sample
}

/** 
 * Reference: https://en.wikipedia.org/wiki/Decibel
 * @param pcmData
 * @param pcmSize 
 * @return 
 */  
int PCM2DecibelRMS(const short *pcmData, size_t pcmSize) {  
    int decibel = 0;  
    double sum = 0;
    for(int i = 0; i < pcmSize; i++)  {  
        sum += pcmData[i] * pcmData[i]; //sum of values square
    }  
    return (int)(10 * log10(sum / (pcmSize * 32768 * 32768)));  //10*log10(RMS), root mean square
} 

int PCM2Decibel(const short *pcmData, size_t pcmSize) {  
    int decibel = 0;  
    double sum = 0;
    for(int i = 0; i < pcmSize; i++)  {  
        sum += abs(pcmData[i]); //sum of values
    }  
    return (int)(20.0 * log10(sum / pcmSize));  
} 










#if 0

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#define SCL A5
#define SDA A4



// Please UNCOMMENT one of the contructor lines below
// U8g2 Contructor List (Frame Buffer)
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 21, /* data=*/ 20, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather M0 Basic Proto + FeatherWing OLED
//U8G2_SSD1306_128X32_UNIVISION_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // Adafruit Feather ESP8266/32u4 Boards + FeatherWing OLED
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);  // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED
//U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
//U8G2_SSD1306_128X32_WINSTAR_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // pin remapping with ESP8266 HW I2C
//U8G2_SSD1306_64X48_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // EastRising 0.66" OLED breakout board, Uno: A4=SDA, A5=SCL, 5V powered
//U8G2_SSD1306_48X64_WINSTAR_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   
//U8G2_SSD1306_64X32_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
//U8G2_SSD1306_64X32_1F_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
//U8G2_SSD1306_96X16_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // EastRising 0.69" OLED
//U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // EastRising 0.42" OLED


// End of constructor list




  unsigned char cp;
void setup(void) {
  u8g2.begin();
  //disableUTF8Print
  //u8g2.enableUTF8Print();		// enable UTF8 support for the Arduino print() function
  //u8g2.disableUTF8Print();
}

void loop(void) {
  char buf[128]={0};
#if 0  
  u8g2.setFont(u8g2_font_unifont_t_chinese1);  // use chinese2 for all the glyphs of "你好世界"
  u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  u8g2.setCursor(0, 15);
  u8g2.print("Hello World!");
  u8g2.setCursor(0, 25);
  u8g2.print("你好世界");		// Chinese "Hello World" 
  u8g2.sendBuffer();
#endif

#if 0
  //u8g2.setFont(u8g2_font_ncenB08_tr); 
  u8g2.setFont(u8g2_font_logisoso16_tf); 
  //u8g2.setFontDirection(0);
  u8g2.clearBuffer();
  sprintf(buf,"Hello world [%d]",cp);
  u8g2.drawStr(0,10,buf);
  //u8g2.drawStr(0,10,"Hello World!");
  u8g2.sendBuffer();
  cp++;
  delay(1000);
#endif
   u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(0,10,"Hello World!");  // write something to the internal memory
  u8g2.sendBuffer();          // transfer internal memory to the display
  delay(1000);  
}

#endif
