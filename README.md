# Part1 自我介绍

很有幸参与到这次Digi-key和硬禾学堂联合举办的这次开发板体验活动。我叫Argon，来自浙江杭州是一名从事 物联网行业）的嵌入式网络工程师。从大学开始就对微电子充满了兴趣，在闲暇的时间也会利用手头的元器件自己搭建一些好玩的DIY产品。

# Part2 硬件介绍

本期使用的开发板是Arduino Nano 33 BLE Sense ,这款开发板板载了丰富的传感器外设并且支持Arduino语法编程，并且这款开发板支持TinyML进行简单的机器学习，非常适合作为方案选型验证。

![FgKx7Ve00GuZqTs1G1H5LZWa9QwB](https://gitee.com/song_zhi_shu/my-image-host/raw/master/img/FgKx7Ve00GuZqTs1G1H5LZWa9QwB)

# Part3 设计思路

### 任务分析

利用NANO-33 BLE的传感器, 搭建一个小型环境监测站用于监测户外环境。待监测的参数包括:

- 周边环境温度（精度：±0.1°C, ±0.1°F）
- 周边环境湿度（精度：±1%）
- 大气压强（精度：±0.1kPa, ±0.1psi）
- 日照强度（用于判断白天/夜晚）
- 周边平均噪声（精度：±1dB）

### 任务拆解

任务可以拆解为以下几个部分：

- 环境搭建，安装所需的编译器，下载所需的资料
- 编写各个外设部分的驱动
- U8g2屏幕显示驱动的移植
- 音频数据处理和解析
- 简单UI的绘制

# Part4 部分功能代码

## MIC部分

因为MP34DTO5传感器采集输出的为瞬时音量大小，因此需要通过音量计算将其转换为分贝。转换代码如下：

```c++
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

```

## 光强部分

光照强度采用了距离传感器APD9960进行获得，该传感器实际得到的为GRB三个的采样值，需要对其进行混光处理计算出实际的白光亮度，详细实现代码如下：

```c++
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

unsigned int uiRGB2Bright(int r_val, int g_val, int b_val)
{
    int Y;
    Y = ((r_val * 299) + (g_val * 587) + (b_val * 114)) / 1000;
    return Y;
}
```

## 压力部分

压力传感器采用的LPS22HB，该传感器可以实时测定大气压强并且Arduino已经对其进行了库函数封装直接调用即可读取压强，详细代码实现如下：

```c++
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
```



## 温湿度部分

温度湿度获取采用了一体化温湿度传感器HTS221，Arduino也对其接口进行了封装，因此直接读取即可，详细代码如下：

```c++
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
```

## U8g2 显示部分

显示部分采用了ssd1306作为显示模块，在U8g2库中已经对ssd1306驱动进行了封装，因此我们只需要在使用的时候调用arduindo接口对其进行相应的实例化和初始化即可，部分代码如下所示：

```c++
/*OLED初始化 U8G2*/
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE); // Adafruit ESP8266/32u4/ARM Boards + FeatherWing OLED

void setup()
{
.........
   	u8g2.begin();
    u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
.........
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
```





## 代码初始化部分

在初始化部分需要对各个外设进行逐一的初始化，初始化代码如下：

```c++
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
```

# 遇到的问题

在项目中习惯性的使能了Arduino的串口，但是并没有使用串口，导致设备上电后必须要打开串口调试助手，串口初始化才能完成。初步怀疑是nano 33 BLE 板子上采用的是HID转TTL串口，因此此处将串口初始化屏蔽。

![image-20210520004153215](https://gitee.com/song_zhi_shu/my-image-host/raw/master/img/image-20210520004153215.png)


# Part5 成果输出

### 视频：https://www.bilibili.com/video/BV11Q4y1o7cL/

### 代码：https://github.com/songzhishuo/FunPack_Arduino_Nano_33

# Part4 心得体会

本期项目让我接触到了处理328p以外的其他arduino开发板，不得不赞叹Arduino生态的强大型。同时感谢@小乐 提供的部分MIC音频转换代码。