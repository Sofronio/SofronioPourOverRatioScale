# SofronioPourOverRatioScale

任何一个电子秤都是由【称重单元】【模数转换器】【单片机】【屏幕】【按钮】【供电】这6部分组成。因此使用现有电子秤的外壳，保留称重单元，更换其余部件，即可做出满足自己要求的电子秤。<br />
本项目是一个支持咖啡粉录入和粉水比显示的，专为手冲咖啡设计的电子秤。<br />
## 改装部件：
- 【称重单元】使用原电子秤自带单元<br />
- 【模数转换器】便宜的电子秤通常使用自带单片机内部的模数转换器，但精度较低，因此使用【HX711】模块代替<br />
- 【单片机】原电子秤单片机通常为牛屎封装，也无法进行二次开发，为了照顾初学者，以及考虑到库文件的支持情况，使用【Arduino Pro Mini】代替<br />
- 【屏幕】原电子秤通常为单色液晶显示，信息量固定，为了显示更多的内容，需要使用点阵屏幕，这里使用【SSD1306 1.3寸 OLED】代替<br />
- 【按钮】原电子秤为贴片轻触开关，但不自己画PCB的话是比较难应用的，这里使用【贴片微动开关】代替<br />
- 【供电】原电子秤通常为2-4节AA/AAA电池供电，由于单片机耗电相对高一些，再考虑到电子秤内部空间，这里使用【14500锂电池】代替，为满足充电需求，使用国产【锂电池充放一体模块】<br />
## 项目结构：
### Calibration
初次连线完成后需要使用Calibration.ino进行校准，通常使用100.0g砝码完成<br />
### PvcDesign
表面覆膜设计，可以代替3D打印的OLED框架，也可以让按钮防水<br />
### RatioPourOverScale
RatioPourOverScale.ino是实现功能的主要程序，其中使用了以下库：<br />
#### AceButton
按钮所使用的库，支持单击、双击、长按、连发<br />
#### StopWatch
秒表所使用的库，支持开始、暂停、继续、复位、精度设定等<br />
#### HX711_ADC
模数转换器所使用的库，支持去皮、称重、窗口平均等功能<br />
#### U8g2lib
OLED所使用的库，支持字体设定、按页更新、旋转显示等功能<br />
### WireConnection.jpg
WireConnection.jpg是连线图<br />
