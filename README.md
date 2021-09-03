# 吉林大学TARS-GO战队装甲板识别代码TarsGoVision
---

## 致谢  
首先在开头感谢东南大学2018年开源代码以及深圳大学、上海交大2019年开源代码对本套代码的完成提供的巨大帮助，希望这套代码也能够帮助其他队伍在 RM 这个舞台上得到更大的提升。

---
## 介绍  
本代码是吉林大学TARS-GO战队Robomaster2020赛季步兵装甲板识别算法（全平台兼容版本），包含并优化了本战队机器人视觉算法（JLURoboVision）的**相机驱动**、**装甲板识别**、**角度解算**三大主要模块。

---
## 主要特性
1. 大幅优化了大恒相机驱动GxCamera，增强了代码封装性及可移植性，重新组织了代码格式，增强可读性及拓展性。
2. 更改了使用的多线程库，由pthread更改为c++11标准的Thread库，突破原先只能在linux系统下运行的限制，实现全平台运行。

---
## 目录
* [1. 功能介绍](#1功能介绍)
* [2. 效果展示](#2效果展示)
* [3. 依赖环境](#3依赖环境)
* [4. 配置与调试](#4配置与调试)
* [5. 整体框架](#5整体框架)
* [6. 实现方案](#6实现方案)
* [7. 总结展望](#7总结展望)
---
## 1.功能介绍
|模块     |功能     |
| ------- | ------ |
|相机驱动| 大恒相机SDK封装，实现相机参数控制及图像采集 |
|装甲板识别| 检测敌方机器人装甲板位置信息并识别其数字 |
|角度解算| 根据上述位置信息解算目标相对枪管的yaw、pitch角度及距离 |
---
## 2.效果展示
### 装甲板识别
装甲板识别采用基于OpenCV的传统算法实现装甲板位置检测，同时采用SVM实现装甲板数字识别。  
考虑战场实际情况，机器人可打击有效范围在1m~7m之间，在此范围内，本套算法**装甲板识别率达98%**，识别得到装甲板在图像中四个顶点、中心点的坐标信息。  
<div align=center>**EnemyColor = BLUE; TargetNum = 1**</div>  

<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/B1.png" width = "600" alt="图2.1 装甲板识别"/>
</div>  


<div align=center>**EnemyColor = RED; TargetNum = 2**</div>  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/R.png" width = "600" alt="图2.2 装甲板识别"/>
</div>  

在640\*480图像分辨率下，**装甲板识别帧率可达340fps左右，引入ROI之后可达420fps**。但考虑到识别帧率对于电控机械延迟的饱和，取消引入ROI操作，以此避免引入ROI之后无法及时探测全局视野情况的问题，加快机器人自瞄响应。  
<div align=center>**640\*480（峰值可达340FPS）**</div>  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/armor640480.gif" width = "600" alt="图2.3 装甲板实时识别帧率"/>
</div>  


<div align=center>**320\*240（峰值可达1400FPS）**</div>  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/armor320240.gif" width = "600" alt="图2.4 装甲板实时识别帧率"/>
</div>


装甲板数字识别采用SVM，通过装甲板位置信息裁剪二值化后的装甲板图像并透射变换，投入训练好的SVM模型中识别，**数字识别准确率可达98%**。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/RealtimeArmor.gif" width = "600" alt="图2.5 装甲板数字识别"/>
</div>

### 角度解算  
角度解算方面使用了两种解算方法分距离挡位运行。第一档使用P4P算法，第二档使用小孔成像原理的PinHole算法。  
此外还引入了相机-枪口的Y轴距离补偿及重力补偿。  
使用标定板测试，角度解算计算的距离误差在10%以内，角度基本与实际吻合。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/pos.jpg" width = "600" alt="图2.7 角度解算测试图"/>
</div>
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/angle_solver.gif" width = "600" alt="图2.7 角度解算测试图"/>
</div>

---
## 3.依赖环境
### 硬件设备
|硬件|型号|参数|
|---|---|---|
|运算平台|Jetson Nano/Intel NUC|B01|
|相机|大恒相机MER-050-560U3C|分辨率640*480 自动曝光3000~5000μs|
|镜头|M0814-MP2|焦距8mm 光圈值4|
### 软件设备
|软件类型|型号|
|---|---|
|OS|Ubuntu 18.04/Windows 10|
|IDE|Qt Creator-4.5.2/Visual Studio 2019|
|Library|OpenCV-3.4.10|
|DRIVE|Galaxy SDK|
---
## 4.Configuration and Debugging
### Project configuration requirement  
1. Windows 10
	- Use Visual Studio to open TarsGoVision.sln
	- 配置项目属性表
	- Link OpenCV link library
2. Ubuntu 18
	- Use Qt Creator to open TarsGoVision-qt.pro
	- Configurate pro file
	- Use the link in pro file to install OpenCV and compile to get .so link library

### Code debugging TODO-List
1. CHange the corresponding directory of the .xml file, as YOUR_PATH_TO below shows. Change that part to the project's absolute path
```
// File: Main/ArmorDetecting.cpp

//Set armor detector prop
detector.loadSVM("YOUR_PATH_TO/TarsGoVision/General/123svm.xml");

//Set angle solver prop
angleSolver.setCameraParam("YOUR_PATH_TO/TarsGoVision/General/camera_params.xml", 1);
```
2. Change the camera's SN number/index number, as YOUR_GALAXY_CAMERA_SN below shows. Change that part to your SN number of the camera, or change YOUR_GALAXY_CAMERA_INDEX to your camera's index number.
```
// File: Main/ImageUpdating.cpp

/*
 *Second init: Open Camera by SN/Index
*/
status = gxCam.openDeviceBySN("YOUR_GALAXY_CAMERA_SN");			//By SN
//status = gxCam.openDeviceByIndex("YOUR_GALAXY_CAMERA_INDEX");	//By Index
GX_VERIFY(status);
```
3. Set camera's ROI, explosure, gain, white balance, and so on
```
// File: Main/ImageUpdating.cpp

/*
 *Third init: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
*/
gxCam.setRoiParam(640, 480, 80, 120);				// ROI
gxCam.setExposureParam(2000, false, 1000, 3000);	// Exposure
gxCam.setGainParam(0, false, 0, 10);				// Gain
gxCam.setWhiteBalanceOn(true);						// WhiteBalance

```
4. Debugging Tools Setting
The specific settings can be seen in [Debugging Tools](### Debugging Tools)，The setting file is:
```
// File: Main/ArmorDetecting.cpp
```
### Debugging single module  
The demo code is for reference：  
[JLUVision_Demos](https://gitee.com/qunshanhe/JLUVision_Demos)  
[Armor_Demo](https://gitee.com/qunshanhe/JLUVision_Demos/tree/master/Armor_Demo)Can be run in Linux(.pro)/Windows(.sln)  
[AngleSolver_Armor_GxCamera](https://gitee.com/qunshanhe/JLUVision_Demos/tree/master/Anglesolver_Armor_GxCamera_Demo)DJI's programs for image acquisition and the angle solver of armors. Connection to a Daheng camera to Linux is needed.  
### Debugging Tools  
There are customized functions in the code, visualizing the light, armor, and angle solver, and we can use the keyboard to control the parameters. It is convenient to use these functions to debug and optimize.

```
//装甲板检测识别调试参数是否输出 Armor recognization: isShown
//param:
//		1.showSrcImg_ON,		  是否展示原图
//		2.bool showSrcBinary_ON,  是否展示二值图
//		3.bool showLights_ON,	  是否展示灯条图
//		4.bool showArmors_ON,	  是否展示装甲板图
//		5.bool textLights_ON,	  是否输出灯条信息
//		6.bool textArmors_ON,	  是否输出装甲板信息
//		7.bool textScores_ON	  是否输出打击度信息
//					   1  2  3  4  5  6  7
detector.showDebugInfo(0, 0, 0, 1, 0, 0, 0);
```

```
//角度解算调试参数是否输出 Angle solver parameters: isShown
//param:
//		1.showCurrentResult,	  是否展示当前解算结果
//		2.bool showTVec,          是否展示目标坐标
//		3.bool showP4P,           是否展示P4P算法计算结果
//		4.bool showPinHole,       是否展示PinHole算法计算结果
//		5.bool showCompensation,  是否输出补偿结果
//		6.bool showCameraParams	  是否输出相机参数
//					      1  2  3  4  5  6
angleSolver.showDebugInfo(1, 1, 1, 1, 1, 0);
```

---
## 5.整体框架
### 文件树  
```
TarsGoVision/
├── AngleSolver
│   └── AngleSolver.h（角度解算模块头文件）
│   ├── AngleSolver.cpp（角度解算模块源文件）
├── Armor
│   ├── Armor.h（装甲板识别模块头文件）
│   ├── LightBar.cpp（灯条类源文件）
│   ├── ArmorBox.cpp（装甲板类源文件）
│   ├── ArmorNumClassifier.cpp（装甲板数字识别类源文件）
│   ├── findLights.cpp（灯条监测相关函数源文件）
│   └── matchArmors.cpp（装甲板匹配相关函数源文件）
│   ├── ArmorDetector.cpp（装甲板识别子类源文件）
├── General
│   ├── 123svm.xml（SVM模型文件）
│   ├── camera_params.xml（相机参数文件）
│   └── General.h（公有内容声明头文件）
├── GxCamera
│   ├── GxCamera.h（大恒相机类头文件）
│   ├── GxCamera.cpp（大恒相机类封装源文件）
│   └── GxSDK（相机SDK包含文件）
│       ├── DxImageProc.h
│       └── GxIAPI.h
├── Main
│   ├── ArmorDetecting.cpp（装甲板识别线程）
│   ├── ImageUpdating.cpp（图像更新线程）
│   └── main.cpp（main函数，程序主入口源文件）

```
### 整体算法流程图  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/Armor.png " width = "800" alt="图4.1 自瞄算法流程图"/>
</div>  

---
## 6.实现方案  
### 装甲板识别  
装甲板识别使用基于检测目标特征的OpenCV传统方法，实现检测识别的中心思想是找出图像中所有敌方颜色灯条，并使用找出的灯条一一拟合并筛选装甲板。  
主要步骤分为：**图像预处理**、**灯条检测**、**装甲板匹配**、**装甲板数字识别**及最终的**目标装甲板选择**。  
1. **图像预处理**  
为检测红/蓝灯条，需要进行颜色提取。颜色提取基本思路有BGR、HSV、通道相减法。  
然而，前两种方法由于需要遍历所有像素点，耗时较长，因此我们选择了**通道相减法**进行颜色提取。  
其原理是在**低曝光**（3000~5000）情况下，蓝色灯条区域的B通道值要远高于R通道值，使用B通道减去R通道再二值化，能提取出蓝色灯条区域，反之亦然。  
此外，我们还对颜色提取二值图进行一次掩膜大小3*3，形状MORPH_ELLIPSE的膨胀操作，用于图像降噪及灯条区域的闭合。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/src_binary.jpg " width = "600" alt="图5.1 颜色提取二值图"/>
</div>  

2. **灯条检测**  
灯条检测主要是先对预处理后的二值图找轮廓（findContours），  
然后对初筛（面积）后的轮廓进行拟合椭圆（fitEllipse），  
使用得到的旋转矩形（RotatedRect）构造灯条实例（LightBar），  
在筛除偏移角过大的灯条后依据灯条中心从左往右排序。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/Light_Monitor.jpg " width = "600" alt="图5.2 灯条识别图"/>
</div>  

3. **装甲板匹配**  
分析装甲板特征可知，装甲板由两个长度相等互相平行的侧面灯条构成，  
因此我们对检测到的灯条进行两两匹配，  
通过判断两个灯条之间的位置信息：角度差大小、错位角大小、灯条长度差比率和X,Y方向投影差比率，  
从而分辨该装甲板是否为合适的装甲板（isSuitableArmor），  
然后将所有判断为合适的装甲板放入预选装甲板数组向量中。  
同时，为了消除“游离灯条”导致的误装甲板，我们针对此种情况编写了eraseErrorRepeatArmor函数，专门用于检测并删除错误装甲板。  

```
/**
 *@brief: detect and delete error armor which is caused by the single lightBar 针对游离灯条导致的错误装甲板进行检测和删除
 */
void eraseErrorRepeatArmor(vector<ArmorBox> & armors)
{
	int length = armors.size();
	vector<ArmorBox>::iterator it = armors.begin();
	for (size_t i = 0; i < length; i++)
		for (size_t j = i + 1; j < length; j++)
		{
			if (armors[i].l_index == armors[j].l_index ||
				armors[i].l_index == armors[j].r_index ||
				armors[i].r_index == armors[j].l_index ||
				armors[i].r_index == armors[j].r_index)
			{
				armors[i].getDeviationAngle() > armors[j].getDeviationAngle() ? armors.erase(it + i) : armors.erase(it + j);
			}
		}
}
```
4. **装甲板数字识别**  
匹配好装甲板后，利用装甲板的顶点在原图的二值图（原图的灰度二值图）中剪切装甲板图，  
使用透射变换将装甲板图变换为SVM模型所需的Size，随后投入SVM识别装甲板数字。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/NumClassifier.png " width = "600" alt="图5.3 装甲板数字识别图"/>
</div>  

5. **目标装甲板选取**  
对上述各项装甲板信息（顶点中心点坐标与枪口锚点距离、面积大小、装甲板数字及其是否与操作手设定匹配）进行加权求和，  
从而获取最佳打击装甲板作为最终的目标装甲板。  
<div align=center>
<img src="https://gitee.com/qunshanhe/JLURoboVision/raw/master/Assets/Armor_Monitor.png " width = "600" alt="图5.4 装甲板识别效果图"/>
</div>  

---
### 角度解算  
角度解算部分使用了两种模型解算枪管直指向目标装甲板所需旋转的yaw和pitch角。  
第一个是**P4P解算**，第二个是**PinHole解算**。  
首先回顾一下相机成像原理，其成像原理公式如下：  
$$ s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \end{bmatrix} \begin{bmatrix} X \\ Y \\ Z \\ 1 \end{bmatrix}$$  
1. P4P解算原理  
由上述相机成像原理可得相机-物点的平移矩阵为：
$$ tVec = \begin{bmatrix} t_x \\ t_y \\ t_z \end{bmatrix} $$  
转角计算公式如下：  
$$ \tan pitch = \frac{t_y}{\sqrt{{t_y}^2 + {t_z}^2}} $$
$$ \tan yaw = \frac{t_x}{t_z} $$

2. 小孔成像原理  
像素点与物理世界坐标系的关系：  
$$ x_{screen} = f_x(\frac{X}{Z}) + c_x $$
$$ y_{screen} = f_y(\frac{Y}{Z}) + c_y $$  
则转角计算公式如下：  
$$ \tan pitch = \frac{X}{Z} = \frac{x_{screen} - c_x}{f_x} $$
$$ \tan yaw = \frac{Y}{Z} = \frac{y_{screen} - c_y}{f_y} $$

---
## 7.总结展望
### 总结  
本套代码主要实现了装甲板识别及大风车的识别这两个模块，结合角度解算模块对识别到的目标信息的解算，获取云台枪口控制转角，随后通过串口传输给下位机。  
装甲板识别与大风车识别模块性能表现不错，识别率和帧率满足比赛需求；角度解算模块经过设计，提升了准确性及鲁棒性。    
同时，代码整体经过封装，具有较强的可移植性。  
### 特色功能  
1. 丰富的调试接口及数据可视化  
代码配备了多个调试用函数，能将代码运行效果及计算参数通过图片或终端实时显示，便于代码调试优化。  
2. 深入底层的图像处理  
在预处理阶段，选用了通道相减进行颜色提取，然而通道相减法需要调用split及thresh等函数，耗时较长，于是我们经过分析算法特点，我们直接通过指针来遍历图像数据，大大加快了该步的运算速度。  
```
//pointer visits all the data of srcImg, the same to bgr channel split 通道相减法的自定义形式，利用指针访问，免去了split、substract和thresh操作，加速了1.7倍
//data of Mat  bgr bgr bgr bgr
uchar *pdata = (uchar*)srcImg.data;
uchar *qdata = (uchar*)srcImg_binary.data;
int srcData = srcImg.rows * srcImg.cols;
if (enemyColor == RED)
{
	for (int i = 0; i < srcData; i++)
	{
		if (*(pdata + 2) - *pdata > armorParam.color_threshold)
			*qdata = 255;
		pdata += 3;
		qdata++;
	}
}
else if (enemyColor == BLUE)
{
	for (int i = 0; i < srcData; i++)
	{
		if (*pdata - *(pdata+2) > armorParam.color_threshold)
			*qdata = 255;
		pdata += 3;
		qdata++;
	}
}
```
3. 目标装甲板加权计分选取  
目标装甲板的选取，我们结合了操作手指定兵种及装甲板实际打击特征（距离枪口的平移向量、打击面积大小）进行加权求和，最终选取打击度得分最大的作为目标装甲板。  
```
/**
 *@brief: compare a_armor to b_armor according to their distance to lastArmor(if exit, not a default armor) and their area and armorNum
 *		  比较a_armor装甲板与b_armor装甲板的打击度，判断a_armor是否比b_armor更适合打击（通过装甲板数字是否与目标装甲板数字匹配，装甲板与lastArmor的距离以及装甲板的面积大小判断）
 */
bool armorCompare(const ArmorBox & a_armor, const ArmorBox & b_armor, const ArmorBox & lastArmor, const int & targetNum)
{
	float a_score = 0;  // shooting value of a_armor a_armor的打击度
	float b_score = 0;  //shooting value of b_armor b_armor的打击度
	a_score += a_armor.armorRect.area(); //area value of a a_armor面积得分
	b_score += b_armor.armorRect.area(); //area value of b b_armor面积得分

	//number(robot type) priorty 设置a、b装甲板的分数
	setNumScore(a_armor.armorNum, targetNum, a_score);
	setNumScore(b_armor.armorNum, targetNum, b_score);

	if (lastArmor.armorNum != 0) {  //if lastArmor.armorRect is not a default armor means there is a true targetArmor in the last frame 上一帧图像中存在目标装甲板
		float a_distance = getPointsDistance(a_armor.center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
		float b_distance = getPointsDistance(b_armor.center, lastArmor.center); //distance score to the lastArmor(if exist) 装甲板距离得分，算负分
		a_score -= a_distance * 2;
		b_score -= b_distance * 2;
	}
	return a_score > b_score; //judge whether a is more valuable according their score 根据打击度判断a是否比b更适合打击
}
```  
4. 角度解算具有两个计算模型分档运行  

### 展望  
1. 卡尔曼滤波预测
2. 计算平台性能提升
3. 代码开机自启动
