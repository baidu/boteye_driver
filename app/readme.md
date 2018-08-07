# README
driver_demo和depth_demo是百度boteye系列双目相机的演示程序，支持百度所有双目相机图像显示、曝光调节、白平衡调节、RGB-IR图像控制，其中depth_demo除了具有driver_demo所有功能外，还集成了RGB深度图和IR深度图计算功能。
# 操作系统
  Linux ubuntu 14.04 or 16.04
# 环境配置
## 依赖包
* Opencv-3.0.0 (官网下载源码包)
```
cd opencv-3.0.0/
mkdir build; cd build
cmake ..
make -j4
sudo make install
```
* libgflags-dev libgoogle-glog-dev
```
apt-get install -y --allow-unauthenticated \n
  libgflags-dev \
  libgoogle-glog-dev
```
## 动态链接库
* libxpparam.so libxpdepth.so
我们会提供这2个动态链接库，用户需将库文件放置于`app/libs/`路径下，方能编译成功。
# 编译方法
## 准备工作
```
cd driver; mkdir build; cd build
cmake ..
make -j4
```
现在已经具备了编译app代码的条件。
## 开始编译
```
cd app; mkdir build; cd build;
cmake ..
make -j4
```
此时，`app/build/`路径下已经编译出了driver_demo和depth_demo可执行文件，恭喜成功编译！
# 运行demo
将sensor和PC通过usb数据线链接，输入命令`dmesg`，可以看到`Product: Baidu_Robotics_vision_xxx`的信息。
输入命令`./driver_demo -helpshort`或`./depth_demo -helpshort`将打印帮助信息，可以查看cammad options的内容和含义。
## driver_demo
* 指令介绍
运行程序的指令如下：
```
driver_demo [-helpshort] [options] [<args>]
   -auto_gain      打开自动增益，默认关闭
   -dev_name       指定camera名，不使用选项时，程序自动寻找camera
      <args>       设备路径，例如：/dev/video0
   -imu_from_image 从图像中获取IMU数据，默认单独发指令获取IMU数据
   -ir_period      设置RGB和IR图像的频率，默认为2
      <args>        0:全部RGB图像
                    1:全部IR图像
                    2:RGB-IR
                    3:RGB-RGB-IR
    -record_path    设置图片保存路径，程序会自动或手动保存图片
      <args>        路径名，例如：~/record/
    -sensor_type    指定camera型号，不使用选项是，程序自动识别camera型号
    -spacebar_mode  与-record_path配合使用，设置手动保存图片模式，按空格键保存
```
例如：`./driver_demo -auto_gain -dev_name /dev/video1 -imu_from_image -record_path ~/record -spacebar_mode`，程序运行后，显示两个画面，左侧为左眼图像，右侧为右眼图像。
![driver_demo双目窗口](https://image.ibb.co/kCJTFe/651071383.jpg)
* 控制功能介绍
程序运行中，可通过按键控制软件的行为，例如曝光时间、自动增益、红外光强度或者切换光照方式等。
### 退出程序
按esc键退出程序。
### 保存图片
设置了保存图片路径时，按空格键保存一张图片。
### 曝光控制
```
'1'：最小曝光时间
'2'：20%曝光时间
'3'：60%曝光时间
'4'：最大曝光时间
'+'/'='：增加曝光，步长为1
'-'    ：减小曝光，步长为1
']'    ：增加曝光，步长为5
'['    ：减小曝光，步长为5
```
### 自动增益控制
```
'A'/'a'：打开或关闭自动增益
```
### 红外光照方式控制
```
'I'/'i'：循环切换红外光照方式，off->only structured->only infrared->both structured and infrared->off
'6'：最小光照强度
'7'：20%光照强度
'8'：60%光照强度
'9'：最大光照强度
'.'：增加光照强度，步长为1
','：减小曝光强度，步长为1
'>'：增加曝光强度，步长为5
'<'：减小曝光强度，步长为5
```
按下“I”键，可开启、关闭、切换IR照射方式，开启IR功能后，会出现img_lr_IR窗口，显示左右眼IR图像。

![开启IR后的窗口](https://image.ibb.co/fKbJgK/Screenshot_from_2018_08_02_16_02_38.png)

## depth_demo
* 指令介绍
运行程序的指令如下：
```
depth_demo [-helpshort] [options] [<args>]
   -auto_gain      打开自动增益，默认关闭
   -calib_yaml     加载RGB矫正文件
      <args>       矫正文件路径，例如：~/calibration/calib.yaml
   -depth          打开RGB深度图功能，默认关闭
   -depth_param_yaml IR深度图配置文件路径
      <args>       文件路径，例如：~/config/ir_depht_param.yaml
   -dev_name       指定camera名，不使用选项时，程序自动寻找camera
      <args>       设备路径，例如：/dev/video0
   -imu_from_image 从图像中获取IMU数据，默认单独发指令获取IMU数据
   -ir_calib_yaml  加载IR矫正文件
      <args>       矫正文件路径，例如：~/calibration/ir_calib.yaml
   -ir_depth       打开IR深度图功能，默认关闭
   -ir_period      设置RGB和IR图像的频率，默认为2
      <args>        0:全部RGB图像
                    1:全部IR图像
                    2:RGB-IR
                    3:RGB-RGB-IR
    -record_path    设置图片保存路径，程序会自动或手动保存图片
      <args>        路径名，例如：~/record/
    -sensor_type    指定camera型号，不使用选项是，程序自动识别camera型号
    -spacebar_mode  与-record_path配合使用，设置手动保存图片模式，按空格键保存
    -wb_mode        设置白平衡模式
      <args>        preset:预设模式
                    disable:关闭白平衡
                    auto:打开自动白平衡
```
dept_demo可以显示深度图，并且具有RGB深度图和IR深度图2种模式。
例如：`./depth_demo -calib_yaml ~/calibration/calib.yaml -depth`，程序运行后，显示两个窗口，img_lr窗口显示左右眼RGB图像，depth_canvas窗口为RGB深度图，分辨率1280*720。

![RGB深度图](https://image.ibb.co/nmjeMK/Screenshot_from_2018_08_02_16_12_17.png)

或者：`./depth_demo -calib_yaml ~/calibration/calib.yaml -dept_param_yaml ~/config/ir_depth_param.yaml -ir_calib_yaml ~/calibration/ir_calib.yaml -ir_depth`，程序运行后，显示3个窗口，img_lr窗口显示左右眼RGB图像，img_lr_IR窗口显示左右眼IR图像，depth_canvas窗口显示IR深度图。

![IR深度图](https://image.ibb.co/mnHkTz/Screenshot_from_2018_08_02_16_17_09.png)

* 控制功能介绍
depth_demo的控制功能和driver_demo相同，可参考前面小节，这里不再重复。


