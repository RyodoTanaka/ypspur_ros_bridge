#Nishida Lab's YP-Spur Robot Driver
###Abut
西田研究室のYP-Spurロボットを動かすためのパッケージです。  
このパッケージでは、速度指令(並進速度、角速度)によるロボットの駆動と、  
オドメトリの計算結果の配信(TFを含む)を行っています。

###Installation
git cloneでインストールして下さい。

```bash
cd <catkin_ws>/src
git clone https://github.com/RyodoTanaka/nishidalab_ypspur_driver.git
```

また、現段階(2015/03/27)では、ジョイスティックによる動作のみができるようになっています。なので、そのパッケージもインストールして下さい。

```bash
cd <catkin_ws>/src
git clone https://github.com/RyodoTanakanishidalab_ypspur_joy_controler.git
```

###Usage
まず最初に、`ypspur-coordinator`を起動する必要があります。ロボットとUSB接続が完了したら、以下のコマンドでそれを実行しましょう。

```bash
cd <nishidalab_ypspur_driver/config
sudo sh nishidalab_ypspur_start.sh
```

うまく行くと以下のような表示が出るはずです。

```bash
++++++++++++++++++++++++++++++++++++++++++++++++++
YamabicoProject-Spur
 Ver. 1.14.0
++++++++++++++++++++++++++++++++++++++++++++++++++
Device Information
 Port    : /dev/ttyACM0 
Warn: Baudrate setting is not supported on this device.
Applying parameters.
YP-Spur coordinator started.
Command analyzer started.
Trajectory control loop started.

```

うまく行かない場合は、USBを挿し直した後、10秒ほど待ってからもう一度上記コマンドを実行しましょう。  
いずれにせよここまで完了したら、ジョイスティックを使ってロボットを動かすため、launchを行います。

```bash
roslaunch nishidalab_ypspur_driver nishidalab_ypspur_joyop.launch
```
