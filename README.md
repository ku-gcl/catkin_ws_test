# Overview
倒立振子のROSコード

# Raspiの設定編
## pipgioの自動起動

[Ubuntuで起動時に自動でShellScriptを実行する方法](https://qiita.com/MAI_onishi/items/74edc40a667dd2dc633e)

```bash
sudo nano /etc/systemd/system/pigpiod.service
```

## 固定IP化
以下の記事を参考に。

[コマンドラインで Ubuntu を固定 IP アドレスにする方法 (なるべく丁寧に解説)](https://qiita.com/noraworld/items/3e232fb7a25ed16c6a63)



# コードのビルド

```bash
cd ~/pendulum_project/catkin_ws_test
catkin_make
```

# ROSの実行と終了
以下のコマンドを実行

```bash
alias roslaunch_kill='source /home/ubuntu/pendulum_project/pendulum_test/cleanup.sh'
alias roslaunch_pendulum='ros_add && roslaunch test pendulum.launch'
```

aliasを作成しておけば、以下のコマンドでよい。

```bash
# 実行
roslaunch_pendulum

# 終了
roslaunch_kill
```

# 複数のPCでROS接続
[ROS講座20 複数のPCでROS接続1](https://qiita.com/srs/items/7d4aeb5e44138f97c770)

## raspiのターミナルで

```bash
# raspiのip
export ROS_IP=`hostname -I | cut -d' ' -f1`
roslaunch_pendulum
```

## pcのターミナルで

```bash
export ROS_MASTER_URI=http://192.168.1.201:11311
# pcのip
export ROS_IP=`hostname -I | cut -d' ' -f1`
rosnode list
```

環境変数の中身を確認するときは

```bash
$ROS_IP
```

## ゲインの計算

ゲインの計算は以下のPythonプログラムで行う。

[SolveRiccatiEquation.ipynb](https://colab.research.google.com/drive/1IcFUoy5qXGN6GaOZdWq6SWmxtl1Ce0IL?usp=sharing)