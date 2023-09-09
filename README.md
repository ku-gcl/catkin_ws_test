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

設定ファイルを作成

```bash
touch /etc/netplan/50-cloud-init.yaml
```

設定ファイルを編集
- `YOUR_ADMIRE_IP`: 希望のipアドレスを入力。`192.168.1.200`とか
- `YOUR_NETWORK`: Wifiのネットワーク名
- `YOUR_PASSWORD`: Wifiのパスワード

```yaml:/etc/netplan/50-cloud-init.yaml
network:
  version: 2
  wifis:
    wlan0:
      optional: false
      dhcp4: false
      dhcp6: false
      addresses: [YOUR_ADMIRE_IP/24]
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8]
      access-points:
        "YOUR_NETWORK":
            password: "YOUR_PASSWORD"
```






作成したyamlファイルを無効にする

```
sudo mv 50-cloud-init.yaml 50-cloud-init.yaml.cp
```


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

## データの表示

実行する
```bash
ros_add && roslaunch test pendulum.launch
```

必要かわからないけど、10Hzでデータを表示する設定コマンド

```bash
ros_add
rosrun test pendulum rate --all 10
```

データの表示コマンド

```bash
rostopic echo imu/data_raw
```