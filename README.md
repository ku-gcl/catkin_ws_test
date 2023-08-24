# Overview
倒立振子のROSコード

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

