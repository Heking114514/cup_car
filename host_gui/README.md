# 小车串口控制台

## 启动

第一次运行先安装依赖：

```powershell
cd D:\Desktop\pcb_config\cup_car\host_gui
py -3 -m pip install -r requirements.txt
```

之后双击 `start_gui.bat`，或在当前目录运行：

```powershell
py -3 main.py
```

不要使用本机 Anaconda 的 `python` 命令，其 PySide6 DLL 环境不可用。

## 使用

1. USB 转串口与 STM32 共地，TX/RX 交叉连接。
2. 选择对应 COM 口，波特率保持 `115200`，点击“连接”。
3. 修改 `vx` 和 `az` 后可单次发送，也可按设定频率连续发送。
4. “急停 / 清零”会立即发送一次 `0.000,0.000`，并把输入值清零。
5. 收到编码器帧后，界面显示左右累计值、单帧增量和最近 20 秒曲线。

STM32 必须处于导航模式，串口速度指令才会控制小车。调试时先架空车轮或断开电机电源。

## 遥控直行数据记录

关闭串口 GUI 和 ROS 串口节点，确保只有一个程序占用串口。先查看端口：

```powershell
py -3 record_control.py --list-ports
```

开始记录后使用 PS2 遥控小车直线行驶，按 `Ctrl+C` 停止：

```powershell
py -3 record_control.py --port COM5
```

日志默认保存到 `logs/straight_run_日期_时间.txt`。记录器只读取 `ENC` 遥测，
不会向小车发送速度指令。Linux 上将 `py -3` 换为 `python3`，串口可以使用
`/dev/ttyUSB0`。

## 串口协议

```text
上位机 -> STM32: vx,az\r\n
STM32 -> 上位机: ENC,mcu_time_ms,sample_sequence,left_total,right_total\r\n
```

示例：

```text
0.250,-1.500
ENC,1250,125,1234,-1188
```

编码器帧中的时间戳和采样序号均为无符号 32 位数，左右累计计数为有符号
32 位数。上位机应使用相邻帧的累计计数差计算轮子位移，并使用 MCU 时间戳差
计算速度；不要使用串口帧到达电脑的时间代替采样时间。累计计数使偶发丢帧不会
丢失行程数据。检测到时间戳或采样序号回退时，应将其视为 STM32 重启并重新建立
里程计基准。

当前车辆参数：轮子半径 `0.0325 m`、编码器每圈 `1925` 计数、轮距
`0.254 m`。左右轮相邻帧位移可按以下方式计算：

```text
meters_per_count = 2 * pi * 0.0325 / 1925
d_left  = (left_total_now  - left_total_prev)  * meters_per_count
d_right = (right_total_now - right_total_prev) * meters_per_count * 1.010
d_center = (d_left + d_right) / 2
d_yaw = (d_right - d_left) / 0.254
```

其中 `1.010` 是当前右侧有效轮径标定系数，应与下位机保持一致。积分位姿时使用
区间中点航向：`x += d_center*cos(yaw+d_yaw/2)`、
`y += d_center*sin(yaw+d_yaw/2)`、`yaw += d_yaw`。
