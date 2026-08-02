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

## 串口协议

```text
上位机 -> STM32: vx,az\r\n
STM32 -> 上位机: ENC,left_total,right_total\r\n
```

示例：

```text
0.250,-1.500
ENC,1234,-1188
```
