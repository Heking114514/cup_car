# cup_car_mc02car 下位机工程

本目录是 STM32H7 下位机工程，目标芯片当前按 `STM32H723VG` 配置。推荐在 Ubuntu 下用 Makefile 编译，用 J-Link 通过 SWD 烧录。

工程路径：

```bash
/home/hjh/cup_corporate/cup_car_mc02car
```

常用输出文件：

```bash
build/Basic_Framework_MC02.hex
build/Basic_Framework_MC02.elf
build/Basic_Framework_MC02.bin
```

## 1. Ubuntu 环境准备

### 1.1 安装编译工具链

推荐使用系统包，避免把大工具链放在主文件夹里占空间：

```bash
sudo apt update
sudo apt install -y make gcc-arm-none-eabi binutils-arm-none-eabi
```

检查：

```bash
arm-none-eabi-gcc --version
make --version
```

本机已经准备了一个简化编译命令：

```bash
stm32-build /home/hjh/cup_corporate/cup_car_mc02car
```

它会优先使用系统里的 `arm-none-eabi-gcc`。如果系统工具链不存在，才会尝试 `/tmp/stm32-arm-toolchain/...` 里的临时工具链。

### 1.2 安装 J-Link

如果还没有安装 SEGGER J-Link，在下载目录里找到 `.deb` 包后安装：

```bash
sudo dpkg -i ~/Downloads/JLink_Linux_V976a_x86_64.deb
sudo apt -f install
```

检查：

```bash
which JLinkExe
JLinkExe
```

看到 J-Link Commander 启动信息即可。进入交互界面后输入：

```text
qc
```

退出。

如果 `JLinkExe` 找不到，但 `/opt/SEGGER/JLink/JLinkExe` 存在，可以加软链接：

```bash
mkdir -p ~/.local/bin
ln -sf /opt/SEGGER/JLink/JLinkExe ~/.local/bin/JLinkExe
```

确认 `~/.local/bin` 在 `PATH` 中：

```bash
echo "$PATH"
```

### 1.3 检查 J-Link 是否连接

插好 J-Link 和板子的 SWD 后：

```bash
lsusb | grep -i j-link
```

也可以直接尝试烧录命令。如果能识别芯片，说明连接正常。

J-Link SWD 线至少需要：

```text
VTref/3V3  -> 板子 3V3 参考电压
GND        -> 板子 GND
SWDIO      -> MCU SWDIO
SWCLK      -> MCU SWCLK
NRST       -> MCU NRST，推荐接
```

### 1.4 可选：检查快捷命令

本机常用两个快捷命令：

```bash
which stm32-build
which jlink-flash
```

如果这两个命令不存在，也不影响开发；可以直接使用后文的 `make` 和 `JLinkExe` 原生命令。

## 2. 编译

在任意目录执行：

```bash
stm32-build /home/hjh/cup_corporate/cup_car_mc02car
```

或进入工程目录后执行：

```bash
cd /home/hjh/cup_corporate/cup_car_mc02car
make -j"$(nproc)"
```

编译成功后会生成：

```bash
build/Basic_Framework_MC02.hex
build/Basic_Framework_MC02.elf
build/Basic_Framework_MC02.bin
```

推荐烧录 `.hex`：

```bash
build/Basic_Framework_MC02.hex
```

## 3. 烧录

### 3.1 推荐方式：`jlink-flash`

本机已经准备了一个简化烧录命令：

```bash
jlink-flash build/Basic_Framework_MC02.hex
```

完整流程：

```bash
cd /home/hjh/cup_corporate/cup_car_mc02car
stm32-build /home/hjh/cup_corporate/cup_car_mc02car
jlink-flash build/Basic_Framework_MC02.hex
```

`jlink-flash` 默认参数：

```text
device = STM32H723VG
interface = SWD
speed = 4000 kHz
```

如需手动指定芯片：

```bash
jlink-flash build/Basic_Framework_MC02.hex STM32H723VG
```

如需降低 SWD 速度：

```bash
JLINK_SPEED=1000 jlink-flash build/Basic_Framework_MC02.hex
```

烧录 `.bin` 时需要指定 Flash 地址，脚本默认是 `0x08000000`：

```bash
jlink-flash build/Basic_Framework_MC02.bin
```

必要时可手动指定：

```bash
JLINK_BIN_ADDR=0x08000000 jlink-flash build/Basic_Framework_MC02.bin
```

### 3.2 原生方式：直接调用 `JLinkExe`

如果没有 `jlink-flash`，可以在工程目录下直接运行：

```bash
cd /home/hjh/cup_corporate/cup_car_mc02car

cat >/tmp/stm32_flash.jlink <<'EOF'
si SWD
speed 4000
device STM32H723VG
connect
r
h
loadfile build/Basic_Framework_MC02.hex
r
g
qc
EOF

JLinkExe < /tmp/stm32_flash.jlink
```

其中关键参数是：

```text
si SWD                         使用 SWD 接口
speed 4000                     SWD 速度 4000 kHz
device STM32H723VG             目标芯片
loadfile build/...hex          烧录 hex 文件
```

如果连接不稳定，把 `speed 4000` 改成：

```text
speed 1000
```

### 3.3 不需要 STM32CubeProgrammer

当前推荐工作流是：

```text
Ubuntu + arm-none-eabi-gcc + Makefile + J-Link
```

只要用 J-Link 烧录，就不需要额外安装 STM32CubeProgrammer。

## 4. 常见问题

### 4.1 `arm-none-eabi-gcc not found`

安装工具链：

```bash
sudo apt update
sudo apt install -y gcc-arm-none-eabi binutils-arm-none-eabi
```

然后重新编译：

```bash
stm32-build /home/hjh/cup_corporate/cup_car_mc02car
```

### 4.2 `JLinkExe not found`

先确认是否安装：

```bash
ls /opt/SEGGER/JLink/JLinkExe
```

如果文件存在，建立软链接：

```bash
mkdir -p ~/.local/bin
ln -sf /opt/SEGGER/JLink/JLinkExe ~/.local/bin/JLinkExe
```

如果文件不存在，安装下载目录里的 J-Link `.deb`：

```bash
sudo dpkg -i ~/Downloads/JLink_Linux_V976a_x86_64.deb
sudo apt -f install
```

### 4.3 J-Link 连不上芯片

优先检查：

```text
1. 板子是否上电
2. J-Link 的 VTref 是否接到板子 3V3
3. GND 是否共地
4. SWDIO/SWCLK 是否接反
5. NRST 是否接好
6. 目标芯片是否是 STM32H723VG
```

可以降低速度再试：

```bash
JLINK_SPEED=1000 jlink-flash build/Basic_Framework_MC02.hex
```

### 4.4 烧录后程序不运行

先确认烧录文件是最新编译出来的：

```bash
ls -lh build/Basic_Framework_MC02.hex
```

重新编译并烧录：

```bash
stm32-build /home/hjh/cup_corporate/cup_car_mc02car
jlink-flash build/Basic_Framework_MC02.hex
```

如果仍不运行，检查供电、BOOT 引脚、复位线和外设短路情况。

## 5. 关键目录

```text
Core/                    STM32CubeMX 生成的底层初始化代码
application/             机器人应用层
modules/                 功能模块
bsp/                     板级支持包
Makefile                 GCC 编译入口
Basic_Framework_MC02.ioc STM32CubeMX 工程配置
```

近期常改文件：

```text
application/robot.c
application/chassis/chassis.c
modules/master_machine/master_process.c
modules/rfid/rfid_reader.c
modules/voice/voice_tts.c
Core/Src/usart.c
```

## 6. 原始框架来源

本工程基于 NCHU 南昌航空大学洪鹰战队开源框架改造：

```text
https://gitee.com/LitzJ/basic_framework_mc02
```

同时参考湖大开源基础框架：

```text
https://gitee.com/hnuyuelurm/basic_framework
```
