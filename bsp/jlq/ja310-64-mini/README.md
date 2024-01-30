# JA310 EVB BSP支持包说明

## 1. 编译说明

RT-Thread内核编译可以参考[RT-Thread Smart版本编译与用户态应用开发]。

在console下进入到`bsp/jlq/ja310-64-mini`目录中，运行以下命令：

```
scons -j4
```

来编译这个板级支持包。
如果编译正确无误，会产生 `rtthread.elf`, `rtthread.bin` 文件。


## 2. 驱动支持情况

| 驱动 | 支持情况  |  备注  |
| ------ | ----  | :------:  |
| UART | 支持 | COM-UART |
| SDIO | 支持 | SD卡 |
| I2C0~I2C5 | 支持 | I2C3 for PM310 test ok |
| PM310 | 支持 | PMIC寄存器读写/ldo设置 |
| RTC | 支持 | RTC设置 |

## 3. JA310 EVB板卡烧入
在console下进入到`bsp/jlq/ja310-64-mini`目录中，运行以下命令：
```
./tools/encode_ja310_image.py Bootloader rtthread.bin 0x1080000 null null null 0
cp rtthread.bin.enc tools/u-boot.bin
```
tools目录下包含JA310 EVB板卡烧入所需的基本bin文件。使用JLQ Software Loader进行烧入。目前支持eMMC和nand的烧入和启动。
正常启动后界面如下：
```
heap: 0x000fb1e0 - 0x02000000

 \ | /
- RT -     Thread Smart Operating System
 / | \     5.0.1 build Jun 21 2023 16:03:20
 2006 - 2022 Copyright by RT-Thread team
[I/utest] utest is initialize success.
[I/utest] total utest testcase num: (0)
JLQ-JA310 RT-Thread Demo!!
msh />
```
## 4. JA310 EVB SD卡
将userapps 编译生成的userspace应用打包成sd.bin 写入sd卡，或者将应用拷贝到FAT32格式的sd卡中。
默认启动后会将sd卡挂载到文件系统根目录/。

通过list device命令，可以查看成功挂载后的sd设备节点。

```
heap: 0x001180d0 - 0x02000000

 \ | /
- RT -     Thread Smart Operating System
 / | \     5.0.1 build Jun 29 2023 17:29:25
 2006 - 2022 Copyright by RT-Thread team
[I/utest] utest is initialize success.
[I/utest] total utest testcase num: (0)
[I/SDIO] SD card capacity 7565312 KB.
SD card file system initialization done!
JLQ-JA310 RT-Thread Demo!!
msh />
msh />ls
Directory /:
hello.elf           279408
ping.elf            282904
pong.elf            280696
tcpclient.elf       289424
tcpserver.elf       289632
udpclient.elf       342704
udpserver.elf       288240
umailbox.elf        294152
vi.elf              411888
webclient.elf       323360
webserver.elf       418512
msh />hello
msh />hello world!

msh />list device
device           type         ref count
-------- -------------------- ----------
sd       Block Device         1
urandom  Miscellaneous Device 0
random   Miscellaneous Device 0
zero     Miscellaneous Device 0
null     Miscellaneous Device 0
ptmx     Character Device     0
rtc      RTC                  0
shm      Unknown              0
console  Character Device     2
uart0    Character Device     1
```

## 5. JA310 RTC
RT-Thread系统内只允许存在一个 RTC 设备，且名称为 "rtc"。
如果使用了JA310硬件RTC, 需要在menuconfig中disable Soft RTC。
### 5.1 查看日期和时间

输入 `date` 即可，大致效果如下：

```
msh />date
local time: Thu Jul  6 14:48:37 2023
timestamps: 1688626117
timezone: UTC+8
msh />
```

### 5.2 设置日期和时间

同样使用 `date` 命令，在命令后面再依次输入 `年` `月` `日` `时` `分` `秒` （中间空格隔开, 24H 制），大致效果如下：

```
msh />date 2023 07 06 15 46 30  # 设置当前时间为 2023-07-06 15:46:30
old: Thu Jul  6 14:49:50 2023
now: Thu Jul  6 15:46:30 2023
msh />date
local time: Thu Jul  6 15:46:32 2023
timestamps: 1688629592
timezone: UTC+8
msh />
```

## 6. JA310 SMP
JA310支持通过psci启动ARM多核CPU.
注意针对Cotex-a55芯片，需要更新最新的libcpu代码，并在配置中打开ARCH_ARM_CORTEX_A55=y。

RT-Thread中新增的补丁，文件`rt-thread/libcpu/aarch64/common/context_gcc.S`如下：

```
.weak rt_hw_cpu_id_set
.type rt_hw_cpu_id_set, @function
rt_hw_cpu_id_set:
    mrs x0, mpidr_el1           /* MPIDR_EL1: Multi-Processor Affinity Register */
#ifdef ARCH_ARM_CORTEX_A55
    lsr x0, x0, #8
#endif
    and x0, x0, #15
    msr tpidr_el1, x0
    ret
```

系统启动后，输入 `ps` 可以查看当前系统各个线程运行在不同cpu的信息，如下：

```
msh />ps
thread   cpu bind pri  status      sp     stack size max used left tick  error
-------- --- ---- ---  ------- ---------- ----------  ------  ---------- ---
jlq_demo N/A   1   21  suspend 0x00000318 0x00000400    77%   0x00000020 EINTRPT
tshell     0   4   20  running 0x000004b8 0x00002000    29%   0x00000002 OK
sys work N/A   4   23  suspend 0x000002a8 0x00002000    08%   0x0000000a OK
mmcsd_de N/A   4   22  suspend 0x00000348 0x00002000    21%   0x00000009 EINTRPT
tsystem  N/A   4   30  suspend 0x00000318 0x00002000    11%   0x00000020 EINTRPT
tidle3     3   3   31  running 0x00000228 0x00002000    10%   0x00000014 OK
tidle2     2   2   31  running 0x00000228 0x00002000    10%   0x0000000c OK
tidle1     1   1   31  running 0x00000258 0x00002000    10%   0x00000005 OK
tidle0   N/A   0   31  ready   0x00000258 0x00002000    30%   0x0000000b OK
timer    N/A   4    4  suspend 0x00000298 0x00002000    10%   0x00000009 OK
msh />
```

关于多线程的使用，可以参考`applications/main.c`中jlq_demo的实现，绑定在特定的cpu上运行。
