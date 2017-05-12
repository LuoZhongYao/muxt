[![Build Status](https://travis-ci.org/LuoZhongYao/muxt.svg?branch=master)](https://travis-ci.org/LuoZhongYao/muxt)

Serial port multiplexing terminal program
-----------------------------------------

Copyright (C) 2017 Luo ZhongYao <LuoZhongYao@gmail.com>

## Introduction ##

muxt 是一个串口控制台复用程序, 通过复用一个串口实现终端复用. 满足
只有一个串口但需要多个控制台的需求. 多数嵌入Linux, QNX 平台都使用一个
串口调试, 很多情况我们需要打开多个终端, 其中一个执行命令,另一查看log.
当前仅确认能在如下平台运行, 理论所有的Linux平台都应该可以运行:

* Fedora 25(x86_64)
* QNX 6.6 (arm)

## Instructions for Use ##

```shell

Usage: muxt [options]
options:
	-p <serport>        : Serial port device to connect to [/dev/ttyS0]
	-b <baudrate>       : MUX mode baudrate (0,9600,19200, ...) [460800]
	-n <number>         : Number of logical serial ports [1]
	-s <symlink-prefix> : Prefix for the symlinks of slave devices (e.g. /dev/mux)
	-C <server command> : Remote service start command [/bin/muxtd -p /dev/ttyS0]
	-h                  : Show this help message
	-d <loglevel>       : Set loglevel: [ERROR | WARNING]
                  ERROR     0x01
                  WARNING   0x02
                  INFO      0x04
                  DEBUG     0x08

Usage: muxtd [options]
options:
	-p <serport>        : Serial port device to connect to [/dev/ttyS0]
	-b <baudrate>       : MUX mode baudrate (0,9600,19200, ...) [460800]
	-s <shell>          : Login shell [/bin/sh]
	-h                  : Show this help message
	-d <loglevel>       : Set loglevel: [ERROR | WARNING]
	                        ERROR     0x01
	                        WARNING   0x02
	                        INFO      0x04
	                        DEBUG     0x08

```

* -p <serport>          指定复用的串口,默认是 /dev/ttyS0
* -b <baudrate>         串口波特率,默认波特率是 460800
* -n <number>           创建的虚拟串口数量,范围是1-31(仅客户端)
* -s <symlink-prefix>   虚拟串口前缀,例如 -s /dev/mux 将会生成 /dev/mux1 /dev/mux2等(仅客户端)
* -s <shell>            虚拟串口调用的shell(仅服务端)
* -C <server command>   远端运行的命令,该命令会直接发给串口,当串口是控制台时,将执行该命令(仅客户端)
* -d <loglevel>         设置输出log等级,默认输出 出错,警告(0x01 | 0x02), 可用值有:
```
    ERROR     0x01    出错信息
    WARNING   0x02    警告信息
    INFO      0x04    常规信息
    DEDUG     0x08    调试信息
```


首先将对应平台的 `muxtd` 拷贝到远程设备上,在本地执行 `muxt` ,并指定 `-C` 参数使远程设备运行
`muxtd`, 如果连接成功, `muxt` 将会生成 `symlink-prefix` 前缀的虚拟串口, 此时本地可通过`picocom`
`minicom`等程序连接远程终端, 实现终端复用.

## Example ##

```shell
 
# 通过串口ttyUSB0,使用115200的波特率,创建4个虚拟串口, 服务端使用/bin/muxtd,串口为ttyS0, shell 是 /bin/bash 
$ muxt -p /dev/ttyUSB0 -b 115200 -n 4 -s ~/mux -C '/bin/muxtd -p /dev/ttyS0 -b 115200 -s /bin/bash'
# 新的终端里面用picocom 连接 ~/mux1, 也可使用tmux, screen等程序复用本地终端
$ picocom ~/mux1
# 新的终端里面用picocom 连接 ~/mux2
$ picocom ~/mux2
# 新的终端里面用picocom 连接 ~/mux3
$ picocom ~/mux3
# 新的终端里面用picocom 连接 ~/mux4
$ picocom ~/mux4

```
