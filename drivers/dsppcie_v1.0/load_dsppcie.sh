#!/bin/sh
module="dsppcie"
device="dsppcie"
mode="777"

# 使用传入到该脚本的所有参数调用`insmod`，同时使用路径名来指定模块位置，
# 这是因为新的modutils默认不会在当前目录中查找模块。
/sbin/insmod ./$module.ko $* || exit 1

# 删除原有节点
rm -f /dev/${device}

major=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)
mknod /dev/${device} c $major 0

chmod $mode /dev/${device}
