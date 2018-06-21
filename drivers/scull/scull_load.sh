#!/bin/sh
module="scull"
device="scull"
mode="664"

# 使用传入到该脚本的所有参数调用`insmod`，同时使用路径名来指定模块位置，
# 这是因为新的modutils默认不会在当前目录中查找模块。
/sbin/insmod ./$module.ko $* || exit 1

# 删除原有节点
rm -f /dev/${device}[0-3]

major=$(awk "\$2==\"$module\" {print \$1}" /proc/devices)
mknod /dev/${device}0 c $major 0
mknod /dev/${device}1 c $major 1
mknod /dev/${device}2 c $major 2
mknod /dev/${device}3 c $major 3
ln -sf ${device}0 /dev/${device}

# 给定适当的组属性及许可，并修改属组。
# 并非所有的发行版都具有staff组，有些有wheel组。
group=0

chgrp $group /dev/${device}[0-3]
chmod $mode /dev/${device}[0-3]
