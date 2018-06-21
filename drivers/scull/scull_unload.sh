#!/bin/sh

module="scull"
device="scull"

/sbin/rmmod $module $* || exit 1

rm -f /dev/${device} /dev/${device}[0-3]
