#!/bin/sh

module="dsppcie"
device="dsppcie"

/sbin/rmmod $module $* || exit 1

rm -f /dev/${device}
