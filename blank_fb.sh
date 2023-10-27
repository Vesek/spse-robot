#!/bin/bash

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

chvt 8
echo 0 > /sys/class/graphics/fbcon/cursor_blink
sleep .1
cat /dev/zero >> /dev/fb0 2> /dev/null
