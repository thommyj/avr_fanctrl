#!/bin/bash

avrdude -c stk500v2 -p t13 -P /dev/ttyUSB0 -U flash:w:fanctrl.hex
