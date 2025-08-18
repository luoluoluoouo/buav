#!/usr/bin/env bash

# In mavlink_shell (nsh>)
# mavlink stop -d /dev/ttyS4
# uxrce_dds_client stop
# uxrce_dds_client start -t serial -d /dev/ttyS4 -b 115200

sudo MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 115200
