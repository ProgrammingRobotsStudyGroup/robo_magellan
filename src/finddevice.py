#!/usr/bin/python

import sys
import bluetooth

device_name = None
device_address = None

# Device name may be supplied by user
if len(sys.argv) > 1:
    device_name = sys.argv[1]

# Discover
nearby_devices = bluetooth.discover_devices()

# List of devices found
print "Devices Found:"
for bdaddr in nearby_devices:
    print ">",bluetooth.lookup_name( bdaddr ), bdaddr
    if device_name == bluetooth.lookup_name( bdaddr ):
        device_address = bdaddr
        break

# Report status if device requested
if device_name is not None:
    if device_address is not None:
        print "Found bluetooth device: ", device_name, " with address: ", device_address
    else:
        print "Could not find bluetooth device: ", device_name

