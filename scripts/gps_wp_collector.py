#!/usr/bin/python

###############################################################################
# Script to record list of waypoints using data from GPS receiver attached
#   to USB port. Currently does not work on Windows.
#
# Based on handler.py from 
#   https://github.com/mw46d/Entdecker/tree/master/Laptop/GPS
#
# Command line arguments: output file path (defaults to /tmp/waypoints.txt)
# 
# Make sure WAYPOINT_RADIUS is set to what you want (~ line 177)
#
# Basic character command inputs:
#   "c[gpst] - Cone: Grass/Pavement/Grass-Stop/Pavement-Stop, l - land, w - waypoint, q - quit"
#
# Output: text file waypoint list readable by Mission Planner
#
###############################################################################

import csv
import math
import serial
import signal
import string
import subprocess
import sys
import termios
import threading
import time
import tty

class Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

class CSVDialect(csv.Dialect):
    delimiter = '\t'
    doublequote = False
    skipinitialspace = True
    lineterminator = '\r\n'
    quoting = csv.QUOTE_NONE

class GpsValue:
    lock = threading.Lock()
    lat = 0.0
    lng = 0.0
    alt = 0.0

class GpsReader(threading.Thread):
    ser = None
    start_called = False
    gps_value = None

    def __init__(self, group = None, target = None, name = None,
                 args = (), kwargs = None, verbose = None):
        threading.Thread.__init__(self, group=group, target = target, name = name,
                                  verbose = verbose)
        self.args = args
        self.kwargs = kwargs
        self.gps_value = args[0]

        return

    def start(self):
        if self.start_called:
            raise RunTimeError

        self.start_called = True
        # fn = subprocess.check_output(["/export/home/marcow/bin/find_last", "sol2_*.pos"]).rstrip()

        # self.fin = open(fn, 'r')
        self.fin = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1)


        i = 0                              # Just read some lines
        line = self.fin.readline()
        while line and i < 10:
            line = self.fin.readline()
            i += 1

        super(GpsReader, self).start()

    def run(self):
        while True:
            line = self.fin.readline()

            if not line:
                where = self.fin.tell()
                time.sleep(0.2)
                self.fin.seek(where)
            else:
                # $GNGGA,170515.00,3742.43741,N,12125.36876,W,1,08,0.96,32.2,M,-28.8,M,,*42
                #        hhmmss.ss,llll.lllll,N,yyyyy.yyyyy,W,1|2- valid,#sats,HDOP,Alt,M,...

                a = line.strip().split(',')
                if a[0] == '$GNGGA' or a[0] == '$GPGGA':
                    lat_read = 0.0
                    long_read = 0.0
                    alt_read = 0.0

                    if int(a[6]) == 0:                   # invalid
                        valid = int(a[6])
                        # print "invalid? %s\r" % valid
                    elif float(a[8]) > 3:                # large hdop:-(
                        hdop_read = float(a[8])
                        # print "hdop: %f\r" % hdop_read
                    else:
                        lat_read = float(a[2])           # assume N
                        long_read = float(a[4])          # assume W
                        alt_read = float(a[9])           # ??

                    if lat_read != 0.0:
                        lat_real = int(lat_read / 100) + ((5 * ((lat_read * 10000) % 1000000) / 3) / 1000000)
                        long_real = 0 - (int(long_read / 100) + ((5 * ((long_read * 10000) % 1000000) / 3) / 1000000))
                        alt_real = alt_read
                        # print "lat= %f, long= %f\r" % (lat_real, long_real)
                    else:
                        lat_real = 0.0
                        long_real = 0.0
                        alt_real = 0.0
                        # print "no fix\r"

                    if self.gps_value != None and lat_read != 0.0:
                        self.gps_value.lock.acquire()
                        try:
                            self.gps_value.lat = lat_real
                            self.gps_value.lng = long_real
                            self.gps_value.alt = alt_real
                            # print "gps %f %f %f\r" % (self.gps_value.lat, self.gps_value.lng, self.gps_value.alt)
                        finally:
                            self.gps_value.lock.release()


def calc_diff(lat1, lng1, lat2, lng2):
    lat1_rad = math.radians(lat1)
    lng1_rad = math.radians(lng1)
    lat2_rad = math.radians(lat2)
    lng2_rad = math.radians(lng2)
    dlat_rad = math.radians(lat2 - lat1)
    dlng_rad = math.radians(lng2 - lng1)
    R = 6371000

    a = math.sin(dlat_rad / 2) * math.sin(dlat_rad / 2) + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlng_rad / 2) * math.sin(dlng_rad / 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    d = R * c

    y = math.sin(dlng_rad) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlng_rad)

    brng = math.degrees(math.atan2(y, x))

    if brng < 0:
        brng += 360

    return (d, brng)

value = GpsValue()
t = GpsReader(args = (value, ))
t.setDaemon(True)
t.start()

if len(sys.argv) > 1:
    fname = sys.argv[1]
else:
    fname = '/tmp/waypoints.txt'

f = open(fname, 'wb')
writer = csv.writer(f, CSVDialect())
writer.writerow(('QGC WPL 120', ))
sequence = 0
writer.writerow((sequence, 1, 0, 16, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
sequence += 1
WAYPOINT_RADIUS = 2.0
ROVER_HOST_LIST = [ '192.168.8.1', '192.168.1.16' ]  # we aren't using this

for h in ROVER_HOST_LIST:
    ping_ret = subprocess.call([ "ping", "-c", "1", "-t", "1", "-q", h ])
    if ping_ret == 0:
       rover_host = h
       break

if ping_ret == 0:
    print "Found rover @ %s" % rover_host

print "c[gpst] - Cone: Grass/Pavement/Grass-Stop/Pavement-Stop, l - land, w - waypoint, q - quit"

getch = Getch()
c = getch().lower()
while c != 'q' and ord(c) != 3:
    value.lock.acquire()
    try:
        my_lat = value.lat
        my_long = value.lng
        my_alt = value.alt
        # print "gps received %f %f %f" % (value.lat, value.lng, value.alt)
    finally:
        value.lock.release()

    if my_lat == 0.0:
        print "No fix:-("
#    elif c == 'b':                             # base point
#        writer.writerow(("#", "BasePoint", my_lat, my_long, my_alt))
#        print "b - %f %f %f" % (my_lat, my_long, my_alt)
#        if ping_ret == 0:
#            r = subprocess.check_output([ "ssh", "-i", "/export/home/marcow/.ssh/id_daheim", "odroid@%s" % rover_host, 'source /opt/ros/indigo/setup.bash && source $HOME/catkin_ws/devel/setup.bash && /home/odroid/catkin_ws/src/mw/mw_mavros/src/mw_mavros/get_gps.py' ])
#            ra = r.strip().split('\t')
#            writer.writerow(ra)
#            l = calc_diff(my_lat, my_long, float(ra[2]), float(ra[3]))
#            writer.writerow(("#", "Diff", l[0], l[1]))
#            print "  diff %f m %f deg" % (l[0], l[1])
    elif c == 'w':                             # waypoint
        writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, my_alt, 1))
        sequence += 1
        print "w - %f %f %f" % (my_lat, my_long, my_alt)
    elif c == 'c':                             # it's a cone
        # altitude codes
        C_REG_GRASS = 1011.0
        C_REG_PVMT = 1020.0
        C_STOP_GRASS = 2011.0
        C_STOP_PVMT = 2020.0
        c1 = getch().lower()
        if c1 == 'g':                          # grass, not last cone
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, C_REG_GRASS, 1))
            sequence += 1
            print 'c - reg-grass %f %f %f' % (my_lat, my_long, C_REG_GRASS)
        elif c1 == 'p':                        # pavement, not last cone
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, C_REG_PVMT, 1))
            sequence += 1
            print 'c - reg-pvmt %f %f %f' % (my_lat, my_long, C_REG_PVMT)
        elif c1 == 's':                        # grass, stop afterwards
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, C_STOP_GRASS, 1))
            sequence += 1
            print 'c - stop-grass %f %f %f' % (my_lat, my_long, C_STOP_GRASS)
        elif c1 == 't':                        # pavement, stop afterwards
            writer.writerow((sequence, 0, 0, 16, 0.0, WAYPOINT_RADIUS, 0.0, 0.0, my_lat, my_long, C_STOP_PVMT, 1))
            sequence += 1
            print 'c - stop-pvmt %f %f %f' % (my_lat, my_long, C_STOP_PVMT)
        else:
            print 'c - unknown %s' % c1
    elif c == 'l':                             # land; is this same as RTL?
        #writer.writerow((sequence, 0, 0, 181, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
        #sequence += 1
        #writer.writerow((sequence, 0, 0, 181, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1))
        #sequence += 1
        writer.writerow((sequence, 0, 0, 21, 0.0, 0.0, 0.0, 0.0, my_lat, my_long, my_alt, 1))
        sequence += 1
        print 'l - %f %f %f' % (my_lat, my_long, my_alt)

    c = getch().lower()

f.close()
