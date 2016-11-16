#!/usr/bin/python

import time
from daemon import runner

class HelloDaemon():
    def __init__(self):
        self.stdin_path   = '/dev/null'
        self.stdout_path  = '/dev/tty'
        self.stderr_path  = '/dev/tty'
        self.pidfile_path = '/tmp/HelloDaemon.pid'
        self.pidfile_timeout = 5
    def run(self):
        while True:
            print("Hello daemon")
            time.sleep(25)

theDaemon = HelloDaemon()
daemon_runner = runner.DaemonRunner(theDaemon)
daemon_runner.do_action()
