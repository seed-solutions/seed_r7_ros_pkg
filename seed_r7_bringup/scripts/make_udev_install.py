#!/usr/bin/env python

import os
import sys
import tempfile
import subprocess 

class UdevInstall:
    def __init__(self):
        a = 3
    def setup_serial(self): 
        choice = raw_input("install setserial yes(y) or none(n) : ")
        if choice in ['y', 'ye', 'yes']:
            subprocess.call('sudo apt-get install setserial')
    def usb_id_write(self):
        with tempfile.TemporaryFile() as tf:
            tf.write('#aero_controller\n')
            tf.write('SUBSYSTEMS=="usb",ATTRS{idVendor}=="43",ATTRS{idProduct}=="61",ATTRS{serial}==111,MODE="666",SYMLINK+="aero_upper"\n')
            tf.write('SUBSYSTEMS=="usb",ATTRS{idVendor}=="43",ATTRS{idProduct}=="61",ATTRS{serial}==123,MODE="666",SYMLINK+="aero_lower"')
            tf.seek(0)
            lines = tf.readlines()
            print lines[1].split(",")
            choice = raw_input("yes(y) or none(n) : ")
            if choice in ['y', 'yes']:
                print "test"
                
               
if __name__ == "__main__" :
    ui = UdevInstall()   
    ui.usb_id_write()