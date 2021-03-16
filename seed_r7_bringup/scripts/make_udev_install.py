#!/usr/bin/env python

import os
import sys
import tempfile
from subprocess import *
import subprocess
import shutil

class UdevInstall:
    def setup_serial(self): 
        choice = raw_input("install setserial yes(y) or none(n) : ")
        if choice in ['y', 'ye', 'yes']:
            os.system('sudo apt-get install setserial')
   
    def usb_id_write(self):
        with tempfile.NamedTemporaryFile() as tf:
            self.filename = tf.name

            header = '#aero_controller\n'
            upper_string = 'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="111",MODE="666",SYMLINK+="aero_upper", RUN+="/bin/setserial /dev/aero_upper low_latency"\n'
            lower_string = 'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}=="123",MODE="666",SYMLINK+="aero_lower", RUN+="/bin/setserial /dev/aero_lower low_latency"\n'
            space = '\n'
            header_hokuyo = '#hokuyo urg\n'
            hokuyo_string = 'SUBSYSTEM=="tty",ATTRS{idVendor}=="15d1",ATTRS{idProduct}=="0000",MODE="666",SYMLINK+="hokuyo"'    
            
            print("Please insert upper USB to PC port")
            choice = raw_input("yes(y) or none(n) : ")
            if choice in ['y', 'yes']:
               upper = upper_string.split(',')

               print(upper[3])
               #prev_upper_id = upper.strip("ATTRS{serial}==")
               p1 = subprocess.Popen(['udevadm','info','-n', '/dev/ttyUSB0'], stdout=subprocess.PIPE)
               p2 = subprocess.Popen(['grep', 'SERIAL_SHORT'], stdin=p1.stdout, stdout=subprocess.PIPE)
               #p1 = subprocess.Popen(['udevadm','info','-n', '/dev/tty0'], stdout=subprocess.PIPE)
               #p2 = subprocess.Popen(['grep', 'MAJOR'], stdin=p1.stdout, stdout=subprocess.PIPE)
               p1.stdout.close()

               out, err = p2.communicate()
               upper[3] = "ATTRS{serial}==\"" + out.split('=')[1].strip() +"\""
               print(upper[3])
               print(upper)
               upper_string = (','.join(upper))
               print(upper_string)
            
            print("Please insert lower USB to PC port")
            choice = raw_input("yes(y) or none(n) : ")
            if choice in ['y', 'yes']:
               lower = lower_string.split(',')

               print(lower[3])
               #prev_upper_id = upper.strip("ATTRS{serial}==")
               p1 = subprocess.Popen(['udevadm','info','-n', '/dev/ttyUSB0'], stdout=subprocess.PIPE)
               p2 = subprocess.Popen(['grep', 'SERIAL_SHORT'], stdin=p1.stdout, stdout=subprocess.PIPE)
               #p1 = subprocess.Popen(['udevadm','info','-n', '/dev/tty0'], stdout=subprocess.PIPE)
               #p2 = subprocess.Popen(['grep', 'MAJOR'], stdin=p1.stdout, stdout=subprocess.PIPE)
               p1.stdout.close()

               out, err = p2.communicate()
               lower[3] = "ATTRS{serial}==\"" + out.split('=')[1].strip() + "\""
               print(lower[3])
               print(lower)
               lower_string = (','.join(lower))
               print(lower_string)
            
            print("write tempfile")
            tf.write(header)
            tf.write(upper_string)
            tf.write(lower_string)
            tf.write(space)
            tf.write(header_hokuyo)
            tf.write(hokuyo_string)
            tf.seek(0)
            print(tf.read())
            subprocess.call(['sudo', 'cp',  self.filename, '/etc/udev/rules.d/90-aero.rules'])

                  
if __name__ == "__main__" :
    ui = UdevInstall()   
    ui.setup_serial()
    ui.usb_id_write()

    
