#!/bin/bash

temp_file=$(mktemp)

#template file created
echo '#aero_controller' >> $temp_file
echo 'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}==111,MODE="666",SYMLINK+="aero_upper", RUN+="/bin/setserial /dev/aero_upper low_latency"' >> "$temp_file"
echo 'SUBSYSTEM=="tty",ATTRS{idVendor}=="0403",ATTRS{idProduct}=="6001",ATTRS{serial}==123,MODE="666",SYMLINK+="aero_lower"',  RUN+="/bin/setserial /dev/aero_lower low_latency" >> "$temp_file"

function setup_serial()
{
   echo "install setserial"
   read -p "yes(y) or none(n) : " input
   case $input in
        [Yy] | [Yy][Ee][Ss] )
      sudo apt-get install setserial
      ;;
   esac
}

function string_check()
{
if [ -z "$1" ]; then
  return 255
fi
}

function usb_id_write(){
echo "PCにupperのUSBコネクタのみ差し込んでください(upperを使用しない場合はnone)"
read -p "yes(y) or none(n) : " input

case $input in 
    [Yy] | [Yy][Ee][Ss] )
    prev_upper_id=$( ( sed -n '2p' | cut -d "," -f4 | awk '{print substr($0, index($0, "="))}'| cut -c 3- ) < "$temp_file" )
    string_check $prev_upper_id
    if [ $? = 255 ] ; then
       echo "Error : not exist prev_upper_id. .tmpfile is broken."
       exit
    fi
    new_upper_id=$(udevadm info -n /dev/ttyUSB0 | grep SERIAL_SHORT | )
    string_check $new_upper_id

    sed -i -e "s/$prev_upper_id/$new_upper_id/g" "$temp_file"
    echo "upper ID: $new_upper_id"
    ;;
    
    * )
        echo "yes または no を入力して下さい.";;
 
esac
echo "PCにlowerのUSBコネクタのみ差し込んでください(lowerを使用しない場合はnone)"
read -p "yes(y) or none(n) : " input
case $input in 
    [Yy] | [Yy][Ee][Ss] )
    prev_lower_id=$( (sed -n '3p' | cut -d "," -f4 | awk '{print substr($0, index($0, "="))}'| cut -c 3-) < "$temp_file" )
    string_check $prev_lower_id
    if [ $? = 255 ] ; then
       echo "Error : not exist prev_upper_id. .tmpfile is broken."
       exit
    fi
    new_lower_id=$(udevadm info -n /dev/ttyUSB0 | grep SERIAL_SHORT | cut -d "=" -f2)
    string_check $new_lower_id

    sed -i -e "s/$prev_lower_id/$new_lower_id/g" $temp_file
    echo "upper ID: $new_lower_id"
    ;;
    
    * )
        echo "yes または no を入力して下さい.";;
 
esac
}

setup_serial
usb_id_write
cat "$temp_file"

sudo cp "$temp_file" /etc/udev/rules.d/90-aero.rules
##　copy check

sudo udevadm control --reload-rules

trap "
rm -f $temp_file
' 0
