# piMonitorStation
IoT driver prototype based on rapsberry pi zero and DHT11 sensor's family.
First of all, i'm not an expert about "kernel stuff" so i'm sorry if my code 
doesn't look like a pro.
Are projects built to learn and develop experience.


In order to cross-compile correctly the driver project for Arm architecture on host machine (usually Intel or Amd):

1 - checkout the project https://github.com/raspberrypi/linux.git

2 - configure the toolchain and compile the kernel following the instruction on raspberry official site 
    https://www.raspberrypi.org/documentation/linux/kernel/building.md. 
    Extra usefull informations about configuration are here https://elinux.org/RPiconfig
3 - Configure on host machine the global variables in order to point to the cross-compiler and/or have 
    the definition of the paths like KERNEL_SRC, ARMTOOLS etc... this depends on you and how you want
    to build the project.
4 - Now you have to options. 
	1 - Apply all the differences from the folder raspberry/linux into the raspberry project
    	    and copy the drivers/misc/dht11unimi into the raspberry project (of course you have to respect 
      	    the folders structure...) and after that recompile the raspberry kernel. The driver will be
 	    compiled and copied into the modules library. To do tha follow the original instruction
	    at point 2.	
            By the way, i've added two simply scripts in order to cross compile the kernel fro raspberry zero
	    and install the modules into mounted micro sd. 
	    The script are buildP0.sh and install.sh, before launch you have to check your usb mount point
	    with lsblk. the script point to sdb so check it before start, in case just change it.
	2 - cross-compile the project locally into this directory using the makefile. If all you global 
            system variables are correct (see the example to have an idea...) go into the driver folder
	    and run "make clean && make", copy you driver ".ko" files into your raspberry and install it.
            I will not cover here how to install manually a driver module. See online refs.
	    When the driver is installed go into the client folder and compile it with "make clean && make"
            copy the client (will run in userspace) dht11unimi_https into your raspberry home.
            

Now, the pinout configuration depends on the device tree, see the file into the overlay order to find/change the pin. 
Once you known the pin, attach the dht11 sensor to the raspberry and start the client in the userspace.

For now, the client is based on my remote microservices, this means that all the data from the driver will
be uploaded on a remote backend service on internet.

To do that a wifi connection is required.

Fell free to change the code client in order to disable the upload/network connection and keep only the
code to read the sensor's data.


	    


# ----------- EXAMPLE OF CROSS-COMPILER RASPBERRY SETTINGS INTO .bashrc file
export KERNEL_SRC=/_your_path_on_os_filesystem/raspberry/linux
# config from official website raspberry
export CCPREFIX=/_your_path_on_os_filesystem/raspberry/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
#CCPREFIX_PI1_Zero_W_Module is not mandatory, is my custom configuration for my eclipse ide and my own makefile (take a look into the makefile)
export CCPREFIX_PI1_Zero_W_Module=r/_your_path_on_os_filesystem/raspberry/tools/arm-bcm2708/arm-bcm2708-linux-gnueabi/bin/arm-bcm2708-linux-gnueabi-
export ARMTOOLS=/_your_path_on_os_filesystem/raspberry/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin
export PATH=$ARMTOOLS:$PATH
export MODULES_TEMP=~/modules

