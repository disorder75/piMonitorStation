PROJECT_ROOT = $(dir $(abspath $(lastword $(MAKEFILE_LIST))))


ifndef KERNEL_SRC
	$(error KERNEL_SRC must be set in the command line)
endif

PWD := $(shell pwd)
ARCH := arm

CROSS_COMPILE ?= ${CCPREFIX_PI1_Zero_W_Module}
CC := ${CCPREFIX_PI1_Zero_W_Module}gcc

#This sample makefile has been setup for a project which contains the following files: main.h, ap-main.c, ap-main.h, ap-gen.c, ap-gen.h   Edit as necessary for your project

#Change output_file_name.a below to your desired executible filename

#Set all your object files (the object files of all the .c files in your project, e.g. main.o my_sub_functions.o )
OBJ = dht11unimi.o

#Set any dependant header files so that if they are edited they cause a complete re-compile 
#(e.g. main.h some_subfunctions.h some_definitions_file.h ), or leave blank
DEPS =  

#Any special libraries you are using in your project (e.g. -lbcm2835 -lrt `pkg-config --libs gtk+-3.0` ), or leave blank
LIBS = -pthread -lssl -lcrypto

#Set any compiler flags you want to use (e.g. -I/usr/include/somefolder `pkg-config --cflags gtk+-3.0` ), or leave blank
CFLAGS = -Wall -g -O2 -std=gnu99
CFLAGS += -I/home/nc/eclipse-workspaceCPP/openssl/raspberry_library/include

# define library paths in addition to /usr/lib
#LDFLAGS = --verbose
#LDFLAGS += -L/home/nc/eclipse-workspaceCPP/openssl/raspberry_library/lib

#Set the compiler you are using ( gcc for C or g++ for C++ )
CC = ${CCPREFIX_PI1_Zero_W_Module}gcc

#Set the filename extensiton of your C files (e.g. .c or .cpp )
EXTENSION = .c

#define a rule that applies to all files ending in the .o suffix, which says that the .o file depends upon the .c version 
#of the file and all the .h files included in the DEPS macro.  Compile each object file
%.o: %$(EXTENSION) $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)  $(LIBS) 

#Combine them into the output file
#Set your desired exe output file name here
dht11unimi_https: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS) $(LIBS) 

#Cleanup
.PHONY: clean

clean:
	rm -f *.o *~ dht11unimi_https *~ 

