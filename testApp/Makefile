#CC = arm-xilinxa9-linux-gnueabi-gcc
CC = gcc
TARGET = testClient_main


#LDFLAGS 	= -s -lpthread 

#ifeq ($(NEW_KERNEL),1)
CFLAGS = -DKERNEL_3_8
LDFLAGS = -lpthread
#endif

OBJ = cli_cmd.o testClient_main.o testClient.o

all: $(OBJ)
	$(CC) -o $(TARGET) $(OBJ) $(LDFLAGS) 
	
clean:
	rm -rf $(TARGET)
	rm -rf *.o
	rm -rf *.bak
	rm -rf ./.*.swp



