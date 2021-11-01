CC = gcc
FILES = *.c */*.c */*/*.c
CFLAGS = $(FILES) -I$(INC) $(LIB) $(WARNING) -o $(OUTPUT)
LDFLAGS = -L/usr/local/lib
OUTPUT = rpilot
INC = .
LIB = -lm -lpthread -ljson-c
WARNING = -Wno-address-of-packed-member -Wimplicit

all:
	@sh -c '$(CC) $(CFLAGS)';\
	EXIT_CODE=$$?;\
	if [ $$EXIT_CODE = 0 ];\
	then echo 'Build success!';\
	else echo 'Build failed!';\
	fi

install:rpilot parameter.json
	@mkdir ~/.rpilot
	@cp parameter.json ~/.rpilot/
	@cp rpilot ~/.rpilot/

uninstall:
	@rm ~/.rpilot -r