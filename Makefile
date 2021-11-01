CC = gcc
FILES = src/*.c src/*/*.c src/*/*/*.c
OUTPUT = raspi-pilot
INC = -Isrc
LIB = -lm -lpthread -ljson-c
WARNING = -Wno-address-of-packed-member -Wimplicit

CFLAGS = $(FILES) $(INC) $(LIB) $(WARNING) -o $(OUTPUT)

all:
	@sh -c '$(CC) $(CFLAGS)';\
	EXIT_CODE=$$?;\
	if [ $$EXIT_CODE = 0 ];\
	then echo 'Build success!';\
	else echo 'Build failed!';\
	fi

install:raspi-pilot parameter.json raspi-pilot.service
	@mkdir ~/.raspi-pilot
	@cp parameter.json ~/.raspi-pilot/
	@cp raspi-pilot /bin/raspi-pilot
	@cp raspi-pilot.service /etc/systemd/system/raspi-pilot.service
	@systemctl enable raspi-pilot.service

update:raspi-pilot
	@cp raspi-pilot /bin/raspi-pilot

uninstall:
	@rm /bin/raspi-pilot
	@rm ~/.raspi-pilot -r
	@rm /etc/systemd/system/raspi-pilot.service