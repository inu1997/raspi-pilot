# RPilot
My Raspberry Pi Rover controller software.  
Communicate with QGourndControl using MAVLink.

## Requirement
* json-c

## Installation
On your raspiberry pi:
```
git clone https://github.com/inu1997/raspi-pilot.git
cd raspi-pilot
make
sudo make install # This would put the binary file to /bin, parameter.json to /root/.raspi-pilot, and create & enable raspi-pilot.service.
```

## Uninstallation
In the git directory:
```
sudo make uninstall
```
This will clean up the binary file, parameter.json, /root/.raspi-pilot and the service.

## Code structrue
* driver
    * I2C/SPI device driver.
* measurement
    * Read sensor value and compute results.
* pilot
    * Handle manual control and update actuator.
* mavlink
    * MAVLink utilities.
* util
    * Coding utilities.

## To-Do
- [x] Correct IMU reading.
- [x] MAVLink support.
- [x] Parameter support.
- [x] Pilotable.
- [ ] Camera stream.
- [ ] More MAVLink message to complete.