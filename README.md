# RPilot
My Raspberry Pi Rover controller software.  
Communicate with QGourndControl using MAVLink.

## Requirement
* json-c

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