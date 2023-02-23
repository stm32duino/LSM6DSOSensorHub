# LSM6DSOSensorHub Arduino Library
By Lorenzo Bini at STMicroelectronics

This library provides functionality for interacting with the Sensor Hub functionality on LSM6DSO sensors.

## BOARD SETUP
If you are using a X-NUCLEO-IKS01A3 board, you must correctly wire up the sensor hub bus to the other sensors. To do this, short the middle two pins for each of the two jumper sets JP8 and JP7, which look like this:

```
JP8		* * * *
JP7		* * * *

Short as following:

JP8		* O-O *
JP7		* O-O *

* represents a pin
O-O represents a jumper and the two pins it is shorting
```

Note that with this setup, normal sensor operation will be unavailable unless specific library functionality is used

## LIBRARY USAGE
1. Create LSM6DSOHub object by passing it a valid LSM6DSOSensor representing the sensor you wish to use as master
2. Make sure the master has been initialized by calling its begin function
3. Initialize the Sensor Hub by calling the resetMaster function
4. Call the addDevice function for each device you would like to read using the sensor hub, up to four (4) devices. Provide, in this order, the I2C address of the device, the register address you wish to read, and how many times you wish to read the device at each read operation. Note that not all devices automatically increment the register address when multiple reads are issued in one operation.
5. Call the begin function to star the sensor hub
6. In your loop function, call waitAndRead whenever you wish to extract a new set of data from your sensors. Note that this function will block until all sensors have been read. Alternatively, you may manually wait and extract data by using the ready and read functions.
7. To actually obtain the data related to one of your sensors, call getSensor. You must provide, in this order, the sensor number (for example, the first sensor that was added is sensor 0, the second sensor 1, and so on), and a pointer to a uint8_t array which is as long as the number of reads you set for that sensor.

For example: a sensor added second with addDevice (0xBE, 0xEF, 4) should be read, after calling waitAndRead, by calling getSensor (1, arr) where arr is a uint8_t arr[4] of length 4.

## USING PASSTHROUGH FUNCTIONALITY
Before activating the sensor hub with begin(), the enablePassthrough() function may be used to gain normal I2C access to the sensors connected to the hub. Note that in this case, sensor hub operation is impossible. Before enabling the sensor hub, the passthrough feature MUST be disabled by calling disablePassthrough(). Passthrough functions MUST NOT be used once the sensor hub is operational. To regain use of these functions afterwards, the sensor must be fully reset using the resetMaster() function, after which it must be configured again.
