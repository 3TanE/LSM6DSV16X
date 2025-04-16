# LSM6DSV16X
Arduino/Rasberry Pi Pico library to support the LSM6DSV16X 3D accelerometer and 3D gyroscope
!!! This Fork is not a in place replacement since i decided to opt for snake case only !!!

## API
### Arduino
This sensor uses I2C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

### Pico
This sensor uses I2C or SPI to communicate.
    Create initialise spi  like this 

    ```
    // Initialize SPI0/1 at the desired baud rate
    spi_init(spi0, SPI_BAUD_RATE);
    // Set GPIO pins to SPI functions
    spi_set_format(spi0, //spi  interface
                    8, // bits per transfer
                    SPI_CPOL_1, //spi mode 3
                    SPI_CPHA_1, //spi mode 3
                    SPI_MSB_FIRST);
    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);
  ```

I2C will follow some time 


If u use it with a Pi Pico
you need to add 
 build_flags = -DPICO for platform io

 or 

target_compile_definitions(<target> PRIVATE PICO)

if you use cmake

### General
An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSV16XSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.enable_x();  
    AccGyr.enable_g();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSV16XSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.enable_x();  
    AccGyr.enable_g();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.get_x_axes(accelerometer);  
    AccGyr.get_g_axes(gyroscope);

## Examples

* LSM6DSV16X_DataLog_Terminal: This application shows how to get data from LSM6DSV16X accelerometer and gyroscope and print them on terminal.

* LSM6DSV16X_6D_Orientation: This application shows how to use LSM6DSV16X accelerometer to find out the 6D orientation and display data on a hyperterminal.

* LSM6DSV16X_Double_Tap_Detection: This application shows how to detect the double tap event using the LSM6DSV16X accelerometer.

* LSM6DSV16X_Free_Fall_Detection: This application shows how to detect the free fall event using the LSM6DSV16X accelerometer.

* LSM6DSV16X_MLC: This application shows how to detect the activity using the LSM6DSV16X Machine Learning Core.

* LSM6DSV16X_Pedometer: This application shows how to use LSM6DSV16X accelerometer to count steps.

* LSM6DSV16X_Qvar_Polling: This application shows how to use LSM6DSV16X Qvar features in polling mode.

* LSM6DSV16X_Sensor_Fusion: This application shows how to use LSM6DSV16X Sensor Fusion features for reading quaternions.

* LSM6DSV16X_Single_Tap_Detection: This application shows how to detect the single tap event using the LSM6DSV16X accelerometer.

* LSM6DSV16X_Tilt_Detection: This application shows how to detect the tilt event using the LSM6DSV16X accelerometer.

* LSM6DSV16X_Wake_Up_Detection: This application shows how to detect the wake-up event using the LSM6DSV16X accelerometer.

* LSM6DSV16X_FIFO_Polling: This application shows how to get accelerometer and gyroscope data from FIFO in pooling mode and print them on terminal.

* LSM6DSV16X_FIFO_Interrupt: This application shows how to get accelerometer and gyroscope data from FIFO using interrupt and print them on terminal.
## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSV16X

The LSM6DSV16X datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsv16x.html