//    mpu9250 - mpu9250 driver for esp32
//    Copyright (C) 2023 akshay bansod <akshayb@gmx.com>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <https://www.gnu.org/licenses/>.



#ifndef MPU_HPP
#define MPU_HPP

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <deque>


#include "registerMap.hpp"

#define MPU_CLK_PIN_DEFAULT  GPIO_NUM_32   // default I2C clock pin 
#define MPU_DIO_PIN_DEFAULT  GPIO_NUM_33   // default I2C data pin
#define MPU_DEFAULT_PORT I2C_NUM_0         // default ESP I2C port pin
#define MPU_DEF_CLK_FREQ (400 * 1000)      // I2C operation frequency
#define MPU_OPE_WAIT_DELAY 100             // I2C operation delay

/**
 * @brief class to handle mpu9250 operations
 * 
 */
class mpu {

public:

    /**
     * @brief raw sensor readings from the imu
     */
    struct rawSensor
    {
        // accelerometer readings
        int16_t aX, aY, aZ;
        // temperature
        int16_t temp;
        // gyroscopic readings
        int16_t gX, gY, gZ;
    } rawData, filteredRaw;


    struct imuData
    {
        float aX, aY, aZ;
        float temp;
        float gX, gY, gZ;

        // float vX, vY, vZ;
        // float X, Y, Z;

        // imuData(rawData& raw);
    } data;

    
    class filter{
        public:
        std::deque<rawSensor> buffer;

        rawSensor& source = rawData;
        imuData& dest = data;

        filter(mpu& parent) : source(parent.rawData), dest(parent.data) {};
        
        void update();

        uint8_t filterSize = 50;
    };


    
    /**
     * @brief Construct a new mpu object
     * 
     * @param clkPin gpio pin to use as I2C clock pin
     * @param dioPin gpio pin to use as I2C data  pin
     */
    mpu(gpio_num_t clkPin = MPU_CLK_PIN_DEFAULT, gpio_num_t dioPin = MPU_DIO_PIN_DEFAULT, bool lsbZero = true);


    /**
     * @brief burst read bytes from device 
     * 
     * @param address address of the first byte to read
     * @param buffer address of the buffer to write the data
     * @param size number of bytes to read, by default it reads one byte
     */
    void read(uint8_t address, uint8_t* buffer, size_t size = 1);


    /**
     * @brief burst write bytes from device 
     * 
     * @param address address of the first byte to write
     * @param buffer address of the buffer to write data from
     * @param size number of bytes to write, by default it writes one byte
     */

    void write(uint8_t address, uint8_t* buffer, size_t size = 1);



    void setSyncFreq(unsigned int freq = 100);

    void beginSyncTask(unsigned int freq = 100);
    void endSyncTask();

private:

    uint8_t address = MPU9250_ADDRESS;

    float syncPeriod = 0.0f;

    TaskHandle_t syncTaskHandle = 0;
    void syncTask();
    static void s_syncTask(void* imu);

};  
#endif //  MPU_HPP