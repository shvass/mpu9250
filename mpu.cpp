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



#include "mpu.hpp"


bool isI2cInit = false;

mpu::mpu( gpio_num_t clkPin, gpio_num_t dioPin, bool lsbZero){

    if(!isI2cInit){

        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = dioPin,        
            .scl_io_num = clkPin,        
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = {.clk_speed = MPU_DEF_CLK_FREQ }, 
            .clk_flags = 0,                         
        };

        i2c_param_config(MPU_DEFAULT_PORT, &conf);
        ESP_ERROR_CHECK(i2c_driver_install(MPU_DEFAULT_PORT, I2C_MODE_MASTER, 0, 0, 0));
        isI2cInit = true;
    }

    address = (lsbZero) ? (MPU9250_ADDRESS) : (MPU9250_ADDRESS | 0x01);
}

void mpu::read(uint8_t address, uint8_t *buffer, size_t size){
    i2c_master_write_read_device(MPU_DEFAULT_PORT, this->address, &address, 1, buffer, size, pdMS_TO_TICKS(MPU_OPE_WAIT_DELAY));
}


void mpu::write(uint8_t address, uint8_t *buffer, size_t size){
    i2c_master_write_to_device(MPU_DEFAULT_PORT, this->address, buffer, size, pdMS_TO_TICKS(MPU_OPE_WAIT_DELAY));
}

void mpu::setSyncFreq(unsigned int freq){
    syncPeriod = 1000.0f / freq;
}

void mpu::beginSyncTask(unsigned int freq){
    syncPeriod = 1000.0f / freq;

    if(!syncTaskHandle) xTaskCreate(mpu::s_syncTask, "mpuSync", 4096, this, 1, &syncTaskHandle);
}

void mpu::endSyncTask(){
    syncPeriod = 0;
    syncTaskHandle = 0;
}

void mpu::syncTask()
{
    while (syncPeriod)
    {
        read(MPU9250_ACCEL_XOUT_H, (uint8_t*) &rawData, sizeof(mpu::rawSensor));

        // add to buffer
        buffers[0].push_back(rawData.aX);
        buffers[1].push_back(rawData.aY);
        buffers[2].push_back(rawData.aZ);
        buffers[3].push_back(rawData.gX);
        buffers[4].push_back(rawData.gY);
        buffers[5].push_back(rawData.gZ);


        vTaskDelay(pdMS_TO_TICKS(syncPeriod));
    }
}

void mpu::s_syncTask(void* ptr){

    mpu* imu = (mpu*)ptr;
    imu->syncTask();
    vTaskDelete(NULL);
};

// mpu::imuData::imuData(rawData &raw){
//     // calculate acceleration

// }

void mpu::filter::update(){

    buffer.push_back(source);

    // filteredRaw.aX ;

}
