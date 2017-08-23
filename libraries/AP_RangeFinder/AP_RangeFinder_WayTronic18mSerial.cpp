/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_WayTronic18mSerial.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;

/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_WayTronic18mSerial::AP_RangeFinder_WayTronic18mSerial(RangeFinder &_ranger, uint8_t instance,
                                                               RangeFinder::RangeFinder_State &_state,
                                                               AP_SerialManager &serial_manager) :
    AP_RangeFinder_Backend(_ranger, instance, _state, MAV_DISTANCE_SENSOR_ULTRASOUND)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

/* 
   detect if a Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_WayTronic18mSerial::detect(RangeFinder &_ranger, uint8_t instance, AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_WayTronic18mSerial::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the lidar
    uint16_t sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();


    for(count; count < nbytes/6;count++){
    	//check if we are in sync
        if (uart->read() == 0xFF) {
        	sum+= (uart->read() -'0')*256; // needed to convert char number to int
            sum+= uart->read()-'0';
            //skip the temperature bits
            uart->read();
            uart->read();
            //skip the checksum bit
            uart->read();
        }
        //we are out of sync so we clean the uart buffer and exit the loop
        else{
       	uart->flush();
        count=0;
        break;
        }
    }

    // we need to write a null (00000000)to prompt another reading

    uart->write('\0');
    if (count == 0) {
        return false;
    }
    reading_cm = 10 * sum / count;
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_WayTronic18mSerial::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 400) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}
