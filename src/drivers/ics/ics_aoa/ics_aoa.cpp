/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "ics_aoa.hpp"

ICS_AOA::ICS_AOA(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(DRV_SENS_DEVTYPE_ICS_AOA, MODULE_NAME, bus, address, bus_frequency),
	ModuleParams(nullptr),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus)

{
}

ICS_AOA::~ICS_AOA()
{
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

void ICS_AOA::start()
{
	// Reset the report ring and state machine.
	_collect_phase = false;

	// Schedule the driver to run on a set interval
	ScheduleOnInterval(ICS_AOA_CONVERSION_INTERVAL);
}

int ICS_AOA::init()
{
	// I2C init (and probe) first.
	if (I2C::init() != OK) {
		return PX4_ERROR;
	}

	// we should find out why we need to wait 200 ms here
	px4_usleep(200000);

	return measure();
}

int ICS_AOA::collect()
{
	// Read from the sensor.
	uint8_t val[4] {};
	perf_begin(_sample_perf);

	int ret = transfer(nullptr, 0, &val[0], 4);

	if (ret != PX4_OK) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t angle[2];
	float _angle[2];
	float _deg[2];

	angle[0] = val[0] << 8 | val[1];
	angle[1] = val[2] << 8 | val[3];

	//Reverse AoA reading
	int _rev = _p_aoa_rev.get();
	if(_rev >= 0){
		_rev = 1;
	}else{
		_rev = -1;
	}

	for(int i = 0; i < 2; i++)
	{
		_angle[i] = normalize((float)angle[i]);
		_deg[i] = ((float)_angle[i]) * _rev;
	}

	// publish
	ics_aoa_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.indicated_aoa = _deg[0];
	msg.filtered_aoa = _filter.apply(_deg[0]);
	msg.indicated_aos = _deg[1];
	msg.filtered_aos = _filter.apply(_deg[1]);

	_ics_aoa_pub.publish(msg);

	perf_end(_sample_perf);
	return PX4_OK;
}

int ICS_AOA::measure()
{
	uint8_t cmd = 0x00;

	// Send the command to begin a measurement.
	int ret = transfer(&cmd, 1, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}

	return PX4_OK;
}

void ICS_AOA::RunImpl()
{
	if (_collect_phase) {
		// Perform collection.
		if (collect() != PX4_OK) {
			PX4_DEBUG("collection error");
			// If error restart the measurement state machine.
			start();
			return;
		}

		// Next phase is measurement.
		_collect_phase = false;
	}

	// Perform measurement.
	if (measure() != PX4_OK) {
		PX4_DEBUG("measure error sensor adress");
	}

	// Next phase is collection.
	_collect_phase = true;

}

float ICS_AOA::normalize(float angle)
{
	float nor_angle = angle / 10;
  	nor_angle = fmod(nor_angle + 180, 360);
  	if (nor_angle < 0) {
    		nor_angle += 360;
  	}
  	return nor_angle - 180;
}

void ICS_AOA::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
