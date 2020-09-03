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

/**
 * @file ics_aoa.hpp
 *
 * @author Nanthawat Saetun <aerojnn@gmail.com>
 *
 * Driver for aoa sensor using AS5048A with ATmega328P (I2C).
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <mathlib/math/filter/MedianFilter.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/ics_aoa.h>

#define FILTER_SIZE	10

/* Configuration Constants */
#define ICS_AOA_BASEADDR 			0x68 	// 7-bit address.
#define ICS_AOA_CONVERSION_INTERVAL 		50000  // 50ms measurement interval, 20Hz.

class ICS_AOA : public device::I2C, public ModuleParams, public I2CSPIDriver<ICS_AOA>
{
public:
	ICS_AOA(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = ICS_AOA_BASEADDR);
	~ICS_AOA() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	int init() override;
	void print_status() override;

	void RunImpl();

private:
	void start();

	//Sends an i2c measure command to check for presence of a sensor.
	int measure();

	//Collects and publish the most recent sensor measurement data from the i2c bus.
	int collect();

	//Normalize to [-180,180]
	float normalize(float angle);

	math::MedianFilter  _filter{FILTER_SIZE};

	bool _collect_phase{false};

	uORB::Publication<ics_aoa_s> _ics_aoa_pub{ORB_ID(ics_aoa)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ICS_AOA_REV>) _p_aoa_rev
	)
};
