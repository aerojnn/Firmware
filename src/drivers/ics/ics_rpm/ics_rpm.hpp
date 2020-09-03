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
 * @file ics_rpm.hpp
 *
 * @author Nanthawat Saetun <aerojnn@gmail.com>
 *
 *  * Driver for RPM sensor using castle with ATmega328P (I2C).
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
#include <uORB/topics/ics_rpm.h>

#define FILTER_SIZE	20

/* Configuration Constants */
#define ICS_RPM_BASEADDR 			0x70 	// 7-bit address.
#define ICS_RPM_CONVERSION_INTERVAL 		50000  // 50ms measurement interval, 20Hz.

class ICS_RPM : public device::I2C, public ModuleParams, public I2CSPIDriver<ICS_RPM>
{
public:
	ICS_RPM(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address = ICS_RPM_BASEADDR);
	~ICS_RPM() override;

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

	math::MedianFilter  _filter1{FILTER_SIZE};
	math::MedianFilter  _filter2{FILTER_SIZE};
	math::MedianFilter  _filter3{FILTER_SIZE};
	math::MedianFilter  _filter4{FILTER_SIZE};
	math::MedianFilter  _filter5{FILTER_SIZE};
	math::MedianFilter  _filter6{FILTER_SIZE};
	math::MedianFilter  _filter7{FILTER_SIZE};
	math::MedianFilter  _filter8{FILTER_SIZE};

	bool _collect_phase{false};

	uORB::Publication<ics_rpm_s> _ics_rpm_pub{ORB_ID(ics_rpm)};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED,  MODULE_NAME": read")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ICS_RPM_MAG>) _p_rpm_magnet
	)
};
