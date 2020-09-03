/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "ics_sensor.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

ICS_SEN::ICS_SEN() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ICS_SEN::~ICS_SEN()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ICS_SEN::init()
{
	ScheduleOnInterval(50000); // 50000 us interval, 20 Hz rate

	return true;
}

void ICS_SEN::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// grab latest data
	if(_ics_aoa_sub.update() || _ics_rpm_sub.update()){

		const ics_aoa_s &aoa = _ics_aoa_sub.get();
		const ics_rpm_s &rpm = _ics_rpm_sub.get();

		// publish sensor data
		ics_sensor_s data{};
		data.timestamp = hrt_absolute_time();

		data.aoa = aoa.filtered_aoa;
		data.aos = aoa.filtered_aos;

		for(int i = 0; i < 8; i++){
			data.rpm[i] = rpm.filtered_rpm[i];
		}

		_ics_sensor_pub.publish(data);
	}

	perf_end(_loop_perf);
}

int ICS_SEN::task_spawn(int argc, char *argv[])
{
	ICS_SEN *instance = new ICS_SEN();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int ICS_SEN::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int ICS_SEN::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ICS_SEN::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ICS sensor hub module running out of a work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ics_sensor", "sensor_hub");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ics_sensor_main(int argc, char *argv[])
{
	return ICS_SEN::main(argc, argv);
}
