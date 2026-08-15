#pragma once

#ifndef __SERIAL_DHOT_H
#define __SERIAL_DHOT_H

#include "serial_dshot_def.h"
#include "serial_dshot_proto.h"
#include <board_config.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

using namespace time_literals;

namespace SerialDShot {
class Pwm : public ModuleBase, public OutputModuleInterface, public Proto {
#define SERIAL_PORT_BAUDRATE	1500000

static constexpr uint16_t DSHOT_DISARM_VALUE = 0;
static constexpr uint16_t DSHOT_MIN_THROTTLE = 1;
static constexpr uint16_t DSHOT_MAX_THROTTLE = 1999;

public:
	static Descriptor desc;

	Pwm(const char *device);
	~Pwm();

	static int task_spawn(int argc, char *argv[]);

	static int print_usage(const char *arg);

	static int custom_command(int argc, char *argv[]);

	int print_status() override;

	int init();

private:
	static constexpr uint8_t _CONFIG_RETRY_COUNT{5};
	static constexpr uint64_t _CONFIG_COMMAND_ACK_TIMEOUT{5000};	/* 5s config time out */

	CONFIG_STATE config_port(uint8_t &retry_cnt);

	CONFIG_STATE config_pwm_output_type();

	CONFIG_STATE config_pwm_output_rate();

	CONFIG_STATE config_abrot_check(CONFIG_STATE cur_state);

	bool config();

	bool send(uint8_t *p_data, size_t size) override {
		if ((p_data == nullptr) || \
		    (size == 0)         || \
		    (_port_fd < 0)      || \
		    (::write(_port_fd, p_data, size) < 0))
			return false;

		return true;
	}

	PROTO_CMD recv();

	void update_params();

	void Run() override;

	bool updateOutputs(float *outputs, unsigned num_outputs, unsigned num_control_groups_updated) override;

	const char* get_type_str(OUTPUT_TYPE type) {
		switch (type) {
			case OUTPUT_TYPE::OUTPUT_DSHOT150:  return "DSHOT-150";
			case OUTPUT_TYPE::OUTPUT_DSHOT300:  return "DSHOT-300";
			case OUTPUT_TYPE::OUTPUT_DSHOT600:  return "DSHOT-600";
			default: break;
		}

		return "unknown type";
	}

	const char* get_state_str(CONFIG_STATE state) {
		switch (state) {
			case CONFIG_STATE::CONFIG_FAILED: return "config failed";
			case CONFIG_STATE::CONFIG_PORT:   return "config port";
			case CONFIG_STATE::CONFIG_TYPE:   return "config type";
			case CONFIG_STATE::CONFIG_FINISH: return "config finish";
			default: break;
		}

		return "unknown state";
	}

	OUTPUT_TYPE _output_type{OUTPUT_TYPE::OUTPUT_DSHOT300};

	CONFIG_STATE _config_state{CONFIG_STATE::CONFIG_PORT};

	CONFIG_STATE _config_failed_reason{CONFIG_STATE::CONFIG_PORT};

	bool _config_done{false};

	uint8_t _config_retry{_CONFIG_RETRY_COUNT};

	int _port_fd{-1};

	char _device[20] {};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	/* Mixer */
	MixingOutput _mixing_output{PARAM_PREFIX, _MAX_SUPPORT_ROTOR_NUM, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SERIAL_ESC_TYPE>) _param_dshot_type
	)
};
};

#endif
