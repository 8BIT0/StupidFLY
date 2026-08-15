#include <poll.h>
#include "serial_dshot.h"
#include <sys/ioctl.h>
#include <termios.h>

using namespace time_literals;
using namespace SerialDShot;

static constexpr unsigned MIN_TOPIC_UPDATE_INTERVAL = 2500; // 2.5 ms -> 400 Hz

ModuleBase::Descriptor SerialDShot::Pwm::desc{task_spawn, custom_command, print_usage};

Pwm::Pwm(const char *device) :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default) {
	_mixing_output.setAllDisarmedValues(DSHOT_DISARM_VALUE);
	_mixing_output.setAllMinValues(DSHOT_MIN_THROTTLE);
	_mixing_output.setAllMaxValues(DSHOT_MAX_THROTTLE);

	_mixing_output.setAllFailsafeValues(UINT16_MAX);

	if (device) {
		strncpy(_device, device, sizeof(_device) - 1);
		_device[sizeof(_device) - 1] = '\0';
	}

	_mixing_output.serDefaultAssign();
	_mixing_output.setLowrateSchedulingInterval(4980);	/* low rate 200Hz */
}

Pwm::~Pwm() {
	if (_port_fd >= 0) {
		::close(_port_fd);
		_port_fd = -1;
	}

	perf_free(_cycle_perf);
}

CONFIG_STATE Pwm::config_abrot_check(CONFIG_STATE cur_state) {
	if (_config_retry == 0) {
		_config_failed_reason = cur_state;
		return CONFIG_STATE::CONFIG_FAILED;
	}

	_config_retry --;
	return cur_state;
}

/* config serial port */
CONFIG_STATE Pwm::config_port(uint8_t &retry_cnt) {
	if (_port_fd < 0) {
		/* open serial port */
		_port_fd = ::open(_device, O_RDWR | O_NONBLOCK);

		/* open port failed */
		if (_port_fd < 0) {
			PX4_ERR("open port %s failed", _device);
			goto port_config_err;
		}

		struct termios opt;
		tcgetattr(_port_fd, &opt);

		opt.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
		opt.c_cflag |= CS8;
		opt.c_cflag |= CLOCAL | CREAD;

		cfsetispeed(&opt, B1500000);
		cfsetospeed(&opt, B1500000);

		opt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
		opt.c_oflag &= ~OPOST;
		opt.c_lflag &= ~(ICANON | ECHO | ISIG);

		tcflush(_port_fd, TCIOFLUSH);
		tcsetattr(_port_fd, TCSANOW, &opt);
	}

	return CONFIG_STATE::CONFIG_TYPE;

port_config_err:
	return config_abrot_check(CONFIG_STATE::CONFIG_PORT);
}

PROTO_CMD Pwm::recv() {
	struct pollfd fd;
	PROTO_CMD cmd = PROTO_CMD::CMD_INVALID;

	fd.fd = _port_fd;
	fd.events = POLLIN;

	PX4_INFO("poll receive");

	if (poll(&fd, 1, _CONFIG_COMMAND_ACK_TIMEOUT) > 0) {
		size_t read_size = ::read(_port_fd, get_rec_buf(), rec_buf_size());

		if (read_size > 0) {
			cmd = parse_cmd(read_size);
			clear_rec_buf();
		}
	} else {
		PX4_ERR("poll failed");
	}

	return cmd;
}

CONFIG_STATE Pwm::config_pwm_output_type() {
	PROTO_CMD cmd = PROTO_CMD::CMD_INVALID;

	set_type(_output_type);
	cmd = recv();

	if (cmd == PROTO_CMD::CMD_ACK_PWM_TYPE)
		return CONFIG_STATE::CONFIG_FINISH;

	/* clear config time */
	if (cmd != PROTO_CMD::CMD_ACK_PWM_TYPE) {
		PX4_WARN("set output type retry ack %d", cmd);
		return config_abrot_check(CONFIG_STATE::CONFIG_TYPE);
	}

	return CONFIG_STATE::CONFIG_FAILED;
}

bool Pwm::config() {
	switch (_config_state) {
		case CONFIG_STATE::CONFIG_PORT : {
			_config_state = config_port(_config_retry);

			if (_config_state == CONFIG_STATE::CONFIG_TYPE) {
				/* reset config retry */
				_config_retry = _CONFIG_RETRY_COUNT;
			} else if (_config_state == CONFIG_STATE::CONFIG_PORT) {
				/* delay 1 sec */
				px4_usleep(1000000);
			}
			break;
		}

		case CONFIG_STATE::CONFIG_TYPE: {
			_config_state = config_pwm_output_type();
			if (_config_state == CONFIG_STATE::CONFIG_FAILED) {
				/* abrot config */
				PX4_ERR("type config failed");
				return false;
			}
			break;
		}

		case CONFIG_STATE::CONFIG_FINISH: return true;
		case CONFIG_STATE::CONFIG_FAILED: return false;
		default: return false;
	}

	return false;
}

void Pwm::Run() {
	if (should_exit()) {
		_mixing_output.unregister();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	/* doing config */
	if (config()) {
		_mixing_output.update();

		if (_parameter_update_sub.updated())
			update_params();

		_mixing_output.updateSubscriptions();
	} else {
		ScheduleDelayed(100_ms);
	}

	perf_end(_cycle_perf);
}

bool Pwm::updateOutputs(float *outputs, unsigned num_outputs, unsigned num_control_groups_updated) {
	if (outputs == nullptr)
		return false;

	uint8_t rotor_count = (num_outputs > _MAX_SUPPORT_ROTOR_NUM) ? _MAX_SUPPORT_ROTOR_NUM : num_outputs;
	uint16_t hw_outputs[rotor_count] = {};

	for (uint8_t i = 0; i < rotor_count; i ++) {
		if (!_mixing_output.isFunctionSet(i))
			outputs[i] = 0;

		hw_outputs[i] = static_cast<uint16_t>(lroundf(outputs[i]));
	}

	pwm_out(hw_outputs, rotor_count);
	return true;
}

void Pwm::update_params() {
	parameter_update_s pupdate;
	_parameter_update_sub.copy(&pupdate);

	updateParams();

	int type = _param_dshot_type.get();

	/* get parameters */
	if (type >= 0) {
		/* type parameter valid */
		_output_type = static_cast<OUTPUT_TYPE>(type);
	}
}

int Pwm::init() {
	update_params();

	ScheduleNow();

	_mixing_output.setMaxTopicUpdateRate(MIN_TOPIC_UPDATE_INTERVAL);

	return PX4_OK;
}

int Pwm::task_spawn(int argc, char *argv[]) {
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'd':
				device_name = myoptarg;
				break;

			default:
				PX4_WARN("unrecognized flag");
				return -1;
		}
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		Pwm *instance = new Pwm(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		instance->init();

		return PX4_OK;
	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);
		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

/* developing */
int Pwm::custom_command(int argc, char **argv) {
	/* start test sequence */

	/* stop all */

	/* revert spin direction */

	return PX4_OK;
}

int Pwm::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This is the Serial Esc driver. max support 8Ch DShot outputs

It supports:
- DShot300, DShot600, DShot1200, DShot2400
- telemetry via separate UART

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("serial_dshot", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");

	/* print command describe */

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int Pwm::print_status() {
	Proto_Statistic_t statistic = get_statistic();

	PX4_INFO("Performance Counters:");
	perf_print_counter(_cycle_perf);

	PX4_INFO("current state:       %s", get_state_str(_config_state));
	if (_config_state == CONFIG_STATE::CONFIG_FAILED) {
		PX4_INFO("failed reason:       %s\n", get_state_str(_config_failed_reason));
	}

	PX4_INFO("Dshot setting");
	PX4_INFO("      Type:          %s",   get_type_str(_output_type));

	PX4_INFO("Statistic");
	PX4_INFO("\ttransmit [ cnt: %d ]", statistic.val_trans_cnt);
	PX4_INFO("\tinvert   [ set: %d ] ---- [ ack: %d ]", statistic.cmd_invert_set_cnt, statistic.cmd_invert_ack_cnt);
	PX4_INFO("\trate     [ set: %d ] ---- [ ack: %d ]", statistic.cmd_rate_set_cnt,   statistic.cmd_rate_ack_cnt);
	PX4_INFO("\ttype     [ set: %d ] ---- [ ack: %d ]", statistic.cmd_type_set_cnt,   statistic.cmd_type_ack_cnt);
	PX4_INFO("\tinvalid  [ cmd: %d ] ---- [ len: %d ]", statistic.invalid_cmd_cnt,    statistic.invalid_cmd_size_cnt);

	return 0;
}

extern "C" __EXPORT int serial_dshot_main(int argc, char *argv[]) {
	return ModuleBase::main(Pwm::desc, argc, argv);
}


