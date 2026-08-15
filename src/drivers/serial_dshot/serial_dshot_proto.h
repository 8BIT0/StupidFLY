#pragma once

#ifndef __SERIAL_DSHOT_PROTO_H
#define __SERIAL_DSHOT_PROTO_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <cstring>
#include <string.h>
#include <stddef.h>
#include "serial_dshot_def.h"

namespace SerialDShot {
class Proto {
public:
	static constexpr uint16_t _MAX_BUFF_SIZE = 1024;

	Proto() = default;
	~Proto() = default;

	void set_type(OUTPUT_TYPE type);

	void set_ch_revert(CHANNLE_DEF ch);

	void pwm_out(uint16_t *pwm_val, uint8_t ch_sum);

	PROTO_CMD parse_cmd(size_t size);

	uint8_t *get_rec_buf() { return _rec_buf; }

	uint16_t rec_buf_size() { return _MAX_BUFF_SIZE; }

	void clear_rec_buf() { memset(_rec_buf, 0, _MAX_BUFF_SIZE); }

	Proto_Statistic_t get_statistic() {
		Proto_Statistic_t statistic_info;

		statistic_info.cmd_invert_ack_cnt = _cmd_invert_ack_cnt;
		statistic_info.cmd_invert_set_cnt = _cmd_invert_set_cnt;

		statistic_info.cmd_rate_ack_cnt = _cmd_rate_ack_cnt;
		statistic_info.cmd_rate_set_cnt = _cmd_rate_set_cnt;

		statistic_info.cmd_type_ack_cnt = _cmd_type_ack_cnt;
		statistic_info.cmd_type_set_cnt = _cmd_type_set_cnt;

		statistic_info.invalid_cmd_cnt = _invalid_cmd_cnt;
		statistic_info.invalid_cmd_size_cnt = _invalid_cmd_size_cnt;

		statistic_info.val_trans_cnt = _val_trans_cnt;

		return statistic_info;
	}

private:
	#define HEADER_MAGIC 0x55AA

	typedef struct __attribute__((aligned(1))) {
		uint16_t h_magic;
		uint8_t cmd;
		uint16_t buf[16];
		uint16_t chk_sum;

		void calcu_check_sum() {
			chk_sum = static_cast<uint16_t>(cmd);
			for (uint8_t i = 0; i < sizeof(buf) / sizeof(buf[0]); i ++) {
				chk_sum += buf[i];
			}
		}

		bool check_rec_valid() {
			if (this->h_magic != HEADER_MAGIC) {
				printf("bad header");
				return false;
			}

			switch (static_cast<PROTO_CMD>(cmd)) {
				case PROTO_CMD::CMD_ACK_PWM_TYPE:
				case PROTO_CMD::CMD_ACK_CHANNEL_REVERT:
					break;
				default: printf("invalid cmd"); return false;
			}

			uint16_t chk = cmd;
			for (uint8_t i = 0; i < sizeof(buf) / sizeof(buf[0]); i ++) {
				chk += buf[i];
			}

			return (chk == chk_sum);
		}

		PROTO_CMD get_cmd() {
			return static_cast<PROTO_CMD>(cmd);
		}
	} frame_t;

	virtual bool send(uint8_t *p_buf, size_t size) { return false; }

	static constexpr size_t _FRAME_SIZE = sizeof(frame_t);

	uint8_t _frame_buf[_FRAME_SIZE] {};

	uint8_t _rec_buf[_MAX_BUFF_SIZE] {};

	uint32_t _val_trans_cnt{0};

	uint32_t _cmd_rate_set_cnt{0};

	uint32_t _cmd_type_set_cnt{0};

	uint32_t _cmd_invert_set_cnt{0};

	uint32_t _invalid_cmd_size_cnt{0};

	uint32_t _invalid_cmd_cnt{0};

	uint32_t _cmd_type_ack_cnt{0};

	uint32_t _cmd_rate_ack_cnt{0};

	uint32_t _cmd_invert_ack_cnt{0};

	void trans(frame_t frame) {
		memcpy(_frame_buf, reinterpret_cast<uint8_t *>(&frame), _FRAME_SIZE);
		send(_frame_buf, _FRAME_SIZE);
		memset(_frame_buf, 0, _FRAME_SIZE);
	}
};
};

#endif

