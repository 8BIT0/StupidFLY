#pragma once

#ifndef __SERIAL_DSHOT_DEF_H
#define __SERIAL_DSHOT_DEF_H

#include <stdint.h>

namespace SerialDShot {
	#define _MAX_SUPPORT_ROTOR_NUM 8

	enum OUTPUT_TYPE : int {
		OUTPUT_DSHOT150  = 0,
		OUTPUT_DSHOT300  = 1,
		OUTPUT_DSHOT600  = 2,
	};

	enum CONFIG_STATE : int {
		CONFIG_FAILED   = -1,
		CONFIG_PORT     = 0,
		CONFIG_TYPE     = 1,
		CONFIG_FINISH   = 2
	};

	enum CHANNLE_DEF : int {
		CHANNEL_DISABLE = 0,
		CHANNEL_CH1     = 1,
		CHANNEL_CH2     = 2,
		CHANNEL_CH3     = 3,
		CHANNEL_CH4     = 4,
		CHANNEL_CH5     = 5,
		CHANNLE_CH6     = 6,
		CHANNEL_CH7     = 7,
		CHANNEL_CH8     = 8
	};

	enum PROTO_CMD : int {
		CMD_INVALID		= -1,

		/* setting command */
		CMD_SET_PWM_TYPE	= 0,
		CMD_SET_CHANNEL_REVERT	= 1,

		/* setting ack */
		CMD_ACK_PWM_TYPE	= 2,
		CMD_ACK_CHANNEL_REVERT	= 3,

		/* transmit channle value */
		CMD_PROTO_CHANNLE_VAL	= 4
	};

	typedef struct {
		uint32_t val_trans_cnt;
		uint32_t cmd_rate_set_cnt;
		uint32_t cmd_type_set_cnt;
		uint32_t cmd_invert_set_cnt;
		uint32_t invalid_cmd_size_cnt;
		uint32_t invalid_cmd_cnt;
		uint32_t cmd_type_ack_cnt;
		uint32_t cmd_rate_ack_cnt;
		uint32_t cmd_invert_ack_cnt;
	} Proto_Statistic_t;
};

#endif
