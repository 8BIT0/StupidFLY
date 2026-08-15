#include "serial_dshot_proto.h"

using namespace SerialDShot;

void Proto::set_type(OUTPUT_TYPE type) {
	frame_t frame{};

	frame.h_magic = HEADER_MAGIC;
	frame.cmd = CMD_SET_PWM_TYPE;

	memset(frame.buf, 0, sizeof(frame.buf));
	frame.buf[0] = static_cast<uint16_t>(type);
	frame.calcu_check_sum();

	trans(frame);

	_cmd_type_set_cnt ++;
}

void Proto::set_ch_revert(CHANNLE_DEF ch) {
	frame_t frame{};

	frame.h_magic = HEADER_MAGIC;
	frame.cmd = CMD_SET_CHANNEL_REVERT;

	memset(frame.buf, 0, sizeof(frame.buf));
	frame.buf[0] = static_cast<uint16_t>(ch);
	frame.calcu_check_sum();

	trans(frame);

	_cmd_invert_set_cnt ++;
}

void Proto::pwm_out(uint16_t *pwm_val, uint8_t ch_sum) {
	frame_t frame{};

	if (ch_sum > static_cast<uint8_t>(CHANNLE_DEF::CHANNEL_CH8))
		return;

	frame.h_magic = HEADER_MAGIC;
	frame.cmd = CMD_PROTO_CHANNLE_VAL;

	memset(frame.buf, 0, sizeof(frame.buf));
	memcpy(frame.buf, pwm_val, sizeof(uint16_t) * ch_sum);
	frame.calcu_check_sum();

	trans(frame);

	_val_trans_cnt ++;
}

PROTO_CMD Proto::parse_cmd(size_t size) {
	frame_t frame;
	PROTO_CMD cmd;
	uint16_t frame_len = _FRAME_SIZE - 1;

	if (size < frame_len) {
		_invalid_cmd_size_cnt ++;
		return PROTO_CMD::CMD_INVALID;
	}

	memset(&frame, 0, _FRAME_SIZE);
	memcpy(reinterpret_cast<uint8_t *>(&frame), _rec_buf, _FRAME_SIZE);

	if (!frame.check_rec_valid()) {
		_invalid_cmd_cnt ++;
		return PROTO_CMD::CMD_INVALID;
	}

	cmd = frame.get_cmd();

	switch (cmd) {
		case PROTO_CMD::CMD_ACK_PWM_TYPE:        _cmd_type_ack_cnt++;   break;
		case PROTO_CMD::CMD_ACK_CHANNEL_REVERT:  _cmd_invert_ack_cnt++; break;
		default: _invalid_cmd_cnt ++; break;
	}

	return cmd;
}
