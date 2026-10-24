#include <lib/heatshrink/heatshrink/heatshrink_decoder.h>
#include <px4_platform_common/atomic.h>
#include <uORB/topics/uORBMessageFieldsGenerated.hpp>
#include <uORB/topics/uORBTopics.hpp>
#include <stdint.h>
#include <sys/mman.h>
#include <string.h>
#include <crc32.h>
#include <cstdio>
#include "uORBOutput.hpp"
#include "fcntl.h"
#include "sys/ioctl.h"

#define Kb 					* 1024
#define _SHARE_MEM_DEV 		"px4_shm"
#define MEM_ALIGN_SIZE		8
#define _SHARE_MEM_MAX_SIZE	128 Kb

#define _TO_BASEINFO_PTR(x) (reinterpret_cast<sync_base_info_t *>(static_cast<void *>(x)))
#define _TO_HEADER_PTR(x) (reinterpret_cast<share_data_header_t *>(static_cast<void *>(x)))

int uORB::Output::_dev_fd = -1;
bool uORB::Output::_upload_heatshrink = false;
uint8_t *uORB::Output::_map_mem = nullptr;
uint32_t uORB::Output::_share_mem_req_size = 0;
uint32_t uORB::Output::_single_mb_size = 0;
uint32_t uORB::Output::_trans_done = 0;
uint32_t uORB::Output::_trans_abrot = 0;
uint8_t uORB::Output::_mailbox_index = 0;
uORB::Output::mailbox_t* uORB::Output::_mb_list[SHARE_MEM_UPDATA_MAILBOX_NUM] = { NULL };
uint8_t* uORB::Output::_base_info_addr = nullptr;
uint8_t* uORB::Output::_compress_sec_addr = nullptr;
uint8_t* uORB::Output::_mailbox_sec_addr = nullptr;

uORB::Output::Output() : CDev(strdup("uORBOutput")) {
}

uORB::Output::~Output() {
}

bool uORB::Output::bus_open() {
	if (_dev_fd >= 0)
		return true;

	get_all_topic_space();
	share_mem_assign();

	PX4_INFO("uORB open share mem dev");
	return true;
}

void uORB::Output::bus_close() {
	if (_dev_fd < 0)
		return;

	::close(_dev_fd);
	munmap(_map_mem, _share_mem_req_size);
	_dev_fd = -1;
}

void uORB::Output::share_mem_assign() {
	/* share memory require */
	if (_share_mem_req_size == 0) {
		PX4_ERR("get share memory size failed");
		return;
	}

	/* open share memory */
	shm_unlink(_SHARE_MEM_DEV);
	_dev_fd = shm_open(_SHARE_MEM_DEV, O_RDWR | O_CREAT, 0666);
	if (_dev_fd < 0) {
		PX4_ERR("open share mem dev failed");
		return;
	}

	if (ftruncate(_dev_fd, _share_mem_req_size) != 0) {
		PX4_ERR("ftruncate failed");
		::close(_dev_fd);
		return;
	}

	_map_mem = (uint8_t *)mmap(NULL, _share_mem_req_size, PROT_WRITE, MAP_SHARED, _dev_fd, 0);
	if (_map_mem == MAP_FAILED) {
		PX4_ERR("mmap failed: %s", strerror(errno));
		::close(_dev_fd);
		_dev_fd = -1;
		return;
	}
	memset(_map_mem, 0, _share_mem_req_size);	/* clear sec */

	PX4_INFO("assign share memory");

	/* assign _map_mem to mailbox */
	const uint32_t compress_size = orb_compressed_message_formats_size();

    /* initial base info */
    _base_info_addr = _map_mem;
    _TO_BASEINFO_PTR(_base_info_addr)->sec_sum         = SHARE_MEM_UPDATA_MAILBOX_NUM;
    _TO_BASEINFO_PTR(_base_info_addr)->single_sec_size = _single_mb_size;
    _TO_BASEINFO_PTR(_base_info_addr)->compress_size   = compress_size;
    _TO_BASEINFO_PTR(_base_info_addr)->topic_sum       = orb_topics_count();

    /* initial compress share section */
	upload_heatshrink_compress(compress_size);

    _mailbox_sec_addr = _compress_sec_addr + compress_size;

    /* initial mailbox share section */
	for (uint8_t i = 0; i < SHARE_MEM_UPDATA_MAILBOX_NUM; i ++) {
		_mb_list[i] = reinterpret_cast<mailbox_t *>(_mailbox_sec_addr + i * _single_mb_size);

		/* fill data section (only header) */
		memset(reinterpret_cast<uint8_t *>(_mb_list[i]), 0, _single_mb_size);
	}
}

void uORB::Output::get_all_topic_space() {
	const struct orb_metadata *const *p_topic_meta_list = orb_get_topics();

	/* traverse orb topic list */
	/* check max length topic data */
	for (size_t i = 0; i < orb_topics_count(); i++) {
		if (_single_mb_size <= (p_topic_meta_list[i]->o_size))
			_single_mb_size = p_topic_meta_list[i]->o_size;
	}
    _single_mb_size += sizeof(share_data_header_t);

    _share_mem_req_size = 0;
    _share_mem_req_size += sizeof(sync_base_info_t);
    _share_mem_req_size += orb_compressed_message_formats_size();
	_share_mem_req_size += _single_mb_size * SHARE_MEM_UPDATA_MAILBOX_NUM;

	PX4_INFO("uORB single section space ---------- %" PRIu32, _single_mb_size);
	PX4_INFO("uORB share memory require space ---- %" PRIu32, _share_mem_req_size);
}

void uORB::Output::upload_heatshrink_compress(const unsigned compressed_formats_size) {
	if (_dev_fd < 0)
		return;

	if (!_upload_heatshrink && _share_mem_req_size) {
		const uint8_t *compressed_formats = orb_compressed_message_formats();
		const uint32_t compress_offset = sizeof(sync_base_info_t);

		uORB::smp_wmb();

        _compress_sec_addr = _base_info_addr + sizeof(sync_base_info_t);
		memcpy(_compress_sec_addr, compressed_formats, compressed_formats_size);

		_upload_heatshrink = true;
		PX4_INFO("upload heatshrink code at %d", compress_offset);
	}
}

// uint8_t uORB::Output::get_msb_pos(uint32_t val) {
// 	if (val == 0)
// 		return 0;

// 	return 31 - __builtin_clz(val);
// }

void uORB::Output::sync(const struct orb_metadata meta, const char *data) {
	share_data_header_t *p_header = nullptr;
	uint8_t *p_share_data = nullptr;
	const hrt_abstime now = hrt_absolute_time();
	uint8_t *tmp_data = const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(data));
	uint8_t topic_ins = 0;

	if ((_dev_fd < 0) || (data == nullptr))
		return;

	/* check topic been subscribed */
	for (int i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		if (orb_exists(&meta, i) == PX4_OK) {
			topic_ins ++;
		}
	}

	/* only transmit topic when subscribed */
	if ((topic_ins == 0) || \
        (p_base_info == nullptr) || \
        (_mb_list[_mailbox_index] == nullptr) || \
        (_mb_list[_mailbox_index]->p_header == nullptr) || \
        (_mb_list[_mailbox_index]->p_data_s == nullptr))
		return;

	lock();

	/* check mailbox is in using */
    p_header = _TO_HEADER_PTR(_mb_list[_mailbox_index]->p_data_s);
    p_share_data = _mb_list[_mailbox_index]->p_data_s + sizeof(share_data_header_t);

    _mailbox_index ++;
    _mailbox_index %= SHARE_MEM_UPDATA_MAILBOX_NUM;

	/* write memory block */
	uORB::smp_wmb();

	/* update header */
	p_header->ts = now;
	p_header->size = meta.o_size;
	p_header->id = meta.o_id;
	memset(p_header->o_name, '\0', sizeof(p_header->o_name));
	strcpy(p_header->o_name, meta.o_name);

	p_header->crc32 = crc32(tmp_data, meta.o_size);

	/* update data */
	memcpy(p_share_data, tmp_data, meta.o_size);

    /* update base info */
    reinterpret_cast<sync_base_info_t *>(_base_info_addr)->update_mailbox_id = _mailbox_index;

	unlock();

	_trans_done ++;
}

