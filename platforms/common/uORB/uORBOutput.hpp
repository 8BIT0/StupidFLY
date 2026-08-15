#pragma once

#include <uORB/uORB.h>
#include <uORB/uORBCommon.hpp>
#include <lib/cdev/CDev.hpp>
#include <semaphore.h>

namespace uORB
{

#define SHARE_MEM_UPDATA_MAILBOX_NUM	16		/* max parallel updata topic number */

class Output : public cdev::CDev {
public:
	typedef struct __attribute__((aligned(8))) {
		char o_name[48];
		uint16_t size;
		uint32_t crc32;
		hrt_abstime ts;
		uint16_t id;
	} share_data_header_t;

	typedef struct __attribute__((aligned(8))) {
		uint8_t sec_sum;		/* receive application use this data create receive thread */
		uint32_t single_sec_size;
		uint32_t compress_size;
		uint16_t topic_sum;
		uint8_t state;			/* link to MailboxState */
	} sync_mailbox_t;

	typedef struct {
		uint8_t *p_mailbox;
		uint8_t *p_data_s;
	} share_sec_t;

	Output();
	~Output();

	static bool bus_open();		/* open share memory */
	static void bus_close();	/* close share memory */
	void sync(const struct orb_metadata meta, const char *data);
private:
	enum class MailboxState : uint8_t {
		MAILBOX_IDLE,
		MAILBOX_UPDATED,
		MAILBOX_ACCESSING,
	};

	// static uint8_t get_msb_pos(uint32_t val);

	static void get_all_topic_space();
	static void share_mem_assign();

	static void upload_heatshrink_compress();

	static bool _upload_heatshrink;
	static int _dev_fd;

	static uint8_t *_map_mem;
	static uint32_t _single_sec_size;
	static uint32_t _share_mem_req_size;
	static share_sec_t _sec_list[SHARE_MEM_UPDATA_MAILBOX_NUM];

	static uint32_t _trans_done;
	static uint32_t _trans_abrot;
};

#if defined(__ARM_ARCH) || defined(__aarch64__)
	inline void smp_wmb() { asm volatile ("dmb ishst" ::: "memory"); }
	inline void smp_rwb() { asm volatile ("dmb ish" ::: "memory"); }
#elif defined(__x86_64__) || defined(__i386__)
	inline void smp_wmb() { asm volatile ("" ::: "memory"); }
	inline void smp_rwb() { asm volatile ("" ::: "memory"); }
#endif

}	//namespace uORB

