#include <px4_platform_common/px4_config.h>
#include <stdio.h>
#include <string.h>

int board_get_px4_guid(px4_guid_t px4_guid) {
	/* TODO get linux uuid */

	return PX4_GUID_BYTE_LENGTH;
}

