#include "LyraImu.h"
#include <time.h>

#if !defined(CONFIG_IIO_GYRO_CONFIG_PATH) || !defined(CONFIG_IIO_ACCEL_CONFIG_PATH) || !defined(CONFIG_IIO_GYRO_DEV_PATH) || !defined(CONFIG_IIO_ACCEL_DEV_PATH)
    #error "define IIO_GYRO and IIO_ACCEL path first"
#endif

#define CONVERT_UINT16_ENDIAN(x) (int16_t)((((uint8_t *)&x)[0] << 8) | ((uint8_t *)(&x))[1])
#define CONVERT_UINT64_ENDIAN(x) ((x & 0x00000000000000FFULL) << 56) | \
                                 ((x & 0x000000000000FF00ULL) << 40) | \
                                 ((x & 0x0000000000FF0000ULL) << 24) | \
                                 ((x & 0x00000000FF000000ULL) << 8)  | \
                                 ((x & 0x000000FF00000000ULL) >> 8)  | \
                                 ((x & 0x0000FF0000000000ULL) >> 24) | \
                                 ((x & 0x00FF000000000000ULL) >> 40) | \
                                 ((x & 0xFF00000000000000ULL) >> 56)


ModuleBase::Descriptor LyraImu::desc{task_spawn, custom_command, print_usage};

LyraImu::LyraImu(Rotation def_rotation = Rotation::ROTATION_NONE):
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _px4_accel(1310988, def_rotation),
    _px4_gyro(1310988, def_rotation) {

    _sensor_interval_us = roundf(1.e6f / _px4_gyro.get_max_rate_hz());

	_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
    _px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);

    _px4_gyro.set_range(math::radians(2000.f));
	_px4_gyro.set_scale(math::radians(2000.f) / static_cast<float>(INT16_MAX - 1));
}

bool LyraImu::init() {
    const float min_interval = FIFO_SAMPLE_DT;
    float interval_us = math::max(roundf((1e6f / (float)_px4_gyro.get_max_rate_hz()) / min_interval) * min_interval, min_interval);
	_sample_watermark = (uint16_t)roundf((float)interval_us / (1e6f / IMU_RATE_HZ));

    PX4_INFO("lyra iio device init");
    PX4_INFO("sample rate: %d", _px4_gyro.get_max_rate_hz());
    PX4_INFO("watermark %d", _sample_watermark);

    /* open iio device icm42688p device */
    /* iio icm42688p config param must be 16g/2000dps 4K-odr */
    /************************** ACC CFG ***********************/
    for (auto acc_cfg : _acclel_config_list) {
        if (strcmp(acc_cfg.file_name(), "buffer/length") == 0) {
            acc_cfg.set(_sample_watermark * FIFO_DATA_SIZE);
        } else if (strcmp(acc_cfg.file_name(), "buffer/watermark") == 0) {
            acc_cfg.set(_sample_watermark);
        }
        acc_cfg.config();
    }

    /************************** GYR CFG ***********************/
    for (auto gyr_cfg : _gyro_config_list) {
        if (strcmp(gyr_cfg.file_name(), "buffer/length") == 0) {
            gyr_cfg.set(_sample_watermark * FIFO_DATA_SIZE);
        } else if (strcmp(gyr_cfg.file_name(), "buffer/watermark") == 0) {
            gyr_cfg.set(_sample_watermark);
        }
        gyr_cfg.config();
    }

    /* open file */
    _gyr_dev_fd = open(CONFIG_IIO_GYRO_DEV_PATH,  O_RDONLY | O_NONBLOCK);
    _acc_dev_fd = open(CONFIG_IIO_ACCEL_DEV_PATH, O_RDONLY | O_NONBLOCK);
    if ((_gyr_dev_fd < 0) || (_acc_dev_fd < 0)) {
        PX4_ERR("open device failed g_dev: %d | a_dev: %d", _gyr_dev_fd, _acc_dev_fd);
        return false;
    }

    PX4_INFO("LyraImu config done g_dev: %d | a_dev: %d", _gyr_dev_fd, _acc_dev_fd);

	ScheduleOnInterval(_sensor_interval_us);

	return true;
}

void LyraImu::Run()
{
	sensor_accel_fifo_s accel{};
    sensor_gyro_fifo_s gyro{};
    iio_bin_data_TypeDef acc_tmp{};
    iio_bin_data_TypeDef gyr_tmp{};
    uint8_t i = 0;
    hrt_abstime now = hrt_absolute_time();

    if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

    /* check accel buffer available */
    uint16_t acc_available = iio_check_buffer_availabel(CONFIG_IIO_ACCEL_CONFIG_PATH);
    uint16_t gyr_available = iio_check_buffer_availabel(CONFIG_IIO_GYRO_CONFIG_PATH);

    if ((_acc_dev_fd > 0) && (_sample_watermark <= acc_available)) {
        /* set accel time stamp and tempra */
        accel.dt = CONVERT_UINT16_ENDIAN(acc_tmp.s_sensor.time_stamp) * 17.06f;

        /* read acc data */
        for (i = 0; i < _sample_watermark; i++) {
            read(_acc_dev_fd, acc_tmp.buf, sizeof(iio_bin_data_TypeDef));

            accel.x[i] = CONVERT_UINT16_ENDIAN(acc_tmp.s_sensor.data.x);
            accel.y[i] = CONVERT_UINT16_ENDIAN(acc_tmp.s_sensor.data.y);
            accel.z[i] = CONVERT_UINT16_ENDIAN(acc_tmp.s_sensor.data.z);

            accel.samples ++;
        }

        // correct frame for publication
        for (i = 0; i < accel.samples; i++) {
            // sensor's frame is +x forward, +y left, +z up
            //  flip y & z to publish right handed with z down (x forward, y right, z down)
            accel.x[i] = accel.x[i];
            accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
            accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
        }

        accel.timestamp_sample = now;
        _px4_accel.updateFIFO(accel);
    }

    if ((_gyr_dev_fd > 0) && (_sample_watermark <= gyr_available)) {
        /* set gyro time stamp and tempra */
        gyro.dt = CONVERT_UINT16_ENDIAN(gyr_tmp.s_sensor.time_stamp) * 17.06f;

        /* read gyr data */
        for (i = 0; i < _sample_watermark; i++) {

            read(_gyr_dev_fd, gyr_tmp.buf, sizeof(iio_bin_data_TypeDef));

            gyro.x[i] = CONVERT_UINT16_ENDIAN(gyr_tmp.s_sensor.data.x);
            gyro.y[i] = CONVERT_UINT16_ENDIAN(gyr_tmp.s_sensor.data.y);
            gyro.z[i] = CONVERT_UINT16_ENDIAN(gyr_tmp.s_sensor.data.z);

            gyro.samples ++;
        }

        // correct frame for publication
        for (i = 0; i < gyro.samples; i++) {
            // sensor's frame is +x forward, +y left, +z up
            //  flip y & z to publish right handed with z down (x forward, y right, z down)
            gyro.x[i] = gyro.x[i];
            gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
            gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
        }

        gyro.timestamp_sample = now;
        _px4_gyro.updateFIFO(gyro);
    }
}

uint16_t LyraImu::iio_check_buffer_availabel(const char *path) {
    char path_tmp[MAX_PATH_LEN];
    char val[32];

    snprintf(path_tmp, sizeof(path_tmp), "%s/%s", path, "buffer/data_available");
    FILE *fd = fopen(path_tmp, "r");

    if (fd == NULL)
        return 0;

    fgets(val, sizeof(val), fd);

    fclose(fd);

    return atoi(val);
}

int LyraImu::task_spawn(int argc, char *argv[]) {
	LyraImu *instance = new LyraImu();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int LyraImu::custom_command(int argc, char *argv[]) {
	return print_usage("unknown command");
}

int LyraImu::print_usage(const char *reason) {
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(R"DESCR_STR(### Description)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("lyra_imu", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int lyra_imu_main(int argc, char *argv[])
{
	return ModuleBase::main(LyraImu::desc, argc, argv);
}


