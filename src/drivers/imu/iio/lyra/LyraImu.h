#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gyro_fifo.h>
#include "LyraImu_IIO_config.h"

class LyraImu : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem {
public:

    static Descriptor desc;

    LyraImu(Rotation def_rotation);

    ~LyraImu() override = default;

    static int task_spawn(int argc, char *argv[]);

    static int custom_command(int argc, char *argv[]);

    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } iio_sensor_axis_data;

    typedef struct {
        iio_sensor_axis_data data;
        int16_t temp;
        hrt_abstime time_stamp;
    } iio_data_TypeDef;

    typedef union {
        iio_data_TypeDef s_sensor;
        uint8_t buf[sizeof(iio_data_TypeDef)];
    } iio_bin_data_TypeDef;

    static constexpr uint16_t IMU_RATE_HZ = 4000;
    static constexpr float FIFO_SAMPLE_DT{1e6f / IMU_RATE_HZ};     // 8000 Hz accel & gyro ODR configured
    static constexpr uint8_t FIFO_DATA_SIZE = 20;
    static constexpr uint32_t MAX_PATH_LEN = 128;

    PX4Accelerometer _px4_accel;
    PX4Gyroscope _px4_gyro;

    int _gyr_dev_fd{-1};
    int _acc_dev_fd{-1};

    int _sample_watermark{-1};

    uint32_t _sensor_interval_us{1000}; /* sample rate 1KHz */

    uint16_t iio_check_buffer_availabel(const char *path);
    void Run() override;

    LyraImu_IIO_Set<int> _acclel_config_list[8] = {
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer/enable",         0,              true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "sampling_frequency",    IMU_RATE_HZ,    true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer0/in_accel_x_en", 1,              true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer0/in_accel_y_en", 1,              true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer0/in_accel_z_en", 1,              true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer/length",         0,              false),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer/watermark",      0,              false),
        LyraImu_IIO_Set<int>(CONFIG_IIO_ACCEL_CONFIG_PATH, "buffer/enable",         1,              true)
    };

    LyraImu_IIO_Set<int> _gyro_config_list[8] = {
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer/enable",           0,             true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "sampling_frequency",      IMU_RATE_HZ,   true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer0/in_anglvel_x_en", 1,             true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer0/in_anglvel_y_en", 1,             true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer0/in_anglvel_z_en", 1,             true),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer/length",           0,             false),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer/watermark",        0,             false),
        LyraImu_IIO_Set<int>(CONFIG_IIO_GYRO_CONFIG_PATH, "buffer/enable",           1,             true)
    };
};

