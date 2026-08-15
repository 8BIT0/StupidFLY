#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stddef.h>

template<typename T>
class LyraImu_IIO_Set {
public:
    LyraImu_IIO_Set(const char* path, const char* file, T data, bool fixed) :
        _path(path),
        _file(file),
        _data(data),
        _fixed(fixed)
    {
        _file_valid = false;

        if ((path != NULL) && (file != nullptr)) {
            if constexpr (std::is_same_v<T, const char *> || std::is_same_v<T, char *>) {
                if (strlen(_data))
                    _str = data;
            } else {
                dataToString();
            }
            _file_valid = true;
        }
    }

    ~LyraImu_IIO_Set() = default;

    bool set(T data) {
        if (_fixed)
            return false;

        if constexpr (std::is_same_v<T, const char *> || std::is_same_v<T, char *>) {
            _str = data;
        } else {
            _data = data;
        }

        dataToString();

        return true;
    };

    void dataToString() {
        /* convert data to string */
        if constexpr (std::is_integral_v<T>) {
            snprintf(_value_str, MAX_VALUE_SIZE, "%d", (int)_data);
        } else if constexpr (std::is_floating_point_v<T>) {
            snprintf(_value_str, MAX_VALUE_SIZE, "%.8f", (double)_data);
        }
    }

    bool config() {
        char file_path[MAX_PATH_LEN];
        char read_out[MAX_VALUE_SIZE];
        char *p_val_str = nullptr;
        FILE *fd = nullptr;
        int ret = 0;

        if (!_file_valid)
            return false;

        memset(file_path, '\0', MAX_PATH_LEN);
        snprintf(file_path, MAX_PATH_LEN, "%s/%s", _path, _file);

        /*********************** open file *******************/
        fd = fopen(file_path, "w+");
        if (fd == nullptr) {
            PX4_ERR("open file %s failed", file_path);
            return false;
        }

        /*********************** write file *******************/
        if (std::is_same_v<T, const char*> || std::is_same_v<T, char*>) {
            ret = fprintf(fd, "%s", _str);
            p_val_str = _str;
        } else {
            ret = fprintf(fd, "%s", _value_str) < 0;
            p_val_str = _value_str;
        }

        if (ret < 0) {
            PX4_ERR("write %s to file %s failed error code %d", p_val_str, file_path, ret);
            fclose(fd);
            return false;
        }

        /*********************** read  file ********************/
        fseek(fd, 0, SEEK_SET);
        clock_t s_time = 0;
        uint16_t line = 0;

        while (fgets(read_out, MAX_VALUE_SIZE, fd) != nullptr) {
            /* check time out */
            uint32_t elapsed_ms = (clock() - s_time) * 1000 / CLOCKS_PER_SEC;
            if (elapsed_ms > CHECK_TIMEOUT_MS) {
                PX4_ERR("check timeout after %d ms", CHECK_TIMEOUT_MS);
                ret = -1;
                break;
            }

            if constexpr (std::is_integral_v<T>) {
                if (atoi(read_out) != _data) {
                    PX4_ERR("operating file %s set failed", file_path);
                    PX4_ERR("value unmatch: %s", _value_str);
                    ret = -1;
                    break;
                }
            } else if constexpr (std::is_floating_point_v<T>) {
                if (atof(read_out) != _data) {
                    PX4_ERR("operating file %s", file_path);
                    PX4_ERR("value unmatch: %s", _value_str);
                    ret = -1;
                    break;
                }
            } else if constexpr (std::is_same_v<T, const char *> || std::is_same_v<T, char *>) {
                if (memcmp(read_out, _str, MAX_VALUE_SIZE) != 0) {
                    PX4_ERR("operating file %s", file_path);
                    PX4_ERR("value unmatch: %s", _str);
                    ret = -1;
                    break;
                }
            }

            line ++;
            if (line > 1) {
                PX4_ERR("file %s line size error", _path);
                ret = -1;
                break;
            }
        }

        fclose(fd);

        if (ret < 0)
            return false;

        return true;
    }

    const char *file_name() {return _file;};

private:
    static constexpr uint32_t MAX_VALUE_SIZE    = 64;
    static constexpr uint32_t MAX_PATH_LEN      = 256;
    static constexpr uint32_t CHECK_TIMEOUT_MS  = 1000;

    const char* _path{nullptr};
    const char* _file{nullptr};
    T _data;
    bool _fixed;

    char _value_str[MAX_VALUE_SIZE];
    char *_str;

    bool _file_valid{false};
};

