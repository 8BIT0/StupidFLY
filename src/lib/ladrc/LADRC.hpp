#pragma once

#ifndef __LADRC_HPP
#define __LADRC_HPP

#include <stdint.h>
#include <math.h>
#include <string.h>

class LADRC {
public:
    LADRC();
    ~LADRC() = default;

    typedef struct {
        float w0;
        float wc;
        float b0;

        float r;

        float v1;
        float v2;

        float z1;
        float z2;
        float z3;

        float u0;
    } ProcessData_TypeDef;

    /* input expactation state and current system state
     * output real control val
     */
    float proc(float exp_v, float cur_v, float dt);

    void set_param(float w0, float wc, float b0) {
        _eso_p_w0 = w0;
        _eso_p_b0 = b0;
        _sef_p_wc = wc;
    }

    ProcessData_TypeDef get_proc_data() {
        ProcessData_TypeDef prc_data;

        memset(reinterpret_cast<uint8_t *>(&prc_data), 0, sizeof(ProcessData_TypeDef));

        prc_data.w0 = _eso_p_w0;
        prc_data.b0 = _eso_p_b0;
        prc_data.wc = _sef_p_wc;

        prc_data.r  = _td_p_r;

        prc_data.v1 = _td_v1;
        prc_data.v2 = _td_v2;

        prc_data.z1 = _eso_z1;
        prc_data.z2 = _eso_z2;
        prc_data.z3 = _eso_z3;

        prc_data.u0 = _sef_u0;

        return prc_data;
    }

private:
    void TD(float exp_v, float cur_v, float dt);
    void ESO(float mea_y, float u, float dt);
    void SEF();

    bool is_param_valid() { return ((_eso_p_w0 > 0.0f) && (_eso_p_b0 > 0.0f) && (_sef_p_wc > 0.0f)); }

    /************** TD *************/
    float td_get_r(float exp_v, float init_v, float dt);

    float _td_p_r{0.0f};        /* track factory */

    float _td_v1{0.0f};         /* track signal */
    float _td_v2{0.0f};         /* track signal differential */

    float _td_v1_lst{0.0f};
    float _td_v2_lst{0.0f};

    /************* ESO *************/
    float eso_get_u(float ctl_u);

    float _eso_p_w0{0.0f};      /* observation factory */
    float _eso_p_b0{0.0f};      /* sys parameter */

    float _eso_z1{0.0f};
    float _eso_z2{0.0f};
    float _eso_z3{0.0f};

    /************* SEF *************/
    float sef_get_Kp();
    float sef_get_Kd();

    float _sef_p_wc{0.0f};      /* controller factory */
    float _sef_u0{0.0f};        /* virtual control value */
};


#endif


