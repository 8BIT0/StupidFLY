#pragma once

#ifndef __LADRC_HPP
#define __LADRC_HPP

#include <stdint.h>
#include <math.h>

class LADRC {
public:
    LADRC();
    ~LADRC() = default;

    /* input expactation state and current system state
     * output real control val
     */
    float proc(float exp_v, float cur_v, float dt);

    void set_param(float w0, float wc, float b0) {
        _eso_p_w0 = w0;
        _eso_p_b0 = b0;
        _sef_p_wc = wc;
    }

private:
    void TD(float exp_v, float cur_v, float dt);
    void ESO(float mea_y, float u, float dt);
    void SEF();

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


