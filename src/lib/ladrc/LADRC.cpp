/*
 *  Author: 8_B!T0
 *  LADRC implement reference website https://zhuanlan.zhihu.com/p/671469224
 */
#include "LADRC.hpp"
#include <stdio.h>

#define MS_PER_S    1000.0f
#define MIN_DT      (1.0f / MS_PER_S)
#define SQUARE(x)   ((x) * (x))
#define CUBIC(x)    ((x) * (x) * (x))

LADRC::LADRC() {
    _td_p_r     = 0.0f;
    _td_v1      = 0.0f;
    _td_v2      = 0.0f;
    _td_v1_lst  = 0.0f;
    _td_v2_lst  = 0.0f;
    _eso_p_w0   = 0.0f;
    _eso_p_b0   = 0.0f;
    _eso_z1     = 0.0f;
    _eso_z2     = 0.0f;
    _eso_z3     = 0.0f;
    _sef_p_wc   = 0.0f;
}

/*
 *        ----------
 *        |        |--v1-->
 * --v0-->|   TD   |
 *        |        |--v2-->
 *        ----------
 */
float LADRC::td_get_r(float exp_v, float init_v, float dt) {
    /* update r
     * r = 4 * (exp_state - cur_state) / Dt
     * r must > 0
     * system pole must be in left half plane
     */
    // float r = 4 * fabs(exp_v - init_v) / dt;
    float r = 2 * _eso_p_w0;

    return r;
}

void LADRC::TD(float exp_v, float init_v, float dt) {
    /* frmular
     * .
     * v1 = v2
     * v1(k + 1) = v1(k) + T * v2(k)
     * .
     * v2 = -2r * v2 - r^2(v1 - v0)
     * v2(k + 1) = v2(k) + T * (-2 * r * v2(k) - r^2 * (v1(k) - v0(k)))
     *
     * v0 target
     * r track factory
     * v1 and v2 is TD output
     */
    _td_p_r = td_get_r(exp_v, init_v, dt);

    _td_v1 = _td_v1_lst + dt * _td_v2_lst;
    _td_v2 = _td_v2_lst + dt * (-2 * _td_p_r * _td_v2_lst - SQUARE(_td_p_r) * (_td_v1_lst - exp_v));

    _td_v1_lst = _td_v1;
    _td_v2_lst = _td_v2;
}

/*
 *
 *          ---------
 * -- e1 -->|       |
 *          |  SEF  |------> u0
 * -- e2 -->|       |
 *          ---------
 */

float LADRC::sef_get_Kp() {
    return (_sef_p_wc * _sef_p_wc);
}

float LADRC::sef_get_Kd() {
    return (2 * _sef_p_wc);
}

float LADRC::SEF() {
    float Kp = sef_get_Kp();
    float Kd = sef_get_Kd();
    float e1 = (_td_v1 - _eso_z1);
    float e2 = (_td_v2 - _eso_z2);

    return (Kp * e1 + Kd * e2);
}

/*
 *                 ---------         |---> z1
 * -- measure y -->|       |----------
 *                 |  ESO  |-------------> z2
 * -- control u -->|       |----------
 *                 ---------         |---> z3
 */

/* disturbance compensation */
float LADRC::eso_get_u(float ctl_u) {
    float output = 0.0f;

    if (is_param_valid() == false)
        return 0.0f;

    output = (ctl_u - _eso_z3 / _eso_p_b0);

    return output;
}

void LADRC::ESO(float mea_y, float u, float dt) {
    float g_l1 = 3 * _eso_p_w0;
    float g_l2 = 3 * SQUARE(_eso_p_w0);
    float g_l3 = CUBIC(_eso_p_w0);
    float err = (mea_y - _eso_z1);

    /* get z1 */
    _eso_z1 += dt * (_eso_z2 + g_l1 * err);

    /* get z2 */
    _eso_z2 += dt * (_eso_z3 + g_l2 * err + _eso_p_b0 * u);

    /* get z3 */
    _eso_z3 += dt * g_l3 * err;

}

/*
 *  exp_v: expected value
 *  cur_v: current measurement value
 */
float LADRC::proc(float exp_v, float cur_v, float dt) {
    /*
     * process order (strict match block diagram)
     * 1. TD
     * 2. SEF → u0
     * 3. disturbance compensation → u
     * 4. ESO update(z1,z2,z3) using measured cur_v & current u
     * first period not process
     */
    float u = 0.0f;

    TD(exp_v, cur_v, dt);
    float u = SEF();
    ESO(cur_v, u, dt);
    u = eso_get_u(u);

    return u;
}

