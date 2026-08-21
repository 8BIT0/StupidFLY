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
    _eso_p_wo   = 0.0f;
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
float LADRC::td_fhan(float exp_v, float dt) {
    /*
     * ADRC fhan
     * d = r * dt^2
     * a0 = h * v2
     * y = v1 + a0
     * a1 = sqrt(d * (d + 8 * |y|))
     * a2 = a0 + sign(y) * (a1 - d) / 2
     * a = (a0 + y) * fsg(y, d) + a2 * (1 - fsg(y, d))
     * fhan = -r * (a / d) * fsg(a, d) - r * sign(a) * (1 - fsg(a, d))
     */
    float d = _td_p_r * SQUARE(dt);
    float a0 = dt * _td_v2;
    float y = (_td_v1 - exp_v) + a0;
    float a1 = sqrtf(d * (d + 8 * fabsf(y)));
    float a2 = a0 + td_sign(y) * (a1 - d) / 2;
    float a = (a0 + y) * td_fsg(y, d) + a2 * (1 - td_fsg(y, d));
    float fhan = -_td_p_r * (a / d) * td_fsg(a, d) - _td_p_r * td_sign(a) * (1 - td_fsg(a, d));
    return fhan;
}

void LADRC::TD(float exp_v, float dt) {
    /*
     * ----------- legacy ---------
     * origin LADRC TD formular
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

    /* use adrc fhan here */
    float fh = td_fhan(exp_v, dt);

    _td_v1 += dt * _td_v2;
    _td_v2 += dt * fh;
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
    float g_l1 = 3 * _eso_p_wo;
    float g_l2 = 3 * SQUARE(_eso_p_wo);
    float g_l3 = CUBIC(_eso_p_wo);
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
    TD(exp_v, dt);
    float u = SEF();
    ESO(cur_v, u, dt);
    u = eso_get_u(u);

    return u;
}

