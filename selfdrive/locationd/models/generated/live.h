#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4754043332532862);
void live_err_fun(double *nom_x, double *delta_x, double *out_3486327650840323934);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3206258889146485027);
void live_H_mod_fun(double *state, double *out_5814422614993047354);
void live_f_fun(double *state, double dt, double *out_8496102306548205565);
void live_F_fun(double *state, double dt, double *out_484621888802379269);
void live_h_4(double *state, double *unused, double *out_4386749140107033805);
void live_H_4(double *state, double *unused, double *out_6778935493254232322);
void live_h_9(double *state, double *unused, double *out_6120782681241824943);
void live_H_9(double *state, double *unused, double *out_4380589645190871824);
void live_h_10(double *state, double *unused, double *out_2605822942370700538);
void live_H_10(double *state, double *unused, double *out_7374247591185266971);
void live_h_12(double *state, double *unused, double *out_3594675024919268009);
void live_H_12(double *state, double *unused, double *out_6648352172423357499);
void live_h_35(double *state, double *unused, double *out_4684687486973321183);
void live_H_35(double *state, double *unused, double *out_8301146523082711918);
void live_h_32(double *state, double *unused, double *out_433474662114768945);
void live_H_32(double *state, double *unused, double *out_8169981738141971449);
void live_h_13(double *state, double *unused, double *out_2599633679028641608);
void live_H_13(double *state, double *unused, double *out_6900500889336801405);
void live_h_14(double *state, double *unused, double *out_6120782681241824943);
void live_H_14(double *state, double *unused, double *out_4380589645190871824);
void live_h_33(double *state, double *unused, double *out_4395715454920679777);
void live_H_33(double *state, double *unused, double *out_5150589518443854314);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}