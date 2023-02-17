#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6406290624312280998);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6610292794556148846);
void gnss_H_mod_fun(double *state, double *out_4190022832125984075);
void gnss_f_fun(double *state, double dt, double *out_5485440970639894550);
void gnss_F_fun(double *state, double dt, double *out_8287619921053647117);
void gnss_h_6(double *state, double *sat_pos, double *out_6986316530236158341);
void gnss_H_6(double *state, double *sat_pos, double *out_5536356530439876366);
void gnss_h_20(double *state, double *sat_pos, double *out_871747740134602654);
void gnss_H_20(double *state, double *sat_pos, double *out_6887040764717437270);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6234951525856667033);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3527993678913903080);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6234951525856667033);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3527993678913903080);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}