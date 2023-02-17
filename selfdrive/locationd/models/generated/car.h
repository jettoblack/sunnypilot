#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_4239990911220653914);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6013891986074755824);
void car_H_mod_fun(double *state, double *out_4325368645737364917);
void car_f_fun(double *state, double dt, double *out_1889004700640037715);
void car_F_fun(double *state, double dt, double *out_529478043801040338);
void car_h_25(double *state, double *unused, double *out_2747349570204871243);
void car_H_25(double *state, double *unused, double *out_1087706381204727692);
void car_h_24(double *state, double *unused, double *out_7901114660987291782);
void car_H_24(double *state, double *unused, double *out_2470219217597595444);
void car_h_30(double *state, double *unused, double *out_890099958049151107);
void car_H_30(double *state, double *unused, double *out_1430626577302520935);
void car_h_26(double *state, double *unused, double *out_4530607195872700425);
void car_H_26(double *state, double *unused, double *out_4829209700078783916);
void car_h_27(double *state, double *unused, double *out_3690859423106533678);
void car_H_27(double *state, double *unused, double *out_744136734497903976);
void car_h_29(double *state, double *unused, double *out_53409894852511328);
void car_H_29(double *state, double *unused, double *out_1940857921616913119);
void car_h_28(double *state, double *unused, double *out_6727645474550866363);
void car_H_28(double *state, double *unused, double *out_3141541095452617455);
void car_h_31(double *state, double *unused, double *out_2145931412591507689);
void car_H_31(double *state, double *unused, double *out_5455417802312135392);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}