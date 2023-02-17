#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4239990911220653914) {
   out_4239990911220653914[0] = delta_x[0] + nom_x[0];
   out_4239990911220653914[1] = delta_x[1] + nom_x[1];
   out_4239990911220653914[2] = delta_x[2] + nom_x[2];
   out_4239990911220653914[3] = delta_x[3] + nom_x[3];
   out_4239990911220653914[4] = delta_x[4] + nom_x[4];
   out_4239990911220653914[5] = delta_x[5] + nom_x[5];
   out_4239990911220653914[6] = delta_x[6] + nom_x[6];
   out_4239990911220653914[7] = delta_x[7] + nom_x[7];
   out_4239990911220653914[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6013891986074755824) {
   out_6013891986074755824[0] = -nom_x[0] + true_x[0];
   out_6013891986074755824[1] = -nom_x[1] + true_x[1];
   out_6013891986074755824[2] = -nom_x[2] + true_x[2];
   out_6013891986074755824[3] = -nom_x[3] + true_x[3];
   out_6013891986074755824[4] = -nom_x[4] + true_x[4];
   out_6013891986074755824[5] = -nom_x[5] + true_x[5];
   out_6013891986074755824[6] = -nom_x[6] + true_x[6];
   out_6013891986074755824[7] = -nom_x[7] + true_x[7];
   out_6013891986074755824[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4325368645737364917) {
   out_4325368645737364917[0] = 1.0;
   out_4325368645737364917[1] = 0;
   out_4325368645737364917[2] = 0;
   out_4325368645737364917[3] = 0;
   out_4325368645737364917[4] = 0;
   out_4325368645737364917[5] = 0;
   out_4325368645737364917[6] = 0;
   out_4325368645737364917[7] = 0;
   out_4325368645737364917[8] = 0;
   out_4325368645737364917[9] = 0;
   out_4325368645737364917[10] = 1.0;
   out_4325368645737364917[11] = 0;
   out_4325368645737364917[12] = 0;
   out_4325368645737364917[13] = 0;
   out_4325368645737364917[14] = 0;
   out_4325368645737364917[15] = 0;
   out_4325368645737364917[16] = 0;
   out_4325368645737364917[17] = 0;
   out_4325368645737364917[18] = 0;
   out_4325368645737364917[19] = 0;
   out_4325368645737364917[20] = 1.0;
   out_4325368645737364917[21] = 0;
   out_4325368645737364917[22] = 0;
   out_4325368645737364917[23] = 0;
   out_4325368645737364917[24] = 0;
   out_4325368645737364917[25] = 0;
   out_4325368645737364917[26] = 0;
   out_4325368645737364917[27] = 0;
   out_4325368645737364917[28] = 0;
   out_4325368645737364917[29] = 0;
   out_4325368645737364917[30] = 1.0;
   out_4325368645737364917[31] = 0;
   out_4325368645737364917[32] = 0;
   out_4325368645737364917[33] = 0;
   out_4325368645737364917[34] = 0;
   out_4325368645737364917[35] = 0;
   out_4325368645737364917[36] = 0;
   out_4325368645737364917[37] = 0;
   out_4325368645737364917[38] = 0;
   out_4325368645737364917[39] = 0;
   out_4325368645737364917[40] = 1.0;
   out_4325368645737364917[41] = 0;
   out_4325368645737364917[42] = 0;
   out_4325368645737364917[43] = 0;
   out_4325368645737364917[44] = 0;
   out_4325368645737364917[45] = 0;
   out_4325368645737364917[46] = 0;
   out_4325368645737364917[47] = 0;
   out_4325368645737364917[48] = 0;
   out_4325368645737364917[49] = 0;
   out_4325368645737364917[50] = 1.0;
   out_4325368645737364917[51] = 0;
   out_4325368645737364917[52] = 0;
   out_4325368645737364917[53] = 0;
   out_4325368645737364917[54] = 0;
   out_4325368645737364917[55] = 0;
   out_4325368645737364917[56] = 0;
   out_4325368645737364917[57] = 0;
   out_4325368645737364917[58] = 0;
   out_4325368645737364917[59] = 0;
   out_4325368645737364917[60] = 1.0;
   out_4325368645737364917[61] = 0;
   out_4325368645737364917[62] = 0;
   out_4325368645737364917[63] = 0;
   out_4325368645737364917[64] = 0;
   out_4325368645737364917[65] = 0;
   out_4325368645737364917[66] = 0;
   out_4325368645737364917[67] = 0;
   out_4325368645737364917[68] = 0;
   out_4325368645737364917[69] = 0;
   out_4325368645737364917[70] = 1.0;
   out_4325368645737364917[71] = 0;
   out_4325368645737364917[72] = 0;
   out_4325368645737364917[73] = 0;
   out_4325368645737364917[74] = 0;
   out_4325368645737364917[75] = 0;
   out_4325368645737364917[76] = 0;
   out_4325368645737364917[77] = 0;
   out_4325368645737364917[78] = 0;
   out_4325368645737364917[79] = 0;
   out_4325368645737364917[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1889004700640037715) {
   out_1889004700640037715[0] = state[0];
   out_1889004700640037715[1] = state[1];
   out_1889004700640037715[2] = state[2];
   out_1889004700640037715[3] = state[3];
   out_1889004700640037715[4] = state[4];
   out_1889004700640037715[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1889004700640037715[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1889004700640037715[7] = state[7];
   out_1889004700640037715[8] = state[8];
}
void F_fun(double *state, double dt, double *out_529478043801040338) {
   out_529478043801040338[0] = 1;
   out_529478043801040338[1] = 0;
   out_529478043801040338[2] = 0;
   out_529478043801040338[3] = 0;
   out_529478043801040338[4] = 0;
   out_529478043801040338[5] = 0;
   out_529478043801040338[6] = 0;
   out_529478043801040338[7] = 0;
   out_529478043801040338[8] = 0;
   out_529478043801040338[9] = 0;
   out_529478043801040338[10] = 1;
   out_529478043801040338[11] = 0;
   out_529478043801040338[12] = 0;
   out_529478043801040338[13] = 0;
   out_529478043801040338[14] = 0;
   out_529478043801040338[15] = 0;
   out_529478043801040338[16] = 0;
   out_529478043801040338[17] = 0;
   out_529478043801040338[18] = 0;
   out_529478043801040338[19] = 0;
   out_529478043801040338[20] = 1;
   out_529478043801040338[21] = 0;
   out_529478043801040338[22] = 0;
   out_529478043801040338[23] = 0;
   out_529478043801040338[24] = 0;
   out_529478043801040338[25] = 0;
   out_529478043801040338[26] = 0;
   out_529478043801040338[27] = 0;
   out_529478043801040338[28] = 0;
   out_529478043801040338[29] = 0;
   out_529478043801040338[30] = 1;
   out_529478043801040338[31] = 0;
   out_529478043801040338[32] = 0;
   out_529478043801040338[33] = 0;
   out_529478043801040338[34] = 0;
   out_529478043801040338[35] = 0;
   out_529478043801040338[36] = 0;
   out_529478043801040338[37] = 0;
   out_529478043801040338[38] = 0;
   out_529478043801040338[39] = 0;
   out_529478043801040338[40] = 1;
   out_529478043801040338[41] = 0;
   out_529478043801040338[42] = 0;
   out_529478043801040338[43] = 0;
   out_529478043801040338[44] = 0;
   out_529478043801040338[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_529478043801040338[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_529478043801040338[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_529478043801040338[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_529478043801040338[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_529478043801040338[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_529478043801040338[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_529478043801040338[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_529478043801040338[53] = -9.8000000000000007*dt;
   out_529478043801040338[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_529478043801040338[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_529478043801040338[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_529478043801040338[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_529478043801040338[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_529478043801040338[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_529478043801040338[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_529478043801040338[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_529478043801040338[62] = 0;
   out_529478043801040338[63] = 0;
   out_529478043801040338[64] = 0;
   out_529478043801040338[65] = 0;
   out_529478043801040338[66] = 0;
   out_529478043801040338[67] = 0;
   out_529478043801040338[68] = 0;
   out_529478043801040338[69] = 0;
   out_529478043801040338[70] = 1;
   out_529478043801040338[71] = 0;
   out_529478043801040338[72] = 0;
   out_529478043801040338[73] = 0;
   out_529478043801040338[74] = 0;
   out_529478043801040338[75] = 0;
   out_529478043801040338[76] = 0;
   out_529478043801040338[77] = 0;
   out_529478043801040338[78] = 0;
   out_529478043801040338[79] = 0;
   out_529478043801040338[80] = 1;
}
void h_25(double *state, double *unused, double *out_2747349570204871243) {
   out_2747349570204871243[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1087706381204727692) {
   out_1087706381204727692[0] = 0;
   out_1087706381204727692[1] = 0;
   out_1087706381204727692[2] = 0;
   out_1087706381204727692[3] = 0;
   out_1087706381204727692[4] = 0;
   out_1087706381204727692[5] = 0;
   out_1087706381204727692[6] = 1;
   out_1087706381204727692[7] = 0;
   out_1087706381204727692[8] = 0;
}
void h_24(double *state, double *unused, double *out_7901114660987291782) {
   out_7901114660987291782[0] = state[4];
   out_7901114660987291782[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2470219217597595444) {
   out_2470219217597595444[0] = 0;
   out_2470219217597595444[1] = 0;
   out_2470219217597595444[2] = 0;
   out_2470219217597595444[3] = 0;
   out_2470219217597595444[4] = 1;
   out_2470219217597595444[5] = 0;
   out_2470219217597595444[6] = 0;
   out_2470219217597595444[7] = 0;
   out_2470219217597595444[8] = 0;
   out_2470219217597595444[9] = 0;
   out_2470219217597595444[10] = 0;
   out_2470219217597595444[11] = 0;
   out_2470219217597595444[12] = 0;
   out_2470219217597595444[13] = 0;
   out_2470219217597595444[14] = 1;
   out_2470219217597595444[15] = 0;
   out_2470219217597595444[16] = 0;
   out_2470219217597595444[17] = 0;
}
void h_30(double *state, double *unused, double *out_890099958049151107) {
   out_890099958049151107[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1430626577302520935) {
   out_1430626577302520935[0] = 0;
   out_1430626577302520935[1] = 0;
   out_1430626577302520935[2] = 0;
   out_1430626577302520935[3] = 0;
   out_1430626577302520935[4] = 1;
   out_1430626577302520935[5] = 0;
   out_1430626577302520935[6] = 0;
   out_1430626577302520935[7] = 0;
   out_1430626577302520935[8] = 0;
}
void h_26(double *state, double *unused, double *out_4530607195872700425) {
   out_4530607195872700425[0] = state[7];
}
void H_26(double *state, double *unused, double *out_4829209700078783916) {
   out_4829209700078783916[0] = 0;
   out_4829209700078783916[1] = 0;
   out_4829209700078783916[2] = 0;
   out_4829209700078783916[3] = 0;
   out_4829209700078783916[4] = 0;
   out_4829209700078783916[5] = 0;
   out_4829209700078783916[6] = 0;
   out_4829209700078783916[7] = 1;
   out_4829209700078783916[8] = 0;
}
void h_27(double *state, double *unused, double *out_3690859423106533678) {
   out_3690859423106533678[0] = state[3];
}
void H_27(double *state, double *unused, double *out_744136734497903976) {
   out_744136734497903976[0] = 0;
   out_744136734497903976[1] = 0;
   out_744136734497903976[2] = 0;
   out_744136734497903976[3] = 1;
   out_744136734497903976[4] = 0;
   out_744136734497903976[5] = 0;
   out_744136734497903976[6] = 0;
   out_744136734497903976[7] = 0;
   out_744136734497903976[8] = 0;
}
void h_29(double *state, double *unused, double *out_53409894852511328) {
   out_53409894852511328[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1940857921616913119) {
   out_1940857921616913119[0] = 0;
   out_1940857921616913119[1] = 1;
   out_1940857921616913119[2] = 0;
   out_1940857921616913119[3] = 0;
   out_1940857921616913119[4] = 0;
   out_1940857921616913119[5] = 0;
   out_1940857921616913119[6] = 0;
   out_1940857921616913119[7] = 0;
   out_1940857921616913119[8] = 0;
}
void h_28(double *state, double *unused, double *out_6727645474550866363) {
   out_6727645474550866363[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3141541095452617455) {
   out_3141541095452617455[0] = 1;
   out_3141541095452617455[1] = 0;
   out_3141541095452617455[2] = 0;
   out_3141541095452617455[3] = 0;
   out_3141541095452617455[4] = 0;
   out_3141541095452617455[5] = 0;
   out_3141541095452617455[6] = 0;
   out_3141541095452617455[7] = 0;
   out_3141541095452617455[8] = 0;
}
void h_31(double *state, double *unused, double *out_2145931412591507689) {
   out_2145931412591507689[0] = state[8];
}
void H_31(double *state, double *unused, double *out_5455417802312135392) {
   out_5455417802312135392[0] = 0;
   out_5455417802312135392[1] = 0;
   out_5455417802312135392[2] = 0;
   out_5455417802312135392[3] = 0;
   out_5455417802312135392[4] = 0;
   out_5455417802312135392[5] = 0;
   out_5455417802312135392[6] = 0;
   out_5455417802312135392[7] = 0;
   out_5455417802312135392[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4239990911220653914) {
  err_fun(nom_x, delta_x, out_4239990911220653914);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6013891986074755824) {
  inv_err_fun(nom_x, true_x, out_6013891986074755824);
}
void car_H_mod_fun(double *state, double *out_4325368645737364917) {
  H_mod_fun(state, out_4325368645737364917);
}
void car_f_fun(double *state, double dt, double *out_1889004700640037715) {
  f_fun(state,  dt, out_1889004700640037715);
}
void car_F_fun(double *state, double dt, double *out_529478043801040338) {
  F_fun(state,  dt, out_529478043801040338);
}
void car_h_25(double *state, double *unused, double *out_2747349570204871243) {
  h_25(state, unused, out_2747349570204871243);
}
void car_H_25(double *state, double *unused, double *out_1087706381204727692) {
  H_25(state, unused, out_1087706381204727692);
}
void car_h_24(double *state, double *unused, double *out_7901114660987291782) {
  h_24(state, unused, out_7901114660987291782);
}
void car_H_24(double *state, double *unused, double *out_2470219217597595444) {
  H_24(state, unused, out_2470219217597595444);
}
void car_h_30(double *state, double *unused, double *out_890099958049151107) {
  h_30(state, unused, out_890099958049151107);
}
void car_H_30(double *state, double *unused, double *out_1430626577302520935) {
  H_30(state, unused, out_1430626577302520935);
}
void car_h_26(double *state, double *unused, double *out_4530607195872700425) {
  h_26(state, unused, out_4530607195872700425);
}
void car_H_26(double *state, double *unused, double *out_4829209700078783916) {
  H_26(state, unused, out_4829209700078783916);
}
void car_h_27(double *state, double *unused, double *out_3690859423106533678) {
  h_27(state, unused, out_3690859423106533678);
}
void car_H_27(double *state, double *unused, double *out_744136734497903976) {
  H_27(state, unused, out_744136734497903976);
}
void car_h_29(double *state, double *unused, double *out_53409894852511328) {
  h_29(state, unused, out_53409894852511328);
}
void car_H_29(double *state, double *unused, double *out_1940857921616913119) {
  H_29(state, unused, out_1940857921616913119);
}
void car_h_28(double *state, double *unused, double *out_6727645474550866363) {
  h_28(state, unused, out_6727645474550866363);
}
void car_H_28(double *state, double *unused, double *out_3141541095452617455) {
  H_28(state, unused, out_3141541095452617455);
}
void car_h_31(double *state, double *unused, double *out_2145931412591507689) {
  h_31(state, unused, out_2145931412591507689);
}
void car_H_31(double *state, double *unused, double *out_5455417802312135392) {
  H_31(state, unused, out_5455417802312135392);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
