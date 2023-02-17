#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_6406290624312280998) {
   out_6406290624312280998[0] = delta_x[0] + nom_x[0];
   out_6406290624312280998[1] = delta_x[1] + nom_x[1];
   out_6406290624312280998[2] = delta_x[2] + nom_x[2];
   out_6406290624312280998[3] = delta_x[3] + nom_x[3];
   out_6406290624312280998[4] = delta_x[4] + nom_x[4];
   out_6406290624312280998[5] = delta_x[5] + nom_x[5];
   out_6406290624312280998[6] = delta_x[6] + nom_x[6];
   out_6406290624312280998[7] = delta_x[7] + nom_x[7];
   out_6406290624312280998[8] = delta_x[8] + nom_x[8];
   out_6406290624312280998[9] = delta_x[9] + nom_x[9];
   out_6406290624312280998[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6610292794556148846) {
   out_6610292794556148846[0] = -nom_x[0] + true_x[0];
   out_6610292794556148846[1] = -nom_x[1] + true_x[1];
   out_6610292794556148846[2] = -nom_x[2] + true_x[2];
   out_6610292794556148846[3] = -nom_x[3] + true_x[3];
   out_6610292794556148846[4] = -nom_x[4] + true_x[4];
   out_6610292794556148846[5] = -nom_x[5] + true_x[5];
   out_6610292794556148846[6] = -nom_x[6] + true_x[6];
   out_6610292794556148846[7] = -nom_x[7] + true_x[7];
   out_6610292794556148846[8] = -nom_x[8] + true_x[8];
   out_6610292794556148846[9] = -nom_x[9] + true_x[9];
   out_6610292794556148846[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4190022832125984075) {
   out_4190022832125984075[0] = 1.0;
   out_4190022832125984075[1] = 0;
   out_4190022832125984075[2] = 0;
   out_4190022832125984075[3] = 0;
   out_4190022832125984075[4] = 0;
   out_4190022832125984075[5] = 0;
   out_4190022832125984075[6] = 0;
   out_4190022832125984075[7] = 0;
   out_4190022832125984075[8] = 0;
   out_4190022832125984075[9] = 0;
   out_4190022832125984075[10] = 0;
   out_4190022832125984075[11] = 0;
   out_4190022832125984075[12] = 1.0;
   out_4190022832125984075[13] = 0;
   out_4190022832125984075[14] = 0;
   out_4190022832125984075[15] = 0;
   out_4190022832125984075[16] = 0;
   out_4190022832125984075[17] = 0;
   out_4190022832125984075[18] = 0;
   out_4190022832125984075[19] = 0;
   out_4190022832125984075[20] = 0;
   out_4190022832125984075[21] = 0;
   out_4190022832125984075[22] = 0;
   out_4190022832125984075[23] = 0;
   out_4190022832125984075[24] = 1.0;
   out_4190022832125984075[25] = 0;
   out_4190022832125984075[26] = 0;
   out_4190022832125984075[27] = 0;
   out_4190022832125984075[28] = 0;
   out_4190022832125984075[29] = 0;
   out_4190022832125984075[30] = 0;
   out_4190022832125984075[31] = 0;
   out_4190022832125984075[32] = 0;
   out_4190022832125984075[33] = 0;
   out_4190022832125984075[34] = 0;
   out_4190022832125984075[35] = 0;
   out_4190022832125984075[36] = 1.0;
   out_4190022832125984075[37] = 0;
   out_4190022832125984075[38] = 0;
   out_4190022832125984075[39] = 0;
   out_4190022832125984075[40] = 0;
   out_4190022832125984075[41] = 0;
   out_4190022832125984075[42] = 0;
   out_4190022832125984075[43] = 0;
   out_4190022832125984075[44] = 0;
   out_4190022832125984075[45] = 0;
   out_4190022832125984075[46] = 0;
   out_4190022832125984075[47] = 0;
   out_4190022832125984075[48] = 1.0;
   out_4190022832125984075[49] = 0;
   out_4190022832125984075[50] = 0;
   out_4190022832125984075[51] = 0;
   out_4190022832125984075[52] = 0;
   out_4190022832125984075[53] = 0;
   out_4190022832125984075[54] = 0;
   out_4190022832125984075[55] = 0;
   out_4190022832125984075[56] = 0;
   out_4190022832125984075[57] = 0;
   out_4190022832125984075[58] = 0;
   out_4190022832125984075[59] = 0;
   out_4190022832125984075[60] = 1.0;
   out_4190022832125984075[61] = 0;
   out_4190022832125984075[62] = 0;
   out_4190022832125984075[63] = 0;
   out_4190022832125984075[64] = 0;
   out_4190022832125984075[65] = 0;
   out_4190022832125984075[66] = 0;
   out_4190022832125984075[67] = 0;
   out_4190022832125984075[68] = 0;
   out_4190022832125984075[69] = 0;
   out_4190022832125984075[70] = 0;
   out_4190022832125984075[71] = 0;
   out_4190022832125984075[72] = 1.0;
   out_4190022832125984075[73] = 0;
   out_4190022832125984075[74] = 0;
   out_4190022832125984075[75] = 0;
   out_4190022832125984075[76] = 0;
   out_4190022832125984075[77] = 0;
   out_4190022832125984075[78] = 0;
   out_4190022832125984075[79] = 0;
   out_4190022832125984075[80] = 0;
   out_4190022832125984075[81] = 0;
   out_4190022832125984075[82] = 0;
   out_4190022832125984075[83] = 0;
   out_4190022832125984075[84] = 1.0;
   out_4190022832125984075[85] = 0;
   out_4190022832125984075[86] = 0;
   out_4190022832125984075[87] = 0;
   out_4190022832125984075[88] = 0;
   out_4190022832125984075[89] = 0;
   out_4190022832125984075[90] = 0;
   out_4190022832125984075[91] = 0;
   out_4190022832125984075[92] = 0;
   out_4190022832125984075[93] = 0;
   out_4190022832125984075[94] = 0;
   out_4190022832125984075[95] = 0;
   out_4190022832125984075[96] = 1.0;
   out_4190022832125984075[97] = 0;
   out_4190022832125984075[98] = 0;
   out_4190022832125984075[99] = 0;
   out_4190022832125984075[100] = 0;
   out_4190022832125984075[101] = 0;
   out_4190022832125984075[102] = 0;
   out_4190022832125984075[103] = 0;
   out_4190022832125984075[104] = 0;
   out_4190022832125984075[105] = 0;
   out_4190022832125984075[106] = 0;
   out_4190022832125984075[107] = 0;
   out_4190022832125984075[108] = 1.0;
   out_4190022832125984075[109] = 0;
   out_4190022832125984075[110] = 0;
   out_4190022832125984075[111] = 0;
   out_4190022832125984075[112] = 0;
   out_4190022832125984075[113] = 0;
   out_4190022832125984075[114] = 0;
   out_4190022832125984075[115] = 0;
   out_4190022832125984075[116] = 0;
   out_4190022832125984075[117] = 0;
   out_4190022832125984075[118] = 0;
   out_4190022832125984075[119] = 0;
   out_4190022832125984075[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_5485440970639894550) {
   out_5485440970639894550[0] = dt*state[3] + state[0];
   out_5485440970639894550[1] = dt*state[4] + state[1];
   out_5485440970639894550[2] = dt*state[5] + state[2];
   out_5485440970639894550[3] = state[3];
   out_5485440970639894550[4] = state[4];
   out_5485440970639894550[5] = state[5];
   out_5485440970639894550[6] = dt*state[7] + state[6];
   out_5485440970639894550[7] = dt*state[8] + state[7];
   out_5485440970639894550[8] = state[8];
   out_5485440970639894550[9] = state[9];
   out_5485440970639894550[10] = state[10];
}
void F_fun(double *state, double dt, double *out_8287619921053647117) {
   out_8287619921053647117[0] = 1;
   out_8287619921053647117[1] = 0;
   out_8287619921053647117[2] = 0;
   out_8287619921053647117[3] = dt;
   out_8287619921053647117[4] = 0;
   out_8287619921053647117[5] = 0;
   out_8287619921053647117[6] = 0;
   out_8287619921053647117[7] = 0;
   out_8287619921053647117[8] = 0;
   out_8287619921053647117[9] = 0;
   out_8287619921053647117[10] = 0;
   out_8287619921053647117[11] = 0;
   out_8287619921053647117[12] = 1;
   out_8287619921053647117[13] = 0;
   out_8287619921053647117[14] = 0;
   out_8287619921053647117[15] = dt;
   out_8287619921053647117[16] = 0;
   out_8287619921053647117[17] = 0;
   out_8287619921053647117[18] = 0;
   out_8287619921053647117[19] = 0;
   out_8287619921053647117[20] = 0;
   out_8287619921053647117[21] = 0;
   out_8287619921053647117[22] = 0;
   out_8287619921053647117[23] = 0;
   out_8287619921053647117[24] = 1;
   out_8287619921053647117[25] = 0;
   out_8287619921053647117[26] = 0;
   out_8287619921053647117[27] = dt;
   out_8287619921053647117[28] = 0;
   out_8287619921053647117[29] = 0;
   out_8287619921053647117[30] = 0;
   out_8287619921053647117[31] = 0;
   out_8287619921053647117[32] = 0;
   out_8287619921053647117[33] = 0;
   out_8287619921053647117[34] = 0;
   out_8287619921053647117[35] = 0;
   out_8287619921053647117[36] = 1;
   out_8287619921053647117[37] = 0;
   out_8287619921053647117[38] = 0;
   out_8287619921053647117[39] = 0;
   out_8287619921053647117[40] = 0;
   out_8287619921053647117[41] = 0;
   out_8287619921053647117[42] = 0;
   out_8287619921053647117[43] = 0;
   out_8287619921053647117[44] = 0;
   out_8287619921053647117[45] = 0;
   out_8287619921053647117[46] = 0;
   out_8287619921053647117[47] = 0;
   out_8287619921053647117[48] = 1;
   out_8287619921053647117[49] = 0;
   out_8287619921053647117[50] = 0;
   out_8287619921053647117[51] = 0;
   out_8287619921053647117[52] = 0;
   out_8287619921053647117[53] = 0;
   out_8287619921053647117[54] = 0;
   out_8287619921053647117[55] = 0;
   out_8287619921053647117[56] = 0;
   out_8287619921053647117[57] = 0;
   out_8287619921053647117[58] = 0;
   out_8287619921053647117[59] = 0;
   out_8287619921053647117[60] = 1;
   out_8287619921053647117[61] = 0;
   out_8287619921053647117[62] = 0;
   out_8287619921053647117[63] = 0;
   out_8287619921053647117[64] = 0;
   out_8287619921053647117[65] = 0;
   out_8287619921053647117[66] = 0;
   out_8287619921053647117[67] = 0;
   out_8287619921053647117[68] = 0;
   out_8287619921053647117[69] = 0;
   out_8287619921053647117[70] = 0;
   out_8287619921053647117[71] = 0;
   out_8287619921053647117[72] = 1;
   out_8287619921053647117[73] = dt;
   out_8287619921053647117[74] = 0;
   out_8287619921053647117[75] = 0;
   out_8287619921053647117[76] = 0;
   out_8287619921053647117[77] = 0;
   out_8287619921053647117[78] = 0;
   out_8287619921053647117[79] = 0;
   out_8287619921053647117[80] = 0;
   out_8287619921053647117[81] = 0;
   out_8287619921053647117[82] = 0;
   out_8287619921053647117[83] = 0;
   out_8287619921053647117[84] = 1;
   out_8287619921053647117[85] = dt;
   out_8287619921053647117[86] = 0;
   out_8287619921053647117[87] = 0;
   out_8287619921053647117[88] = 0;
   out_8287619921053647117[89] = 0;
   out_8287619921053647117[90] = 0;
   out_8287619921053647117[91] = 0;
   out_8287619921053647117[92] = 0;
   out_8287619921053647117[93] = 0;
   out_8287619921053647117[94] = 0;
   out_8287619921053647117[95] = 0;
   out_8287619921053647117[96] = 1;
   out_8287619921053647117[97] = 0;
   out_8287619921053647117[98] = 0;
   out_8287619921053647117[99] = 0;
   out_8287619921053647117[100] = 0;
   out_8287619921053647117[101] = 0;
   out_8287619921053647117[102] = 0;
   out_8287619921053647117[103] = 0;
   out_8287619921053647117[104] = 0;
   out_8287619921053647117[105] = 0;
   out_8287619921053647117[106] = 0;
   out_8287619921053647117[107] = 0;
   out_8287619921053647117[108] = 1;
   out_8287619921053647117[109] = 0;
   out_8287619921053647117[110] = 0;
   out_8287619921053647117[111] = 0;
   out_8287619921053647117[112] = 0;
   out_8287619921053647117[113] = 0;
   out_8287619921053647117[114] = 0;
   out_8287619921053647117[115] = 0;
   out_8287619921053647117[116] = 0;
   out_8287619921053647117[117] = 0;
   out_8287619921053647117[118] = 0;
   out_8287619921053647117[119] = 0;
   out_8287619921053647117[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_6986316530236158341) {
   out_6986316530236158341[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_5536356530439876366) {
   out_5536356530439876366[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5536356530439876366[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5536356530439876366[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5536356530439876366[3] = 0;
   out_5536356530439876366[4] = 0;
   out_5536356530439876366[5] = 0;
   out_5536356530439876366[6] = 1;
   out_5536356530439876366[7] = 0;
   out_5536356530439876366[8] = 0;
   out_5536356530439876366[9] = 0;
   out_5536356530439876366[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_871747740134602654) {
   out_871747740134602654[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_6887040764717437270) {
   out_6887040764717437270[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6887040764717437270[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6887040764717437270[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6887040764717437270[3] = 0;
   out_6887040764717437270[4] = 0;
   out_6887040764717437270[5] = 0;
   out_6887040764717437270[6] = 1;
   out_6887040764717437270[7] = 0;
   out_6887040764717437270[8] = 0;
   out_6887040764717437270[9] = 1;
   out_6887040764717437270[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_6234951525856667033) {
   out_6234951525856667033[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_3527993678913903080) {
   out_3527993678913903080[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[6] = 0;
   out_3527993678913903080[7] = 1;
   out_3527993678913903080[8] = 0;
   out_3527993678913903080[9] = 0;
   out_3527993678913903080[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_6234951525856667033) {
   out_6234951525856667033[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_3527993678913903080) {
   out_3527993678913903080[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_3527993678913903080[6] = 0;
   out_3527993678913903080[7] = 1;
   out_3527993678913903080[8] = 0;
   out_3527993678913903080[9] = 0;
   out_3527993678913903080[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_6406290624312280998) {
  err_fun(nom_x, delta_x, out_6406290624312280998);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_6610292794556148846) {
  inv_err_fun(nom_x, true_x, out_6610292794556148846);
}
void gnss_H_mod_fun(double *state, double *out_4190022832125984075) {
  H_mod_fun(state, out_4190022832125984075);
}
void gnss_f_fun(double *state, double dt, double *out_5485440970639894550) {
  f_fun(state,  dt, out_5485440970639894550);
}
void gnss_F_fun(double *state, double dt, double *out_8287619921053647117) {
  F_fun(state,  dt, out_8287619921053647117);
}
void gnss_h_6(double *state, double *sat_pos, double *out_6986316530236158341) {
  h_6(state, sat_pos, out_6986316530236158341);
}
void gnss_H_6(double *state, double *sat_pos, double *out_5536356530439876366) {
  H_6(state, sat_pos, out_5536356530439876366);
}
void gnss_h_20(double *state, double *sat_pos, double *out_871747740134602654) {
  h_20(state, sat_pos, out_871747740134602654);
}
void gnss_H_20(double *state, double *sat_pos, double *out_6887040764717437270) {
  H_20(state, sat_pos, out_6887040764717437270);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_6234951525856667033) {
  h_7(state, sat_pos_vel, out_6234951525856667033);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_3527993678913903080) {
  H_7(state, sat_pos_vel, out_3527993678913903080);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_6234951525856667033) {
  h_21(state, sat_pos_vel, out_6234951525856667033);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_3527993678913903080) {
  H_21(state, sat_pos_vel, out_3527993678913903080);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
