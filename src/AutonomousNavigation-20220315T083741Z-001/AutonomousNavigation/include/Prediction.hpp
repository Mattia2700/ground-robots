#ifndef PREDICTION_H_
#define PREDICTION_H_
#include <cmath>
#include <vector>
const double dt    = 0.1;
const int observe  = 10;
const int predict  = 30;
const double scale = 100;
const double mass  = 100;


std::vector<std::vector<double>> prediction(std::vector<std::vector<double>> Xin);

// TODO: import automatico dei pesi della rete

static double w1[1][10] = {{1.,-0.6069159,-0.25468042,-0.0277511,0.09962494,0.14905713,0.13802204,0.08058129,-0.01200234,-0.13073254}};
static double b1[1] = {0};
static double w2[1][10] = {{1.,-0.11995354,-0.02714603,0.03142041,0.05727162,0.05864693,0.04254455,0.01426639,-0.02229762,-0.06431456}};
static double b2[1] = {0};
static double w3[10][9] = {{-0.6686067,0.98500955,1.3999611,1.2741863,1.1049917,0.10878889,-0.93157476,-1.5968356,-3.592612},{2.196765,-0.6117248,-1.9365634,-1.9895034,-2.1883636,-1.4219325,0.0325875,1.6893294,2.0400486},{-2.746329,-0.07894088,1.0663413,1.3902779,1.6826941,0.20317505,-0.21933703,-1.0993235,-2.427395},{-0.38411373,1.3354108,1.1789259,1.1501845,0.5254885,0.03687591,-1.3531108,-2.5649061,-3.3674388},{1.8602952,-0.38179415,-1.7500296,-1.8499794,-1.5144608,-1.6017095,-0.37293142,0.8324041,1.9213666},{-1.7251208,0.46941456,1.5509784,1.8010174,0.7561897,0.0394984,-0.25157464,-1.7608807,-2.539563},{2.0004194,-1.0874634,-1.6377838,-2.2694263,-1.5910076,-0.7694005,-0.4079282,1.407109,3.026983},{1.2493962,-0.87311804,-2.2642918,-2.1112173,-1.8381768,-0.31449404,0.5531175,1.7333626,2.5771346},{-1.6882542,0.75458425,1.6909016,1.6578237,1.6157342,0.5511607,-0.58994585,-1.8324205,-3.1175265},{2.5760264,0.02651882,-1.9375126,-2.3848329,-1.5020862,-1.2638243,-0.42626026,1.2870508,2.2970245}};
static double b3[10] = {0.,0.,0.,0.,0.,0.,0.,0.,0.,0.};
static double w4[1][1] = {0.00477548};
static double b4[1] = {0};
static double w5[1][1] = {0.00381281};
static double b5[1] = {0};
static double w6[1][10] = {{-0.4934683,1.1655905,-1.4615451,-0.16931161,0.60575885,-1.0813035,0.75459594,0.6498389,-1.2306241,1.2560695}};
static double b6[1] = {0};
static double w7[1][1] = {1.8216248};
static double b7[1] = {0};

// Keras core layers

template <int Nin, int Nout>
void linear(double in[Nin], double w[Nout][Nin], double b[Nout], double out[Nout]) {
    for(int j=0 ; j<Nout; j++) {
        out[j] = 0;
        for(int i=0; i<Nin; i++) {
            out[j] += in[i]*w[j][i];
        }
        out[j] += b[j];
    }
}

// Keras activation layers

template <int Nin>
void relu(double in[Nin], double out[Nin]) {
    for(int i=0; i<Nin ; i++) {
        out[i] = in[i] > 0 ? in[i] : 0;
    }
}

template <int Nin>
void sigmoid(double in[Nin], double out[Nin]) {
    for(int i=0; i<Nin ; i++) {
        out[i] = 1 / (1 + std::exp(-in[i]) );
    }
}

template <int Nin>
void tanh(double in[Nin], double out[Nin]) {
    for(int i=0; i<Nin ; i++) {
        out[i] = std::tanh(in[i]);
    }
}

// Network SFM forces 1

template <int window>
void netSfmForces1(double input_x[window], double input_y[window], double & out_f0x, double & out_f0y) {
    double dense_x[1];
    double dense_y[1];
    double est_vx[1];
    double est_vy[1];
    double mod_vd[window-1];
    double dense_vd[10];
    double est_vd[1];

    linear<window,1>(input_x, w1, b1, dense_x);
    tanh<1>(input_x, input_x);
    linear<1,1>(dense_x, w4, b4, est_vx);

    linear<window,1>(input_y, w2, b2, dense_y);
    tanh<1>(dense_y, dense_y);
    linear<1,1>(dense_y, w5, b5, est_vy);

    for (int i=0; i<window-1; i++) {
        mod_vd[i] = std::sqrt( std::pow(input_x[i+1]-input_x[i], 2) + std::pow(input_y[i+1]-input_y[i], 2) );
    }
    linear<window-1,10>(mod_vd, w3, b3, dense_vd);
    sigmoid<10>(dense_vd, dense_vd);
    linear<10,1>(dense_vd, w6, b6, est_vd);

    double alpha = std::atan2(input_y[window-1] - input_y[window-2], input_x[window-1] - input_x[window-2]);
    double est_ex = std::cos(alpha);
    double est_ey = std::sin(alpha);

    out_f0x = w7[0][0] * (est_vd[0] * est_ex - est_vx[0]);
    out_f0y = w7[0][0] * (est_vd[0] * est_ey - est_vy[0]);
}

#endif //PREDICTION

