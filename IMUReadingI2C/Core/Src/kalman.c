#include "kalman.h"

#define Q_angle 0.001f
#define Q_bias 0.003f
#define R_measure 0.03f

void Kalman_Init(Kalman_t *kf) {
    kf->angle = 0.0f;
    kf->bias = 0.0f;

    kf->P[0][0] = 0;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 0;
}

float Kalman_getAngle(Kalman_t *kf, float newAngle, float newRate, float dt) {
    // Predict
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    kf->P[0][0] += dt * (dt*kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += Q_bias * dt;

    // Update
    float S = kf->P[0][0] + R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    float y = newAngle - kf->angle;

    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}
