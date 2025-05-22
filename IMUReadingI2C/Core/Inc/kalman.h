

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float angle;
    float bias;
    float rate;

    float P[2][2];
} Kalman_t;

void Kalman_Init(Kalman_t *kf);
float Kalman_getAngle(Kalman_t *kf, float newAngle, float newRate, float dt);

#endif
