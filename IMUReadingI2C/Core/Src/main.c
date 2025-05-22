
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "kalman.h"

Kalman_t kalmanRoll, kalmanPitch;

void configureIO();
void enableClk();
void delay(uint16_t t);
void turnON(uint8_t i);
void turnOFF(uint8_t i);
void setRotationDir(uint8_t i);
void pressBreak();
void freeMotor();
uint16_t getADCVal(uint8_t i);
void initPWM();
void writePWM (float dutyCycle);
void initI2C();
void initIMU();
void controlMotor(float data);
void readIMUData(float* ax, float* ay, float* az,
                 float* gx, float* gy, float* gz);
void setAngles(float ax, float ay, float az,
               float gx, float gy, float gz,
               float* roll, float* pitch);




#define STABILITY_TOLERANCE 10
#define RAD_TO_DEG 57.2957795f
#define MPU6050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define DT 0.01f  // 10ms loop time = 100Hz



int main(void){
	enableClk();
	configureIO();


	initPWM();
	initI2C();

	float roll = 0;   // X
	float pitch = 0;  // Y
	float yaw = 0;    // Z

	float ax, ay, az;      // Accelerometer (g)
	float gx, gy, gz;      // Gyroscope (°/s)



while (1) {

	freeMotor();
	setAngles(ax,  ay,  az,
              gx,  gy,  gz,
              &roll, &pitch);

	readIMUData(&ax, &ay, &az, &gx, &gy, &gz);

	setAngles(ax, ay, az, gx, gy, gz, &roll, &pitch);

	yaw += gz * DT;

	if (yaw >= 360.0f) yaw -= 360.0f;
	if (yaw < 0.0f) yaw += 360.0f;
	if (roll >= 360.0f) roll -= 360.0f;
	if (roll < 0.0f) roll += 360.0f;
	if (pitch >= 360.0f) pitch -= 360.0f;
	if (pitch < 0.0f) pitch += 360.0f;

	if (fabs(roll) > STABILITY_TOLERANCE){
		controlMotor(roll);
	}
	if (fabs(pitch) > STABILITY_TOLERANCE){
		controlMotor(pitch);
	}
	if (fabs(yaw) > STABILITY_TOLERANCE){
		controlMotor(yaw);
	}

	delay(10);


}


}


void configureIO(){

	/*
	 *
	 * LEDS => B13 output "2"
	 * PWM => A1 output AF "A"
	 * HBridge IN3 => B7
	 * HBridge IN4 => B8
	 * HBridge ENB => B9
	 * SDA -> B11
	 * SCL -> B10
	 *
	 * */

	GPIOA -> CRL = 0x444444A4;
	GPIOB -> CRL = 0x24444444;
	GPIOB -> CRH = 0x4424AA44;


}

void enableClk(){
	RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC -> APB1ENR |= 0b11; //enable TIM2 & TIM3
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC -> APB2ENR |= (1 << 9) | (1 << 10); // enable CLK for ADC1
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; //enable I2C2 CLK
}


void turnON(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR |= (1 << 7);  return; // Blue indicates reached goal
		case 1: GPIOB ->ODR |= (1 << 10); return; // Red indicates still processing
		case 2: GPIOB ->ODR |= (1 << 13); return; // Green indicates CAL for first time

		default: break;
	}
}

void turnOFF(uint8_t i){


	switch (i){

		case 0: GPIOA ->ODR &= ~(1 << 7);  return;
		case 1: GPIOB ->ODR &= ~(1 << 10); return;
		case 2: GPIOB ->ODR &= ~(1 << 13); return;

		default: break;
	}
}



void delay(uint16_t t){
	TIM3 ->PSC = 8000-1;
	TIM3 ->ARR = t-1;
	TIM3 ->CR1 |= TIM_CR1_CEN;
	while(!(TIM3->SR & TIM_SR_UIF));
	TIM3->SR &= ~TIM_SR_UIF;
	TIM3 ->CR1 &= ~(TIM_CR1_CEN);
}


void setRotationDir(uint8_t i){
	switch(i){
		case 0 : GPIOB -> ODR |= (1 << 7) ;break; // Counter CLockwise
		case 1 : GPIOB -> ODR |= (1 << 8) ;break; // CLockwise
		default: break;
	}
}


void pressBreak(){
	GPIOB->ODR |= (1 << 7) | (1 << 8);

}


void freeMotor(){
	GPIOB->ODR &= ~((1 << 7) | (1 << 8));
}

void initPWM(){
	RCC ->APB1ENR |= RCC_APB1ENR_TIM2EN; //enable Timer

	TIM2 -> PSC = 8 - 1;
	TIM2 -> ARR = 1000 - 1;

	TIM2->CCMR1 |= (6 << 12); // mode 1 for ch2

	TIM2 -> CCR2 = 0; //duty Cycle =0

	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; // Enable preload

	TIM2->CCER |= TIM_CCER_CC2E;    // Enable output for CH1

	TIM2->CR1 |= TIM_CR1_ARPE;   // Enable auto-reload Preload

	TIM2->EGR |= TIM_EGR_UG;     // Force update to load registers

	TIM2->CR1 |= TIM_CR1_CEN;    // Enable timer

}

void writePWM (float dutyCycle){
	if (dutyCycle > 100){
		dutyCycle = 100;
	}

	TIM2 -> CCR2 = (TIM2->ARR + 1) * dutyCycle / 100;

}

void initI2C(){

	//enabled RCC
	//Configured IO as AF OD

	I2C2-> CR1 &= ~I2C_CR1_PE; // Disable I2C1 before configuring

	// Reset I2C1 peripheral
	I2C2-> CR1 |= I2C_CR1_SWRST;
	I2C2-> CR1 &= ~I2C_CR1_SWRST;

	I2C2 -> CR2 = 8 ; // set Freq.

	I2C2-> CCR &= ~I2C_CCR_FS;  // Standard mode

	//I2C1->CCR &= ~I2C_CCR_DUTY; setting duty here is useless

	I2C2 -> CCR = 40;  // F/(2*speed) -> 8M / ( 2*100K )

	I2C2-> TRISE = 8 + 1 ;  // TRISE = Fpclk1(MHz) + 1 => 8 + 1

	I2C2-> CR1 |= I2C_CR1_PE; //Enable I2C1


}

void controlMotor(float angle) {
    if (fabsf(angle) < STABILITY_TOLERANCE) {
        pressBreak();
        return;
    }

    //uint8_t dir = (angle > 0) ? 0 : 1;
    //setRotationDir(dir);

    float dutyCycle = fabsf(angle) / 90.0f * 100.0f;
    if (dutyCycle > 100.0f) dutyCycle = 100.0f;

    writePWM(dutyCycle);
}


void initIMU() {
    // Wake up the MPU6050
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    I2C2->DR = MPU6050_ADDR << 1; // write
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR1;
    (void)I2C2->SR2;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = PWR_MGMT_1;
    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = 0x00;
    while (!(I2C2->SR1 & I2C_SR1_BTF));
    I2C2->CR1 |= I2C_CR1_STOP;
}

void readIMUData(float* ax, float* ay, float* az,
                 float* gx, float* gy, float* gz) {
    uint8_t rawData[14];

    // Start communication
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    I2C2->DR = MPU6050_ADDR << 1; // Write
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR1; (void)I2C2->SR2;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = ACCEL_XOUT_H;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->CR1 |= I2C_CR1_START; // Repeated start
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    I2C2->DR = (MPU6050_ADDR << 1) | 0x01; // Read
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR1; (void)I2C2->SR2;

    for (int i = 0; i < 13; i++) {
        while (!(I2C2->SR1 & I2C_SR1_RXNE));
        rawData[i] = I2C2->DR;
    }

    I2C2->CR1 &= ~I2C_CR1_ACK; // NACK
    I2C2->CR1 |= I2C_CR1_STOP;
    while (!(I2C2->SR1 & I2C_SR1_RXNE));
    rawData[13] = I2C2->DR;

    int16_t ax_raw = (rawData[0] << 8) | rawData[1];
    int16_t ay_raw = (rawData[2] << 8) | rawData[3];
    int16_t az_raw = (rawData[4] << 8) | rawData[5];
    int16_t gx_raw = (rawData[8] << 8) | rawData[9];
    int16_t gy_raw = (rawData[10] << 8) | rawData[11];
    int16_t gz_raw = (rawData[12] << 8) | rawData[13];

    // Convert raw to physical units (example: depends on sensitivity settings)
    *ax = ax_raw / 16384.0f;  // Assuming ±2g
    *ay = ay_raw / 16384.0f;
    *az = az_raw / 16384.0f;
    *gx = gx_raw / 131.0f;    // Assuming ±250°/s
    *gy = gy_raw / 131.0f;
    *gz = gz_raw / 131.0f;
}


void setAngles(float ax, float ay, float az,
               float gx, float gy, float gz,
               float* roll, float* pitch) {

    float accel_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float accel_pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;

    static uint32_t lastTime = 0;
    uint32_t now = SysTick->VAL;
    float dt = (now - lastTime) / 8000000.0f;
    lastTime = now;

    *roll = Kalman_getAngle(&kalmanRoll, accel_roll, gx, dt);
    *pitch = Kalman_getAngle(&kalmanPitch, accel_pitch, gy, dt);
}

