
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
uint16_t getcurrentPosition();
void initPWM();
void initADC2();
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
#define MPU9050_ADDR 0x68
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define GYRO_XOUT_H  0x43
#define DT 0.01f  // 10ms loop time = 100Hz

#define LOWER_LIMIT 1050
#define UPPER_LIMIT 3400
#define RESET_THRESHOLD 250
#define TOLERANCE 35


float Kp;
float Ki;
float Kd;
int errDerivative = 0 ;
int errIntegral = 0 ;
int prevErr = 0 ;
int err =0;
void PIDController();
int ref = 0 ;
int prevRef = 0 ;
int curr = 0;
int u;


float roll = 0;   // X
float pitch = 0;  // Y
float yaw = 0;    // Z

float ax, ay, az;      // Accelerometer (g)
float gx, gy, gz;      // Gyroscope (°/s)




int main(void){
	enableClk();
	configureIO();


	Kp = 5.0f;
	Ki = 0.5f;
	Kd = 0.8f;

	initADC2();
	initPWM();
	initI2C();
	initIMU();




while (1) {


	freeMotor();

	readIMUData(&ax, &ay, &az, &gx, &gy, &gz);

	if (gx !=0){
			turnON(2);
		}
		else {
			turnOFF(2);
		}
	if (gy !=0){
				turnON(2);
			}
			else {
				turnOFF(2);
			}
	if (gz !=0){
					turnON(2);
				}
				else {
					turnOFF(2);
				}

	setAngles(ax, ay, az, gx, gy, gz, &roll, &pitch);

	yaw += gz * DT;

	if (yaw >= 360.0f) yaw -= 360.0f;
	if (yaw < 0.0f) yaw += 360.0f;
	if (roll >= 360.0f) roll -= 360.0f;
	if (roll < 0.0f) roll += 360.0f;
	if (pitch >= 360.0f) pitch -= 360.0f;
	if (pitch < 0.0f) pitch += 360.0f;

	if (roll > 180.0f) roll = 180.0f;
	if (yaw > 180.0f) yaw = 180.0f;
	if (pitch > 180.0f) pitch = 180.0f;
	if (roll < 0.0f) roll = 0.0f;
	if (yaw < 0.0f) yaw = 0.0f;
	if (pitch < 0.0f) pitch = 0.0f;

	float angle_deg =0;


	if (fabs(roll) > STABILITY_TOLERANCE){
		    angle_deg = roll;
	}
	if (fabs(pitch) > STABILITY_TOLERANCE){
			angle_deg = pitch;
	}
	if (fabs(yaw) > STABILITY_TOLERANCE){
			angle_deg = yaw;
	}
	if (angle_deg >15){
		turnON(2);
	}
	else {
		turnOFF(2);
	}


	// 3400 from Motor Side to 1050 to the Potentiometer Side gives angle from 0 to 180
	// 3400 -> 0
	// 1050 -> 180

	angle_deg = 30; //Hard Coded to test

	ref = (uint16_t) (-13.0556 * angle_deg + 3400);

	curr = getcurrentPosition();

	//if ((ref >= LOWER_LIMIT && ref <= UPPER_LIMIT) && (curr != ref)) PIDController();

	delay(10);


}


}


void configureIO(){

	/*
	 *
	 * LEDS => B13 output "2"
	 * PWM => A1 output AF "A"
	 * ADC => A2 input "0"
	 * HBridge IN3 => B7
	 * HBridge IN4 => B8
	 * HBridge ENB => B9
	 * SDA -> B11
	 * SCL -> B10
	 *
	 * */

	GPIOA -> CRL = 0x444440A4;
	GPIOB -> CRL = 0x24444444;
	GPIOB -> CRH = 0x4424AA22;


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


void initADC2(){

		ADC2->SQR1 = 0x00000000;  // Reset SQR1 (sequence length = 1 by default)
		ADC2->SQR3 = 2; // Channel 2
		ADC2 -> SMPR2 &= 0; // reseting the sample time
		ADC2->SMPR2 |= (0b010 << 6); // Channel 2 7.5 cycles

		ADC2->CR2 |= ADC_CR2_ADON; // ADC on
		for (volatile int i = 0; i < 10000; i++);  // Short delay

		ADC2->CR2 |= ADC_CR2_CAL;          // Start calibration
		while (ADC2->CR2 & ADC_CR2_CAL);   // Wait for calibration to finish

		ADC2->CR2 |= ADC_CR2_EXTTRIG;  // Enable external trigger

		ADC2->CR2 &= ~ADC_CR2_EXTSEL;       // Clear EXTSEL bits
		ADC2->CR2 |= (0b111 << 17);         // Set EXTSEL = 111 for SWSTART

		ADC2->CR2 &= ~ADC_CR2_CONT;  // Clear CONT bit for single conversion mode

		ADC2->CR2 &= ~ADC_CR2_ALIGN;  // 0 = Right alignment (default)

}


void delay(uint16_t t){
	TIM3 ->PSC = 8000-1;
	TIM3 ->ARR = t-1;
	TIM3 ->CR1 |= TIM_CR1_CEN;
	while(!(TIM3->SR & TIM_SR_UIF));
	TIM3->SR &= ~TIM_SR_UIF;
	TIM3 ->CR1 &= ~(TIM_CR1_CEN);
}


uint16_t getADCVal(uint8_t i){
	if (i ==0){
		ADC1->CR2 |= ADC_CR2_SWSTART; // start conversion
			while (!(ADC1->SR & ADC_SR_EOC));     // Wait for conversion complete
			ADC1->SR &= ~(ADC_SR_EOC);
			uint16_t ADCVal = ADC1->DR;  // Read result (10-bit mask)
			return ADCVal;
	}
	else {
		ADC2->CR2 |= ADC_CR2_SWSTART; // start conversion
		while (!(ADC2->SR & ADC_SR_EOC));     // Wait for conversion complete
		ADC2->SR &= ~(ADC_SR_EOC);
		uint16_t ADCVal = ADC2->DR;  // Read result (10-bit mask)
		return ADCVal;
	}


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
	TIM2 -> ARR = 20000 - 1;

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

void controlMotor(float angle_deg) {
	    if (angle_deg < 0.0f) angle_deg = 0.0f;
	    if (angle_deg > 180.0f) angle_deg = 180.0f;

	    float pulse_width_us = 1000.0f + (angle_deg / 180.0f) * 1000.0f;
	    TIM2->CCR1 = (uint16_t)pulse_width_us; // For TIM2 Channel 1
	}




void initIMU() {
    // Wake up the MPU6050
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    I2C2->DR = MPU9050_ADDR << 1; // write
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

    while (!(I2C2->SR1 & (1 << 1))); // check if it's not busy
    // Start communication
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    for (volatile int i = 0; i <10000 ;i++);

    I2C2->DR = MPU9050_ADDR << 1; // Write
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR1; (void)I2C2->SR2;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->DR = ACCEL_XOUT_H;

    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->CR1 |= I2C_CR1_START; // Repeated start
    while (!(I2C2->SR1 & I2C_SR1_SB));
    (void)I2C2->SR1;

    I2C2->DR = (MPU9050_ADDR << 1) | 0x01; // Read
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

uint16_t getcurrentPosition(){ return getADCVal(1);}

// PID -> u(t) = Kp * E(t) + Ki * ∫E(t)dt + Kd * dE(t)/dt

void PIDController() {
    int count = 0;

    while ((fabs(ref - curr) > TOLERANCE) && (count++ < 10000)) {

    	if (fabs(prevRef - ref) > RESET_THRESHOLD){
    		errDerivative =0;
    		errIntegral = 0;

    	}

        curr = getcurrentPosition();
        err = ref - curr;

        // Enforce limits and prevent overshooting
        if ((curr <= LOWER_LIMIT && err < 0) ||
            (curr >= UPPER_LIMIT && err > 0) ||
            curr < LOWER_LIMIT ||
            curr > UPPER_LIMIT) {

            pressBreak();        // Emergency stop

            // Wait until ref is set to move AWAY from the limit
            while (1) {
                curr = getcurrentPosition();
            	readIMUData(&ax, &ay, &az, &gx, &gy, &gz);
            	setAngles(ax, ay, az, gx, gy, gz, &roll, &pitch);

            	float angle_deg =0;
            		if (fabs(roll) > STABILITY_TOLERANCE){
            			    angle_deg = roll;
            			}
            			if (fabs(pitch) > STABILITY_TOLERANCE){
            				angle_deg = pitch;
            			}
            			if (fabs(yaw) > STABILITY_TOLERANCE){
            				angle_deg = yaw;
            			}

            		ref = (uint16_t) (-13.0556 * angle_deg + 3400);
                err = ref - curr;

                if ((curr > LOWER_LIMIT && err > 0) ||
                    (curr < UPPER_LIMIT && err < 0)) {
                    break; // Safe to resume
                }

                if (ref > LOWER_LIMIT && ref < UPPER_LIMIT) break;
            }
        }


        errIntegral += err;
        if (errIntegral > 1000) errIntegral = 1000;
        else if (errIntegral < -1000) errIntegral = -1000;

        errDerivative = err - prevErr;
        prevErr = err;

        u = Kp * err + Ki * errIntegral + Kd * errDerivative;

        float duty = fabs(u);
        duty = duty > 100.0f ? 100.0f : duty;  // Clamp to 100

        freeMotor();  // Remove brake

        if (u > 0) {
            setRotationDir(1);  // Clockwise
        } else {
            setRotationDir(0);  // Counter-clockwise
        }

        writePWM(duty);

    }

    // Stop motor and show finish (BLUE ON)
    pressBreak();
    prevRef = ref;
}



