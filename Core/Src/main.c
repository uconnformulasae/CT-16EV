/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *d
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

//TPS1 0 Nominal 1.6564 :: 100 Nominal 2.2905
//TPS2 0 Nominal 0.8153 :: 100 Nominal 1.4364


// TAKE ID 0x555 byte 2 (3 for apps2) divide by 255 * 3.3 to convert to voltage
//Change FAULT LOW and FAULT high to prevent shutoffs

#define TPS1_0PER 1.4
#define TPS1_100PER 2.3

#define TPS1_FAULT_LOW 0.2
#define TPS1_FAULT_HIGH 3.0

#define TPS2_0PER 0.63
#define TPS2_100PER 1.55

#define TPS2_FAULT_LOW 0.2
#define TPS2_FAULT_HIGH 3.0

#define BPS_Setpoint 0.765

#define APPS_TRIP_PERCENT 0.1

#define TPS_IIR_RATIO 0.

#define ADC_TPS1	&hadc1
#define ADC_TPS2    &hadc2
#define ADC_BPS		&hadc3

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- LAUNCH CONTROL CONFIGURATION ---
#define LAUNCH_CONTROL_ENABLE 1 // Set to 1 to enable, 0 to disable

#if LAUNCH_CONTROL_ENABLE
// --- Tunable Parameters ---
#define LC_ACTIVATION_TPS_THRESHOLD 0.95  // Min throttle % to activate LC
#define LC_DEACTIVATION_SPEED_KPH 40.0    // Speed at which LC disengages
#define LC_TARGET_SLIP_RATIO 0.15         // Target slip ratio (15%)
#define LC_KP 80.0                        // Proportional gain for PI controller
#define LC_KI 40.0                        // Integral gain for PI controller
#define LC_INTEGRAL_WINDUP_GUARD 1000.0   // Anti-windup for the integral term

// --- Vehicle/Sensor Parameters ---
#define FRONT_WHEEL_DIAMETER_M 0.4572     // 18-inch wheel diameter in meters
#define FRONT_WHEEL_PULSES_PER_REV 48.0   // Pulses per revolution for the front wheel speed sensor
#define REAR_WHEEL_DIAMETER_M 0.4572      // 18-inch wheel diameter in meters
#define VEHICLE_GEAR_RATIO 3.8            // Motor to wheel gear ratio

// --- Internal Defines ---
#define LC_ACTIVATION_SPEED_KPH 1.0       // Max speed to be considered "stationary" for activation
#define PI (3.14159265359f)
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
#if LAUNCH_CONTROL_ENABLE
TIM_HandleTypeDef htim4;
#endif

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
#if LAUNCH_CONTROL_ENABLE
// Launch Control State Variables
volatile float front_wheel_speed_kph = 0.0f;
volatile float rear_wheel_speed_kph = 0.0f;
volatile float wheel_slip_ratio = 0.0f;
volatile uint32_t last_pulse_time = 0;
volatile uint8_t first_pulse_captured = 0;

enum {
    LC_STATE_IDLE,
    LC_STATE_ACTIVE,
    LC_STATE_COOLDOWN
};
volatile uint8_t launch_control_state = LC_STATE_IDLE;
float lc_integral_term = 0.0f;
uint32_t launch_control_torque_limit = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
#if LAUNCH_CONTROL_ENABLE
static void MX_TIM4_Init(void);
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if LAUNCH_CONTROL_ENABLE
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
        if (!first_pulse_captured)
        {
            // For the very first pulse, just record the time and wait for the next one.
            last_pulse_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            first_pulse_captured = 1;
            return;
        }

        uint32_t current_pulse_time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        uint32_t pulse_period_ticks;

        if (current_pulse_time > last_pulse_time)
        {
            pulse_period_ticks = current_pulse_time - last_pulse_time;
        }
        else
        {
            // Timer overflowed between pulses
            pulse_period_ticks = (htim->Init.Period - last_pulse_time) + current_pulse_time;
        }
        last_pulse_time = current_pulse_time;

        if (pulse_period_ticks > 0)
        {
            // TIM4 is clocked from APB1, which is 24MHz.
            // The prescaler for TIM4 is set to 23, so the timer clock is 1 MHz.
            float pulse_frequency = 1000000.0f / pulse_period_ticks; // Timer Freq / Ticks
            float wheel_rpm = (pulse_frequency / FRONT_WHEEL_PULSES_PER_REV) * 60.0f;
            float wheel_speed_mps = (wheel_rpm / 60.0f) * PI * FRONT_WHEEL_DIAMETER_M;
            front_wheel_speed_kph = wheel_speed_mps * 3.6f;
        }
        else
        {
            front_wheel_speed_kph = 0.0f;
        }
    }
}
#endif

CAN_RxHeaderTypeDef RxHeader;

volatile uint8_t RxData[8];
volatile uint8_t ddb = 10;
volatile uint32_t torque_limit = 2200; //x10
volatile uint32_t motor_speed = 0;
volatile uint32_t current_limit = 125;
volatile uint32_t bus_voltage = 396;
volatile uint8_t inverter_enabled = 0;
volatile uint8_t inverter_lockout = 1;
volatile uint8_t can_ready = 0;
volatile uint8_t print_ready = 0;
volatile uint8_t ready_to_drive = 0;
volatile uint8_t tps1_oor = 0;
volatile uint8_t tps2_oor = 0;
volatile uint8_t tps_dist_error = 0;
volatile uint16_t rtd_timeout = 199;
volatile uint8_t inv_message = 0;
volatile uint8_t rtd_buzzer_counter = 0;
volatile uint8_t start_disable_debounce = 1;
volatile uint16_t disable_debounce = 999;



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {

		Error_Handler();
	}

	if (RxHeader.StdId == 0x0B1) {
		//torque_limit = RxData[1] << 8 | RxData[0];
	}
	else if (RxHeader.StdId == 0x0AA) {
		inverter_enabled = RxData[6] & 0x01;
		inverter_lockout = (RxData[6] >> 7) & 0x01;
		uint8_t ready_pin_state = (RxData[3] >> 1) & 0x01;
		inv_message = RxData[6];
		if(ready_pin_state){
			rtd_timeout = 0;
		}
	}
	else if (RxHeader.StdId == 0x0A5){
		motor_speed = RxData[3] << 8 | RxData[2];
	}
	else if (RxHeader.StdId == 0x202){
		current_limit = (RxData[1] << 8 | RxData[0]) - 3;
	}
	else if (RxHeader.StdId == 0x600){
			bus_voltage = (RxData[5] << 8 | RxData[4]);
		}


}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		can_ready = 1;
		if(start_disable_debounce){
			disable_debounce += 1;
			disable_debounce = fmin(disable_debounce, 100);
		}
		else{
			disable_debounce = 0;
		}
	}
	else if (htim->Instance == TIM3) {
		print_ready = 1;
		if(ready_to_drive && rtd_buzzer_counter < 100){
			rtd_buzzer_counter += 1;
		}
		rtd_timeout += 1;
		rtd_timeout = fmin(rtd_timeout, 200);
	}
}

double tmap_lut(double tps) {
	double V_MIN = 0.1;
	double V_MAX = 0.9;
	double tps_local = (fmax(V_MIN, fmin(tps, V_MAX)) - V_MIN) * (1 / (V_MAX - V_MIN));
	return tps_local;
}

int torque_lut(double tps) {
	uint32_t torque_limit_local = torque_limit;
	torque_limit_local = fmin(torque_limit, (double) (4200 * current_limit) * 1.0 / fmax(230.4, (double) motor_speed * 0.1076));
	//if(motor_speed < 150){
		//torque_limit_local = fmin(torque_limit_local, 900);
	//}
	if(motor_speed >= 6000){
		torque_limit_local = 300;
	}
	return tps * torque_limit_local;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	for (int i = 0; i < sizeof(RxData); i++) {
				RxData[i] = 0;
			}

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_USB_PCD_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
#if LAUNCH_CONTROL_ENABLE
  MX_TIM4_Init();
#endif
  CAN_TxHeaderTypeDef TxHeader;
    	CAN_TxHeaderTypeDef debugHeader;
    	CAN_TxHeaderTypeDef rtdHeader;
    	uint8_t TxData[8];
    	uint32_t TxMailbox;

    	CAN_FilterTypeDef canfilterconfig;

    	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    	canfilterconfig.FilterBank = 1; // which filter bank to use from the assigned ones
    	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    	canfilterconfig.FilterIdHigh = 0x0B1 << 5;
    	canfilterconfig.FilterIdLow = 0;
    	canfilterconfig.FilterMaskIdHigh = 0x0B1 << 5;
    	canfilterconfig.FilterMaskIdLow = 0x0000;
    	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    	canfilterconfig.FilterBank = 2; // which filter bank to use from the assigned ones
    	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    	canfilterconfig.FilterIdHigh = 0x0AA << 5;
    	canfilterconfig.FilterIdLow = 0;
    	canfilterconfig.FilterMaskIdHigh = 0x0AA << 5;
    	canfilterconfig.FilterMaskIdLow = 0x0000;
    	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    	canfilterconfig.FilterBank = 3; // which filter bank to use from the assigned ones
    	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    	canfilterconfig.FilterIdHigh = 0x0A5 << 5;
    	canfilterconfig.FilterIdLow = 0;
    	canfilterconfig.FilterMaskIdHigh = 0x0A5 << 5;
    	canfilterconfig.FilterMaskIdLow = 0x0000;
    	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    	canfilterconfig.FilterBank = 4; // which filter bank to use from the assigned ones
    	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    	canfilterconfig.FilterIdHigh = 0x202 << 5;
    	canfilterconfig.FilterIdLow = 0;
    	canfilterconfig.FilterMaskIdHigh = 0x202 << 5;
    	canfilterconfig.FilterMaskIdLow = 0x0000;
    	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    	canfilterconfig.FilterBank = 5; // which filter bank to use from the assigned ones
    	canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    	canfilterconfig.FilterIdHigh = 0x600 << 5;
    	canfilterconfig.FilterIdLow = 0;
    	canfilterconfig.FilterMaskIdHigh = 0x600 << 5;
    	canfilterconfig.FilterMaskIdLow = 0x0000;
    	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    	HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

    	HAL_CAN_Start(&hcan);

    	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)
    			!= HAL_OK) {
    		Error_Handler();
    	}

#if LAUNCH_CONTROL_ENABLE
    	// Start the input capture for front wheel speed sensor on pin PB8 (TIM4_CH3)
    	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    	TxHeader.IDE = CAN_ID_STD;
    	  		TxHeader.StdId = 0x0C0;
    	  		TxHeader.RTR = CAN_RTR_DATA;
    	  		TxHeader.DLC = 8;

    	  		debugHeader.IDE = CAN_ID_STD;
    	  		debugHeader.StdId = 0x555;
    	  		debugHeader.RTR = CAN_RTR_DATA;
    	  		debugHeader.DLC = 8;

    	  		rtdHeader.IDE = CAN_ID_STD;
    	  		rtdHeader.StdId = 0x556;
    	  		rtdHeader.RTR = CAN_RTR_DATA;
    	  		rtdHeader.DLC = 1;


    	  		double tps1 = 0;
    	  		double tps2 = 0;
    	  		double tps_combined = 0;
    	  		double bps = 0;
    	  		uint32_t torque_request = 0;
    	  		uint8_t heartbeat_counter = 0;
    	  		uint32_t tps1_adc = 0;
    	  		uint32_t tps2_adc = 0;
    	  		uint32_t bps_adc = 0;
    	  		uint8_t brake_pressed = 0;
    	  		uint8_t bse_error = 0;
    	  		uint8_t should_disable_inverter = 0;

    	  		HAL_TIM_Base_Start_IT(&htim2);
    	  		HAL_TIM_Base_Start_IT(&htim3);

    	  		double tps1_avg = 0;
    	  		double tps2_avg = 0;

    	  		int32_t rtd_debounce = 0;
    	  		uint8_t rtd_raw = 0;
    	  		while (1) {

    	  			HAL_ADC_Start(ADC_BPS);
    	  			HAL_ADC_Start(ADC_TPS1);
    	  			HAL_ADC_Start(ADC_TPS2);
    	  			
    	  			HAL_CAN_AbortTxRequest(&hcan, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

    	  			// Throttle Position Potentiometer 1 Acquire and Calculate
    	  			HAL_ADC_PollForConversion(ADC_TPS1, HAL_MAX_DELAY);
    	  			tps1_adc = HAL_ADC_GetValue(ADC_TPS1);
    	  			double tps1_v = ((double) tps1_adc) / 4095 * 3.3;
    	  			tps1 = (tps1_v - TPS1_0PER) / (TPS1_100PER - TPS1_0PER); // Percentage
    	  			tps1 = fmax(tps1, 0);
    	  			tps1_avg = (tps1_avg == 0) ? tps1 : tps1_avg * TPS_IIR_RATIO + tps1 * (1 - TPS_IIR_RATIO);
    	  			tps1 = tps1_avg;
    	  			

    	  			// Throttle Position Potentiometer 2 Acquire and Calculate
    	  			HAL_ADC_PollForConversion(ADC_TPS2, HAL_MAX_DELAY);
    	  			tps2_adc = HAL_ADC_GetValue(ADC_TPS2);
    	  			double tps2_v = ((double) tps2_adc) / 4095 * 3.3;
    	  			tps2 = (tps2_v - TPS2_0PER) / (TPS2_100PER - TPS2_0PER); // Percentage
    	  			tps2 = fmax(tps2, 0);
    	  			tps2_avg = (tps2_avg == 0) ? tps2 : tps2_avg * TPS_IIR_RATIO + tps2 * (1 - TPS_IIR_RATIO);
    	  			tps2 = tps2_avg;
    	  			
    	  			// TPS and Torque request calculate
    	  			tps_combined = (tps1 + tps2) / 2;
    	  			torque_request = torque_lut(tmap_lut(tps_combined));
    	  			
    	  			// Brake Pressure Acquire and Calculate
    	  			HAL_ADC_PollForConversion(ADC_BPS, HAL_MAX_DELAY);
    	  			bps_adc = HAL_ADC_GetValue(ADC_BPS);
    	  			bps = ((double) bps_adc) / 4095 * 5;
    	  			brake_pressed = bps > BPS_Setpoint;
    	  			
    	  			// Ready to Drive button poll
    	  			rtd_raw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
    	  			rtd_raw &= brake_pressed;
    	  			
    	  			if (rtd_raw){
				rtd_debounce += 3;
				}
				else {
					rtd_debounce -= 4;
				}
    	  			
    	  			rtd_debounce = fmin(fmax(rtd_debounce, 0), 100);

    	  			if (rtd_debounce > 50){
    	  				ready_to_drive = 1;
    	  			}

    	  			ready_to_drive &= rtd_timeout < 20;

    	  			// Ready to Drive dashboard light
    	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, ready_to_drive);

    	  			// Ready to drive Buzzer
    	  			if (ready_to_drive == 0) {
    	  				rtd_buzzer_counter = 0;
    	  			}
    	  			else {
				if(rtd_buzzer_counter < 25){
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
				}
				else{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
				}
    	  			}
    	  			
    	  			// Error States
				tps1_oor = tps1_v < TPS1_FAULT_LOW || tps1_v > TPS1_FAULT_HIGH;
				tps2_oor = tps2_v < TPS2_FAULT_LOW || tps2_v > TPS2_FAULT_HIGH;
				tps1 = fmin(tps1, 100);
				tps2 = fmin(tps2, 100);
    	  			tps_dist_error = fabs(tps1 - tps2) > APPS_TRIP_PERCENT;

    	  			if(!bse_error){
    	  				bse_error = brake_pressed && tps_combined >= 0.1;
    	  			}
    	  			else{
    	  				bse_error = tps_combined >= 0.05;
    	  			}

    	  			// Disable Inverter if any errors present
    	  			start_disable_debounce = tps1_oor || tps2_oor || tps_dist_error || bse_error;
    	  			should_disable_inverter = (disable_debounce > 5) || !ready_to_drive;

#if LAUNCH_CONTROL_ENABLE
                // --- LAUNCH CONTROL LOGIC ---

                // 1. Calculate rear wheel speed from motor speed
                float rear_motor_rpm = motor_speed; // Assuming motor_speed is in RPM
                float rear_wheel_rpm = rear_motor_rpm / VEHICLE_GEAR_RATIO;
                float rear_wheel_speed_mps = (rear_wheel_rpm / 60.0f) * PI * REAR_WHEEL_DIAMETER_M;
                rear_wheel_speed_kph = rear_wheel_speed_mps * 3.6f;

                // 2. Determine Launch Control State
                uint8_t can_activate_lc = !brake_pressed && (tps_combined > LC_ACTIVATION_TPS_THRESHOLD) && (front_wheel_speed_kph < LC_ACTIVATION_SPEED_KPH);

                switch (launch_control_state) {
                    case LC_STATE_IDLE:
                        if (can_activate_lc && ready_to_drive && !should_disable_inverter) {
                            launch_control_state = LC_STATE_ACTIVE;
                            lc_integral_term = 0; // Reset integral term on activation
                        }
                        break;

                    case LC_STATE_ACTIVE:
                        // Deactivation conditions
                        if (!ready_to_drive || should_disable_inverter || (tps_combined < LC_ACTIVATION_TPS_THRESHOLD) || (front_wheel_speed_kph > LC_DEACTIVATION_SPEED_KPH)) {
                            launch_control_state = LC_STATE_COOLDOWN;
                        }
                        break;

                    case LC_STATE_COOLDown:
                        // Stay in cooldown until vehicle is stopped and throttle is released, preventing re-activation
                        if (front_wheel_speed_kph < LC_ACTIVATION_SPEED_KPH && tps_combined < 0.05) {
                            launch_control_state = LC_STATE_IDLE;
                        }
                        break;
                }

                // 3. Calculate Torque Limit if LC is Active
                if (launch_control_state == LC_STATE_ACTIVE) {
                    // Calculate wheel slip. Add a small epsilon to front speed to avoid division by zero when stationary.
                    float front_speed_safe = fmaxf(front_wheel_speed_kph, 0.1f);
                    wheel_slip_ratio = (rear_wheel_speed_kph - front_wheel_speed_kph) / front_speed_safe;
                    wheel_slip_ratio = fmaxf(0.0f, wheel_slip_ratio); // Slip cannot be negative

                    // PI Controller for slip regulation
                    float error = LC_TARGET_SLIP_RATIO - wheel_slip_ratio;

                    // Integral term with anti-windup
                    lc_integral_term += error * LC_KI;
                    if (lc_integral_term > LC_INTEGRAL_WINDUP_GUARD) lc_integral_term = LC_INTEGRAL_WINDUP_GUARD;
                    if (lc_integral_term < -LC_INTEGRAL_WINDUP_GUARD) lc_integral_term = -LC_INTEGRAL_WINDUP_GUARD;

                    // PI output.
                    float pi_output = (error * LC_KP) + lc_integral_term;

					// We calculate a torque limit based on the PI controller output.
					// This limit is subtracted from the maximum possible torque.
					launch_control_torque_limit = torque_lut(1.0) - pi_output;
					launch_control_torque_limit = fmax(0, fmin(torque_limit, launch_control_torque_limit));


                    // IMPORTANT: The LC torque limit can only REDUCE the driver's requested torque.
                    torque_request = fmin(torque_request, launch_control_torque_limit);
                }
#endif

    	  			if (should_disable_inverter) {
    	  				torque_request = 0;
    	  			}

    	  			if (can_ready) {	// Every ~10 (?) ms
    	  				if (inverter_lockout == 1) {
    	  					TxData[0] = torque_request & 0xFF;			// Torque Command lo
    	  					TxData[1] = torque_request >> 8 & 0xFF;		// Torque Command hi
    	  					TxData[2] = 0x00;							// Speed Command lo
    	  					TxData[3] = 0x00;							// Speed Command hi
    	  					TxData[4] = 0x01; // Direction: Reverse = 0x00 | Forward = 0x01;
    	  					TxData[5] = 0x00 | 0x02 | (heartbeat_counter << 4);// 5[0] = Inv enable | 5[1] = Discharge enable | counter
    	  					TxData[6] = 0x00;			// Torque limit lo, 0 = EEprom limit
    	  					TxData[7] = 0x00;			// Torque limit hi, 0 = EEprom limit

    	  					if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
    	  							!= HAL_OK) {
    	  						Error_Handler();
    	  					}
    	  					heartbeat_counter += 1;
    	  					heartbeat_counter = heartbeat_counter & 0x0F;
    	  					can_ready = 0;
    	  					HAL_Delay(10);
    	  				}
    	  			else{
    	  				TxData[0] = torque_request & 0xFF;				// Torque Command lo
    	  				TxData[1] = torque_request >> 8 & 0xFF;			// Torque Command hi
    	  				TxData[2] = 0x00;								// Speed Command lo
    	  				TxData[3] = 0x00;								// Speed Command hi
    	  				TxData[4] = 0x01; 	// Direction: Reverse = 0x00 | Forward = 0x01;
    	  				TxData[5] = (~should_disable_inverter & 0x01) | 0x02 | (heartbeat_counter << 4); // 5[0] = Inv enable | 5[1] = Discharge enable | counter
    	  				TxData[6] = 0x00;				// Torque limit lo, 0 = EEprom limit
    	  				TxData[7] = 0x00;				// Torque limit hi, 0 = EEprom limit

    	  				if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox)
    	  						!= HAL_OK) {
    	  					Error_Handler();
    	  				}
    	  				heartbeat_counter += 1;
    	  				heartbeat_counter = heartbeat_counter & 0x0F;
    	  				can_ready = 0;
    	  			}
    	  			}

    	  			if (print_ready) {	// Every ~100(?) ms
    	  				TxData[0] = rtd_timeout & 0xFF;
    	  				TxData[1] = (bps_adc >> 4) & 0xFF;
    	  				TxData[2] = (tps1_adc >> 4) & 0xFF;
    	  				TxData[3] = (tps2_adc >> 4) & 0xFF;
    	  				TxData[4] = (inverter_lockout << 7) | (inverter_enabled << 6) | (tps_dist_error << 5) | (tps2_oor << 4) | (tps1_oor << 3) | (bse_error << 2) | (!ready_to_drive << 1) | should_disable_inverter;
    	  				TxData[5] = (int) (tps1 * 100) & 0xff;
    	  				TxData[6] = (int) (tps2 * 100) & 0xff;
    	  				TxData[7] = (int) (tmap_lut(tps_combined) * 100) & 0xFF;

    	  				if (HAL_CAN_AddTxMessage(&hcan, &debugHeader, TxData, &TxMailbox)
    	  						!= HAL_OK) {
    	  					Error_Handler();
    	  				}
    	  				TxData[0] = (ready_to_drive) & 0x01;

    	  				if (HAL_CAN_AddTxMessage(&hcan, &rtdHeader, TxData, &TxMailbox)
    	  						!= HAL_OK) {
    	  					Error_Handler();
    	  				}

    	  				uint32_t timeout_start = HAL_GetTick();
    	  				while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) <= 1){
    	  					uint32_t t = HAL_GetTick();
    	  					if(t < timeout_start || (t - timeout_start) > 15){
    	  						break;
    	  					}
    	  				}
    	  				print_ready = 0;
    	  			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    		}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 21;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 8000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 14399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if LAUNCH_CONTROL_ENABLE
/**
  * @brief TIM4 Initialization Function
  * @note  This function configures TIM4 to measure the frequency of an incoming signal on PB8 (TIM4_CH3).
  *        It sets up the timer in Input Capture mode.
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim4.Instance = TIM4;
  // SystemCoreClock is 24MHz. APB1 clock is 24MHz.
  // Prescaler of 23 gives 1MHz timer clock (1us per tick).
  htim4.Init.Prescaler = 23;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535; // Max period for 16-bit timer
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
* @brief TIM_IC MSP Initialization
* This function configures the hardware resources required for TIM4 Input Capture:
*     - TIM4 clock enable
*     - GPIO pin configuration for TIM4_CH3 (PB8)
* @param htim_ic: TIM_IC handle pointer
* @retval None
*/
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(htim_ic->Instance==TIM4)
  {
    /* Peripheral clock enable */
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /**TIM4 GPIO Configuration
    PB8     ------> TIM4_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    // Set pull-up or pull-down based on your sensor's output stage.
    // A pull-up is common for open-drain/collector sensors.
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  }
}
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
