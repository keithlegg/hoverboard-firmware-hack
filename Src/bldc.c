
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"


volatile int posl = 0;
volatile int posr = 0;
volatile int pwml = 0;
volatile int pwmr = 0;
volatile int weakl = 0;
volatile int weakr = 0;

extern volatile int speed;

extern volatile adc_buf_t adc_buffer;

extern volatile uint32_t timeout;

uint32_t buzzerFreq = 0;
uint32_t buzzerPattern = 0;

uint8_t enable = 0;

const int pwm_res = 64000000 / 2 / PWM_FREQ; // = 2000

const uint8_t hall_to_pos[8] = {
    0,
    0,
    2,
    1,
    4,
    5,
    3,
    0,
};

inline void blockPWM(int pwm, int pos, int *u, int *v, int *w) {
  switch(pos) {
    case 0:
      *u = 0;
      *v = pwm;
      *w = -pwm;
      break;
    case 1:
      *u = -pwm;
      *v = pwm;
      *w = 0;
      break;
    case 2:
      *u = -pwm;
      *v = 0;
      *w = pwm;
      break;
    case 3:
      *u = 0;
      *v = -pwm;
      *w = pwm;
      break;
    case 4:
      *u = pwm;
      *v = -pwm;
      *w = 0;
      break;
    case 5:
      *u = pwm;
      *v = 0;
      *w = -pwm;
      break;
    default:
      *u = 0;
      *v = 0;
      *w = 0;
  }
}

inline void blockPhaseCurrent(int pos, int u, int v, int *q) {
  switch(pos) {
    case 0:
      *q = u - v;
      // *u = 0;
      // *v = pwm;
      // *w = -pwm;
      break;
    case 1:
      *q = u;
      // *u = -pwm;
      // *v = pwm;
      // *w = 0;
      break;
    case 2:
      *q = u;
      // *u = -pwm;
      // *v = 0;
      // *w = pwm;
      break;
    case 3:
      *q = v;
      // *u = 0;
      // *v = -pwm;
      // *w = pwm;
      break;
    case 4:
      *q = v;
      // *u = pwm;
      // *v = -pwm;
      // *w = 0;
      break;
    case 5:
      *q = -(u - v);
      // *u = pwm;
      // *v = 0;
      // *w = -pwm;
      break;
    default:
      *q = 0;
      // *u = 0;
      // *v = 0;
      // *w = 0;
  }
}

uint32_t buzzerTimer        = 0;

int offsetcount = 0;
int offsetrl1   = 2000;
int offsetrl2   = 2000;
int offsetrr1   = 2000;
int offsetrr2   = 2000;
int offsetdcl   = 2000;
int offsetdcr   = 2000;

float batteryVoltage = BAT_NUMBER_OF_CELLS * 4.0;

int curl = 0;
// int errorl = 0;
// int kp = 5;
// volatile int cmdl = 0;

int last_pos = 0;
int timer = 0;
const int max_time = PWM_FREQ / 10;
volatile int vel = 0;



/*

NOTES:

    #DISABLE LEFT MOTOR :
        
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;


    #DISABLE RIGHT MOTOR :

        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;


    --------------------------------
    #PIN MAPPINGS - 
        You can run a bidirectional high current H bridge through ANY two pins !!
        a total of 3 independent DC motors!
    
    LEFT :
        GREEN  - LEFT_TIM->LEFT_TIM_U
        BLUE   - LEFT_TIM->LEFT_TIM_V 
        YELLOW - LEFT_TIM->LEFT_TIM_W 

    RIGHT :
        GREEN   - RIGHT_TIM->RIGHT_TIM_U
        BLUE    - RIGHT_TIM->RIGHT_TIM_V
        YELLOW  - RIGHT_TIM->RIGHT_TIM_W

*/

 
#ifdef KEITH_RUN_DCBRUSHED_TEST

    // HERE THERE BE DRAGONS!!

    //determine next position based on hall sensors
    uint8_t hall_ul = 0;
    uint8_t hall_vl = 0;
    uint8_t hall_wl = 0;

    int keith_phase_count = 0;


    int global_power = 0;  //test for all three motor power

    int m1_crnt = 0;  //THIS SETS THE POWER OF MOTOR1 
    int m2_crnt = 0;  //THIS SETS THE POWER OF MOTOR2 
    int m3_crnt = 0;  //THIS SETS THE POWER OF MOTOR3 

    /************/
    // motor one is LEFT GREEN, BLUE wires 

    void motor_one_forward(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(m1_crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(-m1_crnt + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_one_backward(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(-m1_crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(m1_crnt   + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_one_off(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
    }

    /************/
    // motor two is LEFT AND RIGHT YELLOW wires 
    void motor_two_forward(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(m2_crnt   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(-m2_crnt  + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_two_backward(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(-m2_crnt  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(m2_crnt   + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_two_off(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
    }

    /************/
    // motor three is RIGHT GREE, BLUE wires     
    void motor_three_forward(){
          RIGHT_TIM->RIGHT_TIM_U  = CLAMP(m3_crnt  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V  = CLAMP(-m3_crnt + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_three_backward(){
          RIGHT_TIM->RIGHT_TIM_U = CLAMP(-m3_crnt   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V = CLAMP(m3_crnt    + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_three_off(){
          RIGHT_TIM->RIGHT_TIM_U  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
    }

    /************/
    void all_motors_pwm_off(){
          LEFT_TIM->LEFT_TIM_U   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_U = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);      
    }

    void disable_right_motor(){
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; 
    }
    
    void disable_left_motor(){
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; 
    }

    /************/

    void DMA1_Channel1_IRQHandler() {
      DMA1->IFCR = DMA_IFCR_CTCIF1;
      // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

      buzzerTimer++; //also used for ADC / battery voltage checks


      //create square wave for buzzer
      if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerTimer % buzzerFreq == 0) {
          HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
        }
      } else {
          HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
      }

      if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
        batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
      }


      if (buzzerTimer % 1000 == 0) { 
          if (global_power<1000){     
              global_power = global_power + 1;
          }
          if (global_power==1000){
              global_power = 0;
          }

          m1_crnt = global_power;
          m2_crnt = global_power;
          m3_crnt = global_power;

      }


      if (buzzerTimer % 2000 == 0) {
          // count to 5 over and over 
          if (keith_phase_count==12){
            keith_phase_count = 0;
          }else{
              keith_phase_count++;
          }
        
      }



      if(offsetcount < 1000) {  // calibrate ADC offsets
          offsetcount++;
          offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
          offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
          offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
          offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
          offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
          offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
          return;
      }



      //disable left PWM when current limit is reached (current chopping)
      if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
      } else {
        LEFT_TIM->BDTR |= TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
      }
      //disable right PWM when current limit is reached (current chopping)      
      if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
      } else {
        RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
      }

 
      // TEST OF RUNNING DC MOTORS THROUGH VARIOUS WIRES 
      if (keith_phase_count==0){
          all_motors_pwm_off();

      }
      if (keith_phase_count==1){
          motor_one_forward();
      }
      if (keith_phase_count==2){
          all_motors_pwm_off();

      }    
      if (keith_phase_count==3){
          motor_one_backward();
      } 

      if (keith_phase_count==4){
          all_motors_pwm_off();

      }
      if (keith_phase_count==5){
          motor_two_forward();
      }

      if (keith_phase_count==6){
          all_motors_pwm_off();

      }    
      if (keith_phase_count==7){
          motor_two_backward();
      } 

      if (keith_phase_count==8){
          all_motors_pwm_off();
      } 

      if (keith_phase_count==9){
          motor_three_forward();
      }
      if (keith_phase_count==10){
          all_motors_pwm_off();
      }

      if (keith_phase_count==11){
          motor_three_backward();
      }    

      if (keith_phase_count==12){
          all_motors_pwm_off();
      } 

   
    }

#endif 

/********************************************************/


#ifdef KEITH_RUN_BIPOLAR_STEP_TEST

    // HERE THERE BE DRAGONS!!
    // test to run a bipolar stepper motor 
    //derived from the 3 DC motor test example above - only uses 2 motors/coils
    // two of these control PCBs could drive 3 steppers! 


    int DIRECTION = 0;

    // //determine next position based on hall sensors
    // uint8_t hall_ul = 0;
    // uint8_t hall_vl = 0;
    // uint8_t hall_wl = 0;

    int keith_phase_count = 0;


    int m1_crnt = 600;  //THIS SETS THE POWER OF MOTOR1 
    int m2_crnt = 600;  //THIS SETS THE POWER OF MOTOR2 
    int m3_crnt = 600;  //THIS SETS THE POWER OF MOTOR3 

    /************/
    // motor one is LEFT GREEN, BLUE wires 

    void coil_one_forward(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(m1_crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(-m1_crnt + pwm_res / 2, 10, pwm_res-10);
    }
    void coil_one_backward(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(-m1_crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(m1_crnt   + pwm_res / 2, 10, pwm_res-10);
    }
    void coil_one_off(){
          LEFT_TIM->LEFT_TIM_U  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
    }

    /************/
    // motor two is LEFT AND RIGHT YELLOW wires 
    void coil_two_forward(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(m2_crnt   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(-m2_crnt  + pwm_res / 2, 10, pwm_res-10);
    }
    void coil_two_backward(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(-m2_crnt  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(m2_crnt   + pwm_res / 2, 10, pwm_res-10);
    }
    void coil_two_off(){
          LEFT_TIM->LEFT_TIM_W   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
    }


    /************/
    // motor three is RIGHT GREE, BLUE wires     
    void motor_three_forward(){
          RIGHT_TIM->RIGHT_TIM_U  = CLAMP(m3_crnt  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V  = CLAMP(-m3_crnt + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_three_backward(){
          RIGHT_TIM->RIGHT_TIM_U = CLAMP(-m3_crnt   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V = CLAMP(m3_crnt    + pwm_res / 2, 10, pwm_res-10);
    }
    void motor_three_off(){
          RIGHT_TIM->RIGHT_TIM_U  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V  = CLAMP(0   + pwm_res / 2, 10, pwm_res-10);
    }

    /************/
    void all_motors_pwm_off(){
          LEFT_TIM->LEFT_TIM_U   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W   = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_U = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_V = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);
          RIGHT_TIM->RIGHT_TIM_W = CLAMP(0  + pwm_res / 2, 10, pwm_res-10);      
    }

    void disable_right_motor(){
        // right "motor" is actually 3 "drive channels", or 1.5 motors
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; 
    }
    
    void disable_left_motor(){
        // left "motor" is actually 3 "drive channels", or 1.5 motors      
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE; 
    }



    /************/

    void DMA1_Channel1_IRQHandler() 
    {
        DMA1->IFCR = DMA_IFCR_CTCIF1;

        buzzerTimer++; //also used for ADC / battery voltage checks

        //create square wave for buzzer
        if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
          if (buzzerTimer % buzzerFreq == 0) {
            HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
          }
        } else {
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
        }

        if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
          batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
        }


    /************/

        if (buzzerTimer % 100000 == 0) {
            if (DIRECTION==0){
              DIRECTION = 1;
              motor_three_forward();
            }else{
                DIRECTION = 0;
                motor_three_backward();                
            }
          
        }

        if (buzzerTimer % 100 == 0) {
            // count to 5 over and over 
            if (keith_phase_count==4){
              keith_phase_count = 0;
            }else{
                keith_phase_count++;
            }
          
        }

        if(DIRECTION==0){
        
            if(keith_phase_count==0){
                coil_one_forward();
                coil_two_forward();
            }

            if(keith_phase_count==1){
                coil_one_backward();
                coil_two_forward();
            }

            if(keith_phase_count==2){
                coil_one_backward();
                coil_two_backward();
            }

            if(keith_phase_count==3){
                coil_one_forward();
                coil_two_backward();
            }

        }
        
        if(DIRECTION==1){

            if(keith_phase_count==0){
                coil_one_forward();
                coil_two_backward();
            }

            if(keith_phase_count==1){
                coil_one_backward();
                coil_two_backward();
            }

            if(keith_phase_count==2){
                coil_one_backward();
                coil_two_forward();
            }

            if(keith_phase_count==3){
                coil_one_forward();
                coil_two_forward();
            }        

        }


        if(offsetcount < 1000) {  // calibrate ADC offsets
            offsetcount++;
            offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
            offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
            offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
            offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
            offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
            offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
            return;
        }



        //disable left PWM when current limit is reached (current chopping)
        if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
          LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        } else {
          LEFT_TIM->BDTR |= TIM_BDTR_MOE;
        }
        //disable right PWM when current limit is reached (current chopping)      
        if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
          RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
        } else {
          RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
        }

    }

#endif 

/********************************************************/


#ifdef KEITH_RUN_NO_HALLSENSORS

    // HERE THERE BE DRAGONS!!

    //determine next position based on hall sensors
    uint8_t hall_ul = 0;
    uint8_t hall_vl = 0;
    uint8_t hall_wl = 0;

    int keith_phase_count = 0;
    int crnt = 5;


    void DMA1_Channel1_IRQHandler() {
      DMA1->IFCR = DMA_IFCR_CTCIF1;
      // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

      buzzerTimer++; //also used for ADC / battery voltage checks

      //create square wave for buzzer
      if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerTimer % buzzerFreq == 0) {
          HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
        }
      } else {
          HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
      }

      if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
        batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
      }


      if (buzzerTimer % 500 == 0) {
        // count to 8 over and over 
        if (keith_phase_count==6){
          keith_phase_count = 0;
        }else{
            keith_phase_count++;
        }
      
      }



      if(offsetcount < 1000) {  // calibrate ADC offsets
        offsetcount++;
        offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
        offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
        offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
        offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
        offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
        offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
        return;
      }



      //disable PWM when current limit is reached (current chopping)
      if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
      } else {
        LEFT_TIM->BDTR |= TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
      }
      if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
      } else {
        RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
      }

      int ul, vl, wl;
      int ur, vr, wr;

      //determine next position based on hall sensors
      //uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
      //uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
      //uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

      //uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
      //uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
      //uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

      //uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
      //posl          = hall_to_pos[halll];
      //posl += 2;
      //posl %= 6;

      //uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
      //posr          = hall_to_pos[hallr];
      //posr += 2;
      //posr %= 6;


      //blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

      //update PWM channels based on position
      //blockPWM(pwml, posl, &ul, &vl, &wl);
      //blockPWM(pwmr, posr, &ur, &vr, &wr);

      blockPWM(pwml, keith_phase_count, &ul, &vl, &wl); //keith was here

      LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
      LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
      LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

      //RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
      //RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
      //RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);

      /*
      if (keith_phase_count==0){
          LEFT_TIM->LEFT_TIM_U = CLAMP(0      + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(crnt   + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(-crnt  + pwm_res / 2, 10, pwm_res-10);
      }
      if (keith_phase_count==1){
          LEFT_TIM->LEFT_TIM_U = CLAMP(-crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(crnt     + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(0      + pwm_res / 2, 10, pwm_res-10);
      }
      if (keith_phase_count==2){
          LEFT_TIM->LEFT_TIM_U = CLAMP(-crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(0      + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(crnt   + pwm_res / 2, 10, pwm_res-10);
      }    
      if (keith_phase_count==3){
          LEFT_TIM->LEFT_TIM_U = CLAMP(0     + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(-crnt + pwm_res / 2, 10, pwm_res-10);
      } 
      if (keith_phase_count==4){
          LEFT_TIM->LEFT_TIM_U = CLAMP(crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(-crnt + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(0     + pwm_res / 2, 10, pwm_res-10);
      } 
      if (keith_phase_count==5){
          LEFT_TIM->LEFT_TIM_U = CLAMP(crnt  + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_V = CLAMP(0     + pwm_res / 2, 10, pwm_res-10);
          LEFT_TIM->LEFT_TIM_W = CLAMP(-crnt + pwm_res / 2, 10, pwm_res-10);
      } 
      */
      
    }
#endif 


#ifdef NORMAL_BLDC_RUNMODE  
   
    //scan 8 channels with 2ADCs @ 20 clk cycles per sample
    //meaning ~80 ADC clock cycles @ 8MHz until new DMA interrupt =~ 100KHz
    //=640 cpu cycles
    void DMA1_Channel1_IRQHandler() {
      DMA1->IFCR = DMA_IFCR_CTCIF1;
      // HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);

      if(offsetcount < 1000) {  // calibrate ADC offsets
        offsetcount++;
        offsetrl1 = (adc_buffer.rl1 + offsetrl1) / 2;
        offsetrl2 = (adc_buffer.rl2 + offsetrl2) / 2;
        offsetrr1 = (adc_buffer.rr1 + offsetrr1) / 2;
        offsetrr2 = (adc_buffer.rr2 + offsetrr2) / 2;
        offsetdcl = (adc_buffer.dcl + offsetdcl) / 2;
        offsetdcr = (adc_buffer.dcr + offsetdcr) / 2;
        return;
      }

      if (buzzerTimer % 1000 == 0) {  // because you get float rounding errors if it would run every time
        batteryVoltage = batteryVoltage * 0.99 + ((float)adc_buffer.batt1 * ((float)BAT_CALIB_REAL_VOLTAGE / (float)BAT_CALIB_ADC)) * 0.01;
      }

      //disable PWM when current limit is reached (current chopping)
      if(ABS((adc_buffer.dcl - offsetdcl) * MOTOR_AMP_CONV_DC_AMP) > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        LEFT_TIM->BDTR &= ~TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
      } else {
        LEFT_TIM->BDTR |= TIM_BDTR_MOE;
        //HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
      }

      if(ABS((adc_buffer.dcr - offsetdcr) * MOTOR_AMP_CONV_DC_AMP)  > DC_CUR_LIMIT || timeout > TIMEOUT || enable == 0) {
        RIGHT_TIM->BDTR &= ~TIM_BDTR_MOE;
      } else {
        RIGHT_TIM->BDTR |= TIM_BDTR_MOE;
      }

      int ul, vl, wl;
      int ur, vr, wr;

      //determine next position based on hall sensors
      uint8_t hall_ul = !(LEFT_HALL_U_PORT->IDR & LEFT_HALL_U_PIN);
      uint8_t hall_vl = !(LEFT_HALL_V_PORT->IDR & LEFT_HALL_V_PIN);
      uint8_t hall_wl = !(LEFT_HALL_W_PORT->IDR & LEFT_HALL_W_PIN);

      uint8_t hall_ur = !(RIGHT_HALL_U_PORT->IDR & RIGHT_HALL_U_PIN);
      uint8_t hall_vr = !(RIGHT_HALL_V_PORT->IDR & RIGHT_HALL_V_PIN);
      uint8_t hall_wr = !(RIGHT_HALL_W_PORT->IDR & RIGHT_HALL_W_PIN);

      uint8_t halll = hall_ul * 1 + hall_vl * 2 + hall_wl * 4;
      posl          = hall_to_pos[halll];
      posl += 2;
      posl %= 6;

      uint8_t hallr = hall_ur * 1 + hall_vr * 2 + hall_wr * 4;
      posr          = hall_to_pos[hallr];
      posr += 2;
      posr %= 6;

      blockPhaseCurrent(posl, adc_buffer.rl1 - offsetrl1, adc_buffer.rl2 - offsetrl2, &curl);

      //setScopeChannel(2, (adc_buffer.rl1 - offsetrl1) / 8);
      //setScopeChannel(3, (adc_buffer.rl2 - offsetrl2) / 8);


      // uint8_t buzz(uint16_t *notes, uint32_t len){
        // static uint32_t counter = 0;
        // static uint32_t timer = 0;
        // if(len == 0){
            // return(0);
        // }
        
        // struct {
            // uint16_t freq : 4;
            // uint16_t volume : 4;
            // uint16_t time : 8;
        // } note = notes[counter];
        
        // if(timer / 500 == note.time){
            // timer = 0;
            // counter++;
        // }
        
        // if(counter == len){
            // counter = 0;
        // }

        // timer++;
        // return(note.freq);
      // }


      //create square wave for buzzer
      buzzerTimer++;
      if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) {
        if (buzzerTimer % buzzerFreq == 0) {
          HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
        }
      } else {
          HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, 0);
      }

      //update PWM channels based on position
      blockPWM(pwml, posl, &ul, &vl, &wl);
      blockPWM(pwmr, posr, &ur, &vr, &wr);

      int weakul, weakvl, weakwl;
      if (pwml > 0) {
        blockPWM(weakl, (posl+5) % 6, &weakul, &weakvl, &weakwl);
      } else {
        blockPWM(-weakl, (posl+1) % 6, &weakul, &weakvl, &weakwl);
      }
      ul += weakul;
      vl += weakvl;
      wl += weakwl;

      int weakur, weakvr, weakwr;
      if (pwmr > 0) {
        blockPWM(weakr, (posr+5) % 6, &weakur, &weakvr, &weakwr);
      } else {
        blockPWM(-weakr, (posr+1) % 6, &weakur, &weakvr, &weakwr);
      }
      ur += weakur;
      vr += weakvr;
      wr += weakwr;

      LEFT_TIM->LEFT_TIM_U = CLAMP(ul + pwm_res / 2, 10, pwm_res-10);
      LEFT_TIM->LEFT_TIM_V = CLAMP(vl + pwm_res / 2, 10, pwm_res-10);
      LEFT_TIM->LEFT_TIM_W = CLAMP(wl + pwm_res / 2, 10, pwm_res-10);

      RIGHT_TIM->RIGHT_TIM_U = CLAMP(ur + pwm_res / 2, 10, pwm_res-10);
      RIGHT_TIM->RIGHT_TIM_V = CLAMP(vr + pwm_res / 2, 10, pwm_res-10);
      RIGHT_TIM->RIGHT_TIM_W = CLAMP(wr + pwm_res / 2, 10, pwm_res-10);
    }


#endif 

