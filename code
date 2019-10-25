#include <stdio.h>
#include <time.h>
#include <math.h>

#include "SRA18.h"
#include "MPU.h"
#include "TUNING.h"

#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "SRA18.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"

#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"

#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"

#define ENCODER_PHASE_A_1     22
#define ENCODER_PHASE_B_1     23
#define ENCODER_PHASE_A_0     18
#define ENCODER_PHASE_B_0     19


#define ESP_INTR_FLAG_DEFAULT 0


static volatile bool dir_0 = 0;
static volatile int16_t count_0 = 0;
static volatile bool dir_1 = 0;
static volatile int16_t count_1 = 0;
static xQueueHandle gpio_evt_queue_0 = NULL;
static xQueueHandle gpio_evt_queue_1 = NULL;
static gpio_config_t io_conf;
float r,l,c;
float x=0;
float y=0;
float w=0;
int node=0;
int left=1;
int forward=2;
int right=3;
int backward=4;
int direction[50];
int path = 0;
int checkend = 0;
int g = 0;

int z = 0;
void bot_break(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle1, float duty_cycle2)
{
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle1);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle2);
    gpio_set_level(GPIO_NUM0,1);
    gpio_set_level(GPIO_NUM1,1);
    gpio_set_level(GPIO_NUM2,1);
    gpio_set_level(GPIO_NUM3,1);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
}
static void config_isr(int arg)
{
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << arg);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

static void config_isr_neg(int arg)
{
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1 << arg);
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}


static void config_input(int arg)
{
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << arg);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
}

static void config_output(int arg)
{
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << arg);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);    
}

static volatile unsigned int isrcalls=0;
static volatile unsigned int isrcalls2=0;
static void IRAM_ATTR gpio_isr_handler_0(void* arg)
{
    isrcalls++;
    if(gpio_get_level(ENCODER_PHASE_B_0) == 0)
    {
        count_0++;
    }

    else if(gpio_get_level(ENCODER_PHASE_B_0) == 1)
    {
        count_0--;
    }
}

static void IRAM_ATTR gpio_isr_handler_1(void* arg)
{
    isrcalls2++;

    if(gpio_get_level(ENCODER_PHASE_B_1) == 0)
    {
        count_1++;
    }

    else if(gpio_get_level(ENCODER_PHASE_B_1) == 1)
    {
        count_1--;
    }
}



adc1_channel_t channel[6] = {ADC_CHANNEL_7, ADC_CHANNEL_6, ADC_CHANNEL_0, ADC_CHANNEL_3, ADC_CHANNEL_4,ADC_CHANNEL_5};
adc2_channel_t channel2[4] = {ADC2_CHANNEL_4,ADC2_CHANNEL_3,ADC2_CHANNEL_0,ADC2_CHANNEL_2};


int weights[4] = {3,1,-1,-3};

/*
 * Line Following PID Constants
 */
#define kP 1.3
#define kI 0
#define kD 1.9

/*
 * Motor value constraints
 */
float opt = 70;
float lower_pwm_constrain = 60;
float higher_pwm_constrain = 80;
float left_pwm = 0, right_pwm = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error, difference, cumulative_error, correction;

uint32_t adc_reading[6];
float sensor_value[6];
int adc2_reading[4];
int sensor_value2[4];

static void read_sensors()
{
  for(int i = 0; i < 6; i++)
    {
        adc_reading[i] = adc1_get_raw(channel[i]);
    }
    for(int i=0; i<4 ; i++)
   {
        adc2_config_channel_atten(channel2[i],ADC_ATTEN_DB_6);
        adc2_get_raw(channel2[i],ADC_WIDTH_BIT_12,&adc2_reading[i]);
   }
}

static void calc_sensor_values()
{
    for(int i = 0; i < 6; i++)
    {
      adc_reading[i] = adc1_get_raw(channel[i]);
      sensor_value[i] = constrain(map(adc_reading[i], 1736, 4180, 0, 1000),0,1000);
      //printf("%f\t",sensor_value[i]);
    }
    for(int i=0;i<4;i++)
        {
            adc2_config_channel_atten(channel2[i],ADC_ATTEN_DB_6);
            adc2_get_raw(channel2[i],ADC_WIDTH_BIT_12,&adc2_reading[i]);
            sensor_value2[i] = constrain(map(adc2_reading[i], 1736, 4180, 1000,0),0,1000);
            //printf("%d\t",sensor_value2[i]);
        }
//    printf("\n");

}

static void calculate_error()
{
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0, pos = 0;
    
    for(int i = 0; i < 4; i++)
    {
        if(sensor_value[i] > 400)
        {
            all_black_flag = 0;
        }

        weighted_sum += (float)(sensor_value[i]) * (weights[i]);
        sum += sensor_value[i];
        
    }
    
    if(sum != 0)
    {
        pos = weighted_sum / sum;
    }

    if(all_black_flag == 1)
    {
        if(error > 0)
            pos = 2.5;
        else
            pos = -2.5;
    }

    error = pos;

}

static void calculate_correction()
{
    error *= 10;
    difference = (error - prev_error);
    cumulative_error += error;
    
    if(cumulative_error > 30)
    {
        cumulative_error = 30;
    }
    
    else if(cumulative_error < -30)
    {
        cumulative_error = -30;
    }

    correction = kP*error + kI*cumulative_error + kD*difference;
    prev_error = error;
}

void mazerun()
 {
    printf("hello satyam\n");
  if ((sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400 && sensor_value[4]>400 && sensor_value[5]>400) ||
         (sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[4]>400 && sensor_value[5]<400) || 
          (sensor_value[1]>400 && sensor_value[3]>400 && sensor_value[2]>400 && sensor_value[4]<400 && sensor_value[5]>400 && sensor_value2[0]<500 && sensor_value2[1]<400 && sensor_value2[2]<400 && sensor_value2[3]<400 ) || (sensor_value[1]>400 && sensor_value[3]>400 && sensor_value[2]>400 && sensor_value[4]<400 && sensor_value[5]>400 ))
  {
      path++;
        printf("%d\n",direction[path] );
    
      switch ( direction[path] )
      {
        case 1 :
                vTaskDelay(20);
        bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
        vTaskDelay(100);
        int q,s;
        q =1 ;
       // printf(" ,  %d , %d , \n",count_0,count_1);
    while(q)
    {
      //printf("Turn Loop left 21\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]<400 && sensor_value[1]<400 )    
    {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        q=0;
        s=1;
        break;
    }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
      while(s)
    {
      //printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]>400 && sensor_value[1]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        // node++;
        // direction[node]=direction[node]+left;
        printf("%d\n",direction[node]); 
        s=0;
        break;
        }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);

    }

        case 2 :
            for (int i = 0; i < 1000; ++i)
            {
                read_sensors();
                calc_sensor_values();
                calculate_error();
                //sensor();
                calculate_correction();
                left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
                //printf(" %2f\t",left_pwm);

                right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
            //printf(" %2f\t",right_pwm);
                bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
            }
            break;


        case 3 :
        vTaskDelay(20);
    bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
        vTaskDelay(100);
        read_sensors();
        calc_sensor_values();
       printf("spot right\n");
        q=1;
        while(q)
    {
     // printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      //printf("loop spot right\n");
      if(sensor_value[2]>400 && sensor_value[3]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        q=0;
        // node++;
        // direction[node]=direction[node]+right;
        break;
        }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }


      }

      
  }
 }

 void finalrun()
 {
  while (1)
  {
   read_sensors();
   calc_sensor_values();
   calculate_error();
   //sensor();
   calculate_correction();
    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    printf(" %2f\t",left_pwm);

    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    printf(" %2f\t",right_pwm);
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
    mazerun();
    printf("\n");
  }
 }


void maze_solve()
{
if(node==0)
  { 
    direction[node]=forward;
  }
    if(sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400 && sensor_value[4]>400 && sensor_value[5]>400 && (sensor_value2[0]<500 || sensor_value2[1]<400))
    {
        vTaskDelay(15);
        bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
        vTaskDelay(100);
        int q,s;
        q=1;
       // printf(" ,  %d , %d , \n",count_0,count_1);
    while(q)
    {
      //printf("Turn Loop left 21\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]<400 && sensor_value[1]<400 )    
    {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        q=0;
        s=1;
        break;
    }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
      while(s)
    {
      //printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]>400 && sensor_value[1]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        node++;
        direction[node]=direction[node]+left;
        printf("%d\n",direction[node]); 
        s=0;
        break;
        }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);

    }
   }
if(sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[4]>400 && sensor_value[5]<400)
{
    vTaskDelay(15);
    bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
        vTaskDelay(100);
        int q,s;
        q=1;
      //  printf("spot left\n");
    while(q)
    {
      //printf("stop left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]<400 && sensor_value[1]<400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        q=0;
        s=1;
        break;
        }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
        while(s)
    {
     // printf("spot left\n");
      read_sensors();
      calc_sensor_values();
      if(sensor_value[0]>400 && sensor_value[1]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        node++;
        direction[node]=direction[node]+left;
        s=0;
        break;
        }
      bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);

    }

}
if ((sensor_value[2]>400 && sensor_value[3]>400 && sensor_value[5]>500) && (sensor_value2[3] > 400 || sensor_value2[3] > 400) && (sensor_value2[1]<400 || sensor_value2[2] < 400) )

{
    if(z < node)
    {node ++;
        z = node;
        direction[node] = direction[node]+forward;
}
    }
    
if(sensor_value[3]>400 && sensor_value[2]>400 && sensor_value[4]<400 && sensor_value[5]>400 && sensor_value2[0]<500 && sensor_value2[1]<400 && sensor_value2[2]<400 && sensor_value2[3]<400)
{
    vTaskDelay(15);
    bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
        vTaskDelay(100);
        read_sensors();
        calc_sensor_values();
       // printf("spot right\n");
        int q=1;
        while(q)
    {
     // printf("Turn Loop left\n");
      read_sensors();
      calc_sensor_values();
      //printf("loop spot right\n");
      if(sensor_value[2]>400 && sensor_value[3]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(50);
        q=0;
        node++;
        direction[node]=direction[node]+right;
        break;
        }
      bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
    }
}
if(sensor_value[0]<400 && sensor_value[1]<400 && sensor_value[2]<400 && sensor_value[3]<400 && sensor_value[4]<400 && sensor_value[5]<400 && sensor_value2[0]<500 && sensor_value2[1]<400 && sensor_value2[2]<400 && sensor_value2[3]<400)
 {
    vTaskDelay(15);
    //printf("dead end\n");
   // encoder();
    bot_break(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
    vTaskDelay(100);
    int q;
    q=1;
        
    while(q)
    {
     // printf("Turn Loop left dead end\n");
      read_sensors();
      calc_sensor_values();
      //encoder();
      if(sensor_value[2]>400 && sensor_value[3]>400 )    
        {
        bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
        vTaskDelay(5);
        q=0;
        node--;
        break;
    }
    bot_spot_left(MCPWM_UNIT_0,MCPWM_TIMER_0,68,68);
 }
 }
 if(sensor_value[0]>400 && sensor_value[1]>400 && sensor_value[2]>400 && sensor_value[3]>400 && sensor_value[4]>400 && sensor_value[5]>400 && sensor_value2[0]>400 && sensor_value2[1]>400 && sensor_value2[2]>400 && sensor_value2[3]>400)
    {
    bot_break(MCPWM_UNIT_0 , MCPWM_TIMER_0,80,80);
     gpio_set_direction(LED_1,GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_2,GPIO_MODE_OUTPUT); 
    int l = 0;
    while(l<5)
    {

        gpio_set_level(LED_1,0);    //Set LED1 ON
        gpio_set_level(LED_2,0);    //Set LED2 ON

        vTaskDelay(1000 / 10);  //Wait for 1000ms
        


        gpio_set_level(LED_1,1);    //Set LED1 OFF
        gpio_set_level(LED_2,1);    //Set LED2 OFF

        vTaskDelay(1000 / 10); 
        l++;
         //Wait for 1000ms
    // if (pressed_switch(BUTTON_2))
    //     {
    //         finalrun();
    //     }
    }
    g = 1;
    }
}




void line_follow_task(void *arg)
{

  mcpwm_initialize();

  while(1)
  {
    if ( g == 1)
    {
        printf("%d\n",g);
        //vTaskDelay(1000);
        finalrun();
    }
    read_sensors();
    calc_sensor_values();
    calculate_error();
    calculate_correction();
    maze_solve();
  //  printf(" ,  %d , %d , \n",count_0,count_1 );
    left_pwm = constrain((opt - correction), lower_pwm_constrain, higher_pwm_constrain);
    right_pwm = constrain((opt + correction), lower_pwm_constrain, higher_pwm_constrain);
    bot_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, left_pwm, right_pwm);
    printf("node\n %d" ,node);
    for(int i=0;i<=node;i++)
        {
            printf("%d\t",direction[i]);
        }
    printf("\n");

    //bot_spot_right(MCPWM_UNIT_0,MCPWM_TIMER_0,80,80);
}

}

void app_main()
{
    config_isr(ENCODER_PHASE_A_0);
    config_isr(ENCODER_PHASE_A_1);
    
  
    config_input(ENCODER_PHASE_B_0);
    config_input(ENCODER_PHASE_B_1);
    xTaskCreate(&line_follow_task,"line_follow_task",100000,NULL,1,NULL);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENCODER_PHASE_A_0, gpio_isr_handler_0, (void*) ENCODER_PHASE_B_0);
    gpio_isr_handler_add(ENCODER_PHASE_A_1, gpio_isr_handler_1, (void*) ENCODER_PHASE_B_1);
    enable_buttons();
    for (int i = 0; i < 50; i++)
    {

        direction[i]=0;
    }

}
