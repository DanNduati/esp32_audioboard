#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp_adc_cal.h"
#endif

#define input_touch_1 26 //bluetooth on/off when held down for a preset duration
//when button 2 and 3 pressed together clear all paired devices.
#define input_touch_2 25 //volume up
#define input_touch_3 33 //volume down
#define GPIO_INPUT_PIN_SEL  ((1ULL<<input_touch_1) | (1ULL<<input_touch_2) | (1ULL<<input_touch_3))
#define output_led_1 27
#define output_led_2 13
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<output_led_1) | (1ULL<<output_led_2))
#define ESP_INTR_FLAG_DEFAULT 0
#define debounceTimeout 50 
#define btn1_longpress_time 500 //long_button_press_threshold

volatile uint32_t intr_ticks = 0;
uint32_t last_tick = 0; //global no of ticks elapsed 

//btn 1 press duration variables
uint32_t start_pressed=0;
uint32_t end_pressed = 0;
uint32_t press_duration =0;

static xQueueHandle gpio_vol_up_evt_queue = NULL;
static xQueueHandle gpio_vol_down_evt_queue = NULL;
static xQueueHandle gpio_bt_state_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    intr_ticks = xTaskGetTickCount();
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_num == 25){
        xQueueSendFromISR(gpio_vol_up_evt_queue, &gpio_num, NULL);
    }
    else if(gpio_num == 33){
        xQueueSendFromISR(gpio_vol_down_evt_queue, &gpio_num, NULL);
    }
    else{
        //anyedge btn1
        xQueueSendFromISR(gpio_bt_state_evt_queue, &gpio_num, NULL);
    }
}

static char tag[] = "audio_board";
static const adc_channel_t channel = ADC_CHANNEL_6;
static const adc_atten_t atten = ADC_ATTEN_DB_6;

void config_adc(){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel,atten);
}

static void volume_up(void* arg){
    uint32_t io_num;

    for(;;){

        if(xQueueReceive(gpio_vol_up_evt_queue, &io_num, portMAX_DELAY) && (intr_ticks-last_tick)>debounceTimeout) {
            //check for debounce time
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            printf("Button 2 pressed Volume up\n");
        }
        last_tick = intr_ticks;
    }
}

static void volume_down(void* arg){
    uint32_t io_num;
    for(;;){
        if(xQueueReceive(gpio_vol_down_evt_queue, &io_num, portMAX_DELAY) && (intr_ticks-last_tick)>debounceTimeout) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            printf("Button 3 pressed Volume down\n");

        }
        last_tick = intr_ticks;
    }
}

static void set_bt_state(void* arg){
    //check bt state flag
    //compliment(if on turn of and vice versa) the bt state on each long btn1 press
    uint32_t io_num;
    for(;;){
        if(xQueueReceive(gpio_bt_state_evt_queue, &io_num, portMAX_DELAY)){
            printf("Btn 1 pressed/released");
        }
        last_tick = intr_ticks;
    }
}

void config_gpios(){
    //Indicator leds
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set ie:GPIO27/13
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull up and pull down mode
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO26/25/33 
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);
    //change gpio interrupt type for the btn1
    gpio_set_intr_type(input_touch_1, GPIO_INTR_ANYEDGE);

    //create a queue to handle volume up event from isr
    gpio_vol_up_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //create a queue to handle volume down event from isr
    gpio_vol_down_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //create a queue to handle bt on/off from isr
    gpio_bt_state_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio tasks
    xTaskCreate(volume_up, "volume_up", 2048, NULL, 10, NULL);
    xTaskCreate(volume_down, "volume_up", 2048, NULL, 10, NULL);
    xTaskCreate(set_bt_state, "set_bt_state", 2048, NULL, 10, NULL);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler to btn1 gpio pin for bt on and off
    gpio_isr_handler_add(input_touch_1, gpio_isr_handler, (void*) input_touch_1);
    //hook isr handler to volume up gpio pin
    gpio_isr_handler_add(input_touch_2, gpio_isr_handler, (void*) input_touch_2);
    //hook isr handler to volume down gpio pin
    gpio_isr_handler_add(input_touch_3, gpio_isr_handler, (void*) input_touch_3);
}
//pooling the current sense amplifier
uint32_t get_vnsns_value(void){
    uint32_t val = 0;
    val = adc1_get_raw((adc1_channel_t)channel);
    return val;
}


void app_main(void)
{
    ESP_LOGI(tag, "ADC test");
    config_gpios();
    config_adc();
    while (1) {
        uint32_t value = get_vnsns_value();
        printf("The vsns value is:%d\n",value);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
