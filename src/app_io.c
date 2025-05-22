#include "app_io.h"
#include "ble.h"
#include "main.h"
#include "ems_pwm.h"

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/policy.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

LOG_MODULE_REGISTER(APP_IO, LOG_LEVEL_DBG);

#define BLUE_LED_OUT_NODE                   DT_ALIAS(blue_led)       // blue LED node
#define GREEN_LED_OUT_NODE                  DT_ALIAS(green_led) 
#define RED_LED_OUT_NODE                    DT_ALIAS(red_led)        // red LED node


#define SW0_NODE	                    DT_ALIAS(on_button)           // Button node

#define GPIO_PIN                        NRF_GPIO_PIN_MAP(0, 5)
#define GPIO_PIN1                       NRF_GPIO_PIN_MAP(0, 25)

#define STACK_SIZE                            512
#define MY_STACK_SIZE                         500
#define THREAD_PRIORITY                       4

#define PWM_PERIOD   PWM_MSEC(60)

#define PWM_MIN_DUTY_CYCLE 20000000
#define PWM_MAX_DUTY_CYCLE 50000000

#define ADC_THRESHOLD                   70  // Threshold for waking up based on ADC value difference
#define ARRAY_SIZE                      10   // Size of the ADC value array

#define SLEEP_DURATION_MS               5000 // Duration of sleep in milliseconds

#define TIME_WINDOW_MS     10000  // 10-second time window

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

// ADC spec macro
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),


struct gpio_dt_spec red_led_spec = GPIO_DT_SPEC_GET(RED_LED_OUT_NODE, gpios); // Red LED spec

struct gpio_dt_spec green_led_spec = GPIO_DT_SPEC_GET(GREEN_LED_OUT_NODE, gpios); // Red LED spec

struct gpio_dt_spec blue_led_spec = GPIO_DT_SPEC_GET(BLUE_LED_OUT_NODE, gpios); // Red LED spec


extern struct k_msgq ble_msgq;  // BLE message queue

// Define the thread stack
K_THREAD_STACK_DEFINE(dyn_thread_stack_1, MY_STACK_SIZE);
// Define the thread control block
struct k_thread my_thread_data_1;

/* Define the timer object */
static struct k_timer my_timer;

static struct gpio_callback button_cb_data;

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0}); // Button spec

// ADC channels
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)
};

// Thread definitions
#define LED_BLINK_THREAD_STACK_SIZE 1024
#define LED_BLINK_THREAD_PRIORITY 5

K_THREAD_STACK_DEFINE(blue_led_stack_area, LED_BLINK_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(green_led_stack_area, LED_BLINK_THREAD_STACK_SIZE);

struct k_thread blue_led_thread_data;
struct k_thread green_led_thread_data;

// Shared data for controlling threads
static uint32_t pwm_period_blue = 1000000; // Default period for blue LED
static uint32_t pwm_period_green = 1000000;   // Default period for green LED

static bool blue_led_thread_running = false;
static bool green_led_thread_running = false;

// Mutexes to protect access to shared variables
static struct k_mutex blue_period_mutex;
static struct k_mutex green_period_mutex;

#define MIN_DUTY_CYCLE 0
#define MAX_DUTY_CYCLE 10000 // Maximum duty cycle in nanoseconds, adjust as needed
#define STEP_SIZE 100 // Step size for duty cycle increment/decrement, adjust for smoothness
#define STEP_DELAY_MS 10 // Delay between steps in milliseconds, adjust for speed of breathing effect
#define SOLID_BRIGHT_PERIOD 10000 // Set period for solid bright     // Maximum duty cycle for full brightness
#define ZERO_DUTY_CYCLE 0           // Duty cycle for turning off the LED


uint32_t btn_hold_counter = 0;
uint32_t ems_on = 0;
uint16_t raw_batt_val = 0;
int32_t batt_val_mv = 0;
int32_t initial_value = 0;
int32_t current_value = 0;
int32_t final_value = 0;
int charging_threshold;  // Charging threshold will be dynamically set
uint8_t charger_state = 0;   // Initialize the charger state
uint8_t battery_low = 0; 
uint16_t buf;
k_tid_t my_tid1;
k_tid_t my_tid2;
k_tid_t my_tid3;
uint8_t sleep_flag =0;
static int64_t start_time;
const k_tid_t red_led_thread; // LED thread

static int charge_count=0;
static int charge_count1=0;
static int charge=0;
static int discharge=0;

// ADC value array for storing readings
int adc_values_array[ARRAY_SIZE] = {0};
int adc_array_index = 0; // Index to keep track of the array position

volatile bool in_critical_section = false;

const k_tid_t led_thread; // LED thread

extern bool in_standby;

K_SEM_DEFINE(button_sem, 0, 1);

// Battery ADC sequence
struct adc_sequence batt_read_sequence = {
    .buffer = &raw_batt_val,
    .buffer_size = sizeof(raw_batt_val)
};

/* Callback function for the timer */
void timer_expiry_function(struct k_timer *timer_id)
{
    /* Optional: If you want to restart the process for another 30 seconds, restart the timer */
    k_timer_start(&my_timer, K_SECONDS(30), K_NO_WAIT);
}

// Button pressed callback function
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {

    k_sem_give(&button_sem);

	LOG_DBG("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

// Battery read initialization function
static int batt_read_init() {
    int ret = 0;

    if (!device_is_ready(adc_channels[0].dev)) {
        return -ENODEV;
    }

    ret = adc_channel_setup_dt(&adc_channels[0]);
    if (ret) {
        return ret;
    }
      // Initialize ADC sequence
    ret = adc_sequence_init_dt(&adc_channels[0], &batt_read_sequence);
    if (ret) {
          return;
    }

    return 0;
}

static void battery_update()
{
    int ret = 0;

    // Read ADC value
    ret = adc_read(adc_channels[0].dev, &batt_read_sequence);
    if (ret) {
        return;
    }

    batt_val_mv = (int32_t)raw_batt_val;
    ret = adc_raw_to_millivolts_dt(&adc_channels[0], &batt_val_mv);
    if (ret) {
        return;
    }

    /* Read the current battery value */
    current_value = batt_val_mv;
    //LOG_INF("BATTURY VALUE %u",current_value);

    // Set charging thresholds based on batt_val_mv
   
        if (batt_val_mv >= 2000) {
            charging_threshold = 8;
        } else if (batt_val_mv >= 1500 && batt_val_mv < 2000) {
            charging_threshold = 10;
        } else {
            charging_threshold = 12;
        } 

        if(is_pwm_modes_enabled == false)
        {

            if(battery_update_ready)
            {
                k_sleep(K_MSEC(2000)); // Check battery every 2 seconds
                battery_update_ready = false;
            }


        if((final_value - current_value >= 8) && (discharge ==1)){

            charge_count = 0;
            charge_count1 = charge_count1 + 100;

    
            if(charge_count1 > 400)
            {
               // LOG_INF("Button held for more than 9 seconds, rebooting");
    
                // nrf_gpio_pin_clear(GPIO_PIN);
                // nrf_gpio_pin_clear(GPIO_PIN1);
                //gpio_pin_set_raw(red_led_spec.port, red_led_spec.pin, 1);
                charger_state = 0;
                discharge=0;
                
            }
            //charger_state = 0;
            //initial_value = current_value;

        } 
        else if (current_value - initial_value >= charging_threshold) {
            //printk("Value changed from %d to %d with batt_val_mv: %d\n", initial_value, current_value, batt_val_mv);
             // Set charger state to 1
            charge_count1 = 0;
            sleep_flag = 1;
            charge_count = charge_count + 100;

            if(charge_count > 400)

            {
               // LOG_INF("Button held for more than 9 seconds, rebooting");
    
                // nrf_gpio_pin_clear(GPIO_PIN);
                // nrf_gpio_pin_clear(GPIO_PIN1);
                //gpio_pin_set_raw(red_led_spec.port, red_led_spec.pin, 0);
                final_value = batt_val_mv;
                charger_state = 1; 
                discharge =1;
            }
            //gpio_pin_set_raw(red_led_spec.port, red_led_spec.pin, 0);

        }  
        else {
            initial_value = current_value;
           // charger_state = 0;
            charge_count1 = 0;
            charge_count = 0;
            //gpio_pin_set_raw(red_led_spec.port, red_led_spec.pin, 1);
           // printk("Restarting timer with initial value: %d\n", initial_value);
        }
    }
        
    }
    
static void batt_read_update() // Battery read update
{   
    while (1) {

        battery_update();

        struct ble_msg batt_msg = {
            .id = BLE_MSG_ID_BATT_UPDATE,
            .data.batt_mv = batt_val_mv
        };
        k_msgq_put(&ble_msgq, &batt_msg, K_NO_WAIT);
    
        k_sleep(K_MSEC(1000)); // Check battery every 2 seconds
    }
}


static void red_led_main() {
    while (1) {
    int ret = gpio_pin_toggle_dt(&red_led_spec);
        k_sleep(K_MSEC(1000));
    }
}

static void green_led_main() {
    while (1) {
      int  ret = gpio_pin_toggle_dt(&green_led_spec);
        k_sleep(K_MSEC(1000));
    }
}

static void blue_led_main() {
    while (1) {
    int ret = gpio_pin_toggle_dt(&blue_led_spec);
        k_sleep(K_MSEC(1000));
    }
}

int input_main() {
    
    int err = 0;
    int ret;
    int val;
    int btn_hold_counter = 0;
    bool pattern_changed = false; // Flag to track if LED pattern is already changed
    const int hold_duration_threshold = 2000; // 2 seconds in milliseconds

    create_red_thread_from_function();
    
    create_green_thread_from_function();

    create_blue_thread_from_function();

    // Initialize the timer
    k_timer_init(&my_timer, timer_expiry_function, NULL);
    k_timer_start(&my_timer, K_SECONDS(30), K_NO_WAIT);

    // Battery initialization
    if (batt_read_init()) {
        return 1;
    }
    // Button initialization
    if (!gpio_is_ready_dt(&button)) {
        LOG_DBG("Error: button device %s is not ready\n", button.port->name);
        return 0;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    
    if (ret) {
        LOG_DBG("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
        return 0;
    }

    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    
    if (ret) {
        LOG_DBG("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
        return 0;
    }

    // Button callback
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    k_sleep(K_MSEC(100));

    initial_value = batt_val_mv;
    printk("Initial value: %d\n", initial_value);

   	if (!gpio_is_ready_dt(&red_led_spec)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&red_led_spec, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

    	if (!gpio_is_ready_dt(&green_led_spec)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&green_led_spec, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

    	if (!gpio_is_ready_dt(&blue_led_spec)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&blue_led_spec, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

    // Simulate some other operations
    k_sleep(K_SECONDS(1));
    
    // Main loop
    while (1) {
        // Check battery value
        
        if (batt_val_mv < 1600 ) {

            
            if(battery_low == 0){

            k_thread_suspend(my_tid2);
            k_thread_suspend(my_tid3);
            k_thread_start(my_tid1);
            
            battery_low=1;

            }
        }
        // Get button state
        val = gpio_pin_get_dt(&button);
        
        k_msleep(1);  // Sleep to avoid busy waiting

        // Button press handling
        if (val == 1) {

         
            btn_hold_counter++;  // Increment counter every ms while the button is held

            // If button has been held for more than 3 seconds, change the LED pattern
            if (btn_hold_counter >= hold_duration_threshold && !pattern_changed && pairing_on) {

                if(battery_low == 0){

                    k_thread_start(my_tid3);

                }
                
                LOG_INF("Button held for 3 seconds, changed to blinking");

                pattern_changed = true; // Set flag to indicate the pattern has changed
                
                if ((err = bt_le_adv_stop()) == 0) {
                LOG_INF("Advertising stopped successfully");
            }

            else {
                LOG_INF("Cannot stop advertising, err = %d", err);
            }

            if ((err = bt_le_filter_accept_list_clear()) == 0) {
                LOG_INF("Accept list cleared successfully");

                ble_new_adv();

                if ((err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0)) == 0) {
                    LOG_INF("Advertising in pairing mode started");
                } else {
                    LOG_INF("Cannot start advertising, err = %d", err);
                }
            }

            else {
                LOG_INF("Cannot clear accept list, err = %d", err);
            }

            }

        }
        
        else {
            // Reset the counter and pattern_changed flag when the button is released
            btn_hold_counter = 0;
            pattern_changed = false;
        }

        // Handle longer button hold (e.g., reboot) if needed
        if (btn_hold_counter > 5000) {
            
            // stop_blue_led_thread();
            // stop_green_led_thread();
            // stop_red_led_thread();

            LOG_INF("Button held for more than 9 seconds, rebooting");

            nrf_gpio_pin_clear(GPIO_PIN);
            nrf_gpio_pin_clear(GPIO_PIN1);

        }
    }

    return 0;  // Return success
}


K_THREAD_DEFINE(input_thread, 1024, input_main, NULL, NULL, NULL, 2, K_ESSENTIAL, 0);

K_THREAD_DEFINE(batt_thread, 512, batt_read_update, NULL, NULL, NULL, 3, 0, 0);

void create_red_thread_from_function(void) {
    my_tid1 = k_thread_create(&my_thread_data_1, dyn_thread_stack_1,
                             K_THREAD_STACK_SIZEOF(dyn_thread_stack_1),
                             red_led_main,
                             NULL, NULL, NULL,
                             4, 0, K_FOREVER);
}

void create_green_thread_from_function(void) {
    my_tid2 = k_thread_create(&my_thread_data_1, dyn_thread_stack_1,
                             K_THREAD_STACK_SIZEOF(dyn_thread_stack_1),
                             green_led_main,
                             NULL, NULL, NULL,
                             5, 0, K_FOREVER);
}

void create_blue_thread_from_function(void) {
    my_tid3 = k_thread_create(&my_thread_data_1, dyn_thread_stack_1,
                             K_THREAD_STACK_SIZEOF(dyn_thread_stack_1),
                             blue_led_main,
                             NULL, NULL, NULL,
                             6, 0, K_FOREVER);
}