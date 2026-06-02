#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"         
#include "driver/gpio.h"
//#include "esp_wifi.h"
//#include "esp_http_client.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define I2C_MASTER_SDA_IO           1 
#define I2C_MASTER_SCL_IO           2          
#define LCD_ADDR                    0x27 
#define temp_ADDR                   0x18 
#define enc_ADDR 					0x0C

#define LCD_CMD_CLEAR_DISPLAY       0x01
#define LCD_CMD_RETURN_HOME         0x02
#define LCD_CMD_ENTRY_MODE_SET      0x04
#define LCD_CMD_DISPLAY_CONTROL     0x08
#define LCD_CMD_FUNCTION_SET        0x20
#define LCD_CMD_SET_DDRAM_ADDR      0x80

#define LCD_DISPLAY_ON              0x04
#define LCD_BACKLIGHT               0x08
#define LCD_ENABLE_BIT              0x04 
#define LCD_RS_BIT                  0x01

#define TEMP_REG              0x05

#define VIBRATION_PIN               8   

#define SCAN_STEPS         140
#define VERT_STEPS         25
#define LINES_PER_FRAME    16

#define SCAN_DELAY_MS      10
#define FLYBACK_DELAY_MS   10
#define VERT_DELAY_MS      30

#define H_FLYBACK_CUSHION  0
#define V_RESET_CUSHION    (4*VERT_STEPS)
#define V_BACKLASH_COMP	   120      

void lcd_send_nibble(uint8_t, uint8_t);
void lcd_send_byte(uint8_t, uint8_t);
void lcd_put_char(char);
void lcd_put_str(const char*);
void lcd_set_cursor(uint8_t, uint8_t);
void lcd_init();
void stepper_init(int*);
void stepper_run(int*, int*, int);
void LCD_task(void*);
void temp_task(void*);
void vibration_task(void*);  
void stepper_task(void*);
void ADC_task(void*);
void post_task(void*);
                 
static i2c_master_dev_handle_t lcd_handle;
static i2c_master_dev_handle_t temp_handle;
static i2c_master_dev_handle_t encoder_handle;
static QueueHandle_t temp_queue;
static SemaphoreHandle_t vibration_semaphore;   
static SemaphoreHandle_t lcd_mutex;            

int h_pins[4] = {4, 5, 6, 7};
int v_pins[4] = {15, 16, 17, 18};

const int seq[8][4] = {
    {1,0,0,0},
    {1,1,0,0},
    {0,1,0,0},
    {0,1,1,0},
    {0,0,1,0},
    {0,0,1,1},
    {0,0,0,1},
    {1,0,0,1}
};

int h_step = 0;
int v_step = 0;

uint16_t raw_angle = 0;

static void IRAM_ATTR vibration_isr_handler(void *arg) {
    BaseType_t higher_priority_woken = pdFALSE;
    xSemaphoreGiveFromISR(vibration_semaphore, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
}

void app_main(void){
	i2c_master_bus_handle_t bus_handle;
	
	i2c_master_bus_config_t i2c_bus_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = I2C_NUM_0,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.flags.enable_internal_pullup = true
	};

	i2c_device_config_t lcd_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = LCD_ADDR,
		.scl_speed_hz = 100000,
	};

	i2c_device_config_t temp_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address = temp_ADDR,
		.scl_speed_hz = 100000, 
	};
	
	i2c_device_config_t encoder_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = enc_ADDR,
		.scl_speed_hz = 100000,
	};

	i2c_new_master_bus(&i2c_bus_config, &bus_handle);
	i2c_master_bus_reset(bus_handle);
	i2c_master_bus_add_device(bus_handle, &lcd_cfg, &lcd_handle);
	i2c_master_bus_add_device(bus_handle, &temp_cfg, &temp_handle);
	i2c_master_bus_add_device(bus_handle, &encoder_cfg, &encoder_handle);
	
	temp_queue = xQueueCreate(5, sizeof(float));


    vibration_semaphore = xSemaphoreCreateBinary();
    lcd_mutex           = xSemaphoreCreateMutex();

    gpio_config_t vib_cfg = {
        .pin_bit_mask = (1ULL << VIBRATION_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&vib_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(VIBRATION_PIN, vibration_isr_handler, NULL);
	
	stepper_init(h_pins);
	stepper_init(v_pins);
	
	xTaskCreate(LCD_task,       "show",      4096, NULL, 1, NULL);
	xTaskCreate(temp_task,      "measure",   4096, NULL, 1, NULL);
	xTaskCreate(stepper_task,   "move",      4096, NULL, 3, NULL);
    xTaskCreate(vibration_task, "vibration", 2048, NULL, 2, NULL);
	xTaskCreate(ADC_task, "IR", 2048, NULL, 3, NULL); 
	xTaskCreate(post_task, "send", 2048, NULL, 1, NULL); 
}

void lcd_send_nibble(uint8_t nibble, uint8_t mode){
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    i2c_master_transmit(lcd_handle, (uint8_t[]){data | LCD_ENABLE_BIT}, 1, 50);
    i2c_master_transmit(lcd_handle, (uint8_t[]){data & ~LCD_ENABLE_BIT}, 1, 50);
}

void lcd_send_byte(uint8_t val, uint8_t mode){
    lcd_send_nibble(val & 0xF0, mode);
    lcd_send_nibble((val << 4) & 0xF0, mode);
}

void lcd_put_char(char c){
    lcd_send_byte(c, LCD_RS_BIT);
}

void lcd_put_str(const char *str){
    while (*str) lcd_put_char(*str++);
}

void lcd_set_cursor(uint8_t row, uint8_t col){
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_send_byte(LCD_CMD_SET_DDRAM_ADDR | (col + row_offsets[row]), 0);
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50)); 
    lcd_send_nibble(0x30, 0);
    esp_rom_delay_us(5000);
    lcd_send_nibble(0x30, 0);
    esp_rom_delay_us(1000);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x20, 0); 

    lcd_send_byte(LCD_CMD_FUNCTION_SET | 0x08, 0); 
    lcd_send_byte(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON, 0);
	lcd_send_byte(LCD_CMD_ENTRY_MODE_SET | 0x02, 0);
    lcd_send_byte(LCD_CMD_CLEAR_DISPLAY, 0);
    esp_rom_delay_us(2000);
}

void stepper_init(int *pins){
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
}

void stepper_run(int *pins, int *step_index, int dir){
	*step_index += dir;

	 if (*step_index >= 8) *step_index = 0;
	 if (*step_index < 0) *step_index = 7;

	 for (int i = 0; i < 4; i++) {
	     gpio_set_level(pins[i], seq[*step_index][i]);
	 }
}

void temp_task(void *pvParameter){
	uint8_t write_buffer[1] = {TEMP_REG};
	uint8_t read_buffer[2];
	
	float temp;
	
	while (1) {   
		i2c_master_transmit_receive(temp_handle, write_buffer, 1, read_buffer, 2, 1000);
		
		uint8_t upper_byte = read_buffer[0];
		uint8_t lower_byte = read_buffer[1];
		
		upper_byte = upper_byte & 0x0F;

		temp = (upper_byte * 16.0) + (lower_byte / 16.0);

		xQueueSend(temp_queue, &temp, portMAX_DELAY);
		
	    vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}

void LCD_task(void *pvParameters){
	float val;
	char display_buffer[16];

	lcd_init();

    xSemaphoreTake(lcd_mutex, portMAX_DELAY);
	lcd_set_cursor(0, 0);
	lcd_put_str("Temperature:");
	lcd_set_cursor(1, 0);
	lcd_put_str("        ");   
	lcd_put_char(0xDF);   
	lcd_put_char('C');
    xSemaphoreGive(lcd_mutex); 

	while (1) {
		if (xQueueReceive(temp_queue, &val, portMAX_DELAY) == pdTRUE) {
			snprintf(display_buffer, sizeof(display_buffer), "%5.4f", val);

            xSemaphoreTake(lcd_mutex, portMAX_DELAY);
			lcd_set_cursor(1, 0);
			lcd_put_str(display_buffer);
            xSemaphoreGive(lcd_mutex);
		}
	}
}

void vibration_task(void *pvParameters){
    while (1) {
        if (xSemaphoreTake(vibration_semaphore, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(0, 13);
            lcd_put_str("VIB");
            xSemaphoreGive(lcd_mutex);

            vTaskDelay(pdMS_TO_TICKS(500));

            xSemaphoreTake(lcd_mutex, portMAX_DELAY);
            lcd_set_cursor(0, 13);
            lcd_put_str("   ");
            xSemaphoreGive(lcd_mutex);
        }
    }
    vTaskDelete(NULL);
}

void stepper_task(void *pvParameters) {
	
	while (1) {
		for (int line = 0; line < LINES_PER_FRAME; line++) {

		            // =====================
		            // FORWARD SCAN
		            // =====================
		            for (int x = 0; x < SCAN_STEPS; x++) {
		                stepper_run(h_pins, &h_step, +1);
		                vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
		            }

		            vTaskDelay(pdMS_TO_TICKS(50));

		            // =====================
		            // FLYBACK (with cushion)
		            // =====================
		            for (int x = 0; x < SCAN_STEPS + H_FLYBACK_CUSHION; x++) {
		                stepper_run(h_pins, &h_step, -1);
		                vTaskDelay(pdMS_TO_TICKS(FLYBACK_DELAY_MS));
		            }

		            vTaskDelay(pdMS_TO_TICKS(50));

		            // =====================
		            // VERTICAL STEP DOWN
		            // =====================
		            for (int y = 0; y < VERT_STEPS; y++) {
		                stepper_run(v_pins, &v_step, +1);
		                vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
		            }

		            vTaskDelay(pdMS_TO_TICKS(100));
		        }

		        // =====================
		        // VERTICAL RESET (with cushion + backlash compensation)
		        // =====================
		        for (int i = 0; i < (VERT_STEPS * LINES_PER_FRAME) + V_RESET_CUSHION; i++) {
		            stepper_run(v_pins, &v_step, -1);
		            vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
		        }
			

			for (int i = 0; i < V_BACKLASH_COMP; i++) {
			    stepper_run(v_pins, &v_step, +1);
			    vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
			}

		        vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}

void ADC_task(void *pvParameters){
	
}

void post_task(void *pvParameters){
	
}

void encoder_task(void *pvParameters){

		char display_buffer[16];

		const uint8_t write_buffer[1] = {enc_ADDR};
	    uint8_t data[2] = {0, 0};
	    while(1){
		i2c_master_transmit_receive( encoder_handle, write_buffer, 1,  data, 2,  100);
		raw_angle = (data[0] << 8) | data[1];

		float fraw_angle = ((float)raw_angle * 360)/4096;

		snprintf(display_buffer, sizeof(display_buffer), "%.2f", fraw_angle);
		lcd_set_cursor(0,0);
		lcd_put_str("                ");
		lcd_set_cursor(0, 0);
		lcd_put_str(display_buffer);

		vTaskDelay(pdMS_TO_TICKS(100));
		}
}


