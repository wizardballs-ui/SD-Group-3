#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
//#include "esp_wifi.h"
//#include "esp_http_client.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define I2C_MASTER_SDA_IO           23 
#define I2C_MASTER_SCL_IO           22          
#define LCD_ADDR                    0x27 
#define ADC_ADDR                    0x48 

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

#define REG_POINTER_CONVERT         0x00
#define REG_POINTER_CONFIG          0x01

#define SCAN_STEPS         140
#define VERT_STEPS         30
#define LINES_PER_FRAME    10

#define SCAN_DELAY_MS      10
#define FLYBACK_DELAY_MS   10
#define VERT_DELAY_MS      30

#define H_FLYBACK_CUSHION  0     // horizontal overshoot correction
#define V_RESET_CUSHION    (3*VERT_STEPS)     // vertical return overshoot correction
#define V_BACKLASH_COMP	   120   // typically damp during the first 4 lines

void lcd_send_nibble(uint8_t, uint8_t);
void lcd_send_byte(uint8_t, uint8_t);
void lcd_put_char(char);
void lcd_put_str(const char*);
void lcd_set_cursor(uint8_t, uint8_t);
void lcd_init();
void stepper_init(int*);
void stepper_run(int*, int*, int);
void LCD_task(void*);
void ADC_task(void*);
void stepper_task(void*);

static i2c_master_dev_handle_t lcd_handle;
static i2c_master_dev_handle_t adc_handle;
static QueueHandle_t adc_queue;

int h_pins[4] = {32, 33, 25, 26};
int v_pins[4] = {21, 19, 18, 5};

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

	i2c_device_config_t adc_cfg = {
	    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
	    .device_address =ADC_ADDR,
		.scl_speed_hz = 100000, 
	};
	
	i2c_new_master_bus(&i2c_bus_config, &bus_handle);
	i2c_master_bus_reset(bus_handle);
	i2c_master_bus_add_device(bus_handle, &lcd_cfg, &lcd_handle);
	i2c_master_bus_add_device(bus_handle, &adc_cfg, &adc_handle);
	
	adc_queue = xQueueCreate(5, sizeof(int16_t));
	
	stepper_init(h_pins);
	stepper_init(v_pins);
	
	xTaskCreate(LCD_task, "show", 4096, NULL, 1, NULL);
	xTaskCreate(ADC_task, "measure", 4096, NULL, 1, NULL);
	xTaskCreate(stepper_task, "move", 4096, NULL, 3, NULL);
}

void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | LCD_BACKLIGHT;
    i2c_master_transmit(lcd_handle, (uint8_t[]){data | LCD_ENABLE_BIT}, 1, 50);
    i2c_master_transmit(lcd_handle, (uint8_t[]){data & ~LCD_ENABLE_BIT}, 1, 50);
}

void lcd_send_byte(uint8_t val, uint8_t mode) {
    lcd_send_nibble(val & 0xF0, mode);
    lcd_send_nibble((val << 4) & 0xF0, mode);
}

void lcd_put_char(char c) {
    lcd_send_byte(c, LCD_RS_BIT);
}

void lcd_put_str(const char *str) {
    while (*str) lcd_put_char(*str++);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
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

void ADC_task(void *pvParameter){
	uint8_t config_cmd[3] = {REG_POINTER_CONFIG, 0xC3, 0x83};
	uint8_t read_cmd[1] = {REG_POINTER_CONVERT};
	uint8_t adc_buffer[2];
	    
	int16_t raw_adc;
	int16_t voltage_mv;
	
	int16_t TESTERATOR = 5;
	
	while (1) {
	    /*    
		i2c_master_transmit(adc_handle, config_cmd, 3, 1000);   
	    vTaskDelay(pdMS_TO_TICKS(10));
  
	    i2c_master_transmit_receive(adc_handle, read_cmd, 1, adc_buffer, 2, 1000);
	            
	    raw_adc = (adc_buffer[0] << 8) | adc_buffer[1];   
	         
	    voltage_mv = raw_adc / 8;
	    */        
		//xQueueSend(adc_queue, &voltage_mv, portMAX_DELAY);
		
		xQueueSend(adc_queue, &TESTERATOR, portMAX_DELAY);
		
		TESTERATOR++;
		
	    vTaskDelay(pdMS_TO_TICKS(500));
	}
	vTaskDelete(NULL);
}

void LCD_task(void *pvParameters) {
	int16_t val;
	char display_buffer[16];

	lcd_init();
	lcd_set_cursor(0, 0);
	lcd_put_str("temp:");

	while (1) {
		if (xQueueReceive(adc_queue, &val, portMAX_DELAY) == pdTRUE) {

			snprintf(display_buffer, sizeof(display_buffer), "%5d", val);
	            
			lcd_set_cursor(1, 0);
			lcd_put_str(display_buffer);
		}
	}
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
				    stepper_run(v_pins, &v_step, +1);  // preload downward direction
				    vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
				}

		        vTaskDelay(pdMS_TO_TICKS(1000));
	}
	vTaskDelete(NULL);
}

