#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =====================
// PINS
// =====================
int H_PINS[4] = {23, 22, 21, 19};
int V_PINS[4] = {18, 5, 4, 0};

// =====================
// HALF-STEP SEQUENCE
// =====================
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

// =====================
// STATE
// =====================
int h_step = 0;
int v_step = 0;

// =====================
// PARAMETERS
// =====================
#define H_STEPS        80
#define VERT_STEPS         25
#define LINES_PER_FRAME    10

#define SCAN_DELAY_MS      10
#define FLYBACK_DELAY_MS   10
#define VERT_DELAY_MS      30

// =====================
// CUSHIONS (IMPORTANT)
// =====================
#define H_FLYBACK_CUSHION  0     // horizontal overshoot correction
#define V_RESET_CUSHION   (3*VERT_STEPS)     // vertical return undershoot correction
#define V_BACKLASH_COMP (3.5*VERT_STEPS)   // typically damp during the first 4 lines

// =====================
// INIT
// =====================
void init_motor(int *pins)
{
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
}

// =====================
// STEP MOTOR
// =====================
void step_motor(int *pins, int *step_index, int dir)
{
    *step_index += dir;

    if (*step_index >= 8) *step_index = 0;
    if (*step_index < 0) *step_index = 7;

    for (int i = 0; i < 4; i++) {
        gpio_set_level(pins[i], seq[*step_index][i]);
    }
}

// =====================
// MAIN
// =====================
void app_main(void)
{
    init_motor(H_PINS);
    init_motor(V_PINS);

    while (1) {

        for (int line = 0; line < LINES_PER_FRAME; line++) {

            // =====================
            // FORWARD SCAN
            // =====================
            for (int x = 0; x < H_STEPS; x++) {
                step_motor(H_PINS, &h_step, +1);
                vTaskDelay(pdMS_TO_TICKS(SCAN_DELAY_MS));
            }

            vTaskDelay(pdMS_TO_TICKS(50));

            // =====================
            // FLYBACK (with cushion)
            // =====================
            for (int x = 0; x < H_STEPS + H_FLYBACK_CUSHION; x++) {
                step_motor(H_PINS, &h_step, -1);
                vTaskDelay(pdMS_TO_TICKS(FLYBACK_DELAY_MS));
            }

            vTaskDelay(pdMS_TO_TICKS(50));

            // =====================
            // VERTICAL STEP DOWN
            // =====================
            for (int y = 0; y < VERT_STEPS; y++) {
                step_motor(V_PINS, &v_step, +1);
                vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }

        // =====================
        // VERTICAL RESET (with cushion + backlash compensation)
        // =====================
        for (int i = 0; i < (VERT_STEPS * LINES_PER_FRAME) + V_RESET_CUSHION; i++) {
            step_motor(V_PINS, &v_step, -1);
            vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
        }

		for (int i = 0; i < V_BACKLASH_COMP; i++) {
		    step_motor(V_PINS, &v_step, +1);  // preload downward direction
		    vTaskDelay(pdMS_TO_TICKS(VERT_DELAY_MS));
		}

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}