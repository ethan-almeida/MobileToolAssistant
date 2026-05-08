#include "dwm_triangulation.h"
#include <string.h>

#define POSITION_AVG_COUNT 5

// Active anchor
typedef enum {
    ANCHOR_LEFT = 0,
    ANCHOR_RIGHT
} AnchorActive;


//for spi tests
uint8_t dwm1_device_id[4] ={0};
uint8_t dwm2_device_id[4] ={0};


int dwm1_dist_cm = 0;
int dwm2_dist_cm = 0;


typedef struct
{
    SPI_HandleTypeDef *hspi;

    DWM_Module left_anchor;
    DWM_Module right_anchor;

    AnchorActive active_anchor;
    bool standby_state;

    int left_dist_cm;
    int right_dist_cm;

    int left_sum;
    int right_sum;
    int left_count;
    int right_count;

    int left_avg;
    int right_avg;

    bool left_avg_ready;
    bool right_avg_ready;

    RemotePosition remote_pos;
    bool new_position_ready;

} DWM_TriangulationState;

static DWM_TriangulationState s_tri;


/**
 * Initializes the triangulation.
 * Configures GPIO pins, resets both DWM modules, and performs initial 
 * register configuration via SPI.
 * @param hspi Pointer to the SPI handle used for communication.
 */
void DWM_Triangulation_Init(SPI_HandleTypeDef *hspi)
{
    memset(&s_tri, 0, sizeof(s_tri));

    s_tri.hspi = hspi;
    s_tri.left_anchor.cs_port = DWM1_CS_N_GPIO_Port;
    s_tri.left_anchor.cs_pin = DWM1_CS_N_Pin;
    s_tri.left_anchor.reset_port = DWM1_RESET_N_GPIO_Port;
    s_tri.left_anchor.reset_pin = DWM1_RESET_N_Pin;
    s_tri.left_anchor.antenna_delay = 32848.80003;
    s_tri.left_anchor.offset_cm = 0;

    s_tri.right_anchor.cs_port = DWM2_CS_N_GPIO_Port;
    s_tri.right_anchor.cs_pin = DWM2_CS_N_Pin;
    s_tri.right_anchor.reset_port = DWM2_RESET_N_GPIO_Port;
    s_tri.right_anchor.reset_pin = DWM2_RESET_N_Pin;
    s_tri.right_anchor.antenna_delay = 32823.2235;
    s_tri.right_anchor.offset_cm = 0;



    s_tri.active_anchor = ANCHOR_LEFT;
    s_tri.remote_pos.valid = 0;
    s_tri.new_position_ready = false;

    s_tri.standby_state = true;

    dwm_reset(&s_tri.right_anchor);
    dwm_reset(&s_tri.left_anchor);

    dwm_configure(&s_tri.left_anchor);
    dwm_configure(&s_tri.right_anchor);

    // SPI starts slow PRESCALER = 256 for configuration, then increase to PRESCALER_8
    HAL_SPI_DeInit(s_tri.hspi);
    s_tri.hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    HAL_SPI_Init(s_tri.hspi);

	uint8_t device_id_right[4] = {0};
	dwm_read_reg(&s_tri.left_anchor, 0x00, device_id_right, 4);

	uint8_t device_id_left[4] = {0};
	dwm_read_reg(&s_tri.right_anchor, 0x00, device_id_left, 4);


	 int dummy;
}

/**
 * Main state machine step for distance gathering.
 * Alternates ranging between left and right anchors, averages results, 
 * and triggers a new position calculation once enough samples are collected.
 */
void DWM_Triangulation_TaskStep(void)
{
	dwm_read_reg(&s_tri.left_anchor, 0x00, dwm1_device_id, 4);


	dwm_read_reg(&s_tri.right_anchor, 0x00, dwm2_device_id, 4);


    bool finished = false;

    if (s_tri.active_anchor == ANCHOR_LEFT)
    {
        finished = robot_ranging_step(&s_tri.left_anchor,
                                      &s_tri.left_dist_cm,
                                      &s_tri.standby_state);

        if (finished)
        {
            s_tri.left_sum += s_tri.left_dist_cm;
            s_tri.left_count++;

            if (s_tri.left_count >= POSITION_AVG_COUNT)
            {
                s_tri.left_avg = s_tri.left_sum / POSITION_AVG_COUNT;
                s_tri.left_avg_ready = true;
                s_tri.left_sum = 0;
                s_tri.left_count = 0;

                dwm1_dist_cm = s_tri.left_avg;
            }

            s_tri.active_anchor = ANCHOR_RIGHT;
        }
    }
    else
    {
        finished = robot_ranging_step(&s_tri.right_anchor,
                                      &s_tri.right_dist_cm,
                                      &s_tri.standby_state);

        if (finished)
        {
            s_tri.right_sum += s_tri.right_dist_cm;
            s_tri.right_count++;

            if (s_tri.right_count >= POSITION_AVG_COUNT)
            { 
                s_tri.right_avg = s_tri.right_sum / POSITION_AVG_COUNT;
                s_tri.right_avg_ready = true;
                s_tri.right_sum = 0;
                s_tri.right_count = 0;
                dwm2_dist_cm = s_tri.right_avg;
            }

            s_tri.active_anchor = ANCHOR_LEFT;
        }
    }

    if (s_tri.left_avg_ready && s_tri.right_avg_ready)
    {
        s_tri.remote_pos = calculate_remote_position(s_tri.left_avg,
                                                     s_tri.right_avg);

        s_tri.new_position_ready = true;
        s_tri.left_avg_ready = false;
        s_tri.right_avg_ready = false;

    }
}


/**
 * Retrieves the most recently calculated remote position.
 * @param pos Pointer to a RemotePosition struct.
 * @return true if a new valid position was available, false otherwise.
 */
bool DWM_Triangulation_GetPosition(RemotePosition *pos)
{
    if ((pos == NULL) || (!s_tri.new_position_ready))
    {
        return false;
    }

    *pos = s_tri.remote_pos;
    s_tri.new_position_ready = false;
    return true;
}


bool get_standby_state(){
	return s_tri.standby_state;
}


void getDeviceID(){


}



