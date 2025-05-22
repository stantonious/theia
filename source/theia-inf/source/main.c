
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "xensiv_bgt60trxx_mtb.h"
#include "xensiv_bgt60trxx_platform.h"
#include "radar_settings.h"
#include "cy_scb_spi.h"

#include "FreeRTOS.h"
#include "task.h"
/* Model to use */
#include "models/model.h"
#include "udp_server.h"

#include <support_functions.h>
#include <arm_math.h>

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* These sizes and masks must be aligned. */
#define DEFAULT_FIFO_SETTING 6144
#define RING_BUFFER_SIZE 0x00010000 /* 64k samples */
#define RING_BUFFER_MASK 0x0000FFFF
#define RING_BUFFER_MASK32 0x00007FFF                      /* Mask for 32-bits */
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD (0xFF000000UL) /* Write addr 7f<<1 | 0x01 */
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS (17U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS (16U)
#define XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS (9U)

/* RTOS tasks */
#define MAIN_TASK_NAME "main_task"
#define MAIN_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE * 8)
#define MAIN_TASK_PRIORITY (configMAX_PRIORITIES - 2)
#define UDP_SERVER_TASK_NAME "udp_server_task"
#define UDP_SERVER_STACK_SIZE (configMINIMAL_STACK_SIZE * 60)
#define UDP_SERVER_PRIORITY (configMAX_PRIORITIES - 3)

#define CHIRP_OFFSET 0

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void main_task(void *pvParameters);

static TaskHandle_t main_task_handler;
TaskHandle_t udp_server_handle;

/*******************************************************************************
 * Ringbuffer array
 ***********************************************************************2*******/
uint16_t bgt60_tensor_ring[RING_BUFFER_SIZE]; /* This is the size of the internal fifo. */
uint32_t *ring32;
int bgt60_ring_next_to_write = 0;
int bgt60_ring_next_to_read = 0;
int bgt60_ring_level = 0;
struct inf_result inf_results;
int report_class = -2;
int sample_offset = CHIRP_OFFSET;
float32_t avg_chirp[NUM_SAMPLES_PER_CHIRP];

/*******************************************************************************
 * Preset configuration variables and structs.
 *******************************************************************************/
struct xensiv_bgt60trxx_type
{
    uint32_t fifo_addr;
    uint16_t fifo_size;
    xensiv_bgt60trxx_device_t device;
};

struct radar_config radar_configs;

/*******************************************************************************
 * Types
 *******************************************************************************/

typedef struct
{
    xensiv_bgt60trxx_mtb_t bgt60_obj;
    uint16_t bgt60_buffer0[16384];
    uint16_t bgt60_buffer1[16384];
    float bgt60_send_buffer[RING_BUFFER_SIZE];
    bool have_data;
    int skipped_frames;
} dev_bgt60trxx_t;

enum spi_state
{
    NONE = 0,
    IDLE,
    BURST_PENDING,
    FIFO_READ_PENDING,
    FIFO_READ_DONE
};

enum spi_state sp_state = IDLE;

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static bool radar_init(dev_bgt60trxx_t *radar, cyhal_spi_t *spi);
static void load_presets(void);
static void spi_set_data_width(CySCB_Type *base, uint32_t data_width);
bool board_set_clocks(void);

cyhal_spi_t spi;
dev_bgt60trxx_t radar;

static __NO_RETURN void main_task(void *pvParameters)
{
    (void)pvParameters;

    const char *class_map[] = IMAI_DATA_OUT_SYMBOLS;

    radar_init(&radar, &spi);

    if (xensiv_bgt60trxx_soft_reset(&radar.bgt60_obj.dev, XENSIV_BGT60TRXX_RESET_FIFO) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        printf("Fifo reset error error.\r\n");
    }

    radar.have_data = false;
    if (xensiv_bgt60trxx_start_frame(&radar.bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        printf("Start frame error.\r\n");
    }

    /* ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H\x1b[?25l;");

    /* Reset the local fifo. */
    bgt60_ring_next_to_write = 0;
    bgt60_ring_next_to_read = 0;
    bgt60_ring_level = 0;

    for (;;)
    {
        int a;
        int words_a_24;
        int fifo_size_32;
        int imai_result_enqueue;
        float model_out[IMAI_DATA_OUT_COUNT];

        xensiv_bgt60trxx_t *p_device = &radar.bgt60_obj.dev;

        void *iface = p_device->iface;
        xensiv_bgt60trxx_mtb_iface_t *mtb_iface = iface;

        /* Check if SPI transfer is active. If it is active then skip everything. */
        if (0UL != (CY_SCB_SPI_TRANSFER_ACTIVE &
                    Cy_SCB_SPI_GetTransferStatus(mtb_iface->spi->base, &(mtb_iface->spi->context))))
        {
            continue;
        }

        switch (sp_state)
        {
        case IDLE:
            a = cyhal_gpio_read(CYBSP_RSPI_IRQ); /* P11_0 */
            if (a)
            {
                printf("bar\n");
                /* Make Chip select */
                xensiv_bgt60trxx_platform_spi_cs_set(iface, false);

                /* Read current fifo state */
                uint32_t *head = (uint32_t *)radar.bgt60_buffer0;
                const xensiv_bgt60trxx_mtb_iface_t *mtb_iface = iface;

                *head = XENSIV_BGT60TRXX_SPI_BURST_MODE_CMD |
                        (p_device->type->fifo_addr << XENSIV_BGT60TRXX_SPI_BURST_MODE_SADR_POS) | /* Addr 0x60.. */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_RWB_POS) |                          /* Read mode */
                        (0 << XENSIV_BGT60TRXX_SPI_BURST_MODE_LEN_POS);                           /* Read until termination. */

                /* Ensure correct byte order for sending the command */
                *head = xensiv_bgt60trxx_platform_word_reverse(*head);

                spi_set_data_width(mtb_iface->spi->base, 8U);
                Cy_SCB_SetByteMode(mtb_iface->spi->base, true);

                Cy_SCB_SPI_Transfer(mtb_iface->spi->base, (uint8_t *)radar.bgt60_buffer0,
                                    (uint8_t *)radar.bgt60_buffer1, /* Can be set to NULL, Recieved data is discarded. */
                                    4,
                                    &(mtb_iface->spi->context));

                sp_state = BURST_PENDING;
            }
            break;

        case BURST_PENDING:

            words_a_24 = radar_configs.fifo_int_level & 0xFFFFFFC0;

            spi_set_data_width(mtb_iface->spi->base, 12U);

            Cy_SCB_SetByteMode(mtb_iface->spi->base, false);

            Cy_SCB_SPI_Transfer(mtb_iface->spi->base,
                                (uint8_t *)radar.bgt60_buffer0, /* Can be set to NULL, Recieved data is discarded. */
                                (uint8_t *)radar.bgt60_buffer1,
                                words_a_24,
                                &(mtb_iface->spi->context));

            /* At this point it might be a good idea to check if there is another interrupt from the radar pending. */
            /* If there is it should be possible to swap buffers and issue a second read of data. */
            /* In this state the radar burst mode is still active. */
            sp_state = FIFO_READ_PENDING;
            break;

        case FIFO_READ_PENDING:
            /* Clear the Chip Select. Done when the next polling allows for it. */
            xensiv_bgt60trxx_platform_spi_cs_set(iface, true);
            sp_state = FIFO_READ_DONE;
            break;

        case FIFO_READ_DONE:
            sp_state = IDLE;
            break;

        default:
            break;
        }

        if (sp_state == FIFO_READ_DONE)
        {
            ring32 = (uint32_t *)bgt60_tensor_ring; /* This will point to a 32k buffer. */
            fifo_size_32 = radar_configs.fifo_int_level >> 1;

            for (int x = 0; x < fifo_size_32; x++)
            {
                ring32[bgt60_ring_next_to_write] = ((uint32_t *)(radar.bgt60_buffer1))[x]; /* radar->full_buffer[x]; */
                bgt60_ring_level += 2;                                                     /* Copying 2 samples at a time. */
                bgt60_ring_next_to_write++;
                bgt60_ring_next_to_write &= RING_BUFFER_MASK32;
            }

            for (int i = 0; i < radar_configs.num_samples_per_frame * 2; i += 1)
            {
                //(radar.bgt60_send_buffer)[t] = ((int16_t *)bgt60_tensor_ring)[bgt60_ring_next_to_read] * (1.0f);
                (radar.bgt60_send_buffer)[i] = ((float32_t)(bgt60_tensor_ring[bgt60_ring_next_to_read]) / 4096.0F);
                // t++;
                bgt60_ring_level -= 1;
                bgt60_ring_next_to_read++;
                bgt60_ring_next_to_read &= RING_BUFFER_MASK;
            }

            arm_fill_f32(0, avg_chirp, NUM_SAMPLES_PER_CHIRP);

            for (int chirp = 0; chirp < P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME * 2; chirp++)
            {
                arm_add_f32(avg_chirp, &radar.bgt60_send_buffer[NUM_SAMPLES_PER_CHIRP * chirp], avg_chirp, NUM_SAMPLES_PER_CHIRP);
            }
            // arm_scale_f32(avg_chirp, 1.0f / P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME, avg_chirp, NUM_SAMPLES_PER_CHIRP);
            for (int i = 0; i < NUM_SAMPLES_PER_CHIRP; i++)
            {
                avg_chirp[i] *= 1.0f / P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
            }
            //printf("time %i\n", xTaskGetTickCount() / pdMS_TO_TICKS(1));

            /* Move cursor home */
            // printf("\033[H");
            radar.have_data = false;
            imai_result_enqueue = IMAI_enqueue(&avg_chirp[sample_offset]);
            if (imai_result_enqueue != IMAI_RET_SUCCESS){
                printf("doh!!! enqueue error\n");
            }
            if (xensiv_bgt60trxx_start_frame(&radar.bgt60_obj.dev, false) != XENSIV_BGT60TRXX_STATUS_OK)
            {
                printf("start error\n");
            }
            int imai_result = IMAI_dequeue(model_out);
            if (imai_result!= IMAI_RET_SUCCESS){
                printf("doh!!! dequeue error\n");
            }else{
                printf("we good\n");
            }
            if (xensiv_bgt60trxx_start_frame(&radar.bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
            {
                printf("start true error\n");
            }
            int16_t best_label = 0;
            float max_score = -1000.0f;

            switch (imai_result)
            {
            case IMAI_RET_SUCCESS:
                for (uint8_t i = 0; i < IMAI_DATA_OUT_COUNT; i++)
                {
                    printf("label: %-10s: score: %.2f\r\n", class_map[i], model_out[i]);
                    if (model_out[i] > max_score)
                    {
                        max_score = model_out[i];
                        best_label = i;
                    }
                }
                if (report_class == -1)
                {
                        inf_results.class = best_label;
                        inf_results.conf = max_score;
                        xTaskNotifyGive(udp_server_handle);
                }
                else
                {
                    inf_results.class = report_class;
                    inf_results.conf = model_out[report_class];
                    xTaskNotifyGive(udp_server_handle);
                }
                break;
            case IMAI_RET_NODATA:
                break;
            case IMAI_RET_ERROR:
                /* Something went wrong, stop the program */
                printf("Unable to perform inference. Unknown error occurred.\n");
                break;
            }
        }
    }
}

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 * This is the main function for CM4 CPU. It initializes BSP, radar, SPI and the
 * ML model. It reads data from radar sensor continuously, processes it within
 * the model and displays the output.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    /* Initialize the device and board peripherals */
    cy_rslt_t result;
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    board_set_clocks();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    result = cyhal_spi_init(
        &spi,
        CYBSP_RSPI_MOSI,
        CYBSP_RSPI_MISO,
        CYBSP_RSPI_CLK,
        NC,
        NULL,
        8,
        CYHAL_SPI_MODE_00_MSB,
        false);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cyhal_spi_set_frequency(&spi, 18750000UL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    load_presets();

    /* Init DEEPCRAFT AI model */
    IMAI_init();

    /* Create the RTOS task */
    if (xTaskCreate(main_task, MAIN_TASK_NAME, MAIN_TASK_STACK_SIZE, NULL, MAIN_TASK_PRIORITY, &main_task_handler) != pdPASS)
    {
        CY_ASSERT(0);
    }
    if (xTaskCreate(udp_server_task, UDP_SERVER_TASK_NAME, UDP_SERVER_STACK_SIZE, NULL, UDP_SERVER_PRIORITY, &udp_server_handle) != pdPASS)
    {
        CY_ASSERT(0);
    }

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    CY_ASSERT(0);
}

/*******************************************************************************
 * Function Name: radar_init
 ********************************************************************************
 * Summary:
 * This function configures the SPI interface, initializes radar and interrupt
 * service routine to indicate the availability of radar data.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  Success or error
 *
 *******************************************************************************/
static bool radar_init(dev_bgt60trxx_t *radar, cyhal_spi_t *spi)
{
    cy_rslt_t result;

    radar->have_data = false;
    radar->skipped_frames = 0;

    /* Reduce drive strength to improve EMI */
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_RSPI_MOSI), CYHAL_GET_PIN(CYBSP_RSPI_MOSI), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_RSPI_MOSI), CYHAL_GET_PIN(CYBSP_RSPI_MOSI), CY_GPIO_DRIVE_1_8);
    Cy_GPIO_SetSlewRate(CYHAL_GET_PORTADDR(CYBSP_RSPI_CLK), CYHAL_GET_PIN(CYBSP_RSPI_CLK), CY_GPIO_SLEW_FAST);
    Cy_GPIO_SetDriveSel(CYHAL_GET_PORTADDR(CYBSP_RSPI_CLK), CYHAL_GET_PIN(CYBSP_RSPI_CLK), CY_GPIO_DRIVE_1_8);

    /* Initialization uses preset 0 */
    result = xensiv_bgt60trxx_mtb_init(
        &radar->bgt60_obj,
        spi,
        CYBSP_RSPI_CS,
        CYBSP_RXRES_L,
        radar_configs.register_list,
        radar_configs.number_of_regs);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("ERROR: xensiv_bgt60trxx_mtb_init failed\n");
        return false;
    }

    result = cyhal_gpio_init(CYBSP_RSPI_IRQ, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);

    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    xensiv_bgt60trxx_set_fifo_limit(&radar->bgt60_obj.dev, radar_configs.fifo_int_level);

    return true;
}
/*******************************************************************************
 * Platform functions implementation
 ******************************************************************************/
static void spi_set_data_width(CySCB_Type *base, uint32_t data_width)
{
    CY_ASSERT(CY_SCB_SPI_IS_DATA_WIDTH_VALID(data_width));

    CY_REG32_CLR_SET(SCB_TX_CTRL(base),
                     SCB_TX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
    CY_REG32_CLR_SET(SCB_RX_CTRL(base),
                     SCB_RX_CTRL_DATA_WIDTH,
                     (uint32_t)data_width - 1U);
}

/*******************************************************************************
 * Summary:
 *   This just loads preset from header file constants to enable selection of
 *   various radar presets.
 * See radar_settings.h for information of the sources of radar settings.
 *
 *******************************************************************************/
static void load_presets()
{
    radar_configs.start_freq = P3_XENSIV_BGT60TRXX_CONF_START_FREQ_HZ;
    radar_configs.end_freq = P3_XENSIV_BGT60TRXX_CONF_END_FREQ_HZ;
    radar_configs.samples_per_chirp = P3_XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP;
    radar_configs.chirps_per_frame = P3_XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME;
    radar_configs.rx_antennas = P3_XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS;
    radar_configs.tx_antennas = P3_XENSIV_BGT60TRXX_CONF_NUM_TX_ANTENNAS;
    radar_configs.sample_rate = P3_XENSIV_BGT60TRXX_CONF_SAMPLE_RATE;
    radar_configs.chirp_repetition_time = P3_XENSIV_BGT60TRXX_CONF_CHIRP_REPETITION_TIME_S;
    radar_configs.frame_repetition_time = P3_XENSIV_BGT60TRXX_CONF_FRAME_REPETITION_TIME_S;

    radar_configs.frame_rate = (int)(0.5 + 1.0 / radar_configs.frame_repetition_time);
    radar_configs.num_samples_per_frame =
        radar_configs.samples_per_chirp *
        radar_configs.chirps_per_frame *
        radar_configs.rx_antennas;

    radar_configs.fifo_int_level = DEFAULT_FIFO_SETTING; /* This will equal 75% of the fifo. 12288 samples */

    radar_configs.number_of_regs = P3_XENSIV_BGT60TRXX_CONF_NUM_REGS;
    radar_configs.register_list = register_list_p3;
}
/*******************************************************************************
 * Function Name: board_set_clocks
 ********************************************************************************
 * Summary:
 *   Reinitializes the clocks. FLL is dedicated for the audio done elseware.
 *    PLL0 is not modified and it needs to be at 96Mhz for the usb clock.
 *    PLL1 is set to 150 MHz and will drive the CPU Fast clock.
 *    Peripheral speed will be half of the fast clock. I.e. 75 MHz
 *    SPI clock can now be set to 75/4 = 18.75 MHz.
 *
 *
 * Return:
 *   True if initialization is successful, otherwise false.
 *******************************************************************************/
bool board_set_clocks(void)
{
    cy_rslt_t result;
    cyhal_clock_t peri;
    cyhal_clock_t hf_0;
    cyhal_clock_t PLL_1;

#define FAST_FREQ 150000000

    result = cyhal_clock_reserve(&peri, &CYHAL_CLOCK_PERI);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_reserve(&hf_0, &CYHAL_CLOCK_HF[0]);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_reserve(&PLL_1, &CYHAL_CLOCK_PLL[1]);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set the divider of peripheral clock first. */
    result = cyhal_clock_set_divider(&peri, 2);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set the frequency of PLL_1 and enable it. */
    result = cyhal_clock_set_frequency(&PLL_1, FAST_FREQ, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    result = cyhal_clock_set_enabled(&PLL_1, true, true);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    /* Set HF_0 clock to the source of PLL_1. */
    result = cyhal_clock_set_source(&hf_0, &CYHAL_CLOCK_PLL[1]);
    if (result != CY_RSLT_SUCCESS)
    {
        return false;
    }

    return true;
}
