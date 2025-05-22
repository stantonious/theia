
/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <strings.h>

/* Cypress secure socket header file */
#include "cy_secure_sockets.h"

/* Wi-Fi connection manager header files */
#include "cy_wcm.h"
#include "cy_wcm_error.h"

/* UDP server task header file. */
#include "udp_server.h"
#include "theia_settings.h"
#include <support_functions.h>

/*******************************************************************************
 * Macros
 ********************************************************************************/
/* Buffer size to store the incoming messages from server, in bytes. */
#define MAX_UDP_RECV_BUFFER_SIZE (20)

/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static cy_rslt_t connect_to_wifi_ap(void);
static cy_rslt_t create_udp_server_socket(void);
static cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg);
static char udp_buf[Q_DEPTH * DOWNSAMPLE_CHIRP * sizeof(float32)] = {0};

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
/* Secure socket variables. */
cy_socket_sockaddr_t udp_server_addr, peer_addr;
static cy_socket_t server_handle;

/* Flag variable to track client connection status,
 * set to True when START_COMM_MSG is received from client. */
bool client_connected = false;

extern float32_t avg_chirp_q[Q_DEPTH * DOWNSAMPLE_CHIRP];
extern uint16_t buf_write_pos;
extern uint16_t buf_read_pos;

extern int sample_offset;

/*******************************************************************************
 * Function Name: udp_server_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote UDP client.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void udp_server_task(void *arg)
{
    cy_rslt_t result;

    printf("starting Secure Sockets initialized\n");
    /* Variable to store number of bytes sent over UDP socket. */
    uint32_t bytes_sent = 0;
    uint32_t ul_notification = 0x00;

    /* Connect to Wi-Fi AP */
    if (connect_to_wifi_ap() != CY_RSLT_SUCCESS)
    {
        printf("\n Failed to connect to Wi-Fi AP.\n");
        CY_ASSERT(0);
    }

    /* Secure Sockets initialization */
    result = cy_socket_init();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Secure Sockets initialization failed!\n");
        CY_ASSERT(0);
    }
    printf("Secure Sockets initialized\n");

    /* Create UDP Server*/
    result = create_udp_server_socket();
    if (result != CY_RSLT_SUCCESS)
    {
        printf("UDP Server Socket creation failed. Error: %" PRIu32 "\n", result);
        CY_ASSERT(0);
    }

    /* Toggle the LED on/off command sent to client on every button press */
    while (true)
    {
        /* Wait until a notification is received from the user button ISR. */
        xTaskNotifyWait(0, 0, &ul_notification, portMAX_DELAY);


        if (client_connected)
        {
            uint32_t capture_time = xTaskGetTickCount() / pdMS_TO_TICKS(1);
            uint16_t amt = (Q_DEPTH - buf_read_pos) * sizeof(float32_t) * DOWNSAMPLE_CHIRP;
            printf("amt %i-%i=%i\n ",Q_DEPTH,buf_read_pos,amt );
            printf("read/write pos %i-%i %d\n ",buf_read_pos,buf_write_pos,amt );
            memcpy(&udp_buf[0], &capture_time, sizeof(uint32_t));
            if (amt > MAX_UDP_XMIT)
            {
                amt = MAX_UDP_XMIT;
                uint16_t num_block_left = amt / (DOWNSAMPLE_CHIRP * sizeof(float32_t));
                memcpy(&udp_buf[sizeof(uint32_t)], &avg_chirp_q[buf_read_pos * DOWNSAMPLE_CHIRP], amt);
                buf_read_pos += num_block_left;
                printf("copied full buf %i\n ", amt);
            }
            else
            {
                memcpy(&udp_buf[sizeof(uint32_t)], &avg_chirp_q[buf_read_pos * DOWNSAMPLE_CHIRP], amt);
                buf_read_pos = 0; // roll read pointer
                uint16_t amt_left = MAX_UDP_XMIT - amt;
                printf("copied %i\n ", amt);
                memcpy(&udp_buf[sizeof(uint32_t) + amt], &avg_chirp_q[buf_read_pos * DOWNSAMPLE_CHIRP], amt_left);
                printf("than copied %i\n ", amt_left);
                uint16_t written_blocks = amt_left / (DOWNSAMPLE_CHIRP * sizeof(float32_t));
                buf_read_pos += written_blocks;
            }
            result = cy_socket_sendto(server_handle, &udp_buf[0], MAX_UDP_XMIT + sizeof(uint32_t), CY_SOCKET_FLAGS_NONE,
                                      &peer_addr, sizeof(cy_socket_sockaddr_t), &bytes_sent);
            printf("sending!! %i\n", peer_addr.port);
            if (result == CY_RSLT_SUCCESS)
            {
            }
            else
            {
                printf("Failed to send command to client. Error: %" PRIu32 "\n", result);
            }
        }
        else
        {
            //printf("client not connected\n");
        }
    }
}

/*******************************************************************************
 * Function Name: connect_to_wifi_ap()
 *******************************************************************************
 * Summary:
 *  Connects to Wi-Fi AP using the user-configured credentials, retries up to a
 *  configured number of times until the connection succeeds.
 *
 *******************************************************************************/
cy_rslt_t connect_to_wifi_ap(void)
{
    cy_rslt_t result;

    /* Variables used by Wi-Fi connection manager. */
    cy_wcm_connect_params_t wifi_conn_param;

    cy_wcm_config_t wifi_config = {
        .interface = CY_WCM_INTERFACE_TYPE_STA};

    cy_wcm_ip_address_t ip_address;

    /* Variable to track the number of connection retries to the Wi-Fi AP specified
     * by WIFI_SSID macro */
    int conn_retries = 0;

    /* Initialize Wi-Fi connection manager. */
    result = cy_wcm_init(&wifi_config);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi Connection Manager initialization failed!\n");
        return result;
    }
    printf("Wi-Fi Connection Manager initialized. \n");

    /* Set the Wi-Fi SSID, password and security type. */
    memset(&wifi_conn_param, 0, sizeof(cy_wcm_connect_params_t));
    memcpy(wifi_conn_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
    memcpy(wifi_conn_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
    wifi_conn_param.ap_credentials.security = WIFI_SECURITY_TYPE;

    /* Join the Wi-Fi AP. */
    for (conn_retries = 0; conn_retries < MAX_WIFI_CONN_RETRIES; conn_retries++)
    {
        result = cy_wcm_connect_ap(&wifi_conn_param, &ip_address);

        if (result == CY_RSLT_SUCCESS)
        {
            printf("Successfully connected to Wi-Fi network '%s'.\n",
                   wifi_conn_param.ap_credentials.SSID);
            printf("IP Address Assigned: %d.%d.%d.%d\n", (uint8)ip_address.ip.v4,
                   (uint8)(ip_address.ip.v4 >> 8), (uint8)(ip_address.ip.v4 >> 16),
                   (uint8)(ip_address.ip.v4 >> 24));

            /* IP address and UDP port number of the UDP server */
            udp_server_addr.ip_address.ip.v4 = ip_address.ip.v4;
            udp_server_addr.ip_address.version = CY_SOCKET_IP_VER_V4;
            udp_server_addr.port = UDP_SERVER_PORT;
            return result;
        }

        printf("Connection to Wi-Fi network failed with error code %d."
               "Retrying in %d ms...\n",
               (int)result, WIFI_CONN_RETRY_INTERVAL_MSEC);
        vTaskDelay(pdMS_TO_TICKS(WIFI_CONN_RETRY_INTERVAL_MSEC));
    }

    /* Stop retrying after maximum retry attempts. */
    printf("Exceeded maximum Wi-Fi connection attempts\n");

    return result;
}

/*******************************************************************************
 * Function Name: create_udp_server_socket
 *******************************************************************************
 * Summary:
 *  Function to create a socket and set the socket options
 *
 *******************************************************************************/
cy_rslt_t create_udp_server_socket(void)
{
    cy_rslt_t result;

    /* Variable used to set socket options. */
    cy_socket_opt_callback_t udp_recv_option = {
        .callback = udp_server_recv_handler,
        .arg = NULL};

    /* Create a UDP server socket. */
    result = cy_socket_create(CY_SOCKET_DOMAIN_AF_INET, CY_SOCKET_TYPE_DGRAM, CY_SOCKET_IPPROTO_UDP, &server_handle);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Register the callback function to handle messages received from UDP client. */
    result = cy_socket_setsockopt(server_handle, CY_SOCKET_SOL_SOCKET,
                                  CY_SOCKET_SO_RECEIVE_CALLBACK,
                                  &udp_recv_option, sizeof(cy_socket_opt_callback_t));
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Bind the UDP socket created to Server IP address and port. */
    result = cy_socket_bind(server_handle, &udp_server_addr, sizeof(udp_server_addr));
    if (result == CY_RSLT_SUCCESS)
    {
        printf("Socket bound to port: %d\n", udp_server_addr.port);
    }

    return result;
}

cy_rslt_t udp_server_recv_handler(cy_socket_t socket_handle, void *arg)
{
    cy_rslt_t result;

    /* Variable to store the number of bytes received. */
    uint32_t bytes_received = 0;

    /* Buffer to store data received from Client. */
    static char message_buffer[MAX_UDP_RECV_BUFFER_SIZE] = {0};

    /* Receive incoming message from UDP server. */
    result = cy_socket_recvfrom(server_handle, message_buffer, MAX_UDP_RECV_BUFFER_SIZE,
                                CY_SOCKET_FLAGS_NONE, &peer_addr, NULL,
                                &bytes_received);

    message_buffer[bytes_received] = '\0';

    printf("Received %i, %s\n", peer_addr.port, &message_buffer[0]);

    sscanf(message_buffer, "%d", &sample_offset);
    printf("setting modes %i %i\n", sample_offset);

    if (result == CY_RSLT_SUCCESS)
    {
        client_connected = true;
    }
    else
    {
        printf("Failed to receive message from client. Error: %" PRIu32 "\n", result);
        return result;
    }

    return result;
}

/* [] END OF FILE */
