#ifndef UDP_SERVER_H_
#define UDP_SERVER_H_

#define WIFI_SSID                                 "stanton-xfinity"
#define WIFI_PASSWORD                             "B00BF00D77"


/* Security type of the Wi-Fi access point. See 'cy_wcm_security_t' structure
 * in "cy_wcm.h" for more details.
 */
#define WIFI_SECURITY_TYPE                        CY_WCM_SECURITY_WPA2_AES_PSK

/* Maximum number of connection retries to a Wi-Fi network. */
#define MAX_WIFI_CONN_RETRIES                     (10u)

/* Wi-Fi re-connection time interval in milliseconds */
#define WIFI_CONN_RETRY_INTERVAL_MSEC             (1000)

/* UDP server related macros. */
#define UDP_SERVER_PORT                           (57345)
#define UDP_SERVER_MAX_PENDING_CONNECTIONS        (3)
#define UDP_SERVER_RECV_TIMEOUT_MS                (500)

#define NUM_SAMPLES_PER_CHIRP 128
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void udp_server_task(void *arg);

#endif /* UDP_SERVER_H_ */
