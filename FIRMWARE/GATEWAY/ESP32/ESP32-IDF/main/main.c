/**
 * @file hello_world_main.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief
 * @version 1.0
 * @date 15-07_2023
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "sdkconfig.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_sntp.h"
#include "esp_attr.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "mqtt_client.h"

#include "string.h"
#include "time.h"
#include "sys/time.h"


typedef struct LoRa_Link_Struct
{
    uint16_t Node_ID;
    uint16_t Node_Status; 
    uint16_t Node_Battery_Voltage;
    uint16_t Node_Period;
} Link_Struct_t;
typedef struct LoRa_Data_Struct
{
    Link_Struct_t Link;
    uint16_t Node_Soil_Temperature;
    uint16_t Node_Air_Temperature;
    uint16_t Node_Soil_Humidity;
    uint16_t Node_Air_Humidity;
    uint16_t Node_Salinity;
    uint16_t Node_Conductivity;
    uint16_t Node_pH;
    uint16_t Node_N;
    uint16_t Node_P;
    uint16_t Node_K;
} Data_Struct_t;

typedef struct Server_CMD_Struct
{
    uint16_t Target_Node_ID;
    uint16_t Target_Node_Status;
    uint16_t Target_Node_Period;
    uint16_t Gateway_Relay;
} Server_CMD_Struct_t;

#define LED_PIN (4)
#define TXD_PIN (17)
#define RXD_PIN (16)

#define RX_BUF_SIZE (512)
#define TX_BUF_SIZE (512)

static const char *TAG = "MAIN ESP32";

static void tx_task(void *arg);
static void rx_task(void *arg);
static void blink_task(void *arg);
static void smartconfig_task(void *arg);
static void wifi_task(void *arg);

static int sendData(const char *logName, const char *data);
static void initialise_wifi(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static void log_error_if_nonzero(const char *message, int error_code);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

static TaskHandle_t UART_RX_TASK_Handle;
static TaskHandle_t UART_TX_TASK_Handle;
static TaskHandle_t BLINK_TASK_Handle;
static TaskHandle_t WIFI_TASK_Handle;
static TaskHandle_t SC_TASK_Handle;

static QueueHandle_t uart_queue_rx;
static QueueHandle_t uart_queue_tx;
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t s_uart_event_group;
static nvs_handle_t my_nvs_handle;
static esp_mqtt_client_handle_t mqtt_client = NULL;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

static const int CMD_ACK_BIT = BIT0;
static const int CMD_NACK_BIT = BIT1;

volatile static bool is_WiFi_allowed = false;
volatile static bool is_WiFi_connected = false;
volatile static bool is_MQTT_connected = false;
volatile static bool is_use_ESPTOUCH = false;
volatile static bool is_WiFi_task_running = false;
volatile static bool is_SC_task_running = false;

static char SSID[100] = {0};
static char PASS[100] = {0};
static size_t lengthSSID = 100;
static size_t lengthPASS = 100;

static time_t now;
static struct tm timeinfo = {0};

esp_mqtt_client_config_t mqtt_cfg = {
    .uri = "mqtt://test.mosquitto.org",
};

const char link_mode_string[] = "LINK";
const char normal_mode_string[] = "NORMAL";
const char retry_mode_string[] = "RETRY";
const int retry_count = 10;


void app_main(void)
{
    // ESP_ERROR_CHECK(nvs_flash_init());
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        printf("Retry nvs_flash_init");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Open
    printf("\n");
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    err = nvs_open("wifi", NVS_READWRITE, &my_nvs_handle);
    if (err != ESP_OK)  printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    else
    {
        printf("Done\n");
        nvs_get_str(my_nvs_handle, "ssid", &SSID, &lengthSSID);
        nvs_get_str(my_nvs_handle, "pass", &PASS, &lengthPASS);
        printf("Check stored SSID: %s\n", SSID);
        printf("Check stored PASS: %s\n", PASS);
    }

    ESP_LOGW("MAIN", "NOW ENTER MAIN");

    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0));
    // uart_param_config(UART_NUM_2, &uart_config);

    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_INPUT_OUTPUT);

    uart_queue_rx = xQueueCreate(5U, sizeof(Data_Struct_t));
    uart_queue_tx = xQueueCreate(5U, sizeof(Server_CMD_Struct_t));

    s_uart_event_group = xEventGroupCreate();

    if ((uart_queue_rx == NULL) || (uart_queue_tx == NULL) || (s_uart_event_group == NULL)) {ESP_LOGE(TAG, "CANT INIT"); while(1);}

    // vTaskDelay(3000 / portTICK_RATE_MS);

    xTaskCreate(rx_task, "uart_rx_task", 8192, NULL, configMAX_PRIORITIES - 0, &UART_RX_TASK_Handle);
    xTaskCreate(tx_task, "uart_tx_task", 8192, NULL, configMAX_PRIORITIES - 1, &UART_TX_TASK_Handle);
    xTaskCreate(blink_task, "blink_task", 8192, NULL, configMAX_PRIORITIES - 2, &BLINK_TASK_Handle);

    ESP_LOGW("MAIN", "NOW INITIALIZE WIFI");

    initialise_wifi();

    ESP_LOGW("MAIN", "NOW EXIT MAIN");
}

//------------------------------------------------------------------------------------------------------------------------

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_2, data, len);
    ESP_LOGI(logName, "Wrote %d bytes", txBytes);
    return txBytes;
}

//------------------------------------------------------------------------------------------------------------------------

static void tx_task(void *arg)
{
    static const char *TX_TASK_TAG = "TX_TASK";
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    EventBits_t uxBits;
    Server_CMD_Struct_t msg_to_send;
    uint32_t timeKeeper = xTaskGetTickCount();
    while (1)
    {
        if (uxQueueMessagesWaiting(uart_queue_tx) != 0U)
        {
            while (uxQueueMessagesWaiting(uart_queue_tx) != 0U)
            {
                xQueueReceive(uart_queue_tx, &msg_to_send, 100 / portTICK_RATE_MS);
                uart_write_bytes(UART_NUM_2, &msg_to_send, sizeof(Server_CMD_Struct_t));
                uxBits = xEventGroupWaitBits(s_uart_event_group, (CMD_ACK_BIT | CMD_NACK_BIT), true, false, 250 / portTICK_RATE_MS);
                if ((uxBits & CMD_ACK_BIT) != 0U)   {ESP_LOGI(TX_TASK_TAG, "Send CMD Success!");}
                else    xQueueSend(uart_queue_tx, &msg_to_send, 100 / portTICK_RATE_MS);
                vTaskDelay(200/portTICK_RATE_MS);
            }
        }

        if ((xTaskGetTickCount() - timeKeeper) >= 90000)
        {
            sendData(TX_TASK_TAG, "Check STM32");
            timeKeeper = xTaskGetTickCount();
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void rx_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t * data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    static uint32_t rxBytes = 0;
    static char * pChar = NULL;
    while (1)
    {
        rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 100 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            
            pChar = strstr((const char *)data, "MQTT-");
            if (pChar != NULL)
            {
                ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
                xQueueSend(uart_queue_rx, &pChar[5], 100 / portTICK_RATE_MS);
            }

            // pChar = strstr((const char *)data, "TIME-");
            // if (pChar != NULL)  {ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);}

            if          (strstr((const char *)data, "CMD-OK")   != NULL)    xEventGroupSetBits(s_uart_event_group, CMD_ACK_BIT);
            else if     (strstr((const char *)data, "CMD-NOK")  != NULL)    xEventGroupSetBits(s_uart_event_group, CMD_NACK_BIT);

            if          (strstr((const char *)data, "WIFI-ON")   != NULL)   xTaskCreate(wifi_task, "wifi_task", 8192, NULL, configMAX_PRIORITIES - 2, &WIFI_TASK_Handle);
            else if     (strstr((const char *)data, "WIFI-OFF")   != NULL)
            {
                is_WiFi_allowed = false;
                if (is_WiFi_task_running == true)   {vTaskDelete(WIFI_TASK_Handle); is_WiFi_task_running = false;}
                if (is_SC_task_running == true)     {vTaskDelete(SC_TASK_Handle); is_SC_task_running = false; esp_smartconfig_stop();}
                esp_mqtt_client_disconnect(mqtt_client);
                esp_mqtt_client_stop(mqtt_client);
                ESP_ERROR_CHECK(esp_wifi_disconnect());
                is_WiFi_connected = false;
                is_MQTT_connected = false;
            }
        }
        rxBytes = 0;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    free(data);
}

static void blink_task(void *arg)
{
    static const char *BLINK_TASK_TAG = "BLINK_TASK";
    static char temp[2][200];
    static char * pChar = NULL;
    static uint32_t len = 0;
    static Data_Struct_t data_to_send;
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    uint32_t timeKeeper = xTaskGetTickCount();
    uint32_t timeKeeperMQTT = timeKeeper;
    uint32_t timeKeeperBlink = timeKeeper;

    while (1)
    {
        if ((xTaskGetTickCount() - timeKeeperBlink) >= 1000)
        {
            gpio_set_level(LED_PIN, (gpio_get_level(LED_PIN) == 0)? (1U):(0U));
            timeKeeperBlink = xTaskGetTickCount();
        }

        if ((is_WiFi_connected == true) && (is_MQTT_connected == true) && (uxQueueMessagesWaiting(uart_queue_rx) != 0U))
        {
            memset(temp, '\0', sizeof(temp));
            if (xQueueReceive(uart_queue_rx, &data_to_send, 100 / portTICK_RATE_MS) != pdTRUE)  continue;
            if      (data_to_send.Link.Node_Status == 2U)    pChar = &link_mode_string;
            else if (data_to_send.Link.Node_Status == 4U)    pChar = &normal_mode_string;
            else if (data_to_send.Link.Node_Status == 8U)    pChar = &retry_mode_string;
            else                                             continue;
            sprintf(&temp[0], "SolGarden/Node/%04X/link", data_to_send.Link.Node_ID);
            len = sprintf(  &temp[1], "[{\"Node ID\": \"0x%04X\",\"Node Mode\": \"%s\",\"Battery Voltage\": %d,\"Period\": %d}]", 
                            data_to_send.Link.Node_ID, 
                            pChar, 
                            data_to_send.Link.Node_Battery_Voltage, 
                            data_to_send.Link.Node_Period);
            ESP_LOGI("MQTT", "Topic: '%s'", temp[0]);
            ESP_LOGI("MQTT", "Payload: '%s'", temp[1]);
            if (len > 0)    {esp_mqtt_client_publish(mqtt_client, temp[0], temp[1], len, 0, 0); ESP_LOGI("MQTT", "Valid payload length -> Send MQTT...");}
            else {ESP_LOGI("MQTT", "Invalid payload length -> Put back to RX Queue"); xQueueSend(uart_queue_rx, &data_to_send, 100 / portTICK_RATE_MS);}

            memset(temp, '\0', sizeof(temp));
            vTaskDelay(500 / portTICK_PERIOD_MS);

            sprintf(&temp[0], "SolGarden/Node/%04X/data", data_to_send.Link.Node_ID);
            len = sprintf(  &temp[1], "[{\"Soil Temp.\": %2.1f,\"Air Temp.\": %2.1f,\"Soil Humd.\": %2.1f,\"Air Humd.\": %2.1f,\"Salinity\": %d,\"Conductivity\": %d,\"pH\": %.1f,\"N\": %d,\"P\": %d,\"K\": %d}]",
                            (float)(data_to_send.Node_Soil_Temperature/10.0), 
                            (float)(data_to_send.Node_Air_Temperature/10.0), 
                            (float)(data_to_send.Node_Soil_Humidity/10.0), 
                            (float)(data_to_send.Node_Air_Humidity/10.0), 
                            (uint16_t)data_to_send.Node_Salinity, 
                            (uint16_t)data_to_send.Node_Conductivity, 
                            (float)(data_to_send.Node_pH/10.0), 
                            (uint16_t)(data_to_send.Node_N/10), 
                            (uint16_t)(data_to_send.Node_P/10), 
                            (uint16_t)(data_to_send.Node_K/10));
            ESP_LOGI("MQTT", "Topic: '%s'", temp[0]);
            ESP_LOGI("MQTT", "Payload: '%s'", temp[1]);
            if (len > 0)    {esp_mqtt_client_publish(mqtt_client, temp[0], temp[1], len, 0, 0); ESP_LOGI("MQTT", "Valid payload length -> Send MQTT...");}
            else {ESP_LOGI("MQTT", "Invalid payload length -> Put back to RX Queue"); xQueueSend(uart_queue_rx, &data_to_send, 100 / portTICK_RATE_MS);}
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        if ((is_WiFi_connected == true) && (is_MQTT_connected == false))
        {
            esp_mqtt_client_start(mqtt_client);
            timeKeeperMQTT = xTaskGetTickCount();
            while ((is_MQTT_connected == false) && ((xTaskGetTickCount() - timeKeeperMQTT) < 5000));
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void smartconfig_task(void *arg)
{
    EventBits_t uxBits;
    is_SC_task_running = true;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
    while (1)
    {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to Access Point (AP)");
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            is_WiFi_connected = true;
            ESP_LOGI(TAG, "Smartconfig Over");
            esp_smartconfig_stop();
            is_SC_task_running = false;
            vTaskDelete(NULL);
        }
    }
}

static void wifi_task(void *arg)
{
    static const char *BLINK_TASK_TAG = "WIFI_TASK";
    is_WiFi_connected = false;
    is_MQTT_connected = false;
    is_use_ESPTOUCH = false;
    is_WiFi_allowed = true;
    is_WiFi_task_running = true;
    wifi_config_t wifi_config;
    bzero(&wifi_config, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid, SSID, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, PASS, sizeof(wifi_config.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_disconnect());
    esp_wifi_connect();
    uint32_t timeKeeper = xTaskGetTickCount();
    while (1)
    {
        if ((is_WiFi_connected == false) && (is_use_ESPTOUCH == false) && ((xTaskGetTickCount() - timeKeeper) >= 20000))
        {
            is_use_ESPTOUCH = true;
            ESP_LOGI("WIFI", "WiFi change to ESP_TOUCH");
            xTaskCreate(smartconfig_task, "ESP_TOUCH", 4096, NULL, configMAX_PRIORITIES - 2, &SC_TASK_Handle);
        }

        if (is_WiFi_connected == true)
        {
            int retry = 0;
            static char strftime_buf[64] = {0};

            time(&now);
            localtime_r(&now, &timeinfo);
            // Is time set? If not, tm_year will be (1970 - 1900).
            if (timeinfo.tm_year < (2016 - 1900))
            {
                ESP_LOGI("SNTP", "Time is not set yet. Connecting to WiFi and getting time over NTP.");
                ESP_LOGI("SNTP", "Initializing SNTP");
                sntp_setoperatingmode(SNTP_OPMODE_POLL);
                sntp_setservername(0, "pool.ntp.org");
                sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
                sntp_init();
                // wait for time to be set
                while ((sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET) && (++retry < retry_count))
                {
                    ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                }
                time(&now);
                localtime_r(&now, &timeinfo);
                // update 'now' variable with current time
                time(&now);
            }
            // Set timezone to IndoChina Time
            setenv("TZ", "GTM-7", 1);
            tzset();
            localtime_r(&now, &timeinfo);
            strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
            ESP_LOGI("SNTP", "The current date/time in Viet Nam is: %s", strftime_buf);
            ESP_LOG_BUFFER_HEXDUMP("TIME", &timeinfo, sizeof(struct tm), ESP_LOG_INFO);
            uart_write_bytes(UART_NUM_2, "TIME-", 5U);
            uart_write_bytes(UART_NUM_2, &timeinfo, sizeof(struct tm));

            if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH)
            {
                struct timeval outdelta;
                while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS)
                {
                    adjtime(NULL, &outdelta);
                    ESP_LOGI("SNTP", "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                                (long)outdelta.tv_sec,
                                outdelta.tv_usec/1000,
                                outdelta.tv_usec%1000);
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                }
            }
            is_WiFi_task_running = false;
            vTaskDelete(WIFI_TASK_Handle);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

//------------------------------------------------------------------------------------------------------------------------

static void initialise_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_START))
    {
        return;
    }
    else if ((event_base == WIFI_EVENT) && (event_id == WIFI_EVENT_STA_DISCONNECTED))
    {
        is_WiFi_connected = false;
        uart_write_bytes(UART_NUM_2, "WIFI-DISCONNECTED", 17U);
        if (is_WiFi_allowed == true)    esp_wifi_connect();
    }
    else if ((event_base == IP_EVENT) && (event_id == IP_EVENT_STA_GOT_IP))
    {
        if (is_use_ESPTOUCH == false)
        {
            is_WiFi_connected = true;
            uart_write_bytes(UART_NUM_2, "WIFI-CONNECTED", 14U);
        }
        else xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    }
    else if ((event_base == SC_EVENT) && (event_id == SC_EVENT_SCAN_DONE))
    {
        ESP_LOGI(TAG, "Scan done");
    }
    else if ((event_base == SC_EVENT) && (event_id == SC_EVENT_FOUND_CHANNEL))
    {
        ESP_LOGI(TAG, "Found channel");
    }
    else if ((event_base == SC_EVENT) && (event_id == SC_EVENT_GOT_SSID_PSWD))
    {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = {0};
        uint8_t password[65] = {0};
        uint8_t rvd_data[33] = {0};

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true)
        {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
    
        esp_err_t err;
        err = nvs_set_str(my_nvs_handle, "ssid", (const char *)ssid);
        if (err != ESP_OK)  ESP_LOGI(TAG, "ERROR SAVING SSID");
        err = nvs_set_str(my_nvs_handle, "pass", (const char *)password);
        if (err != ESP_OK)  ESP_LOGI(TAG, "ERROR SAVING PASS");
        ESP_LOGI(TAG, "----");
        nvs_get_str(my_nvs_handle, "ssid", &SSID, &lengthSSID);
        nvs_get_str(my_nvs_handle, "pass", &PASS, &lengthPASS);
        printf("Check stored SSID: %s\n", SSID);
        printf("Check stored PASS: %s\n", PASS);

        if (evt->type == SC_TYPE_ESPTOUCH_V2)
        {
            ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i = 0; i < 33; i++)
            {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
        esp_wifi_connect();
    }
    else if ((event_base == SC_EVENT) && (event_id == SC_EVENT_SEND_ACK_DONE))
    {
        if (is_use_ESPTOUCH == true)    xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

//------------------------------------------------------------------------------------------------------------------------

/**
 * @brief Event handler registered to receive MQTT events
 *
 *        This function is called by the MQTT client event loop.
 *
 * @param handler_args User data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id)
    {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            uart_write_bytes(UART_NUM_2, "MQTT-CONNECTED", 14U);
            is_MQTT_connected = true;
            msg_id = esp_mqtt_client_subscribe(client, "SolGarden/Gateway/0x2508", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            uart_write_bytes(UART_NUM_2, "MQTT-DISCONNECTED", 17U);
            is_MQTT_connected = false;
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
            {
                log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
                log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
                log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
                ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

//------------------------------------------------------------------------------------------------------------------------

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}





