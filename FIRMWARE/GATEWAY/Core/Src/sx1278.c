/**
 * @file sx1278.c
 * @author Kyuubi0323 (nguyenvankhoi8d@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <time.h>

#include "main.h"
#include "cmsis_os.h"

#include "spi.h"
#include "gpio.h"

#include "sx1278.h"
#include "common.h"
#include "event_groups.h"

extern SPI_HandleTypeDef hspi1;
static const char *TAG = "SX1278_GATEWAY";
//static spi_device_handle_t spi_handle;//esp

EventGroupHandle_t sx1278_evt_group;
sx1278_network_t sx1278_network = {0};
sx1278_attr_cfg_t attr_cfg_temp = {0};

void reverse(char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

int intToStr(int x, char str[], int d)
{
    int i = 0;
    if (x == 0)
        str[i++] = '0';

    while (x)
    {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }

    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

void ftoa(double n, char *res, int afterpoint)
{
    int ipart = (int)n;
    double fpart = n - (double)ipart;
    int i = intToStr(ipart, res, 0);
    if (afterpoint != 0)
    {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

//void IRAM_ATTR sx1278_intr_handler(void *arg)
//{
//    gpio_num_t pin = (gpio_num_t)arg;
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if (pin == SX1278_DIO0_PIN)
//        xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO0_BIT, &xHigherPriorityTaskWoken);
//    // else if (pin == SX1278_DIO3_PIN)
//    //     xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO3_BIT, &xHigherPriorityTaskWoken);
//    // else if (pin == SX1278_DIO4_PIN)
//    //     xEventGroupSetBitsFromISR(sx1278_evt_group, SX1278_DIO4_BIT, &xHigherPriorityTaskWoken);
//}

uint8_t sx1278_read_reg(uint8_t reg)
{
	uint8_t txByte = reg & 0x7f;
	uint8_t rxByte = 0x00;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &txByte, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Receive(&hspi1, &rxByte, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    return rxByte;
}

void sx1278_write_reg(uint8_t reg, uint8_t val)
{
	uint8_t ureg = reg | 0x80;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &ureg, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi1, &val, 1, 1000);
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void sx1278_reset(void)
{
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4, GPIO_PIN_RESET);
    vTaskDelay(10 / portTICK_RATE_MS);
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
    vTaskDelay(10 / portTICK_RATE_MS);
}

void sx1278_sleep(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void sx1278_standby(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

void sx1278_rx_contiuous(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

void sx1278_rx_single(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_SINGLE);
}

void sx1278_tx(void)
{
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
}

void sx1278_set_irq(uint8_t val)
{
    sx1278_write_reg(REG_DIO_MAPPING_1, val);
}

void sx1278_cad(void)
{
    sx1278_set_irq(0x80);
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_CAD);
}

void sx1278_set_tx_power(uint8_t output_power)
{
    if (output_power > 15)
    {
        LOG(TAG, "Invalid output power");
        return;
    }
    // PA output pin: PA_BOOST pin
    sx1278_write_reg(REG_PA_CONFIG, PA_BOOST | output_power);
}

void sx1278_set_LNA_gain(uint8_t gain)
{
    if (gain > 6)
    {
        LOG(TAG, "Invalid gain");
        return;
    }

    if (gain == 0)
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x04);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_3, 0x00);
        sx1278_write_reg(REG_LNA, sx1278_read_reg(REG_LNA) | (gain << 5));
    }
}

void sx1278_set_freq(uint64_t freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    sx1278_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    sx1278_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    sx1278_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void sx1278_set_bandwidth(long band)
{
    int bw;
    if (band <= 7.8E3)
        bw = 0;
    else if (band <= 10.4E3)
        bw = 1;
    else if (band <= 15.6E3)
        bw = 2;
    else if (band <= 20.8E3)
        bw = 3;
    else if (band <= 31.25E3)
        bw = 4;
    else if (band <= 41.7E3)
        bw = 5;
    else if (band <= 62.5E3)
        bw = 6;
    else if (band <= 125E3)
        bw = 7;
    else if (band <= 250E3)
        bw = 8;
    else
        bw = 9;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

void sx1278_set_sf(uint8_t sf)
{
    if (sf < 6 || sf > 12)
    {
        LOG(TAG, "Invalid spreading factor");
        return;
    }

    if (sf == 6)
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
    }
    else
    {
        sx1278_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
        sx1278_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
    }
    sx1278_write_reg(REG_MODEM_CONFIG_2, (sx1278_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

void sx1278_set_cr(uint8_t cr)
{
    if (cr < 5 || cr > 8)
    {
        LOG(TAG, "Invalid coding rate");
        return;
    }

    cr = cr - 4;
    sx1278_write_reg(REG_MODEM_CONFIG_1, (sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

void sx1278_set_header(bool en, uint32_t size)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
    else
    {
        sx1278_write_reg(REG_MODEM_CONFIG_1, sx1278_read_reg(REG_MODEM_CONFIG_1) | 0x01);
        sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    }
}

void sx1278_set_crc(bool en)
{
    if (en)
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) | 0x04);
    else
        sx1278_write_reg(REG_MODEM_CONFIG_2, sx1278_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

void sx1278_set_preamble(int len)
{
    sx1278_write_reg(REG_PREAMBLE_MSB, (uint8_t)(len >> 8));
    sx1278_write_reg(REG_PREAMBLE_LSB, (uint8_t)(len >> 0));
}

int sx1278_get_rssi(void)
{
    return (sx1278_read_reg(REG_PKT_RSSI_VALUE) - 164);
}

float sx1278_get_snr(void)
{
    return ((int8_t)sx1278_read_reg(REG_PKT_SNR_VALUE) * 0.25);
}

void sx1278_init(void)
{
    sx1278_reset();
    uint8_t ver = sx1278_read_reg(REG_VERSION);
    char data_log[20];
    sprintf(data_log, "Version: 0x%02x", (char)ver);
    LOG(TAG, data_log);
    sx1278_sleep();
    sx1278_write_reg(REG_FIFO_RX_BASE_ADDR, 0x00);
    sx1278_write_reg(REG_FIFO_TX_BASE_ADDR, 0x00);
    sx1278_set_LNA_gain(0);
    sx1278_set_tx_power(15);
    sx1278_set_freq(434E6);
    sx1278_set_bandwidth(125E3);
    sx1278_set_sf(7);
    sx1278_set_cr(5);
    sx1278_set_preamble(12);
    sx1278_set_header(true, 0);
    sx1278_set_crc(true);
    // sx1278_set_irq(0x00);
    sx1278_standby();
}

void sx1278_send_data(uint8_t *data_send, int size)
{
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, 0);
    sx1278_write_reg(REG_PAYLOAD_LENGTH, 0);
    for (int index = 0; index < size; index++)
    {
        sx1278_write_reg(REG_FIFO, data_send[index]);
    }
    sx1278_write_reg(REG_PAYLOAD_LENGTH, size);
    // Start transmission and wait for conclusion
    sx1278_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
    while (!(sx1278_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK))
    {
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    sx1278_write_reg(REG_IRQ_FLAGS, irq);
    // sx1278_sleep();
}

void sx1278_start_recv_data(void)
{
    sx1278_set_irq(0x00);
    sx1278_rx_contiuous();
}

sx1278_err_t sx1278_recv_data(uint8_t *data_recv, int *rssi, float *snr, sx1278_node_slot_t *node_slot)
{
    int irq = sx1278_read_reg(REG_IRQ_FLAGS);
    memset((char *)data_recv, '\0', strlen((char *)data_recv));
    sx1278_write_reg(REG_IRQ_FLAGS, irq);

    if (!(irq & IRQ_RX_DONE_MASK))
    {
        LOG(TAG, "Invalid RxDone Interrupt");
        return SX1278_INVALID_RX_DONE;
    }

    if (!(irq & IRQ_VALID_HEADER_MASK))
    {
        LOG(TAG, "Invalid Header Interrupt");
        return SX1278_INVALID_HEADER;
    }

    if (irq & IRQ_PAYLOAD_CRC_ERROR_MASK)
    {
        LOG(TAG, "Payload Crc Error Interrupt");
        return SX1278_PAYLOAD_CRC_ERROR;
    }

    int len = sx1278_read_reg(REG_RX_NB_BYTES);
    *rssi = sx1278_get_rssi();
    *snr = sx1278_get_snr();
    sx1278_standby();
    sx1278_write_reg(REG_FIFO_ADDR_PTR, sx1278_read_reg(REG_FIFO_RX_CURRENT_ADDR));
    for (int index = 0; index < len; index++)
    {
        data_recv[index] = sx1278_read_reg(REG_FIFO);
    }
    return parse_packet(data_recv, node_slot);
}

int get_random_value(int min, int max)
{
    int random;
    if (max <= min)
    {
        LOG(TAG, "Range error");
        return 0;
    }
    return random = min + rand() % (max + 1 - min);
}

uint8_t get_crc_value(uint8_t *data, int len)
{
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        crc ^= data[i];
    }
    return crc;
}

void send_request(sx1278_opcode_type_t opcode, sx1278_node_slot_t node_slot, uint8_t *packet)
{
    memset((char *)packet, '\0', strlen((char *)packet));
    if (opcode == UPLINK_TX_REQUEST_OPCODE)
    {
        sprintf((char *)packet, "$,%d,%d,%d,%d,%s,*", opcode, node_slot.node_id, sx1278_network.gate_id, node_slot.period, node_slot.threshold);
        int size = strlen((char *)packet);
        packet[size] = get_crc_value(packet, size);
    }
    //LOG(TAG, "PACKET: %s, LEN: %d, CRC: 0x%02x", packet, strlen((char *)packet), packet[strlen((char *)packet) - 1]);
    sx1278_send_data(packet, strlen((char *)packet));
}

sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_slot_t *node_slot)
{
    sx1278_packet_t packet;
    char packet_log[100];
    sprintf(packet_log, " 0x%02x", (char) packet_data);
    LOG(TAG, packet_log);
    int packet_len = strlen((char *)packet_data);
    packet.crc = get_crc_value(packet_data, packet_len - 1);
    if (packet.crc != packet_data[packet_len - 1])
    {
        LOG(TAG, "ERROR CRC");
        return SX1278_NOT_OK;
    }
    else
    {
        int res = sscanf((char *)packet_data,"$,%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],%[^,],*", packet.opcode, packet.node_id, packet.gate_id, packet.temp, packet.battery, packet.period, packet.threshold);
        if (res < 7)
        {
            LOG(TAG, "ERROR PACKET");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.node_id) != node_slot->node_id)
        {
            LOG(TAG, "ERROR NODE_ID");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.opcode) != DOWNLINK_RX_DATA_OPCODE)
        {
            LOG(TAG, "ERROR OPCODE");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.gate_id) != sx1278_network.gate_id)
        {
            LOG(TAG, "ERROR GATE_ID");
            return SX1278_NOT_OK;
        }
        if (atoi(packet.period) != node_slot->period)
        {
            LOG(TAG, "ERROR PERIOD");
            return SX1278_NOT_OK;
        }
        if (strcmp(packet.threshold, node_slot->threshold) != 0)
        {
            LOG(TAG, "ERROR THRESHOLD");
            return SX1278_NOT_OK;
        }
        strcpy(node_slot->temp, packet.temp);
        strcpy(node_slot->battery, packet.battery);
        strcpy(node_slot->PH, packet.PH);
        strcpy(node_slot->humid, packet.humid);
        strcpy(node_slot->soil, packet.soil);
        char packet_recv[100];
        sprintf(packet_recv, "RECV PACKET: opcode:%s, node_id:%s, gate_id:%s, temp:%s, battery:%s, period:%s, threshold:%ls", packet.opcode, packet.node_id, packet.gate_id, packet.temp, packet.battery, packet.period, packet.threshold);
        LOG(TAG, packet_recv);
        return SX1278_OK;
    }
}

bool listen_before_talk(void)
{
    EventBits_t evt_bits;
    while (1)
    {
        xEventGroupClearBits(sx1278_evt_group, SX1278_DIO0_BIT);
        sx1278_cad();
        evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
        if (evt_bits & SX1278_DIO0_BIT) // CadDone
        {
            int irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            LOG(TAG, "Cad timeout");
            sx1278_standby();
            return true;
        }
        else
        {
            int irq = sx1278_read_reg(REG_IRQ_FLAGS);
            sx1278_write_reg(REG_IRQ_FLAGS, irq);
            sx1278_standby();
            TickType_t time_delay = (TickType_t)get_random_value(0, 50);
            char time[30];
            sprintf(time,"Cad Detected, sleep in %lx",time_delay);
            LOG(TAG, time);
            vTaskDelay(time_delay / portTICK_RATE_MS);
        }
    }
}

void sx1278_task(void *param)
{
    uint8_t data_send[128] = {0};
    uint8_t data_recv[128] = {0};
    float snr;
    int rssi;
    EventBits_t evt_bits;
    TickType_t period_tick;
    int slot_cnt = 0;
    sx1278_gpio_init();
    sx1278_spi_init();
    sx1278_init();
    sx1278_evt_group = xEventGroupCreate();
    sx1278_network.total_slots = NW_DEFAULT_TOTAL_SLOTS;
    sx1278_network.gate_id = 100;
    while (1)
    {
        if (sx1278_network.flags.network_run == true)
        {
            if (slot_cnt == 0)
            {
                if (attr_cfg_temp.period < 5)
                {
                    for (int i = 0; i < NW_DEFAULT_TOTAL_SLOTS; i++)
                    {
                        sx1278_network.node_slots[i].period = NW_DEFAULT_PERIOD;
                    }
                }
                else
                {
                    for (int i = 0; i < NW_DEFAULT_TOTAL_SLOTS; i++)
                    {
                        sx1278_network.node_slots[i].period = attr_cfg_temp.period;
                    }
                }
                for (int i = 0; i < NW_DEFAULT_TOTAL_SLOTS; i++)
                {
                    char threshold[10] = {0};
                    ftoa(attr_cfg_temp.threshold[i], threshold, 1);
                    strcpy(sx1278_network.node_slots[i].threshold, threshold);
                }
            }
            period_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "Period: %d", sx1278_network.node_slots[slot_cnt].period);
            while ((uint32_t)(xTaskGetTickCount() - period_tick) / 1000 < ((uint32_t)sx1278_network.node_slots[slot_cnt].period) / portTICK_RATE_MS)
            {
                ESP_LOGI(TAG, "Slot %d, listening channel", slot_cnt);
                listen_before_talk();
                ESP_LOGI(TAG, "Send opcode %d", UPLINK_TX_REQUEST_OPCODE);
                send_request(UPLINK_TX_REQUEST_OPCODE, sx1278_network.node_slots[slot_cnt], data_send);
                ESP_LOGI(TAG, "Start recv opcode");
                sx1278_start_recv_data();
                evt_bits = xEventGroupWaitBits(sx1278_evt_group, SX1278_DIO0_BIT, pdTRUE, pdFALSE, 1000 / portTICK_PERIOD_MS);
                if (evt_bits & SX1278_DIO0_BIT) // RxDone
                {
                    ESP_LOGI(TAG, "RxDone");
                    if (sx1278_recv_data(data_recv, &rssi, &snr, &sx1278_network.node_slots[slot_cnt]) == SX1278_OK)
                    {
                        // DO SOMETHING
                        while ((uint32_t)(xTaskGetTickCount() - period_tick) / 1000 < ((uint32_t)sx1278_network.node_slots[slot_cnt].period) / portTICK_RATE_MS)
                        {
                            vTaskDelay(10 / portTICK_RATE_MS);
                        }
                    }
                }
                else
                {
                    sx1278_standby();
                    ESP_LOGI(TAG, "Timeout");
                }
            }
            if (slot_cnt == NW_DEFAULT_TOTAL_SLOTS - 1)
                slot_cnt = 0;
            else
            {
                if (sx1278_network.node_slots[slot_cnt + 1].node_id != 0)
                    slot_cnt++;
                else
                    slot_cnt = 0;
            }
        }
        else
        {
            vTaskDelay(10 / portTICK_RATE_MS);
        }
    }
}
