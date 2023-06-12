/**
 * @file sx1278.h
 * @author Kyuubi0323 (nguyenvankhoi8d@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-04-20
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef INC_SX1278_H_
#define INC_SX1278_H_
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
// Register definitions
#define REG_FIFO                        0x00
#define REG_OP_MODE                     0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                     0x07
#define REG_FRF_LSB                     0x08
#define REG_PA_CONFIG                   0x09
#define REG_LNA                         0x0c
#define REG_FIFO_ADDR_PTR               0x0d
#define REG_FIFO_TX_BASE_ADDR           0x0e
#define REG_FIFO_RX_BASE_ADDR           0x0f
#define REG_FIFO_RX_CURRENT_ADDR        0x10
#define REG_IRQ_FLAGS                   0x12
#define REG_RX_NB_BYTES                 0x13
#define REG_PKT_SNR_VALUE               0x19
#define REG_PKT_RSSI_VALUE              0x1a
#define REG_MODEM_CONFIG_1              0x1d
#define REG_MODEM_CONFIG_2              0x1e
#define REG_PREAMBLE_MSB                0x20
#define REG_PREAMBLE_LSB                0x21
#define REG_PAYLOAD_LENGTH              0x22
#define REG_MODEM_CONFIG_3              0x26
#define REG_RSSI_WIDEBAND               0x2c
#define REG_DETECTION_OPTIMIZE          0x31
#define REG_DETECTION_THRESHOLD         0x37
#define REG_SYNC_WORD                   0x39
#define REG_DIO_MAPPING_1               0x40
#define REG_VERSION                     0x42

// Transceiver modes
#define MODE_LONG_RANGE_MODE            0x80
#define MODE_SLEEP                      0x00
#define MODE_STDBY                      0x01
#define MODE_TX                         0x03
#define MODE_RX_CONTINUOUS              0x05
#define MODE_RX_SINGLE                  0x06
#define MODE_CAD                        0x07

// PA configuration
#define PA_BOOST                        0x80

// IRQ masks
#define IRQ_TX_DONE_MASK                0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK      0x20
#define IRQ_RX_DONE_MASK                0x40
#define IRQ_VALID_HEADER_MASK           0x10
#define IRQ_CAD_DETECTED_MASK			0x01
#define IRQ_CAD_DONE_MASK				0x04

#define PA_OUTPUT_RFO_PIN               0
#define PA_OUTPUT_PA_BOOST_PIN          1

#define TIMEOUT_RESET                   100

#define ACK                             0x06
#define NACK                            0x15

#define UPLINK_TX_REQUEST_OPCODE 		10
#define DOWNLINK_RX_DATA_OPCODE 		20

#define SX1278_DIO0_BIT 				(1 <<0)
#define SX1278_DIO3_BIT 				(1 <<1)
#define SX1278_DIO4_BIT 				(1 <<2)

#define BUFF "KYUUBI KYUUBI"

#define NW_DEFAULT_TOTAL_SLOTS 			10
#define NW_DEFAULT_PERIOD       		5
#define NW_DEFAULT_THRESHOLD 			25.0

typedef int sx1278_opcode_type_t;

typedef enum
{
    SX1278_OK,
    SX1278_NOT_OK,
    SX1278_INVALID_RX_DONE,
    SX1278_PAYLOAD_CRC_ERROR,
    SX1278_INVALID_HEADER,
} sx1278_err_t;

typedef struct
{
    int node_id;
    int slot_id;
    int period;
    char threshold[10];
    char EC[10];
    char soil[10];
    char PH[10];
    char humid[10];
    char temp[10];
    char battery[10];
} sx1278_node_slot_t;

typedef struct
{
    char opcode[5];
    char node_id[5];
    char gate_id[5];
    char soil[10];
    char EC[10];
	char PH[10];
	char humid[10];
    char temp[10];
	char battery[10];
    char period[5];
    char threshold[10];
    uint8_t crc;
} sx1278_packet_t;

typedef struct
{
    float threshold[10];
    int period;
} sx1278_attr_cfg_t;

typedef struct
{
    bool network_run;
} sx1278_flag_t;

typedef struct
{
    int total_slots;
    int gate_id;
    sx1278_node_slot_t node_slots[NW_DEFAULT_TOTAL_SLOTS];
    sx1278_flag_t flags;
} sx1278_network_t;

void sx1278_task(void *param);
sx1278_err_t parse_packet(uint8_t *packet_data, sx1278_node_slot_t *node_slot);


#endif /* INC_SX1278_H_ */
