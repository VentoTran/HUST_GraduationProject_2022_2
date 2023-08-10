/**
 * @file mqtt.c
 * @author Vento (zseefvhu12345@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include "mqtt.h"
#include "sim.h"
#include "MQTTPacket.h"
#include "common.h"

#include "stdio.h"
#include "string.h"

#if FREERTOS == 1
#include "cmsis_os2.h"
#endif


MQTT_t MQTT = {
    .mqttServer.host = "test.mosquitto.org",
    .mqttServer.port = 1883,
    .mqttClient.clientID = "SolGarden-0x2508",
    .mqttClient.keepAliveInterval = 60,
    .mqttClient.username = "",
    .mqttClient.pass = "",
};
static char temp[1]= {26};

//-------------------------------------------------------------------------------------------------------------------------------------

/**
 * @brief 
 * 
 * @param mqtt 
 * @param len 
 * @param response 
 * @return true 
 * @return false 
 */
bool MQTT_SendMQTT(char* mqtt, uint8_t len, char* response)
{
    SIM_sendATCommand_withLength(mqtt, len);

    osDelay(CMD_DELAY_VERYSHORT);

    SIM_sendATCommand_withLength((char*)&temp, 1U);

    uint32_t timeOut = HAL_GetTick();
    isWaiting4Response = true;

    while ((isWaiting4Response == true) && ((HAL_GetTick() - timeOut) <= CMD_DELAY_MEDIUM))
    {
#if FREERTOS == 1
        osDelay(200);
#else
        HAL_Delay(200);
#endif
    }

    if (strstr((const char *)responseBuffer, (const char *)response) != NULL)   return true;
    return false;
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool MQTT_Connect(void)
{
    MQTT.mqttReceive.newEvent = 0;
    MQTT.mqttServer.connect = 0;
    bool state = true;
    char str1[150] = {0};
    unsigned char buf[150] = {0};

    sprintf(str1, "AT+CIPOPEN=0,\"TCP\",\"%s\",\"%d\"\r\n", MQTT.mqttServer.host, MQTT.mqttServer.port);
    state &= SIM_sendATCommandResponse(str1, "OK", CMD_DELAY_MEDIUM);

#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
    
    if (state == true)
    {
        MQTTPacket_connectData datas = MQTTPacket_connectData_initializer;
        datas.username.cstring = MQTT.mqttClient.username;
        datas.password.cstring = MQTT.mqttClient.pass;
        datas.clientID.cstring = MQTT.mqttClient.clientID;
        datas.keepAliveInterval = MQTT.mqttClient.keepAliveInterval;
        datas.cleansession = 1;
        int mqtt_len = MQTTSerialize_connect(buf, sizeof(buf), &datas);

#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        state &= SIM_sendATCommandResponse("AT+CIPSEND=0\r\n", ">", CMD_DELAY_MEDIUM);

        if (state == true)
        {
            state &= MQTT_SendMQTT((char*)buf, mqtt_len, "OK");
            if (state == true)  MQTT.mqttServer.connect = 1;
			return state;
        }
    }
    return false;
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @return true 
 * @return false 
 */
bool MQTT_Pub(char *topic, char *payload)
{
    unsigned char buf[256] = {0};
    bool state = true;

    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;
    
    int mqtt_len = MQTTSerialize_publish(buf, sizeof(buf), 0, 0, 0, 0, topicString, (unsigned char *)payload, (int)strlen(payload));

#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif

    state &= SIM_sendATCommandResponse("AT+CIPSEND=0\r\n", ">", CMD_DELAY_MEDIUM);

    if (state == true)
    {
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "OK");
        return state;
    }
    return false;
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 */
bool MQTT_PubUint8(char *topic, uint8_t payload)
{
    char str[32] = {0};
    sprintf(str, "%u", payload);
    return MQTT_Pub(topic, str);
}


void MQTT_PubUint16(char *topic, uint16_t payload)
{

}


void MQTT_PubUint32(char *topic, uint32_t payload)
{

}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @param digit 
 */
bool MQTT_PubFloat(char *topic, float payload, uint8_t digit)
{
    char str[32] = {0};
    ftoa(payload, str, digit);
    return MQTT_Pub(topic, str);
}

/**
 * @brief 
 * 
 * @param topic 
 * @param payload 
 * @param digit 
 */
bool MQTT_PubDouble(char *topic, double payload, uint8_t digit)
{
    char str[32] = {0};
    ftoa(payload, str, digit);
    return MQTT_Pub(topic, str);
}

/**
 * @brief 
 * 
 * @return true 
 * @return false 
 */
bool MQTT_PingReq(void)
{
    unsigned char buf[16] = {0};
    bool state = true;

    int mqtt_len = MQTTSerialize_pingreq(buf, sizeof(buf));

#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif

    state &= SIM_sendATCommandResponse("AT+CIPSEND=0\r\n", ">", CMD_DELAY_MEDIUM);

    if (state == true)
    {
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "SEND OK");
        return state;
    }
    return false;
}

/**
 * @brief 
 * 
 * @param topic 
 * @return true 
 * @return false 
 */
bool MQTT_Sub(char *topic)
{
    unsigned char buf[256] = {0};
    bool state = true;

    MQTTString topicString = MQTTString_initializer;
    topicString.cstring = topic;

    int mqtt_len = MQTTSerialize_subscribe(buf, sizeof(buf), 0, 1, 1, &topicString, 0);

#if FREERTOS == 1
    osDelay(CMD_DELAY_VERYSHORT);
#else
    HAL_Delay(CMD_DELAY_VERYSHORT);
#endif

    state &= SIM_sendATCommandResponse("AT+CIPSEND=0\r\n", ">", CMD_DELAY_MEDIUM);

    if (state == true)
    {
#if FREERTOS == 1
        osDelay(CMD_DELAY_VERYSHORT);
#else
        HAL_Delay(CMD_DELAY_VERYSHORT);
#endif
        state &= MQTT_SendMQTT((char*)buf, mqtt_len, "OK");
        return state;
    }
	return false;
}

/**
 * @brief 
 * 
 * @param buf 
 */
void MQTT_Receive(unsigned char *buf)
{
    memset(MQTT.mqttReceive.topic, 0, sizeof(MQTT.mqttReceive.topic));
    memset(MQTT.mqttReceive.payload, 0, sizeof(MQTT.mqttReceive.payload));
    MQTTString receivedTopic;
    unsigned char *payload;
    MQTTDeserialize_publish(&MQTT.mqttReceive.dup, &MQTT.mqttReceive.qos, &MQTT.mqttReceive.retained,
                            &MQTT.mqttReceive.msgId,
                            &receivedTopic, &payload, &MQTT.mqttReceive.payloadLen, buf,
                            sizeof(buf));
    memcpy(MQTT.mqttReceive.topic, receivedTopic.lenstring.data, receivedTopic.lenstring.len);
    MQTT.mqttReceive.topicLen = receivedTopic.lenstring.len;
    memcpy(MQTT.mqttReceive.payload, payload, MQTT.mqttReceive.payloadLen);
    MQTT.mqttReceive.newEvent = 1;
}

/**
 * @brief 
 * 
 * @param response 
 */
void MQTT_Callback(const char* response)
{
    uint8_t * pHelper = response;
    bool is_MQTT_Receive = false;

    while ((*pHelper != '\0') && (is_MQTT_Receive == false))
    {
        if ((pHelper[0] == '\n') && (pHelper[1] == '+'))
        {
            pHelper++;
            if (strstr(pHelper, "+IPD") != NULL)
            {
                is_MQTT_Receive = true;
            }
        }
        else    pHelper++;
    }

    if (is_MQTT_Receive == true)
    {
        uint8_t byteRecv = 0;
        sscanf(pHelper, "+IPD%d\r\n", &byteRecv);
        if (byteRecv > 20)
        {
            MQTT_Receive(pHelper + 8);
            logPC("\nReceive MQTT!\nTopic='%s'\nPayload='%s'\n", MQTT.mqttReceive.topic, MQTT.mqttReceive.payload);
        }
    }
}










