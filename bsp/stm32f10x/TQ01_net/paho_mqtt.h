#ifndef __PAHO_MQTT_H__
#define __PAHO_MQTT_H__

#include <stdint.h>

#include <MQTTPacket.h>

#include <rtthread.h>

#ifdef MQTT_USING_TLS
#include <tls_client.h>
#endif

#ifndef PKG_PAHOMQTT_SUBSCRIBE_HANDLERS
#define MAX_MESSAGE_HANDLERS 1 /* redefinable - how many subscriptions do you want? */
#else
#define MAX_MESSAGE_HANDLERS PKG_PAHOMQTT_SUBSCRIBE_HANDLERS
#endif

#define MAX_PACKET_ID 65535 /* according to the MQTT specification - do not change! */

#define MQTT_SOCKET_TIMEO 6000

#ifdef MQTT_USING_TLS
#define MQTT_TLS_READ_BUFFER 4096
#endif

enum QoS
{
    QOS0,
    QOS1,
    QOS2
} __align(4);
typedef enum
{
    HTTP_POST = 1,
    HTTP_GET = 2,
} http_method;
/* all failure return codes must be negative */
enum returnCode
{
    PAHO_ERROR = -3,
    PAHO_BUFFER_OVERFLOW = -2,
    PAHO_FAILURE = -1,
    PAHO_SUCCESS = 0
};

typedef struct MQTTMessage
{
    enum QoS qos;
    unsigned char retained;
    unsigned char dup;
    unsigned short id;
    void *payload;
    size_t payloadlen;
} MQTTMessage;

typedef struct MessageData
{
    MQTTMessage *message;
    MQTTString *topicName;
} MessageData;

typedef struct MQTTClient MQTTClient;

struct MQTTClient
{
    const char *uri;
    int sock;

    MQTTPacket_connectData condata;

    unsigned int next_packetid, command_timeout_ms;
    size_t buf_size, readbuf_size;
    unsigned char *buf, *readbuf;
    int isconnected;
    int isInformed;
    int isQRcodegeted;
    int isparameterPutted;
    unsigned int keepAliveInterval;
    char ping_outstanding;
    unsigned int TimingInterval;
    unsigned int RealtimeInterval;

    uint32_t tick_ping;
    uint32_t tick_timeing;
    uint32_t tick_realtime;

    void (*connect_callback)(MQTTClient *);
    void (*online_callback)(MQTTClient *);
    void (*offline_callback)(MQTTClient *);

    struct MessagesubHandlers
    {
        char *topicFilter;
        void (*callback)(MQTTClient *, MessageData *);
        enum QoS qos;
    } messagesubHandlers[MAX_MESSAGE_HANDLERS]; /* Message handlers are indexed by subscription topic */

    void (*defaultMessageHandler)(MQTTClient *, MessageData *);

    /* publish interface */
#if defined(RT_USING_POSIX) && (defined(RT_USING_DFS_NET) || defined(SAL_USING_POSIX))
    int pub_pipe[2];
#else
    int pub_sock;
    int pub_port;
#endif /* RT_USING_POSIX && (RT_USING_DFS_NET || SAL_USING_POSIX) */

#ifdef MQTT_USING_TLS
    /* mbedtls session struct*/
    MbedTLSSession *tls_session;
#endif
};

/**
 * This function start a mqtt worker thread.
 *
 * @param client the pointer of MQTT context structure
 *
 * @return the error code, 0 on start successfully.
 */
extern int paho_mqtt_start(MQTTClient *client);

/**
 * This function publish message to specified mqtt topic.
 *
 * @param c the pointer of MQTT context structure
 * @param topicFilter topic filter name
 * @param message the pointer of MQTTMessage structure
 *
 * @return the error code, 0 on subscribe successfully.
 */
extern int MQTTPublish(MQTTClient *c, const char *topicName, MQTTMessage *message); /* copy */

int keepalive(MQTTClient *c);

int eland_http_request(http_method method,
                       char *request_uri, //uri
                       char *host_name,   //host
                       char *http_body,   //BODY
                       char *response);   //response 指針

int MQTT_CMD(MQTTClient *c, const char *cmd);
void mqtt_send_cmd(const char *send_str);
#endif /* __PAHO_MQTT_H__ */
