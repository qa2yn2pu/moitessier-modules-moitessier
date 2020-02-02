#ifndef __HEADER_H__
#define __HEADER_H__

#include "typedefs.h"
#include <linux/types.h>

/* the size must be a multiple of 4 and use always uint32_t or 4* uint8_t or 2* uint16_t --> not data type change that is not on a 32 bit border  */
typedef struct
{
    uint32_t    sof;
    uint8_t     headerLen;
    uint8_t     version;
    uint8_t     type;
    uint8_t     resv1;
    uint32_t    source;
    uint32_t    target;
    uint16_t    resv2;
    uint16_t    payloadLen;
    uint16_t    payloadCRC;
    uint16_t	headerCRC;
}header_t;

#define CMD_FIELD_NOT_USED      0

typedef enum
{
    SOURCE_AIS1         = (1 << 0),
    SOURCE_AIS2         = (1 << 1),
    SOURCE_GNSS         = (1 << 2),
    SOURCE_PRESSURE     = (1 << 3),
    SOURCE_HUMIDITY     = (1 << 4),
    SOURCE_TEMPERATURE  = (1 << 5),
    SOURCE_SYSTEM       = (1 << 31),
}headerSource_t;

typedef enum
{
    CMD_CATEGORY_REQUEST        = 1,
    CMD_CATEGORY_RESPONSE,
}headerCmdCategory_t;

typedef enum
{
    CMD_INFO                = 1,
    CMD_CONFIG              = 4,
}headerCmd_t;

typedef enum
{
    CMD_INFO_SYSTEM         = 1,
}headerCmdInfo_t;

typedef struct
{
    /* if you change anything on these fields (add or remove elements, order) than you will need to change the
       main code as well */
    uint8_t category;
    uint8_t cmd;
    uint8_t cmdSub;
    uint8_t param;
}request_t;

typedef struct
{
    /* if you change anything on these fields (add or remove elements, order) than you will need to change the
       main code as well */
    uint8_t category;
    uint8_t cmd;
    uint8_t cmdSub;
}response_t;

#define HEADER_SOF_IDX						0
#define HEADER_SOF_FIELD_LEN				4
#define HEADER_LEN_IDX						(HEADER_SOF_IDX + HEADER_SOF_FIELD_LEN)
#define HEADER_LEN_FIELD_LEN				2
#define HEADER_VERSION_IDX					(HEADER_LEN_IDX + HEADER_LEN_FIELD_LEN)
#define HEADER_VERSION_FIELD_LEN			2
#define HEADER_TYPE_IDX						(HEADER_VERSION_IDX + HEADER_VERSION_FIELD_LEN)
#define HEADER_TYPE_FIELD_LEN				2
#define HEADER_RESV1_IDX					(HEADER_TYPE_IDX + HEADER_TYPE_FIELD_LEN)
#define HEADER_RESV1_FIELD_LEN				2
#define HEADER_SOURCE_IDX					(HEADER_RESV1_IDX + HEADER_RESV1_FIELD_LEN)
#define HEADER_SOURCE_FIELD_LEN				8
#define HEADER_TARGET_IDX					(HEADER_SOURCE_IDX + HEADER_SOURCE_FIELD_LEN)
#define HEADER_TARGET_FIELD_LEN				8
#define HEADER_RESV2_IDX					    (HEADER_TARGET_IDX + HEADER_TARGET_FIELD_LEN)
#define HEADER_RESV2_FIELD_LEN				4
#define HEADER_PAYLOAD_LEN_IDX				(HEADER_RESV2_IDX + HEADER_RESV2_FIELD_LEN)
#define HEADER_PAYLOAD_LEN_FIELD_LEN		4
//#define HEADER_PAYLOAD_LEN_IDX				(HEADER_TARGET_IDX + HEADER_TARGET_FIELD_LEN)
//#define HEADER_PAYLOAD_LEN_FIELD_LEN		4
#define HEADER_PAYLOAD_CRC_IDX				(HEADER_PAYLOAD_LEN_IDX + HEADER_PAYLOAD_LEN_FIELD_LEN)
#define HEADER_PAYLOAD_CRC_FIELD_LEN	    4
#define HEADER_CRC_IDX				        (HEADER_PAYLOAD_CRC_IDX + HEADER_PAYLOAD_CRC_FIELD_LEN)
#define HEADER_CRC_FIELD_LEN	            4
#define HEADER_PAYLOAD_IDX					(HEADER_CRC_IDX + HEADER_CRC_FIELD_LEN)

#define HEADER_SOF_VAL						0x021EE110
#define HEADER_LEN_VAL						HEADER_PAYLOAD_IDX
#define HEADER_VERSION_VAL					2
#define HEADER_TYPE_LISTENER_TALKER_VAL		0
#define HEADER_TYPE_CMD_VAL		            1
#define HEADER_TARGET_ALL_VAL               0xFFFFFFFF
#define HEADER_TARGET_UNSPECIFIED_VAL       0
#define HEADER_SOURCE_UNSPECIFIED_VAL       0

void HEADER_uint8_tToAsciiHex(uint8_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated);
void HEADER_uint16_tToAsciiHex(uint16_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated);
void HEADER_uint32_tToAsciiHex(uint32_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated);
uint8_t HEADER_asciiHexToUint8_t(uint8_t *hex);
uint16_t HEADER_asciiHexToUint16_t(uint8_t *hex);
uint32_t HEADER_asciiHexToUint32_t(uint8_t *hex);
int8_t HEADER_set(header_t *header, uint8_t *buf, uint32_t bufSize);
int8_t HEADER_get(uint8_t *buf, uint32_t bufSize, header_t *header);
int8_t HEADER_setDefaults(header_t *header);
uint8_t HEADER_size(void);
int8_t HEADER_crc(const uint8_t *buf, const uint32_t bufSize, uint16_t *crc);
int8_t HEADER_find(const uint8_t *buf, const uint32_t bufSize, const uint32_t startIdx, uint32_t *idx);
int8_t HEADER_isValid(header_t *header);
int8_t HEADER_payloadIsValid(header_t* header, uint8_t* payload);



#endif /* __HEADER_H__ */