#include "header.h"
#include "crc.h"

static uint8_t HEADER_toHexDigit(uint8_t n)
{
    if (n < 10) {
        return n + '0';
    }
    else {
        return (n - 10) + 'A';
    }
}

void HEADER_uint8_tToAsciiHex(uint8_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated)
{
    *(hex + 0) = HEADER_toHexDigit(c / 0x10);
    *(hex + 1) = HEADER_toHexDigit(c % 0x10);
    if(nullTerminated == NULL_TERMINATED)
        *(hex + 2) = '\0';
}

void HEADER_uint16_tToAsciiHex(uint16_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated)
{
    HEADER_uint8_tToAsciiHex((uint8_t)(c >> 8), hex, nullTerminated);
    HEADER_uint8_tToAsciiHex((uint8_t)(c >> 0), hex + 2, nullTerminated);
}

void HEADER_uint32_tToAsciiHex(uint32_t c, uint8_t *hex, NullTermination_TypeDef nullTerminated)
{
    HEADER_uint16_tToAsciiHex((uint16_t)(c >> 16), hex, nullTerminated);
    HEADER_uint16_tToAsciiHex((uint16_t)(c >> 0), hex + 4, nullTerminated);
}

static uint8_t HEADER_fromHexDigit(uint8_t hex)
{
    if (hex >= 'A')
        return hex - 'A' + 10;
    else
        return hex - '0';
}

uint8_t HEADER_asciiHexToUint8_t(uint8_t *hex)
{
    return (HEADER_fromHexDigit(hex[0]) << 4) + HEADER_fromHexDigit(hex[1]);
}

uint16_t HEADER_asciiHexToUint16_t(uint8_t *hex)
{
    return (((uint16_t)HEADER_fromHexDigit(hex[0]) << 12) + ((uint16_t)HEADER_fromHexDigit(hex[1]) << 8) + ((uint16_t)HEADER_fromHexDigit(hex[2]) << 4) + (uint16_t)HEADER_fromHexDigit(hex[3]));
}

uint32_t HEADER_asciiHexToUint32_t(uint8_t *hex)
{
    return (((uint32_t)HEADER_asciiHexToUint16_t(hex + 0) << 16) + (uint32_t)HEADER_asciiHexToUint16_t(hex + 4));
}

int8_t HEADER_set(header_t *header, uint8_t *buf, uint32_t bufSize)
{
    uint8_t i = 0;
    int8_t rc = -1;
    
    if(bufSize >= sizeof(header_t) && header && buf)
    {
        for(i = 0; i < HEADER_SOF_FIELD_LEN; i++)    
            buf[HEADER_SOF_IDX + i] = ((header->sof >> ((HEADER_SOF_FIELD_LEN - 1 - i) * 8)) & 0xFF);
        HEADER_uint8_tToAsciiHex(header->headerLen, buf + HEADER_LEN_IDX, NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(header->version, buf + HEADER_VERSION_IDX, NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(header->type, buf + HEADER_TYPE_IDX, NOT_NULL_TERMINATED);
        HEADER_uint8_tToAsciiHex(header->resv1, buf + HEADER_RESV1_IDX, NOT_NULL_TERMINATED);
        HEADER_uint32_tToAsciiHex(header->source, buf + HEADER_SOURCE_IDX, NOT_NULL_TERMINATED);
        HEADER_uint32_tToAsciiHex(header->target, buf + HEADER_TARGET_IDX, NOT_NULL_TERMINATED);
        HEADER_uint16_tToAsciiHex(header->resv2, buf + HEADER_RESV2_IDX, NOT_NULL_TERMINATED);
        HEADER_uint16_tToAsciiHex(header->payloadLen, buf + HEADER_PAYLOAD_LEN_IDX, NOT_NULL_TERMINATED);
        HEADER_uint16_tToAsciiHex(header->payloadCRC, buf + HEADER_PAYLOAD_CRC_IDX, NOT_NULL_TERMINATED);
        HEADER_uint16_tToAsciiHex(header->headerCRC, buf + HEADER_CRC_IDX, NOT_NULL_TERMINATED);
        rc = 0;
    }

    return rc;
}

int8_t HEADER_get(uint8_t *buf, uint32_t bufSize, header_t *header)
{
    uint8_t i = 0;
    int8_t rc = -1;
    
    if(bufSize >= sizeof(header_t) && header && buf)
    {
        header->sof = 0;
        for(i = 0; i < HEADER_SOF_FIELD_LEN; i++)
            header->sof     |= (buf[HEADER_SOF_IDX + i] << ((HEADER_SOF_FIELD_LEN - 1 - i) * 8));
        header->headerLen   = HEADER_asciiHexToUint8_t(buf + HEADER_LEN_IDX);
        header->version     = HEADER_asciiHexToUint8_t(buf + HEADER_VERSION_IDX);  
        header->type        = HEADER_asciiHexToUint8_t(buf + HEADER_TYPE_IDX);
        header->resv1       = HEADER_asciiHexToUint8_t(buf + HEADER_RESV1_IDX);
        header->source      = HEADER_asciiHexToUint32_t(buf + HEADER_SOURCE_IDX);
        header->target      = HEADER_asciiHexToUint32_t(buf + HEADER_TARGET_IDX);
        header->resv2       = HEADER_asciiHexToUint16_t(buf + HEADER_RESV2_IDX);
        header->payloadLen  = HEADER_asciiHexToUint16_t(buf + HEADER_PAYLOAD_LEN_IDX);
        header->payloadCRC  = HEADER_asciiHexToUint16_t(buf + HEADER_PAYLOAD_CRC_IDX);
        header->headerCRC   = HEADER_asciiHexToUint16_t(buf + HEADER_CRC_IDX);
        rc = 0;
    }
    
    return rc;
}

int8_t HEADER_setDefaults(header_t *header)
{
    int8_t rc = -1;
    
    if(header)
    {
        header->sof         = HEADER_SOF_VAL;
    	header->headerLen   = HEADER_LEN_VAL;
        header->version     = HEADER_VERSION_VAL;
    	header->type        = HEADER_TYPE_LISTENER_TALKER_VAL;
    	
    	/* the following values must be set by the application */
    	header->resv1       = 0;
    	header->source      = HEADER_SOURCE_UNSPECIFIED_VAL; 
    	header->target      = HEADER_TARGET_UNSPECIFIED_VAL;
    	header->resv2       = 0;
    	header->payloadLen  = 0;       
    	header->payloadCRC  = 0;
    	header->headerCRC   = 0;
    	
    	rc = 0;
    }
    
    return rc;
}

/* get the size of the header */
uint8_t HEADER_size(void)
{
    return HEADER_LEN_VAL;
}

int8_t HEADER_crc(const uint8_t *buf, const uint32_t bufSize, uint16_t *crc)
{
    int8_t rc = -1;
    uint32_t i = 0;
    uint32_t c = 0;
    
    if(crc && buf)
    {
        CRC_init(&c);
        for(i = 0; i < bufSize; i++)
        {
            c = CRC_calc(buf[i], c, 8);
        }
        
        *crc = (uint16_t)(c & 0xFFFF);
        rc = 0;
    }
    return rc;
}

int8_t HEADER_find(const uint8_t *buf, const uint32_t bufSize, const uint32_t startIdx, uint32_t *idx)
{
    uint32_t i = 0;
    
    if(bufSize < HEADER_size() || idx == NULL || (startIdx + HEADER_SOF_FIELD_LEN) > bufSize)
        return -1;
        
    *idx = 0;
            
    for(i = startIdx; i < bufSize - startIdx; i++)
    {
        if(buf[i] == ((HEADER_SOF_VAL >> 24) & 0xFF) && \
           buf[i + 1] == ((HEADER_SOF_VAL >> 16) & 0xFF) && \
           buf[i + 2] == ((HEADER_SOF_VAL >> 8) & 0xFF) && \
           buf[i + 3] == ((HEADER_SOF_VAL >> 0) & 0xFF))
        {
            *idx = i;
            return 0;
        }
    }
    
    return -1;
}

int8_t HEADER_isValid(header_t *header)
{
    uint16_t crc = 0;
    
    if(header == NULL)
        return -1;
        
    if(HEADER_crc((uint8_t*)header, sizeof(header_t) - sizeof(uint16_t), &crc))
    {
        return -1;
    }
    
    if(crc == header->headerCRC)
        return 0;
        
    return -1;
}

int8_t HEADER_payloadIsValid(header_t* header, uint8_t* payload)
{
    uint16_t crc = 0;
    
    if(header == NULL || payload == NULL)
        return -1;
        
    if(HEADER_crc(payload, header->payloadLen, &crc))
    {
        return -1;
    }
    
    if(crc == header->payloadCRC)
        return 0;
        
    return -1;
}