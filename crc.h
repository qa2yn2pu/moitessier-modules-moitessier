#ifndef __CRC_H__
#define __CRC_H__

#include <linux/types.h>

void CRC_init(uint32_t *crc);
uint32_t CRC_calc(uint8_t dat, uint32_t crc, uint8_t bitCnt);

#endif /* __CRC_H__ */
