#include "narbis/nc_crc32.h"

/* Nibble-table CRC-32: 64-byte table keeps flash/RAM cost negligible and
 * is plenty fast for OTA chunk rates. Verified against zlib.crc32 by
 * test_host/tests/t_proto_roundtrip.c and tools/tests. */
static const uint32_t crc_tab4[16] = {
    0x00000000, 0x1DB71064, 0x3B6E20C8, 0x26D930AC,
    0x76DC4190, 0x6B6B51F4, 0x4DB26158, 0x5005713C,
    0xEDB88320, 0xF00F9344, 0xD6D6A3E8, 0xCB61B38C,
    0x9B64C2B0, 0x86D3D2D4, 0xA00AE278, 0xBDBDF21C
};

uint32_t nc_crc32(uint32_t crc, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    crc = ~crc;
    while (len--) {
        crc ^= *p++;
        crc = (crc >> 4) ^ crc_tab4[crc & 0x0F];
        crc = (crc >> 4) ^ crc_tab4[crc & 0x0F];
    }
    return ~crc;
}
