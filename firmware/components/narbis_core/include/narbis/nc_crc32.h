/* CRC-32 (ISO-HDLC / zlib polynomial 0xEDB88320), used by OTA image
 * verification and host<->device fixture checks. Matches python zlib.crc32. */
#pragma once
#include <stdint.h>
#include <stddef.h>

uint32_t nc_crc32(uint32_t crc, const void *buf, size_t len); /* seed 0 */
