#ifndef __PPU_ADDRESS_SPACE_H
#define __PPU_ADDRESS_SPACE_H

#ifdef USE_ADDR64
typedef uint64_t ppu_address_t;
#else
typedef uint32_t ppu_address_t;
#endif

#endif

