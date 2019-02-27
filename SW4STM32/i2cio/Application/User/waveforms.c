/*
int16_t sin16_C( uint16_t theta )
{
  static const uint16_t base[] =
  { 0, 6393, 12539, 18204, 23170, 27245, 30273, 32137 };
  static const uint8_t slope[] =
  { 49, 48, 44, 38, 31, 23, 14, 4 };

  uint16_t offset = (theta & 0x3FFF) >> 3; // 0..2047
  if( theta & 0x4000 ) offset = 2047 - offset;

    uint8_t section = offset / 256; // 0..7
    uint16_t b   = base[section];
    uint8_t  m   = slope[section];

    uint8_t secoffset8 = (uint8_t)(offset) / 2;

    uint16_t mx = m * secoffset8;
    int16_t  y  = mx + b;

    if( theta & 0x8000 ) y = -y;

    return y;
}

*/