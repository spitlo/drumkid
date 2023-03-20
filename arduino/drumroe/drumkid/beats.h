// Default order of samples: kick, hat, snare, rim, tom

// Beats will be controlled by toggle switches so just keep one for now

#define NUM_BEATS 1

const byte beats[NUM_BEATS][5][2] PROGMEM = {
  {
    {B10100000,B00100000,},
    {B10101010,B10101010,},
    {B00001001,B01001000,},
    {B00001000,B00001001,},
    {B00000000,B00000001,},
  },
};
