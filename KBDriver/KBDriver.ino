#include "TCA9555.h"

#define SIO (uint8_t*)(0xd0000000)

unsigned char reverseBits(unsigned char b){
   return ((b * 0x0802LU & 0x22110LU) | (b * 0x8020LU & 0x88440LU)) * 0x10101LU >> 16; 
}

uint32_t GPIO_read()
{
  return *(uint32_t*)(SIO+0x4);
}
void GPIO_set(int mask)
{
  *(uint32_t*)(SIO+0x14)=mask;
}
void GPIO_clr(int mask)
{
  *(uint32_t*)(SIO+0x18)=mask;
}

int KB_GPIO_get(int pin)
{
  switch(pin){
    case 16: return 15;
    case 17: return 14;
    case 18: return 13;
    case 19: return 12;
    case 20: return 11;
    case 21: return 10;
    case 23: return 8;
    case 22: return 9;
    case 24: return 7;
  }
  return -1;
}

uint32_t KB_read(){
  uint32_t res = tca1.read8(0);
  res |= tca0.read8(0) << 8; 
  uint32_t internal_gpio = GPIO_read();
  res |= reverseBits(internal_gpio>>7) << 17;
  res | (internal_pio & (1<<15)) << 1;

  return res;
}

bool KB_write(uint32_t mask){
  tca1.write8(0,mask);
  tca0.write8(0,mask>>8);
  uint32_t set_mask = reverseBits(mask>>17)<<7 | (mask&(1<<16) >> 1),
           clr_mask = reverseBits((~mask)>>17)<<7 | ((~mask)&(1<<16) >> 1);
  GPIO_set(set_mask);
  GPIO_clr(clr_mask);
  
}
/*
  std::set<std::pair<int, int>> scan()
  {
    write(&TCA9555::setPolarity16, ~0x0);
    std::set<std::pair<int, int>> found;

    for(int i=0; i<32; i++){
      uint64_t selected = 1<<i;
      writePin(0);
      writeMode(~selected);

      uint64_t result = readPin() & ~selected & ~( i==5 ? 1<<4 : 0);
      for(auto [a,b] : found) result &= ~(1<<a);
      Serial.println(i);

      if(!result) continue;

      int triggered=0;
      while((triggered = getLane(result))>=0){
        found.insert(std::make_pair(i, triggered));
        result &= ~(1<<triggered);
      }
    }

    return found;
  }*/

TCA9555 tca0(0x20, &Wire1),
        tca1(0x21, &Wire1);

void setup() {

  Serial.println("Init I2C Wire");

  Wire1.setSDA(26);
  Wire1.setSCL(27);
  Wire1.setClock(50000);
  Wire1.begin();


  Serial.println("Init TCAs");

  tca0.begin(OUTPUT);
  tca0.setPolarity16(~0);

  tca1.begin(OUTPUT);
  tca1.setPolarity16(~0);

  Serial.println("Setup internal GPIOs");
  for(int i=0; i<16; i++) pinMode(i, OUTPUT);

}

void loop() {
  GPIO_set(1<<25);
  delay(500);
  GPIO_clr(1<<25);
  delay(500);
}
