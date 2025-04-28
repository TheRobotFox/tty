#include "TCA9555.h"
#include <array>
#include <set>

void waitForButton()
{
  while(!digitalRead(2));
  while(digitalRead(2));
  delay(100);
}

int getLane(uint64_t val){
  for(int i=0; i<64;i++){
    if(val&(1<<i)) return i;
  }
  return -1;
}
template<typename C>
void printBits(C val){
  for(int i=8*sizeof(C)-1; i>=0;i--){
    Serial.print(val&(1<<i) ? '1' : '0');
  }
  Serial.print('\n');
}

template<std::size_t N>
class TCAS
{

  std::array<TCA9555, N> m_tcas;

/*
  uint64_t read(uint16_t (TCA9555::*fn)())
  {
    uint64_t result = 0;
    for(int i=0; i<m_tcas.size(); i++)
      result |= (m_tcas[i].*fn)() << (16*(4-i));
    return result;
  }
*/
public:
  constexpr TCAS(std::array<TCA9555, N> arr) : m_tcas(arr)
  {}

  bool write(bool (TCA9555::*fn)(uint16_t), uint64_t mask)
  {
    bool result = true;
    for(int i=0; i<m_tcas.size(); i++)
      result &= (m_tcas[i].*fn)(mask>>(16*i));
    return result;
  }

  bool init()
  {
    Serial.println("Init I2C Wire");
    Wire1.setSDA(26);
    Wire1.setSCL(27);
    Wire1.begin();
    Wire1.setClock(50000);

    bool success = true;
    for(auto &t : m_tcas)
      success &= t.begin(OUTPUT);
    return success;
  }

  bool writeMode(uint64_t mask)
  {
    return write(&TCA9555::pinMode16, mask);
  }

  bool writePin(uint64_t mask)
  {
    return write(&TCA9555::write16, mask);
  }

  uint64_t readPin()
  {
    uint64_t result = 0;  
    for(int i=0; i<m_tcas.size(); i++)
      result |= (uint64_t)(m_tcas[i].read16()) << (16*(i));
    return result;
  }


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
  }

  int lastError()
  {
    int error = 0;
    for(auto &t : m_tcas) if (error = t.lastError()) return error;
    return 0;
  }
};

TCAS<2> tca({TCA9555(0x20, &Wire1), TCA9555(0x21, &Wire1)}); 

void setup() {

  Serial.begin(115200);
  delay(1000);
  Serial.println(tca.init() ? "yes" : "no");
  pinMode(2, INPUT);
}

void loop() {

  auto connected = tca.scan();
  for(auto [a,b] : connected) Serial.printf("(%d,%d) ", a,b);
  if(connected.size()) Serial.print('\n');
  delay(300);
  while(true);

  int error = tca.lastError();
  if(error) Serial.println("Error: " + String(error));
}
