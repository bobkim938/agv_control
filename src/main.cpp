#include <Arduino.h>
#include <ArduinoRS485.h>
#include <bitset>

std::bitset<88> key1{0x010601000000000000978A};
std::bitset<88> key2{0x010602000000000000A48A};
std::bitset<88> key3{0x010604000000000000C28A};
std::bitset<88> key4{0x0106080000000000000E8A};
std::bitset<88> key5{0x010610000000000000968B};
std::bitset<88> key6{0x0106000010000000004689};
std::bitset<88> key7{0x010600000800000000668B};
std::bitset<88> key8{0x010600000400000000768A};
std::bitset<88> key9{0x010600000200000000FE8A};
std::bitset<88> key10{0x010600000100000000BA8A};


void setup() {
  RS485.begin(9600);
  
}

void loop() {
  
}
