#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <string>
#include <stdexcept>
using byte = uint8_t;
constexpr int LOW=0,HIGH=1,INPUT=0,OUTPUT=1;
constexpr byte A0=54,A1=55,A2=56,A3=57,A4=58,A5=59,A6=60,A7=61,LED_BUILTIN=13;
#define F(x) x
template<class T> T constrain(T x,T lo,T hi) { return x<lo?lo:(x>hi?hi:x); }
struct Device {
  uint32_t now=0;
  int adc[8]={},pwm[70]={},level[70]={},room=63,blockingPrints=0;
  size_t readCount=0,readAt=0;
  std::string tx,rx;
};
inline Device* activeDevice=nullptr;
inline uint32_t millis() { return activeDevice->now; }
inline void pinMode(byte,int) {}
inline void digitalWrite(byte p,int v) { activeDevice->level[p]=v; }
inline void analogWrite(byte p,int v) { activeDevice->pwm[p]=v; }
inline int analogRead(byte p) { return activeDevice->adc[p-A0]; }
struct SerialProxy {
  void begin(unsigned long) {}
  int available() { return int(activeDevice->rx.size()-activeDevice->readAt); }
  int read() {
    if (!available()) return -1;
    ++activeDevice->readCount;
    return (unsigned char)activeDevice->rx[activeDevice->readAt++];
  }
  int availableForWrite() { return activeDevice->room; }
  size_t write(const uint8_t* data,size_t count) {
    if (int(count)>activeDevice->room) throw std::runtime_error("TX capacity exceeded");
    activeDevice->room-=int(count);
    activeDevice->tx.append(reinterpret_cast<const char*>(data),count);
    return count;
  }
  void println(const char* text) {
    const std::string line=std::string(text)+"\r\n";
    if (int(line.size())>activeDevice->room) ++activeDevice->blockingPrints;
    activeDevice->room=activeDevice->room>int(line.size())?activeDevice->room-int(line.size()):0;
    activeDevice->tx+=line;
  }
};
inline SerialProxy Serial;
