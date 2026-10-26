#pragma once
#include <string>
#define HIGH 1
#define LOW 0
class String {
 public:
  std::string value;
  String() = default;
  String(const char *s) : value(s) {}
  template<class T> explicit String(T n) : value(std::to_string(n)) {}
  String &operator+=(const String &s) { value += s.value; return *this; }
  String &operator+=(const char *s) { value += s; return *this; }
  String &operator+=(char c) { value += c; return *this; }
};
extern unsigned long testNow;
extern int testPin, testLevel;
inline unsigned long millis() { return testNow; }
inline void digitalWrite(int pin, int level) { testPin = pin; testLevel = level; }
