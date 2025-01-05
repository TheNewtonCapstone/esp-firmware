#pragma once

#include <driver/pcnt.h>

static void pcnt_overflow_handler(void* arg);

class Encoder {
 public:
  Encoder(const unsigned int pin_a, const unsigned int pin_b, const int cpr);
  ~Encoder();

  void update();

  int get_position();
  float get_velocity();

 private:
  void init_pcnt();

  const unsigned int m_pin_a;
  const unsigned int m_pin_b;
  const int m_cpr;
  int m_position;
  float m_velocity;
  int m_last_count;
  uint32_t m_last_time;
  pcnt_unit_t m_pcnt_unit;
};