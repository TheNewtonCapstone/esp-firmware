#pragma once

#include "driver/pulse_cnt.h"

class Encoder {
 public:
  Encoder(unsigned int pin_a, unsigned int pin_b, int cpr);
  ~Encoder();

  void update();  // Call this periodically to update velocity
  int get_position() const;
  float get_velocity() const;

 private:
  void init_pcnt();
  static bool on_reach(pcnt_unit_handle_t unit,
                       const pcnt_watch_event_data_t *edata, void *user_ctx);

  const unsigned int m_pin_a;
  const unsigned int m_pin_b;
  const int m_cpr;
  int m_position;
  float m_velocity;
  int m_last_count;
  int m_last_time;
  pcnt_unit_handle_t m_pcnt_unit;
};
