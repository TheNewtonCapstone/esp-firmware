#include "encoder.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static void pcnt_overflow_handler(void* arg) {
  Encoder* enc = static_cast<Encoder*>(arg);
  uint32_t status = 0;
  pcnt_get_event_status(enc->m_pcnt_unit, &status);

  if (status & PCNT_EVT_H_LIM) {
    enc->m_position += INT16_MAX;
  } else if (status & PCNT_EVT_L_LIM) {
    enc->m_position -= INT16_MAX;
  }
}

Encoder::Encoder(const unsigned int pin_a, const unsigned int pin_b,
                 const int cpr)
    : m_pin_a(pin_a),
      m_pin_b(pin_b),
      m_cpr(cpr),
      m_position(0),
      m_velocity(0.0f),
      m_last_count(0),
      m_last_time(0) {
  init_pcnt();
}

Encoder::~Encoder() { pcnt_isr_service_uninstall(); }

void Encoder::init_pcnt() {
  // Configure PCNT unit
  pcnt_config_t pcnt_config = {
      .pulse_gpio_num = static_cast<int>(m_pin_a),
      .ctrl_gpio_num = static_cast<int>(m_pin_b),
      .channel = PCNT_CHANNEL_0,
      .unit = PCNT_UNIT_0,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DEC,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_REVERSE,
      .counter_h_lim = INT16_MAX,
      .counter_l_lim = INT16_MIN,
  };
  pcnt_unit_config(&pcnt_config);

  // Enable input filter to eliminate glitches
  pcnt_set_filter_value(pcnt_config.unit, 100);
  pcnt_filter_enable(pcnt_config.unit);

  // Initialize counter
  pcnt_counter_pause(pcnt_config.unit);
  pcnt_counter_clear(pcnt_config.unit);

  // Register ISR handler for overflow
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(pcnt_config.unit, pcnt_overflow_handler, this);

  // Enable events
  pcnt_event_enable(pcnt_config.unit, PCNT_EVT_H_LIM);
  pcnt_event_enable(pcnt_config.unit, PCNT_EVT_L_LIM);

  // Start counting
  pcnt_counter_resume(pcnt_config.unit);

  m_pcnt_unit = pcnt_config.unit;
}

void Encoder::update() {
  int16_t count = 0;
  pcnt_get_counter_value(m_pcnt_unit, &count);

  // Calculate position
  int32_t delta = count - m_last_count;
  m_position += delta;
  m_last_count = count;

  // Calculate velocity
  uint32_t now = esp_timer_get_time();
  float dt = (now - m_last_time) / 1e6f;  // Convert microseconds to seconds
  if (dt > 0) {
    m_velocity =
        (delta / static_cast<float>(m_cpr)) / dt;  // Revolutions per second
  }
  m_last_time = now;
}

int Encoder::get_position() { return m_position; }

float Encoder::get_velocity() { return m_velocity; }
