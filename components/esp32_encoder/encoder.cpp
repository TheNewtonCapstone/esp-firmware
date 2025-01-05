#include "encoder.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

Encoder::Encoder(unsigned int pin_a, unsigned int pin_b, int cpr)
    : m_pin_a(pin_a),
      m_pin_b(pin_b),
      m_cpr(cpr),
      m_position(0),
      m_velocity(0.0f),
      m_last_count(0),
      m_last_time(0),
      m_pcnt_unit(nullptr) {
  init_pcnt();
}

Encoder::~Encoder() {
  if (m_pcnt_unit) {
    pcnt_unit_disable(m_pcnt_unit);
    pcnt_del_unit(m_pcnt_unit);
  }
}

void Encoder::init_pcnt() {
  // Configure PCNT unit
  pcnt_unit_config_t unit_config = {
      .low_limit = INT16_MIN,
      .high_limit = INT16_MAX,
      .intr_priority = 1,
      .flags = {.accum_count = 1},
  };
  pcnt_new_unit(&unit_config, &m_pcnt_unit);

  // Set edge and level actions for quadrature encoder
  pcnt_chan_config_t chan_a_config = {
      .edge_gpio_num = static_cast<int>(m_pin_a),
      .level_gpio_num = static_cast<int>(m_pin_b),
      .flags = {},
  };
  pcnt_channel_handle_t chan_a;
  pcnt_new_channel(m_pcnt_unit, &chan_a_config, &chan_a);

  pcnt_chan_config_t chan_b_config = {
      .edge_gpio_num = static_cast<int>(m_pin_b),
      .level_gpio_num = static_cast<int>(m_pin_a),
      .flags = {},
  };
  pcnt_channel_handle_t chan_b;
  pcnt_new_channel(m_pcnt_unit, &chan_b_config, &chan_b);

  // Add watch points for overflow handling
  pcnt_event_callbacks_t cbs = {
      .on_reach = on_reach,
  };
  pcnt_unit_register_event_callbacks(m_pcnt_unit, &cbs, this);
  pcnt_unit_add_watch_point(m_pcnt_unit, INT16_MAX);
  pcnt_unit_add_watch_point(m_pcnt_unit, INT16_MIN);

  // Enable PCNT unit
  pcnt_unit_enable(m_pcnt_unit);
  pcnt_unit_start(m_pcnt_unit);
}

bool Encoder::on_reach(pcnt_unit_handle_t unit,
                       const pcnt_watch_event_data_t *edata, void *user_ctx) {
  Encoder *enc = static_cast<Encoder *>(user_ctx);
  if (edata->watch_point_value == INT16_MAX) {
    enc->m_position += INT16_MAX;
  } else if (edata->watch_point_value == INT16_MIN) {
    enc->m_position -= INT16_MAX;
  }
  return false;  // Return false to indicate the event is not handled completely
}

void Encoder::update() {
  int count = 0;
  pcnt_unit_get_count(m_pcnt_unit, &count);
  printf("Count: %d\n", count);

  // Calculate position
  int delta = count - m_last_count;
  m_position += delta;
  m_last_count = count;

  // Calculate velocity
  unsigned int now = esp_timer_get_time();
  float dt = (now - m_last_time) / 1e6f;  // Convert microseconds to seconds
  if (dt > 0) {
    m_velocity =
        (delta / static_cast<float>(m_cpr)) / dt;  // Revolutions per second
  }
  m_last_time = now;
}

int Encoder::get_position() const { return m_position; }

float Encoder::get_velocity() const { return m_velocity; }
