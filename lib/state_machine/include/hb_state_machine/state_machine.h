#pragma once

#include <Arduino.h>

#define ALL_STATES uint32_t(-1)
#define ALL_STATES_NO_SELF uint32_t(-2)

typedef void (*state_change_callback_t)(uint32_t from_state, uint32_t to_state);
typedef void (*state_action_t)();

typedef std::unordered_map<uint32_t, std::unordered_map<uint32_t, state_change_callback_t>> transition_graph_t;
typedef std::unordered_map<uint32_t, state_action_t> state_action_map_t;

struct transition_t {
  uint32_t from_state;
  uint32_t to_state;
  state_change_callback_t callback = nullptr;
};

struct state_machine_config_t {
  uint32_t initial_state;

  std::vector<uint32_t> states;
  std::vector<transition_t> transitions;
  state_action_map_t state_actions;

  bool should_log = false;

  bool verify() const;
};

class state_machine_t {
public:
  uint32_t current_state;

private:
  state_action_t current_state_action;

  transition_graph_t transitions;
  state_action_map_t state_actions;

  bool should_log = false;

public:
  state_machine_t() = default;
  state_machine_t(const state_machine_config_t &config);

  void loop();
  void set_state(uint32_t new_state);
};