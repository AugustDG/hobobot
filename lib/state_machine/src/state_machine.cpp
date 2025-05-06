#include "hb_state_machine/state_machine.h"

#include "utils.hpp"
#include <unordered_set>

bool state_machine_config_t::verify() const {
  // check if all states are unique
  std::unordered_set<uint32_t> unique_states(states.begin(), states.end());
  unique_states.erase(ALL_STATES);
  unique_states.erase(ALL_STATES_NO_SELF);

  if (unique_states.size() != states.size())
    return false;

  // check if all states can be reached from at least one other state
  std::unordered_set<uint32_t> reachable_states;
  for (const auto &transition : transitions)
    reachable_states.insert(transition.to_state);

  if (reachable_states.size() != states.size())
    return false;

  return true;
}

state_machine_t::state_machine_t(const state_machine_config_t &config) {
  current_state = config.initial_state;
  current_state_action = config.state_actions.at(config.initial_state);
  transitions.clear();
  state_actions.clear();
  should_log = config.should_log;

  for (const auto &transition : config.transitions) {
    // transition from specific state to another specific state
    if (transition.from_state != ALL_STATES && transition.from_state != ALL_STATES_NO_SELF) {
      transitions[transition.from_state][transition.to_state] = transition.callback;
      continue;
    }

    // from ALL_STATES(_NO_SELF) to a specific state
    for (const auto &state : config.states) {
      // if we're transitioning to the same state, but it's ALL_STATES (not ALL_STATES_NO_SELF)
      // we don't want to add a self-loop
      if (state == transition.to_state && transition.from_state == ALL_STATES)
        continue;

      // we go through all states and transition to the target state
      transitions[state][transition.to_state] = transition.callback;
    }

    // from a specific state to ALL_STATES(_NO_SELF)
    for (const auto &state : config.states) {
      // if we're transitioning to the same state, but it's ALL_STATES (not ALL_STATES_NO_SELF)
      // we don't want to add a self-loop
      if (state == transition.from_state && transition.to_state == ALL_STATES)
        continue;

      // we go through all states and transition to the target state
      transitions[transition.from_state][state] = transition.callback;
    }
  }

  for (const auto &state_callback : config.state_actions)
    state_actions[state_callback.first] = state_callback.second;
}

void state_machine_t::loop() { current_state_action(); }

void state_machine_t::set_state(uint32_t new_state) {
  if (current_state == new_state) {
    Serial.printf("State is already %u\n", new_state);
    return;
  }

  auto from_state = current_state;
  auto to_state = new_state;

  auto from_state_it = transitions.find(from_state);
  if (from_state_it == transitions.end())
    return;

  auto to_state_it = from_state_it->second.find(to_state);
  if (to_state_it == from_state_it->second.end()) {
    Serial.printf("Non-existent transition from %u to %u\n", from_state, to_state);
    return;
  }

  auto callback = to_state_it->second;
  if (callback)
    callback(from_state, to_state);

  auto state_action_it = state_actions.find(to_state);
  current_state_action = state_action_it->second;
  current_state = to_state;

  if (should_log)
    Serial.printf("State changed from %u to %u\n", from_state, to_state);
}
