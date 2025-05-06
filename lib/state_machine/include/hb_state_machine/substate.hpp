#pragma once

struct substate_t; // forward declaration

typedef bool (*pre_condition_t)();
typedef void (*pre_action_t)();
typedef void (*action_t)();
typedef void (*post_action_t)(substate_t *substate);
typedef bool (*condition_t)();

struct substate_t {
  pre_condition_t pre_condition = nullptr;
  pre_action_t pre_action = nullptr;
  action_t action;
  post_action_t post_action = nullptr;
  condition_t condition;

  bool pre_action_called = false;

  inline bool execute() {
    if (pre_condition && !pre_condition())
      return false; // pre-condition not met

    if (pre_action && !pre_action_called) {
      pre_action();
      pre_action_called = true;
    }

    if (action)
      action();

    if (condition()) {
      if (post_action)
        post_action(this);

      return true; // condition met
    }

    return false; // condition not met
  }

  inline void reset() { pre_action_called = false; }
};