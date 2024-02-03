// Original Author: Greg https://github.com/gmaldona/unique_thread.cpp
// Feature: stop adding another thread to the thread pool if the lock is not available

#pragma once

#include <memory>
#include <thread>

#include "vex.h"

namespace orange {
  typedef void (*func_ptr)(void);

  class unique_thread {
  public:
    unique_thread();

    static void execute(func_ptr callback);

    static std::unique_ptr<vex::mutex> &&get_lock();

    static func_ptr get_callback();
    static void set_callback(func_ptr callback);

  private:
    static std::unique_ptr<vex::mutex> lock;
    static func_ptr _callback;
  };

}  // namespace orange
