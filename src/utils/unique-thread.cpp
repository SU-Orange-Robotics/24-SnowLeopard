// Original Author: Greg https://github.com/gmaldona/unique_thread.cpp

#include "utils/unique-thread.h"

#include "vex.h"

using namespace orange;

std::unique_ptr<vex::mutex> unique_thread::lock(new vex::mutex());
func_ptr unique_thread::_callback = nullptr;

unique_thread::unique_thread() = default;

std::unique_ptr<vex::mutex> &&unique_thread::get_lock() {
   return std::move(lock);
}

func_ptr unique_thread::get_callback() { return _callback; }

void unique_thread::set_callback(func_ptr callback) { _callback = callback; }

void unique_thread::execute(func_ptr callback) {
   if (unique_thread::get_lock()->try_lock()) {
      unique_thread::set_callback(callback);
      vex::thread t([]() {
         unique_thread::get_callback()();
         unique_thread::get_lock()->unlock();
      });
      t.detach();
   }
}
