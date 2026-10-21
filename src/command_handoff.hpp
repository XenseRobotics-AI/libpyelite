#pragma once

#include <chrono>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace libpyelite {
// One outstanding station sample per arm. Unlike a latest-target mailbox,
// collision-approved output samples must not be overwritten before delivery.
// Report each reverse-socket attempt; only success frees the slot.
class CommandHandoff {
 public:
  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    published_=acknowledged_=attempts_=0;
    stopped_=false;
  }
  std::uint64_t publish() {
    std::lock_guard<std::mutex> lock(mutex_);
    if(stopped_ || published_!=acknowledged_)
      throw std::logic_error("Station command overwritten before sender acknowledgement");
    return ++published_;
  }
  void completeAttempt(std::uint64_t sequence, bool success) {
    std::lock_guard<std::mutex> lock(mutex_);
    // A repeated write of an older snapshot cannot acknowledge the next one.
    if(sequence==published_) {
      ++attempts_;
      if(success) acknowledged_=sequence;
      changed_.notify_all();
    }
  }
  bool waitFor(std::chrono::steady_clock::duration timeout, const std::atomic<bool> *running=nullptr) {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto cancelled=[&] { return stopped_ || (running && !running->load()); };
    while(!cancelled() && published_!=acknowledged_) {
      const auto attempts=attempts_;
      // Retrying a failed write is live progress, but does not free the slot.
      // The sender owns the configured consecutive-write-failure budget.
      if(!changed_.wait_for(lock,timeout,[&] {
        return cancelled() || published_==acknowledged_ || attempts_!=attempts;
      })) return false;
    }
    return !cancelled() && published_==acknowledged_;
  }
  void wake() {
    std::lock_guard<std::mutex> lock(mutex_);
    changed_.notify_all();
  }
  void stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    stopped_=true;
    changed_.notify_all();
  }
 private:
  std::mutex mutex_;
  std::condition_variable changed_;
  std::uint64_t published_{0}, acknowledged_{0}, attempts_{0};
  bool stopped_{false};
};
}  // namespace libpyelite
