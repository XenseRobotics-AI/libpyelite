// Hardware-free scheduling regression. Compile with: c++ -std=c++17 -pthread
// tests/test_command_handoff.cpp -o /tmp/test_command_handoff
#include "../src/command_handoff.hpp"
#include <array>
#include <atomic>
#include <cassert>
#include <future>
#include <thread>

using libpyelite::CommandHandoff;
using namespace std::chrono_literals;

int main() {
  CommandHandoff handoff;
  const auto first=handoff.publish();
  bool rejected=false;
  try { handoff.publish(); } catch(const std::logic_error &) { rejected=true; }
  assert(rejected);
  // A failed socket write does not acknowledge or advance anything.
  handoff.completeAttempt(first,false);
  assert(!handoff.waitFor(1ms));
  handoff.completeAttempt(first,true);
  assert(handoff.waitFor(0ms));
  const auto second=handoff.publish();
  handoff.completeAttempt(first,true);  // late repeat of an old sender snapshot
  assert(!handoff.waitFor(0ms));
  handoff.completeAttempt(second,true);
  assert(handoff.waitFor(0ms));
  handoff.publish();
  auto waiting=std::async(std::launch::async,[&] { return handoff.waitFor(2s); });
  handoff.stop();
  assert(!waiting.get());
  handoff.reset();
  assert(handoff.waitFor(0ms));
  assert(handoff.publish()==1);
  // Stopping the coordinator wakes its wait without poisoning a driver's slot.
  std::atomic<bool> running{true};
  auto cancelled=std::async(std::launch::async,[&] { return handoff.waitFor(2s,&running); });
  running=false;
  handoff.wake();
  assert(!cancelled.get());
  handoff.completeAttempt(1,true);
  assert(handoff.waitFor(0ms));
  const auto retry=handoff.publish();
  auto retrying=std::async(std::launch::async,[&] {
    for(int i=0;i<30;++i) {
      std::this_thread::sleep_for(5ms);
      handoff.completeAttempt(retry,false);
    }
    handoff.completeAttempt(retry,true);
  });
  // Live retries may outlast the planner watchdog without dropping the frame.
  assert(handoff.waitFor(100ms));
  retrying.get();

  // Two asynchronous senders, with repeated reads, delayed periods and failed
  // writes. A producer may publish the next *pair* only after both succeeded.
  struct Arm {
    CommandHandoff handoff;
    std::mutex mutex;
    std::uint64_t snapshot{0}, delivered{0};
    unsigned failures{0};
  };
  std::array<Arm,2> arms;
  std::atomic<bool> done{false};
  std::array<std::thread,2> senders;
  for(std::size_t a=0;a<2;++a) senders[a]=std::thread([&,a] {
    auto &arm=arms[a];
    unsigned tick=0;
    while(!done) {
      std::this_thread::sleep_for(++tick%13==0 ? 3ms : 1ms);
      std::uint64_t sequence;
      { std::lock_guard<std::mutex> lock(arm.mutex); sequence=arm.snapshot; }
      if(tick%(a+3)==0) {
        ++arm.failures; arm.handoff.completeAttempt(sequence,false); continue;
      }
      // No skipped output sample, even when the other arm is faster.
      assert(sequence==arm.delivered || sequence==arm.delivered+1);
      arm.delivered=sequence;
      arm.handoff.completeAttempt(sequence,true);
    }
  });
  for(std::uint64_t sequence=1;sequence<=300;++sequence) {
    for(auto &arm:arms) assert(arm.handoff.waitFor(2s));
    for(auto &arm:arms) {
      std::lock_guard<std::mutex> lock(arm.mutex);
      arm.snapshot=arm.handoff.publish();
      assert(arm.snapshot==sequence);
    }
  }
  for(auto &arm:arms) assert(arm.handoff.waitFor(2s));
  done=true;
  for(auto &sender:senders) sender.join();
  for(auto &arm:arms) {
    assert(arm.delivered==300);
    assert(arm.failures>0);
  }
}
