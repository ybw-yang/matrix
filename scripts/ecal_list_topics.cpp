// List all eCAL topics currently registered (publishers + subscribers).
// Build: g++ -std=c++17 scripts/ecal_list_topics.cpp -o /tmp/ecal_list_topics -lecal_core -pthread
#include <ecal/ecal.h>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <unordered_map>

int main(int argc, char** argv) {
  eCAL::Initialize(argc, argv, "ecal_list_topics", eCAL::Init::All);
  // give the monitoring layer time to collect registrations
  std::this_thread::sleep_for(std::chrono::seconds(3));
  std::unordered_map<std::string, eCAL::SDataTypeInformation> topics;
  eCAL::Util::GetTopics(topics);
  std::printf("=== %zu eCAL topics ===\n", topics.size());
  for (const auto& kv : topics)
    std::printf("  %-40s  [%s] %s\n", kv.first.c_str(),
                kv.second.encoding.c_str(), kv.second.name.c_str());
  eCAL::Finalize();
  return 0;
}
