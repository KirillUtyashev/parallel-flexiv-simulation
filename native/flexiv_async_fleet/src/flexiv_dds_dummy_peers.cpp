/**
 * Lightweight complementary Sim Plugin peers for discovery-capacity tests.
 *
 * FlexivNode is present in the vendor archive but is not part of its public
 * external-simulator header set.  This declaration mirrors that stable ABI so
 * the probe reuses the real participant, topics, message types, and QoS.
 */

#include "flexiv/sim_plugin/data.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <vector>

#include <sys/resource.h>

namespace flexiv {
namespace sim_plugin {

class FlexivNode final
{
public:
    FlexivNode(const std::string& robot_sn, bool debug_print = false);
    virtual ~FlexivNode();
    bool connected() const;
    bool SendRobotCommands(const SimRobotCommands& robot_commands);
    bool WaitForRobotStates();
    SimRobotStates robot_states() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} // namespace sim_plugin
} // namespace flexiv

namespace {

std::atomic<bool> g_stop {false};

void HandleSignal(int)
{
    g_stop.store(true);
}

size_t ParseSize(const char* value, const char* label)
{
    char* end = nullptr;
    const unsigned long result = std::strtoul(value, &end, 10);
    if (value == end || *end != '\0') {
        throw std::invalid_argument(std::string("invalid ") + label + ": " + value);
    }
    return static_cast<size_t>(result);
}

} // namespace

int main(int argc, char** argv)
{
    size_t timeout_ms = 10'000;
    size_t hold_ms = 2'000;
    size_t creation_parallelism = 16;
    std::vector<std::string> serials;
    try {
        for (int index = 1; index < argc; ++index) {
            const std::string argument(argv[index]);
            if (argument == "--timeout-ms" && index + 1 < argc) {
                timeout_ms = ParseSize(argv[++index], "timeout");
            }
            else if (argument == "--hold-ms" && index + 1 < argc) {
                hold_ms = ParseSize(argv[++index], "hold duration");
            }
            else if (argument == "--creation-parallelism" && index + 1 < argc) {
                creation_parallelism
                    = ParseSize(argv[++index], "creation parallelism");
                if (creation_parallelism == 0) {
                    throw std::invalid_argument(
                        "creation parallelism must be positive");
                }
            }
            else {
                serials.push_back(argument);
            }
        }
        if (serials.empty()) {
            throw std::invalid_argument("at least one robot serial is required");
        }
    }
    catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << '\n';
        return 64;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);
    // Production Flexiv simulations are separate processes. This compact probe
    // puts all complementary DDS participants in one process, so lift only its
    // soft descriptor limit to avoid introducing an artificial shared ceiling.
    struct rlimit descriptor_limit {};
    if (getrlimit(RLIMIT_NOFILE, &descriptor_limit) != 0) {
        std::cerr << "ERROR: failed to read file-descriptor limit" << std::endl;
        return 70;
    }
    const rlim_t required_descriptors = static_cast<rlim_t>(serials.size()) * 16 + 256;
    if (descriptor_limit.rlim_cur < required_descriptors) {
        descriptor_limit.rlim_cur
            = std::min(descriptor_limit.rlim_max, required_descriptors);
        if (setrlimit(RLIMIT_NOFILE, &descriptor_limit) != 0) {
            std::cerr << "ERROR: failed to raise file-descriptor limit" << std::endl;
            return 70;
        }
    }
    std::vector<std::unique_ptr<flexiv::sim_plugin::FlexivNode>> peers(
        serials.size());
    std::vector<std::string> creation_errors(serials.size());
    std::atomic<size_t> next_index {0};
    const size_t creator_count = std::min(creation_parallelism, serials.size());
    std::vector<std::thread> creators;
    creators.reserve(creator_count);
    for (size_t creator = 0; creator < creator_count; ++creator) {
        creators.emplace_back([&] {
            while (!g_stop.load()) {
                const size_t index = next_index.fetch_add(1);
                if (index >= serials.size()) {
                    return;
                }
                try {
                    peers[index] = std::make_unique<flexiv::sim_plugin::FlexivNode>(
                        serials[index], false);
                }
                catch (const std::exception& error) {
                    creation_errors[index] = error.what();
                }
                catch (...) {
                    creation_errors[index] = "unknown exception";
                }
            }
        });
    }
    for (auto& creator : creators) {
        creator.join();
    }

    const size_t created = std::count_if(peers.begin(), peers.end(),
        [](const auto& peer) { return peer != nullptr; });
    const size_t creation_error_count = std::count_if(creation_errors.begin(),
        creation_errors.end(), [](const auto& error) { return !error.empty(); });
    const auto first_creation_error = std::find_if(creation_errors.begin(),
        creation_errors.end(), [](const auto& error) { return !error.empty(); });
    if (first_creation_error != creation_errors.end()) {
        std::cerr << "DDS_DUMMY_FIRST_CREATION_ERROR index="
                  << std::distance(creation_errors.begin(), first_creation_error)
                  << " message=" << *first_creation_error << std::endl;
    }

    size_t maximum_connected = 0;
    const auto deadline
        = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!g_stop.load() && std::chrono::steady_clock::now() < deadline) {
        size_t connected = 0;
        for (const auto& peer : peers) {
            connected += peer != nullptr && peer->connected() ? 1U : 0U;
        }
        maximum_connected = std::max(maximum_connected, connected);
        if (connected == serials.size()) {
            const auto hold_deadline
                = std::chrono::steady_clock::now() + std::chrono::milliseconds(hold_ms);
            while (!g_stop.load() && std::chrono::steady_clock::now() < hold_deadline) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    std::cout << "DDS_DUMMY_RESULT requested=" << serials.size()
              << " created=" << created
              << " connected=" << maximum_connected
              << " creation_error="
              << (creation_error_count == 0 ? "none" : "present")
              << " creation_errors=" << creation_error_count
              << std::endl;
    const int result
        = created == serials.size() && maximum_connected == serials.size() ? 0 : 2;
    // This process owns only disposable DDS probe resources.  Fast DDS tears
    // down participants serially and can add ~0.5 s per peer; process exit lets
    // the OS close all transports at once and keeps the sweep intentionally fast.
    std::_Exit(result);
}
