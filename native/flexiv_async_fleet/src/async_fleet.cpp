#include "flexiv/sim_plugin/async_fleet.hpp"

#include "flexiv/sim_plugin/data.hpp"
#include "flexiv/sim_plugin/user_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>

namespace flexiv {
namespace sim_plugin {
namespace {

struct StateFrame
{
    uint64_t servo_cycle = 0;
    std::vector<float> q;
    std::vector<float> dq;
};

struct Endpoint
{
    Endpoint(std::string endpoint_serial, size_t endpoint_dof)
    : serial(std::move(endpoint_serial))
    , latest_efforts(endpoint_dof, 0.0F)
    {
    }

    std::string serial;
    mutable std::mutex mutex;
    std::condition_variable state_available;
    std::optional<StateFrame> pending_state;
    std::vector<float> latest_efforts;
    std::thread worker;
    AsyncEndpointStatistics statistics;
    std::deque<double> recent_turnaround_us;
};

} // namespace

class AsyncFleet::Impl
{
public:
    Impl(const std::vector<std::string>& serials, size_t dof, size_t queue_capacity,
        unsigned int command_wait_timeout_ms)
    : dof_(dof)
    , command_wait_timeout_ms_(command_wait_timeout_ms)
    {
        if (serials.empty()) {
            throw std::invalid_argument("AsyncFleet requires at least one serial");
        }
        if (dof_ == 0) {
            throw std::invalid_argument("AsyncFleet dof must be positive");
        }
        if (queue_capacity == 0) {
            throw std::invalid_argument("AsyncFleet queue capacity must be positive");
        }
        if (command_wait_timeout_ms_ == 0) {
            throw std::invalid_argument("AsyncFleet command wait timeout must be positive");
        }

        endpoints_.reserve(serials.size());
        for (const auto& serial : serials) {
            if (serial.empty()) {
                throw std::invalid_argument("AsyncFleet serials must not be empty");
            }
            endpoints_.emplace_back(std::make_unique<Endpoint>(serial, dof_));
            endpoints_.back()->statistics.serial = serial;
        }
        for (auto& endpoint : endpoints_) {
            endpoint->worker = std::thread([this, raw_endpoint = endpoint.get()]() {
                WorkerLoop(*raw_endpoint);
            });
        }
    }

    ~Impl()
    {
        Stop();
    }

    uint64_t PublishStates(uint64_t servo_cycle, const float* positions,
        const float* velocities, size_t robot_count, size_t dof)
    {
        if (positions == nullptr || velocities == nullptr) {
            throw std::invalid_argument("AsyncFleet state pointers must not be null");
        }
        if (robot_count != endpoints_.size() || dof != dof_) {
            throw std::invalid_argument("AsyncFleet state shape does not match construction");
        }

        uint64_t dropped = 0;
        for (size_t row = 0; row < endpoints_.size(); ++row) {
            auto& endpoint = *endpoints_[row];
            StateFrame frame;
            frame.servo_cycle = servo_cycle;
            const auto offset = row * dof_;
            frame.q.assign(positions + offset, positions + offset + dof_);
            frame.dq.assign(velocities + offset, velocities + offset + dof_);

            {
                std::lock_guard<std::mutex> lock(endpoint.mutex);
                if (endpoint.pending_state.has_value()) {
                    ++endpoint.statistics.state_frames_coalesced;
                    ++endpoint.statistics.state_frames_superseded;
                    ++endpoint.statistics.state_frames_dropped;
                    ++dropped;
                }
                endpoint.pending_state = std::move(frame);
                ++endpoint.statistics.states_published;
                endpoint.statistics.last_published_cycle = servo_cycle;
                endpoint.statistics.queue_depth = 1;
                endpoint.statistics.queue_high_watermark = std::max(
                    endpoint.statistics.queue_high_watermark, size_t {1});
            }
            endpoint.state_available.notify_one();
        }
        return dropped;
    }

    void CopyCommands(float* efforts, uint64_t* command_cycles, uint8_t* connected,
        uint8_t* command_valid) const
    {
        if (efforts == nullptr || command_cycles == nullptr || connected == nullptr
            || command_valid == nullptr) {
            throw std::invalid_argument("AsyncFleet command output pointers must not be null");
        }
        for (size_t row = 0; row < endpoints_.size(); ++row) {
            const auto& endpoint = *endpoints_[row];
            std::lock_guard<std::mutex> lock(endpoint.mutex);
            std::memcpy(efforts + row * dof_, endpoint.latest_efforts.data(),
                dof_ * sizeof(float));
            command_cycles[row] = endpoint.statistics.command_cycle;
            connected[row] = endpoint.statistics.connected ? 1U : 0U;
            command_valid[row] = endpoint.statistics.command_valid ? 1U : 0U;
        }
    }

    size_t WaitForCommandCycle(uint64_t servo_cycle, unsigned int timeout_us)
    {
        auto ready_count = [this, servo_cycle]() {
            size_t connected = 0;
            size_t ready = 0;
            for (const auto& endpoint : endpoints_) {
                std::lock_guard<std::mutex> lock(endpoint->mutex);
                if (endpoint->statistics.connected) {
                    ++connected;
                    if (endpoint->statistics.command_valid
                        && endpoint->statistics.command_cycle >= servo_cycle) {
                        ++ready;
                    }
                }
            }
            return std::make_pair(ready, connected);
        };

        if (timeout_us > 0) {
            std::unique_lock<std::mutex> lock(command_update_mutex_);
            command_update_.wait_for(lock, std::chrono::microseconds(timeout_us), [&]() {
                const auto counts = ready_count();
                return stop_requested_.load() || counts.second == 0
                    || counts.first == counts.second;
            });
        }
        return ready_count().first;
    }

    std::vector<AsyncEndpointStatistics> Statistics() const
    {
        std::vector<AsyncEndpointStatistics> result;
        result.reserve(endpoints_.size());
        for (const auto& endpoint : endpoints_) {
            std::lock_guard<std::mutex> lock(endpoint->mutex);
            auto statistics = endpoint->statistics;
            statistics.queue_depth = endpoint->pending_state.has_value() ? 1 : 0;
            statistics.recent_command_turnaround_us.assign(
                endpoint->recent_turnaround_us.begin(), endpoint->recent_turnaround_us.end());
            result.emplace_back(std::move(statistics));
        }
        return result;
    }

    void Stop()
    {
        if (stop_requested_.exchange(true)) {
            return;
        }
        for (auto& endpoint : endpoints_) {
            endpoint->state_available.notify_all();
        }
        NotifyCommandUpdate();
        for (auto& endpoint : endpoints_) {
            if (endpoint->worker.joinable()) {
                endpoint->worker.join();
            }
        }
    }

    size_t robot_count() const
    {
        return endpoints_.size();
    }

    size_t dof() const
    {
        return dof_;
    }

private:
    void NotifyCommandUpdate()
    {
        std::lock_guard<std::mutex> lock(command_update_mutex_);
        command_update_.notify_all();
    }

    void UpdateConnection(Endpoint& endpoint, bool connected)
    {
        bool changed = false;
        {
            std::lock_guard<std::mutex> lock(endpoint.mutex);
            changed = endpoint.statistics.connected != connected;
            if (!connected) {
                endpoint.statistics.command_valid = false;
            }
            endpoint.statistics.connected = connected;
        }
        if (changed) {
            NotifyCommandUpdate();
        }
    }

    void UpdatePhase(Endpoint& endpoint, const char* phase)
    {
        std::lock_guard<std::mutex> lock(endpoint.mutex);
        endpoint.statistics.phase = phase;
    }

    bool TakeNextState(Endpoint& endpoint, StateFrame& state)
    {
        std::unique_lock<std::mutex> lock(endpoint.mutex);
        endpoint.state_available.wait_for(lock, std::chrono::milliseconds(10), [this, &endpoint]() {
            return stop_requested_.load() || endpoint.pending_state.has_value();
        });
        if (stop_requested_.load()) {
            return false;
        }
        if (!endpoint.pending_state.has_value()) {
            return false;
        }
        state = std::move(*endpoint.pending_state);
        endpoint.pending_state.reset();
        endpoint.statistics.queue_depth = 0;
        return true;
    }

    void WorkerLoop(Endpoint& endpoint)
    {
        {
            std::lock_guard<std::mutex> lock(endpoint.mutex);
            endpoint.statistics.worker_running = true;
        }
        try {
            UpdatePhase(endpoint, "initializing");
            UserNode user_node(endpoint.serial, false);
            bool command_channel_synchronized = false;
            while (!stop_requested_.load()) {
                UpdatePhase(endpoint, "waiting_for_state");
                StateFrame state;
                if (!TakeNextState(endpoint, state)) {
                    if (!stop_requested_.load()) {
                        UpdateConnection(endpoint, user_node.connected());
                    }
                    continue;
                }

                const bool connected_before_send = user_node.connected();
                UpdateConnection(endpoint, connected_before_send);
                if (!connected_before_send) {
                    command_channel_synchronized = false;
                }
                UpdatePhase(endpoint,
                    connected_before_send ? "publishing_state" : "waiting_for_connection");
                const bool state_sent = user_node.SendRobotStates(
                    SimRobotStates(state.servo_cycle, state.q, state.dq));
                {
                    std::lock_guard<std::mutex> lock(endpoint.mutex);
                    endpoint.statistics.last_sent_cycle = state.servo_cycle;
                    if (state_sent) {
                        ++endpoint.statistics.states_sent;
                    }
                    else {
                        ++endpoint.statistics.send_failures;
                    }
                }
                if (!state_sent || !connected_before_send) {
                    command_channel_synchronized = false;
                    {
                        std::lock_guard<std::mutex> lock(endpoint.mutex);
                        endpoint.statistics.command_valid = false;
                    }
                    NotifyCommandUpdate();
                    continue;
                }

                const auto wait_start = std::chrono::steady_clock::now();
                if (!user_node.connected()) {
                    UpdateConnection(endpoint, false);
                    command_channel_synchronized = false;
                    continue;
                }
                UpdatePhase(endpoint, command_channel_synchronized ? "waiting_for_command"
                                                                   : "seeking_command_sync");
                constexpr unsigned int kRecoveryProbeTimeoutMs = 2;
                const unsigned int wait_timeout_ms = command_channel_synchronized
                    ? command_wait_timeout_ms_
                    : std::min(command_wait_timeout_ms_, kRecoveryProbeTimeoutMs);
                if (!user_node.WaitForRobotCommands(wait_timeout_ms)) {
                    {
                        std::lock_guard<std::mutex> lock(endpoint.mutex);
                        ++endpoint.statistics.command_wait_timeouts;
                        ++endpoint.statistics.consecutive_command_wait_timeouts;
                        endpoint.statistics.maximum_consecutive_command_wait_timeouts
                            = std::max(
                                endpoint.statistics.maximum_consecutive_command_wait_timeouts,
                                endpoint.statistics.consecutive_command_wait_timeouts);
                        endpoint.statistics.last_command_timeout_cycle = state.servo_cycle;
                        endpoint.statistics.command_valid = false;
                        if (command_channel_synchronized) {
                            ++endpoint.statistics.recovery_events;
                        }
                        endpoint.statistics.phase = "recovering_command_sync";
                    }
                    command_channel_synchronized = false;
                    NotifyCommandUpdate();
                    // A missing command must never stop state publication. Return to
                    // the outer loop and publish the newest mailbox state, but never
                    // expose a response until state/command pairing is re-established.
                    continue;
                }

                const auto command = user_node.robot_commands();
                const auto wait_end = std::chrono::steady_clock::now();
                const double turnaround_us = std::chrono::duration<double, std::micro>(
                    wait_end - wait_start)
                                                 .count();
                {
                    std::lock_guard<std::mutex> lock(endpoint.mutex);
                    ++endpoint.statistics.command_packets_received;
                }

                if (!command_channel_synchronized) {
                    // The first packet after one or more state retries can correspond
                    // to any outstanding state. Discard it, then drain every queued
                    // packet until the channel has been quiet for a conservative
                    // interval. The next state sent after that quiet point has exactly
                    // one possible response and can be paired safely.
                    {
                        std::lock_guard<std::mutex> lock(endpoint.mutex);
                        endpoint.statistics.command_valid = false;
                        ++endpoint.statistics.synchronization_command_discards;
                        endpoint.statistics.phase = "draining_command_backlog";
                    }
                    NotifyCommandUpdate();

                    constexpr unsigned int kMinimumSyncQuietPeriodMs = 10;
                    constexpr size_t kMaximumSyncDrainPackets = 4096;
                    const unsigned int quiet_period_ms = std::max(
                        kMinimumSyncQuietPeriodMs, command_wait_timeout_ms_);
                    size_t drained_packets = 0;
                    bool channel_quiet = false;
                    while (!stop_requested_.load() && user_node.connected()
                        && drained_packets < kMaximumSyncDrainPackets) {
                        if (!user_node.WaitForRobotCommands(quiet_period_ms)) {
                            channel_quiet = true;
                            break;
                        }
                        static_cast<void>(user_node.robot_commands());
                        ++drained_packets;
                        std::lock_guard<std::mutex> lock(endpoint.mutex);
                        ++endpoint.statistics.command_packets_received;
                        ++endpoint.statistics.synchronization_command_discards;
                    }

                    if (channel_quiet && user_node.connected()
                        && !stop_requested_.load()) {
                        command_channel_synchronized = true;
                        std::lock_guard<std::mutex> lock(endpoint.mutex);
                        endpoint.statistics.connected = true;
                        endpoint.statistics.consecutive_command_wait_timeouts = 0;
                        ++endpoint.statistics.synchronization_events;
                        endpoint.statistics.phase = "command_sync_established";
                    }
                    else {
                        command_channel_synchronized = false;
                        UpdateConnection(endpoint, user_node.connected());
                    }
                    continue;
                }

                {
                    std::lock_guard<std::mutex> lock(endpoint.mutex);
                    endpoint.statistics.connected = true;
                    endpoint.statistics.consecutive_command_wait_timeouts = 0;
                    endpoint.statistics.phase = "active";
                    if (command.target_drives.size() != dof_) {
                        endpoint.statistics.command_valid = false;
                        ++endpoint.statistics.invalid_commands;
                    }
                    else {
                        endpoint.latest_efforts = command.target_drives;
                        endpoint.statistics.command_cycle = state.servo_cycle;
                        endpoint.statistics.command_valid = true;
                        ++endpoint.statistics.commands_received;
                        endpoint.statistics.last_command_turnaround_us = turnaround_us;
                        endpoint.statistics.maximum_command_turnaround_us = std::max(
                            endpoint.statistics.maximum_command_turnaround_us, turnaround_us);
                        if (endpoint.recent_turnaround_us.size() >= 4096) {
                            endpoint.recent_turnaround_us.pop_front();
                        }
                        endpoint.recent_turnaround_us.push_back(turnaround_us);
                    }
                }
                NotifyCommandUpdate();
            }
        }
        catch (const std::exception& exception) {
            std::lock_guard<std::mutex> lock(endpoint.mutex);
            endpoint.statistics.worker_error = exception.what();
            endpoint.statistics.phase = "error";
            endpoint.statistics.connected = false;
            endpoint.statistics.command_valid = false;
        }
        catch (...) {
            std::lock_guard<std::mutex> lock(endpoint.mutex);
            endpoint.statistics.worker_error = "unknown native worker exception";
            endpoint.statistics.phase = "error";
            endpoint.statistics.connected = false;
            endpoint.statistics.command_valid = false;
        }
        std::lock_guard<std::mutex> lock(endpoint.mutex);
        endpoint.statistics.worker_running = false;
        endpoint.statistics.connected = false;
        endpoint.statistics.command_valid = false;
        if (endpoint.statistics.worker_error.empty()) {
            endpoint.statistics.phase = "stopped";
        }
    }

    size_t dof_;
    unsigned int command_wait_timeout_ms_;
    std::atomic<bool> stop_requested_ {false};
    std::vector<std::unique_ptr<Endpoint>> endpoints_;
    std::mutex command_update_mutex_;
    std::condition_variable command_update_;
};

AsyncFleet::AsyncFleet(const std::vector<std::string>& serials, size_t dof,
    size_t queue_capacity, unsigned int command_wait_timeout_ms)
: impl_(std::make_unique<Impl>(serials, dof, queue_capacity, command_wait_timeout_ms))
{
}

AsyncFleet::~AsyncFleet() = default;

size_t AsyncFleet::robot_count() const
{
    return impl_->robot_count();
}

size_t AsyncFleet::dof() const
{
    return impl_->dof();
}

uint64_t AsyncFleet::PublishStates(uint64_t servo_cycle, const float* positions,
    const float* velocities, size_t robot_count, size_t dof)
{
    return impl_->PublishStates(servo_cycle, positions, velocities, robot_count, dof);
}

size_t AsyncFleet::WaitForCommandCycle(uint64_t servo_cycle, unsigned int timeout_us)
{
    return impl_->WaitForCommandCycle(servo_cycle, timeout_us);
}

void AsyncFleet::CopyCommands(float* efforts, uint64_t* command_cycles, uint8_t* connected,
    uint8_t* command_valid) const
{
    impl_->CopyCommands(efforts, command_cycles, connected, command_valid);
}

std::vector<AsyncEndpointStatistics> AsyncFleet::Statistics() const
{
    return impl_->Statistics();
}

void AsyncFleet::Stop()
{
    impl_->Stop();
}

} // namespace sim_plugin
} // namespace flexiv
