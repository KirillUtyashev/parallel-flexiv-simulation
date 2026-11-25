/**
 * @file async_fleet.hpp
 * @brief Non-blocking fleet wrapper around one UserNode per simulated robot.
 */

#ifndef FLEXIV_SIM_PLUGIN_ASYNC_FLEET_HPP_
#define FLEXIV_SIM_PLUGIN_ASYNC_FLEET_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace flexiv {
namespace sim_plugin {

/** Low-frequency diagnostic snapshot for one asynchronous endpoint. */
struct AsyncEndpointStatistics
{
    std::string serial;
    bool worker_running = false;
    bool connected = false;
    bool command_valid = false;
    std::string phase = "starting";
    uint64_t last_published_cycle = 0;
    uint64_t last_sent_cycle = 0;
    uint64_t command_cycle = 0;
    uint64_t states_published = 0;
    uint64_t states_sent = 0;
    uint64_t commands_received = 0;
    uint64_t command_packets_received = 0;
    uint64_t synchronization_events = 0;
    uint64_t synchronization_command_discards = 0;
    uint64_t recovery_events = 0;
    uint64_t command_wait_timeouts = 0;
    uint64_t consecutive_command_wait_timeouts = 0;
    uint64_t maximum_consecutive_command_wait_timeouts = 0;
    uint64_t last_command_timeout_cycle = 0;
    uint64_t state_queue_overflows = 0;
    uint64_t state_frames_coalesced = 0;
    uint64_t state_frames_dropped = 0;
    uint64_t state_frames_superseded = 0;
    uint64_t send_failures = 0;
    uint64_t invalid_commands = 0;
    size_t queue_depth = 0;
    size_t queue_high_watermark = 0;
    double last_command_turnaround_us = 0.0;
    double maximum_command_turnaround_us = 0.0;
    std::vector<double> recent_command_turnaround_us;
    std::string worker_error;
};

/**
 * @brief Owns one UserNode worker per robot and exposes batched, non-blocking exchange.
 *
 * UserNode methods are called only by their owning native worker. PublishStates() and
 * CopyCommands() perform bounded memory copies and never wait for RobotControlApp. Each
 * endpoint owns a latest-state mailbox: when physics outruns transport, an obsolete pending
 * state is coalesced rather than delaying the newest state behind a backlog. Commands are
 * exposed only while the endpoint has strict one-state/one-response synchronization. After a
 * timeout, recovery responses are discarded until the command channel is quiet so a late
 * response can never be attributed to a newer state.
 */
class AsyncFleet
{
public:
    AsyncFleet(const std::vector<std::string>& serials, size_t dof, size_t queue_capacity = 8,
        unsigned int command_wait_timeout_ms = 50);
    ~AsyncFleet();

    AsyncFleet(const AsyncFleet&) = delete;
    AsyncFleet& operator=(const AsyncFleet&) = delete;
    AsyncFleet(AsyncFleet&&) = delete;
    AsyncFleet& operator=(AsyncFleet&&) = delete;

    size_t robot_count() const;
    size_t dof() const;

    /**
     * Publish one state row per endpoint. If an unread state is already pending, it is
     * superseded by the newest state and the returned coalesced-frame count is incremented.
     */
    uint64_t PublishStates(uint64_t servo_cycle, const float* positions,
        const float* velocities, size_t robot_count, size_t dof);

    /**
     * Wait once for every currently connected worker to publish a command for at
     * least @p servo_cycle. Workers continue to wait for their controllers in
     * parallel, so the deadline is independent of fleet size.
     * @return Number of connected endpoints whose command reached the cycle.
     */
    size_t WaitForCommandCycle(uint64_t servo_cycle, unsigned int timeout_us);

    /** Copy the latest command/status snapshot into caller-owned contiguous arrays. */
    void CopyCommands(float* efforts, uint64_t* command_cycles, uint8_t* connected,
        uint8_t* command_valid) const;

    std::vector<AsyncEndpointStatistics> Statistics() const;

    /** Idempotently stop and join every native worker. */
    void Stop();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace sim_plugin
} // namespace flexiv

#endif // FLEXIV_SIM_PLUGIN_ASYNC_FLEET_HPP_
