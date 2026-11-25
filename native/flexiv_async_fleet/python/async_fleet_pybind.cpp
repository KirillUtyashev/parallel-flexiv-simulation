#include "flexiv/sim_plugin/async_fleet.hpp"
#include "flexiv/sim_plugin/dds_diagnostics.hpp"

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace py = pybind11;
using flexiv::sim_plugin::AsyncEndpointStatistics;
using flexiv::sim_plugin::AsyncFleet;
using flexiv::sim_plugin::GetSharedDdsDiagnostics;

namespace {

template <typename T>
void RequireShape(const py::array_t<T, py::array::c_style>& array,
    const std::vector<py::ssize_t>& shape, const char* label)
{
    if (array.ndim() != static_cast<py::ssize_t>(shape.size())) {
        throw std::invalid_argument(std::string(label) + " has the wrong rank");
    }
    for (py::ssize_t axis = 0; axis < array.ndim(); ++axis) {
        if (array.shape(axis) != shape[static_cast<size_t>(axis)]) {
            throw std::invalid_argument(std::string(label) + " has the wrong shape");
        }
    }
}

py::dict StatisticsToDict(const AsyncEndpointStatistics& statistics)
{
    py::dict result;
    result["serial"] = statistics.serial;
    result["worker_running"] = statistics.worker_running;
    result["connected"] = statistics.connected;
    result["command_valid"] = statistics.command_valid;
    result["phase"] = statistics.phase;
    result["last_published_cycle"] = statistics.last_published_cycle;
    result["last_sent_cycle"] = statistics.last_sent_cycle;
    result["command_cycle"] = statistics.command_cycle;
    result["states_published"] = statistics.states_published;
    result["states_sent"] = statistics.states_sent;
    result["commands_received"] = statistics.commands_received;
    result["command_packets_received"] = statistics.command_packets_received;
    result["synchronization_events"] = statistics.synchronization_events;
    result["synchronization_command_discards"]
        = statistics.synchronization_command_discards;
    result["recovery_events"] = statistics.recovery_events;
    result["command_wait_timeouts"] = statistics.command_wait_timeouts;
    result["consecutive_command_wait_timeouts"]
        = statistics.consecutive_command_wait_timeouts;
    result["maximum_consecutive_command_wait_timeouts"]
        = statistics.maximum_consecutive_command_wait_timeouts;
    result["last_command_timeout_cycle"] = statistics.last_command_timeout_cycle;
    result["state_queue_overflows"] = statistics.state_queue_overflows;
    result["state_frames_coalesced"] = statistics.state_frames_coalesced;
    result["state_frames_dropped"] = statistics.state_frames_dropped;
    result["state_frames_superseded"] = statistics.state_frames_superseded;
    result["send_failures"] = statistics.send_failures;
    result["invalid_commands"] = statistics.invalid_commands;
    result["queue_depth"] = statistics.queue_depth;
    result["queue_high_watermark"] = statistics.queue_high_watermark;
    result["last_command_turnaround_us"] = statistics.last_command_turnaround_us;
    result["maximum_command_turnaround_us"] = statistics.maximum_command_turnaround_us;
    result["recent_command_turnaround_us"] = statistics.recent_command_turnaround_us;
    result["worker_error"] = statistics.worker_error;
    return result;
}

} // namespace

PYBIND11_MODULE(_flexiv_async_fleet, module)
{
    module.doc() = "Native asynchronous fleet wrapper for Flexiv Sim Plugin UserNode";
    module.attr("__version__") = "1.4.0-async6";
#if defined(FLEXIV_ASYNC_DDS_SHARED_PARTICIPANT)
    module.attr("dds_shared_participant") = true;
#else
    module.attr("dds_shared_participant") = false;
#endif

    module.def("shared_dds_diagnostics", []() {
        const auto diagnostics = GetSharedDdsDiagnostics();
        py::dict result;
        result["local_participants"] = diagnostics.local_participants;
        result["references"] = diagnostics.references;
        result["discovered_participants"] = diagnostics.discovered_participants;
        result["participant_allocation"] = py::make_tuple(diagnostics.participant_initial,
            diagnostics.participant_maximum, diagnostics.participant_increment);
        result["reader_allocation"] = py::make_tuple(
            diagnostics.reader_initial, diagnostics.reader_maximum,
            diagnostics.reader_increment);
        result["writer_allocation"] = py::make_tuple(
            diagnostics.writer_initial, diagnostics.writer_maximum,
            diagnostics.writer_increment);
        return result;
    });

    py::class_<AsyncFleet>(module, "AsyncFleet")
        .def(py::init<const std::vector<std::string>&, size_t, size_t, unsigned int>(),
            py::arg("serials"), py::arg("dof"), py::arg("queue_capacity") = 8,
            py::arg("command_wait_timeout_ms") = 50)
        .def_property_readonly("robot_count", &AsyncFleet::robot_count)
        .def_property_readonly("dof", &AsyncFleet::dof)
        .def("publish_states",
            [](AsyncFleet& fleet, uint64_t servo_cycle,
                py::array_t<float, py::array::c_style> positions,
                py::array_t<float, py::array::c_style> velocities) {
                const std::vector<py::ssize_t> shape {
                    static_cast<py::ssize_t>(fleet.robot_count()),
                    static_cast<py::ssize_t>(fleet.dof())};
                RequireShape(positions, shape, "positions");
                RequireShape(velocities, shape, "velocities");
                py::gil_scoped_release release;
                return fleet.PublishStates(servo_cycle, positions.data(), velocities.data(),
                    fleet.robot_count(), fleet.dof());
            },
            py::arg("servo_cycle"), py::arg("positions"), py::arg("velocities"))
        .def("wait_for_command_cycle",
            [](AsyncFleet& fleet, uint64_t servo_cycle, unsigned int timeout_us) {
                py::gil_scoped_release release;
                return fleet.WaitForCommandCycle(servo_cycle, timeout_us);
            },
            py::arg("servo_cycle"), py::arg("timeout_us"))
        .def("read_commands",
            [](const AsyncFleet& fleet,
                py::array_t<float, py::array::c_style> efforts,
                py::array_t<uint64_t, py::array::c_style> command_cycles,
                py::array_t<uint8_t, py::array::c_style> connected,
                py::array_t<uint8_t, py::array::c_style> command_valid) {
                RequireShape(efforts,
                    {static_cast<py::ssize_t>(fleet.robot_count()),
                        static_cast<py::ssize_t>(fleet.dof())},
                    "efforts");
                const std::vector<py::ssize_t> vector_shape {
                    static_cast<py::ssize_t>(fleet.robot_count())};
                RequireShape(command_cycles, vector_shape, "command_cycles");
                RequireShape(connected, vector_shape, "connected");
                RequireShape(command_valid, vector_shape, "command_valid");
                py::gil_scoped_release release;
                fleet.CopyCommands(efforts.mutable_data(), command_cycles.mutable_data(),
                    connected.mutable_data(), command_valid.mutable_data());
            },
            py::arg("efforts"), py::arg("command_cycles"), py::arg("connected"),
            py::arg("command_valid"))
        .def("statistics",
            [](const AsyncFleet& fleet) {
                const auto native_statistics = fleet.Statistics();
                py::list result;
                for (const auto& statistics : native_statistics) {
                    result.append(StatisticsToDict(statistics));
                }
                return result;
            })
        .def("stop", [](AsyncFleet& fleet) {
            py::gil_scoped_release release;
            fleet.Stop();
        })
        .def("__enter__", [](AsyncFleet& fleet) -> AsyncFleet& { return fleet; },
            py::return_value_policy::reference_internal)
        .def("__exit__", [](AsyncFleet& fleet, const py::object&, const py::object&,
                             const py::object&) {
            py::gil_scoped_release release;
            fleet.Stop();
        });
}
