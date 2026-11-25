/**
 * @file dds_diagnostics.hpp
 * @brief Read-only diagnostics for the async fleet's shared DDS participant.
 */

#ifndef FLEXIV_SIM_PLUGIN_DDS_DIAGNOSTICS_HPP_
#define FLEXIV_SIM_PLUGIN_DDS_DIAGNOSTICS_HPP_

#include <cstddef>

namespace flexiv {
namespace sim_plugin {

struct SharedDdsDiagnostics
{
    size_t local_participants = 0;
    size_t references = 0;
    size_t discovered_participants = 0;
    size_t participant_initial = 0;
    size_t participant_maximum = 0;
    size_t participant_increment = 0;
    size_t reader_initial = 0;
    size_t reader_maximum = 0;
    size_t reader_increment = 0;
    size_t writer_initial = 0;
    size_t writer_maximum = 0;
    size_t writer_increment = 0;
};

/** Return one aggregate snapshot for every participant managed by the shim. */
SharedDdsDiagnostics GetSharedDdsDiagnostics();

} // namespace sim_plugin
} // namespace flexiv

#endif // FLEXIV_SIM_PLUGIN_DDS_DIAGNOSTICS_HPP_
