/**
 * @file shared_dds_participant.cpp
 * @brief Reference-counted Fast DDS participant shared by all async UserNodes.
 */

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TopicDescription.hpp>

#include "flexiv/sim_plugin/dds_diagnostics.hpp"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace {

using eprosima::fastdds::dds::DomainId_t;
using eprosima::fastdds::dds::DomainParticipant;
using eprosima::fastdds::dds::DomainParticipantFactory;
using eprosima::fastdds::dds::DomainParticipantListener;
using eprosima::fastdds::dds::DomainParticipantQos;
using eprosima::fastdds::dds::InstanceHandle_t;
using eprosima::fastdds::dds::DataReader;
using eprosima::fastdds::dds::DataReaderListener;
using eprosima::fastdds::dds::DataReaderQos;
using eprosima::fastdds::dds::DataWriter;
using eprosima::fastdds::dds::DataWriterListener;
using eprosima::fastdds::dds::DataWriterQos;
using eprosima::fastdds::dds::Publisher;
using eprosima::fastdds::dds::StatusMask;
using eprosima::fastdds::dds::Subscriber;
using eprosima::fastdds::dds::Topic;
using eprosima::fastdds::dds::TopicDescription;
using eprosima::fastrtps::types::ReturnCode_t;

struct SharedParticipant
{
    DomainParticipantFactory* factory;
    DomainId_t domain_id;
    DomainParticipant* participant;
    size_t references;
};

std::mutex g_participants_mutex;
std::mutex g_endpoint_creation_mutex;
std::vector<SharedParticipant> g_participants;

} // namespace

namespace flexiv {
namespace sim_plugin {

SharedDdsDiagnostics GetSharedDdsDiagnostics()
{
    std::lock_guard<std::mutex> lock(g_participants_mutex);
    SharedDdsDiagnostics result;
    result.local_participants = g_participants.size();
    for (const auto& entry : g_participants) {
        result.references += entry.references;
        std::vector<InstanceHandle_t> handles;
        if (entry.participant->get_discovered_participants(handles)
            == ReturnCode_t(ReturnCode_t::RETCODE_OK)) {
            result.discovered_participants += handles.size();
        }
        const auto& allocation = entry.participant->get_qos().allocation();
        result.participant_initial += allocation.participants.initial;
        result.participant_maximum += allocation.participants.maximum;
        result.participant_increment += allocation.participants.increment;
        result.reader_initial += allocation.readers.initial;
        result.reader_maximum += allocation.readers.maximum;
        result.reader_increment += allocation.readers.increment;
        result.writer_initial += allocation.writers.initial;
        result.writer_maximum += allocation.writers.maximum;
        result.writer_increment += allocation.writers.increment;
    }
    return result;
}

} // namespace sim_plugin
} // namespace flexiv

extern "C" DomainParticipant* flexiv_async_create_shared_participant(
    DomainParticipantFactory* factory, DomainId_t domain_id,
    const DomainParticipantQos& qos, DomainParticipantListener* listener,
    const StatusMask& mask)
{
    std::lock_guard<std::mutex> lock(g_participants_mutex);
    for (auto& entry : g_participants) {
        if (entry.factory == factory && entry.domain_id == domain_id) {
            ++entry.references;
            return entry.participant;
        }
    }

    DomainParticipant* participant
        = factory->create_participant(domain_id, qos, listener, mask);
    if (participant != nullptr) {
        g_participants.push_back({factory, domain_id, participant, 1});
    }
    return participant;
}

extern "C" ReturnCode_t flexiv_async_delete_shared_participant(
    DomainParticipantFactory* factory, DomainParticipant* participant)
{
    bool managed_participant = false;
    bool delete_participant = false;
    {
        std::lock_guard<std::mutex> lock(g_participants_mutex);
        for (auto iterator = g_participants.begin(); iterator != g_participants.end();
             ++iterator) {
            if (iterator->factory == factory && iterator->participant == participant) {
                managed_participant = true;
                if (--iterator->references == 0) {
                    g_participants.erase(iterator);
                    delete_participant = true;
                }
                break;
            }
        }
    }
    if (managed_participant && !delete_participant) {
        return ReturnCode_t(ReturnCode_t::RETCODE_OK);
    }
    return factory->delete_participant(participant);
}

extern "C" DataReader* flexiv_async_create_unique_datareader(Subscriber* subscriber,
    TopicDescription* topic, const DataReaderQos& qos, DataReaderListener* listener,
    const StatusMask& mask)
{
    std::lock_guard<std::mutex> lock(g_endpoint_creation_mutex);
    DataReaderQos unique_qos(qos);
    unique_qos.endpoint().user_defined_id = -1;
    unique_qos.endpoint().entity_id = -1;
    return subscriber->create_datareader(topic, unique_qos, listener, mask);
}

extern "C" DataWriter* flexiv_async_create_unique_datawriter(Publisher* publisher,
    Topic* topic, const DataWriterQos& qos, DataWriterListener* listener,
    const StatusMask& mask)
{
    std::lock_guard<std::mutex> lock(g_endpoint_creation_mutex);
    DataWriterQos unique_qos(qos);
    unique_qos.endpoint().user_defined_id = -1;
    unique_qos.endpoint().entity_id = -1;
    return publisher->create_datawriter(topic, unique_qos, listener, mask);
}
