#ifndef COMMON_DDS_IMPL_DDS_HPP_
#define COMMON_DDS_IMPL_DDS_HPP_

#include <string>
#include <memory>
#include <dds/dds.h>

#include "common/dds_impl/dds_config.hpp"
#include "common/dds_impl/publisher.hpp"
#include "common/dds_impl/subscriber.hpp"

/**
 * @brief Main DDS communication class
 */
class DDS {
public:
    DDS(const std::string& node_name);
    ~DDS();

    // Prevent copying
    DDS(const DDS&) = delete;
    DDS& operator=(const DDS&) = delete;

    /**
     * @brief Create a publisher for a specific message type
     * @tparam MessageT The message type for the publisher
     * @param topic_name The name of the topic
     * @param topic_descriptor The DDS topic descriptor
     * @return std::unique_ptr<Publisher<MessageT>> Smart pointer to the created publisher
     */
    template<typename MessageT>
    std::unique_ptr<Publisher<MessageT>> create_publisher_dds(
        const std::string& topic_name, 
        const dds_topic_descriptor_t* topic_descriptor) 
    {
        return std::make_unique<Publisher<MessageT>>(
            node_name_, topic_name, m_dds_participant, m_dds_qos, topic_descriptor);
    }

    /**
     * @brief Create a subscription for a specific message type
     * @tparam T The message type for the subscription
     * @param topic_name The name of the topic
     * @param topic_descriptor The DDS topic descriptor
     * @param callback The callback function to be called when a message is received
     * @return std::unique_ptr<SubscriberBase> Pointer to the created subscriber
     */
    template<typename T>
    std::unique_ptr<SubscriberBase> create_subscription_dds(const std::string& topic_name, 
                            const dds_topic_descriptor_t* topic_descriptor, 
                            callback_subscriber<T> callback) 
    {
        return std::make_unique<Subscriber<T>>(
            node_name_, topic_name, m_dds_participant, m_dds_qos, topic_descriptor, callback);
    }

private:
    std::string node_name_;
    dds_entity_t m_dds_participant;
    dds_qos_t* m_dds_qos;
};

#endif // COMMON_DDS_IMPL_DDS_HPP_