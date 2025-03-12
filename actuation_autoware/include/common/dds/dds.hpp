#ifndef COMMON_DDS_IMPL_DDS_HPP_
#define COMMON_DDS_IMPL_DDS_HPP_

#include <string>
#include <memory>
#include <dds/dds.h>

#include "common/dds/dds_config.hpp"
#include "common/dds/publisher.hpp"
#include "common/dds/subscriber.hpp"

/**
 * @brief Main DDS communication class
 */
class DDS {
public:
    DDS(const std::string& node_name)
    : node_name_(node_name)
    , m_dds_participant(0)
    , m_dds_qos(nullptr)
    {
        // Initialize DDS settings
        struct ddsi_config dds_cfg;
        init_config(dds_cfg);
        fprintf(stderr, "Node: %s -> Creating DDS domain with raw config\n", node_name_.c_str());

        // Create a DDS domain with raw config
        dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ID, &dds_cfg);
        if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
            printf("Error: Node: %s -> dds_create_domain_with_rawconfig: %s\n", 
                node_name_.c_str(), dds_strretcode(-domain));
            exit(-1);
        }
        fprintf(stderr, "Node: %s -> DDS domain created with DOMAIN_ID: %d\n", node_name_.c_str(), DDS_DOMAIN_ID);

        // Create a DDS participant
        m_dds_participant = dds_create_participant(DDS_DOMAIN_ID, NULL, NULL);
        if (m_dds_participant < 0) {
            printf("Error: Node: %s -> dds_create_participant: %s\n", 
                node_name_.c_str(), dds_strretcode(-m_dds_participant));
            exit(-1);
        }
        fprintf(stderr, "Node: %s -> DDS participant created\n", node_name_.c_str());
        
        // Reliable QoS
        m_dds_qos = dds_create_qos();
        dds_qset_reliability(m_dds_qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));
        fprintf(stderr, "Node: %s -> DDS QoS created\n", node_name_.c_str());
    }

    ~DDS() 
    {
        // Delete DDS participant (this will clean up all DDS entities)
        dds_return_t rc = dds_delete(m_dds_participant);
        if (rc != DDS_RETCODE_OK) {
            printf("Error: Node: %s -> dds_delete: %s\n", 
                node_name_.c_str(), dds_strretcode(-rc));
            exit(-1);
        }
        
        // Delete QoS separately
        dds_delete_qos(m_dds_qos);
    }

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