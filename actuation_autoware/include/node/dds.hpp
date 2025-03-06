#ifndef DDS_HPP
#define DDS_HPP

// PORT
#if defined(NATIVE_LINUX)
#include <linux/printk.h>
#elif defined(NATIVE_ZEPHYR)
#include <zephyr/sys/printk.h>
#endif

#include <stdlib.h>
#include <memory>
#include <string>

#include "node/dds_config.hpp"

/**
 * @brief Publisher class for DDS communication
 * @tparam MessageT The message type to publish
 */
template<typename MessageT>
class Publisher {
public:
    Publisher(dds_entity_t dds_participant, dds_qos_t* dds_qos, 
             const std::string& topic_name, 
             const dds_topic_descriptor_t* topic_descriptor, 
             const std::string& node_name) : m_dds_writer(0) {
        dds_entity_t topic = dds_create_topic(dds_participant, topic_descriptor, 
                                            topic_name.c_str(), NULL, NULL);
        if (topic < 0) {
            printk("Error: node %s: dds_create_topic (%s): %s\n", 
                   node_name.c_str(), topic_name.c_str(), dds_strretcode(-topic));
            exit(-1);
        }

        m_dds_writer = dds_create_writer(dds_participant, topic, dds_qos, NULL);
        if (m_dds_writer < 0) {
            printk("Error: node %s: dds_create_writer (%s): %s\n", 
                   node_name.c_str(), topic_name.c_str(), dds_strretcode(-m_dds_writer));
            exit(-1);
        }
    }

    /**
     * @brief Publish a message
     * @param message The message to publish
     * @return true if successful, false otherwise
     */
    bool publish(const MessageT& message) {
        dds_return_t rc = dds_write(m_dds_writer, &message);
        if (rc < 0) {
            printk("Error: node %s: dds_write (%s): %s\n", node_name_.c_str(), topic_name_.c_str(), dds_strretcode(-rc));
            return false;
        }
        return true;
    }    

private:
    dds_entity_t m_dds_writer;
};

/**
 * @brief Main DDS communication class
 */
class DDS {
public:
    DDS(const std::string& node_name);
    ~DDS();

    /**
     * @brief Create a publisher for a specific message type
     * @param topic_name The name of the topic
     * @param topic_descriptor The DDS topic descriptor
     * @return std::unique_ptr<Publisher<MessageT>> Smart pointer to the created publisher
     */
    template<typename MessageT>
    std::unique_ptr<Publisher<MessageT>> create_publisher(
        const std::string& topic_name, 
        const dds_topic_descriptor_t* topic_descriptor) {
        return std::make_unique<Publisher<MessageT>>(
            m_dds_participant, m_dds_qos, topic_name, topic_descriptor, node_name_);
    }

    /**
     * @brief Create a subscription for a specific message type
     * @tparam T The message type for the subscription
     * @param topic_name The name of the topic
     * @param topic_descriptor The DDS topic descriptor
     * @param callback The callback function to handle received messages
     * @return bool true if successful, false otherwise
     */
    template<typename T>
    bool create_subscription(const std::string& topic_name, 
                           const dds_topic_descriptor_t* topic_descriptor, 
                           void (*callback)(T*)) {
        dds_entity_t topic = dds_create_topic(m_dds_participant, topic_descriptor, topic_name.c_str(), NULL, NULL);
        if (topic < 0) {
            printk("Error: node %s: dds_create_topic (%s): %s\n", node_name_.c_str(), topic_name.c_str(), dds_strretcode(-topic));
            return false;
        }

        // Create a listener and pass the callback directly
        dds_listener_t* listener = dds_create_listener((void*)callback);
        if (!listener) {
            return false;
        }
        
        // Capture node_name_ for use in lambda
        std::string node_name = node_name_;
        
        // Set the data available callback
        dds_lset_data_available(listener, [node_name](dds_entity_t reader, void* arg) {
            auto typed_callback = reinterpret_cast<void (*)(T*)>(arg);
            T* msg = nullptr;   // if we need to store the message for longer than the callback, we need to allocate it on the heap dynamically
            dds_sample_info_t info;
            
            dds_return_t rc = dds_take(reader, (void**)&msg, &info, 1, 1);
            if (rc > 0 && info.valid_data && msg) {
                typed_callback(msg); // Synchronous call - msg is valid during callback
            }
            else if (rc < 0) {
                printk("Error: node %s: dds_take failed: %s\n", node_name.c_str(), dds_strretcode(-rc));
            }
        });

        dds_entity_t reader = dds_create_reader(m_dds_participant, topic, m_dds_qos, listener);
        if (reader < 0) {
            printk("Error: node %s: dds_create_reader (%s): %s\n", node_name_.c_str(), topic_name.c_str(), dds_strretcode(-reader));
            dds_delete_listener(listener);
            return false;
        }

        dds_delete_listener(listener); // TODO: Check if we can delete the listener as it is copied into the reader.
        return true;
    }

private:
    std::string node_name_;
    dds_entity_t m_dds_participant;
    dds_qos_t* m_dds_qos;
};

#endif // DDS_HPP