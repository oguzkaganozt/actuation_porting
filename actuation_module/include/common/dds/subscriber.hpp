#ifndef COMMON__SUBSCRIBER_HPP_
#define COMMON__SUBSCRIBER_HPP_

#include <memory>
#include <string>

#include "common/dds/dds_helper.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

template<typename T>
using callback_subscriber = void (*)(T& msg, void* arg);

template<typename T>
class Subscriber {
public:
    Subscriber(const std::string& node_name, const std::string& topic_name, 
                dds_entity_t dds_participant, dds_qos_t* dds_qos, 
                const dds_topic_descriptor_t* topic_descriptor,
                callback_subscriber<T> callback, void* arg)
            : node_name_(node_name)
            , m_dds_participant(dds_participant)
            , callback_(callback)
            , arg_(arg)
    {
        // Manipulate topic name and topic descriptor for ROS2
        std::string topic_name_ros2 = transformTopicName(topic_name);
        dds_topic_descriptor_t topic_descriptor_ros2 = transformTopicDescriptor(topic_descriptor);
        topic_name_ = topic_name_ros2;

        // Create a DDS topic
        dds_entity_t topic = dds_create_topic(m_dds_participant, &topic_descriptor_ros2, 
                                                topic_name_.c_str(), NULL, NULL);
        if (topic < 0) {
            log_error("Error: %s -> dds_create_topic (%s): %s\n", 
                   node_name_.c_str(), topic_name_.c_str(), dds_strretcode(-topic));
            return;
        }

        // Create a DDS listener
        dds_listener_t* listener = dds_create_listener(this);
        if (!listener) {
            log_error("Error: %s -> dds_create_listener\n", node_name_.c_str());
            return;
        }
        dds_lset_data_available(listener, on_msg_dds);

        // Create a DDS reader
        dds_entity_t reader = dds_create_reader(m_dds_participant, topic, dds_qos, listener);
        if (reader < 0) {
            log_error("Error: %s -> dds_create_reader (%s): %s\n", 
                   node_name_.c_str(), topic_name.c_str(), dds_strretcode(-reader));
            dds_delete_listener(listener);
            return;
        }

        // Delete the listener explicitly //TODO: check if this is valid
        dds_delete_listener(listener);

        log_info("%s -> Subscriber created for topic %s\n", node_name_.c_str(), topic_name_.c_str());
    }

private:
    std::string node_name_;
    std::string topic_name_;
    dds_entity_t m_dds_participant;
    callback_subscriber<T> callback_;
    void* arg_;
    
    static void on_msg_dds(dds_entity_t reader, void * arg) {
        Subscriber<T>* subscriber = static_cast<Subscriber<T>*>(arg);
        static T msg;
        void* msg_pointer = reinterpret_cast<void *>(&msg);
        dds_sample_info_t info;

        log_debug("%s -> Message received topic: %s\n", subscriber->node_name_.c_str(), subscriber->topic_name_.c_str());

        dds_return_t rc = dds_take(reader, &msg_pointer, &info, 1, 1);
        if (rc > 0 && info.valid_data) {
            subscriber->callback_(msg, subscriber->arg_);
        }
        else if (rc < 0) {
            // TODO: think about removing this as it is common to fail to take a message in the first place ?
            log_debug("Error: %s -> dds_take failed: %s\n", 
                    subscriber->node_name_.c_str(), dds_strretcode(-rc));
        }
    }
};

#endif  // COMMON__SUBSCRIBER_HPP_
