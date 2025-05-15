#ifndef COMMON__SUBSCRIBER_HPP_
#define COMMON__SUBSCRIBER_HPP_

#include <memory>
#include <string>
#include <deque>
#include <pthread.h>

#include "common/dds/helper.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

class ISubscriptionHandler {
public:
    virtual ~ISubscriptionHandler() = default;
    virtual bool is_data_available() = 0;
    virtual void process_next_message() = 0;
};

template<typename T>
using callback_subscriber = void (*)(const T* msg, void* arg);

template<typename T>
class Subscriber : public ISubscriptionHandler {
public:
    Subscriber(const std::string& node_name, const std::string& topic_name, 
                dds_entity_t dds_participant, dds_qos_t* dds_qos, 
                const dds_topic_descriptor_t* topic_descriptor,
                callback_subscriber<T> callback, void* arg)
            : node_name_(node_name)
            , topic_name_(topic_name)
            , m_dds_participant(dds_participant)
            , callback_(callback)
            , arg_(arg)
            , m_reader_entity(0)
    {
        pthread_mutex_init(&data_queue_mutex_, nullptr);

        // Manipulate topic name and topic descriptor for ROS2 compatibility
        std::string topic_name_ros2 = transformTopicName(topic_name);
        dds_topic_descriptor_t topic_descriptor_ros2 = transformTopicDescriptor(topic_descriptor);

        // Create a DDS topic
        dds_entity_t topic = dds_create_topic(m_dds_participant, &topic_descriptor_ros2, 
                                                topic_name_ros2.c_str(), NULL, NULL);
        if (topic < 0) {
            log_error("%s -> dds_create_topic (%s): %s\n", 
                   node_name_.c_str(), topic_name_ros2.c_str(), dds_strretcode(-topic));
            return;
        }

        // Create a DDS listener
        dds_listener_t* listener = dds_create_listener(this);
        if (!listener) {
            log_error("%s -> dds_create_listener\n", node_name_.c_str());
            dds_delete(topic);
            return;
        }
        dds_lset_data_available(listener, Subscriber<T>::on_msg_dds_static);

        // Create a DDS reader
        m_reader_entity = dds_create_reader(m_dds_participant, topic, dds_qos, listener);
        if (m_reader_entity < 0) {
            log_error("%s -> dds_create_reader (%s): %s\n", 
                   node_name_.c_str(), topic_name_ros2.c_str(), dds_strretcode(-m_reader_entity));
            dds_delete_listener(listener);
            dds_delete(topic);
            return;
        }

        m_listener_ = listener;
        log_info("%s -> Subscriber created for topic %s\n", node_name_.c_str(), topic_name_ros2.c_str());
    }

    ~Subscriber() {
        if (m_reader_entity != 0) {
            dds_delete(m_reader_entity);
        }
        
        if (m_listener_) {
            dds_delete_listener(m_listener_);
        }
        pthread_mutex_destroy(&data_queue_mutex_);
    }

    bool is_data_available() override {
        pthread_mutex_lock(&data_queue_mutex_);
        bool available = !message_queue_.empty();
        pthread_mutex_unlock(&data_queue_mutex_);
        return available;
    }

    void process_next_message() override {
        void* msg_ptr = nullptr;
        bool should_process = false;

        pthread_mutex_lock(&data_queue_mutex_);
        if (!message_queue_.empty()) {
            msg_ptr = message_queue_.front();
            message_queue_.pop_front();
            should_process = true;
        }
        pthread_mutex_unlock(&data_queue_mutex_);

        if (should_process && callback_) {
            // T msg = *static_cast<T*>(msg_ptr);
            callback_(static_cast<const T*>(msg_ptr), arg_);
        }
    }
    
    void internal_on_data_available(dds_entity_t reader) {
        int count;
        static void* msg_ptr = nullptr;
        dds_sample_info_t info;

        count = dds_take(reader, &msg_ptr, &info, 1, 1);
        if (count < 0) {
            if (count != DDS_RETCODE_NO_DATA && count != DDS_RETCODE_TRY_AGAIN) {
                 log_debug("Error: %s -> dds_take failed for topic %s: %s\n", 
                        node_name_.c_str(), topic_name_.c_str(), dds_strretcode(-count));
            }
        } else if (count > 0) {
            if (info.valid_data) {
                pthread_mutex_lock(&data_queue_mutex_);
                auto size = message_queue_.size();
                if (size != 0) {
                    log_warn_throttle("%s -> %d unprocessed messages for topic %s\n", 
                            node_name_.c_str(), size, topic_name_.c_str());
                }
                message_queue_.push_back(msg_ptr);
                pthread_mutex_unlock(&data_queue_mutex_);
            }
        }
    }

private:
    std::string node_name_;
    std::string topic_name_;
    dds_entity_t m_dds_participant;
    callback_subscriber<T> callback_;
    void* arg_;
    dds_entity_t m_reader_entity;
    dds_listener_t* m_listener_{nullptr};

    std::deque<void*> message_queue_;
    pthread_mutex_t data_queue_mutex_;

    static void on_msg_dds_static(dds_entity_t reader, void* subscriber_ptr) {
        Subscriber<T>* subscriber_instance = static_cast<Subscriber<T>*>(subscriber_ptr);
        if (subscriber_instance) {
            subscriber_instance->internal_on_data_available(reader);
        } else {
            log_error("%s -> on_msg_dds_static called with null instance for reader %d\n", \
                        "UnknownNode", reader);
        }
    }
};

#endif  // COMMON__SUBSCRIBER_HPP_
