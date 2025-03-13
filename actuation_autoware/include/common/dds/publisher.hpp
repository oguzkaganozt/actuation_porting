#ifndef COMMON__PUBLISHER_HPP_
#define COMMON__PUBLISHER_HPP_

#include <string>
#include <dds/dds.h>

/**
 * @brief Publisher class for DDS communication
 * @tparam MessageT The message type to publish
 */
template<typename MessageT>
class Publisher {
public:
    Publisher(const std::string& node_name, const std::string& topic_name, 
              dds_entity_t dds_participant, dds_qos_t* dds_qos,  
              const dds_topic_descriptor_t* topic_descriptor) 
        : node_name_(node_name)
        , topic_name_(topic_name)
        , m_dds_participant(dds_participant)
        , m_dds_writer(0) {
        
        dds_entity_t topic = dds_create_topic(dds_participant, topic_descriptor, 
                                                topic_name.c_str(), NULL, NULL);
        if (topic < 0) {
            fprintf(stderr, "Error: node %s: dds_create_topic (%s): %s\n", 
                   node_name.c_str(), topic_name.c_str(), dds_strretcode(-topic));
            exit(-1);
        }

        m_dds_writer = dds_create_writer(dds_participant, topic, dds_qos, NULL);
        if (m_dds_writer < 0) {
            fprintf(stderr, "Error: node %s: dds_create_writer (%s): %s\n", 
                   node_name.c_str(), topic_name.c_str(), dds_strretcode(-m_dds_writer));
            exit(-1);
        }

        fprintf(stderr, "Node: %s, Publisher created for topic %s\n", node_name.c_str(), topic_name.c_str());
    }

    bool publish(const MessageT& message) {
        dds_return_t rc = dds_write(m_dds_writer, &message);
        if (rc < 0) {
            fprintf(stderr, "Error: node %s: dds_write (%s): %s\n", 
                   node_name_.c_str(), topic_name_.c_str(), dds_strretcode(-rc));
            return false;
        }

        fprintf(stderr, "Node: %s, Message published to topic %s: rc: %d\n", node_name_.c_str(), topic_name_.c_str(), rc);
        return true;
    }    

private:
    std::string node_name_;
    std::string topic_name_;
    dds_entity_t m_dds_participant;
    dds_entity_t m_dds_writer;
};

#endif // PUBLISHER_HPP 