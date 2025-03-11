#include "common/dds_impl/dds.hpp"

DDS::DDS(const std::string& node_name)
    : node_name_(node_name)
    , m_dds_participant(0)
    , m_dds_qos(nullptr)
{
    // Initialize DDS settings
    struct ddsi_config dds_cfg;
    init_config(dds_cfg);

    // Create a DDS domain with raw config
    dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ID, &dds_cfg);
    if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
        printf("Error: node %s: dds_create_domain_with_rawconfig: %s\n", 
               node_name_.c_str(), dds_strretcode(-domain));
        exit(-1);
    }

    // Create a DDS participant
    m_dds_participant = dds_create_participant(DDS_DOMAIN_ID, NULL, NULL);
    if (m_dds_participant < 0) {
        printf("Error: node %s: dds_create_participant: %s\n", 
               node_name_.c_str(), dds_strretcode(-m_dds_participant));
        exit(-1);
    }

    // Reliable QoS
    m_dds_qos = dds_create_qos();
    dds_qset_reliability(m_dds_qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));
}

DDS::~DDS() {
    // Delete DDS participant (this will clean up all DDS entities)
    dds_return_t rc = dds_delete(m_dds_participant);
    if (rc != DDS_RETCODE_OK) {
        printf("Error: node %s: dds_delete: %s\n", 
               node_name_.c_str(), dds_strretcode(-rc));
        exit(-1);
    }
    
    // Delete QoS separately
    dds_delete_qos(m_dds_qos);
}
