#include "node/dds.hpp"

DDS::DDS(const std::string& node_name)
    : node_name_(node_name)
{
    // Initialize DDS settings
    struct ddsi_config dds_cfg;
    init_config(dds_cfg);

    // Create a DDS domain
    dds_entity_t domain = dds_create_domain_with_rawconfig(DDS_DOMAIN_ID, &dds_cfg);
    if (domain < 0 && domain != DDS_RETCODE_PRECONDITION_NOT_MET) {
        printk("Error: node %s: dds_create_domain_with_rawconfig: %s\n", node_name.c_str(), dds_strretcode(-domain));
        k_panic();
    }

    // Create a DDS participant
    m_dds_participant = dds_create_participant(DDS_DOMAIN_ID, NULL, NULL);
    if (m_dds_participant < 0) {
        printk("Error: node %s: dds_create_participant: %s\n", node_name.c_str(), dds_strretcode(-m_dds_participant));
        k_panic();
    }

    // Reliable QoS
    m_dds_qos = dds_create_qos();
    dds_qset_reliability(m_dds_qos, DDS_RELIABILITY_RELIABLE, DDS_MSECS(30));
}

DDS::~DDS() {
    // Delete publishers first
    for (auto publisher : m_publishers) {
        delete publisher;
    }

    // Delete DDS participant (this will clean up all DDS entities)
    dds_retcode_t rc = dds_delete(m_dds_participant);
    if (rc != DDS_RETCODE_OK) {
        printk("Error: node %s: dds_delete: %s\n", node_name_.c_str(), dds_strretcode(-rc));
        k_panic();
    }
    
    // Delete QoS
    dds_delete_qos(m_dds_qos);
}
