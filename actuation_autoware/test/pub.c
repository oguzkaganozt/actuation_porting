#include <zephyr/kernel.h>

#include <stdio.h>
#include <stdlib.h>
#include <dds/dds.h>
#include <dds/ddsi/ddsi_config.h>
#include "PoseStamped.h"

void init_config(struct ddsi_config *cfg)
{
  ddsi_config_init_default (cfg);
  cfg->rbuf_size = 40 * 1024;
  cfg->rmsg_chunk_size = 20 * 1024;
  //cfg->tracemask = DDS_LC_ALL;
  cfg->tracemask = DDS_LC_INFO | DDS_LC_CONFIG;
  cfg->tracefile = "stderr";
  cfg->tracefp = NULL;
  cfg->multiple_recv_threads = DDSI_BOOLDEF_FALSE;
  //cfg->max_msg_size = 1452;
  cfg->retransmit_merging = DDSI_REXMIT_MERGE_ALWAYS;
  //cfg->transport_selector = DDSI_TRANS_UDP6;
  //cfg->defaultMulticastAddressString = "239.255.0.2";

  struct ddsi_config_network_interface_listelem *ifcfg = malloc(sizeof *ifcfg);
  memset(ifcfg, 0, sizeof *ifcfg);
  ifcfg->next = NULL;
  ifcfg->cfg.prefer_multicast = true;
  ifcfg->cfg.name = CONFIG_DDS_NETWORK_INTERFACE;
  cfg->network_interfaces = ifcfg;
  
#if defined(CONFIG_NET_CONFIG_PEER_IPV6_ADDR)
  if (strlen(CONFIG_NET_CONFIG_PEER_IPV6_ADDR) > 0) {
    struct ddsi_config_peer_listelem *peer = malloc(sizeof *peer);
    peer->next = NULL;
    peer->peer = CONFIG_NET_CONFIG_PEER_IPV6_ADDR;
    cfg->peers = peer;
  }
#elif defined(CONFIG_NET_CONFIG_PEER_IPV4_ADDR)
  if (strlen(CONFIG_NET_CONFIG_PEER_IPV4_ADDR) > 0) {
    struct ddsi_config_peer_listelem *peer = malloc(sizeof *peer);
    peer->next = NULL;
    peer->peer = CONFIG_NET_CONFIG_PEER_IPV4_ADDR;
    cfg->peers = peer;
  }
#endif
}

void helloworld_publisher()
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  geometry_msgs_msg_PoseStamped msg;
  uint32_t status = 0;
  struct ddsi_config config;

  init_config(&config);
  dds_entity_t domain = dds_create_domain_with_rawconfig (1, &config);

  /* Create a Participant. */
  participant = dds_create_participant (1, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &geometry_msgs_msg_PoseStamped_desc, "PoseStampedMsg", NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a Writer. */
  writer = dds_create_writer (participant, topic, NULL, NULL);
  if (writer < 0)
    DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush (stdout);

  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));

  while(!(status & DDS_PUBLICATION_MATCHED_STATUS))
  {
    rc = dds_get_status_changes (writer, &status);
    if (rc != DDS_RETCODE_OK)
      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));

    /* Polling sleep. */
    dds_sleepfor (DDS_MSECS (20));
  }

  /* Create a message to write. */
  msg.header.stamp.sec = 1;
  msg.header.stamp.nanosec = 0;
  // memcpy(msg.header.frame_id, "map", 4);
  msg.header.frame_id = "map";
  msg.pose.position.x = 1.0;
  msg.pose.position.y = 2.0;
  msg.pose.position.z = 3.0;

  printf ("=== [Publisher]  Writing : ");
  printf ("Message (%"PRId32", %s)\n", msg.header.stamp, msg.header.frame_id);
  fflush (stdout);

  rc = dds_write (writer, &msg);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  rc = dds_delete (domain);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete (domain): %s\n", dds_strretcode(-rc));
}

#define MAX_SAMPLES 1

int main(void)
{
    printf("CycloneDDS Publisher! %s\n", CONFIG_BOARD);
    sleep(7);
    helloworld_publisher();
    
    printf("Done\n");
    return 0;
}
