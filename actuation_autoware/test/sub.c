#include <zephyr/kernel.h>

#include <stdio.h>
#include <stdlib.h>
#include <dds/dds.h>
#include <dds/ddsi/ddsi_config.h>
#include "PoseStamped.h"
// geometry_msgs_msg_PoseStamped;

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

  // Discovery
#ifndef NATIVE_SIM
  cfg->participantIndex = DDSI_PARTICIPANT_INDEX_AUTO;
  cfg->maxAutoParticipantIndex = 60;
  cfg->allowMulticast = DDSI_AMC_SPDP;
#endif

  struct ddsi_config_network_interface_listelem *ifcfg = malloc(sizeof *ifcfg);
  memset(ifcfg, 0, sizeof *ifcfg);
  ifcfg->next = NULL;
  ifcfg->cfg.prefer_multicast = false;
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

#define MAX_SAMPLES 1

void helloworld_subscriber()
{
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t reader;
  geometry_msgs_msg_PoseStamped *msg;
  void *samples[MAX_SAMPLES];
  dds_sample_info_t infos[MAX_SAMPLES];
  dds_return_t rc;
  dds_qos_t *qos;
  struct ddsi_config config;

  init_config(&config);
  dds_entity_t domain = dds_create_domain_with_rawconfig (1, &config);
  if (domain < 0)
      DDS_FATAL("dds_create_domain_with_rawconfig: %s\n", dds_strretcode(-domain));

  /* Create a Participant. */
  participant = dds_create_participant (1, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic (
    participant, &geometry_msgs_msg_PoseStamped_desc, "PoseStampedMsg", NULL, NULL);
  if (topic < 0)
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a reliable Reader. */
  qos = dds_create_qos ();
  dds_qset_reliability (qos, DDS_RELIABILITY_RELIABLE, DDS_SECS (10));

  reader = dds_create_reader (participant, topic, qos, NULL);
  if (reader < 0)
    DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  printf ("\n=== [Subscriber] Waiting for a sample ...\n");
  fflush (stdout);

  /* Initialize sample buffer, by pointing the void pointer within
   * the buffer array to a valid sample memory location. */
  samples[0] = geometry_msgs_msg_PoseStamped__alloc ();

  /* Poll until data has been read. */  
  while (true)
  {
    /* Do the actual read.
     * The return value contains the number of read samples. */
    rc = dds_read (reader, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
    if (rc < 0)
      DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));

    /* Check if we read some data and it is valid. */
    if ((rc > 0) && (infos[0].valid_data))
    {
      /* Print Message. */
      msg = (geometry_msgs_msg_PoseStamped*) samples[0];
      printf ("=== [Subscriber] Received : ");
      printf ("Message (%"PRId32", %s)\n", msg->header.stamp, msg->header.frame_id);
      fflush (stdout);
      break;
    }
    else
    {
      /* Polling sleep. */
      dds_sleepfor (DDS_MSECS (20));
    }
  }

  /* Free the data location. */
  geometry_msgs_msg_PoseStamped_free (samples[0], DDS_FREE_ALL);

  /* Deleting the participant will delete all its children recursively as well. */
  rc = dds_delete (participant);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete (participant): %s\n", dds_strretcode(-rc));

  rc = dds_delete (domain);
  if (rc != DDS_RETCODE_OK)
    DDS_FATAL("dds_delete (domain): %s\n", dds_strretcode(-rc));
}

int main(void)
{
    printf("CycloneDDS Subscriber! %s\n", CONFIG_BOARD);
    sleep(7);

    helloworld_subscriber();

    printf("Done\n");
    return 0;
}
