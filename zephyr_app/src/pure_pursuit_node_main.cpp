// Copyright (c) 2022-2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#include <pthread.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>

#include <iostream>
#include <memory>
#include <vector>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"
#include "zephyr_app.hpp"

static K_THREAD_STACK_DEFINE(main_stack_area, ACTUATION_STACK_SIZE);

#if defined(CONFIG_NET_DHCPV4)
/* Semaphore to indicate a lease has been acquired. */
static K_SEM_DEFINE(got_address, 0, 1);

static struct net_mgmt_event_callback mgmt_cb;
#endif  // CONFIG_NET_DHCPV4

static void * main_thread(void * arg)
{
  (void) arg;
  try {
    using autoware::motion::control::pure_pursuit_nodes::PurePursuitNode;
    const auto nd_ptr = std::make_shared<PurePursuitNode>("pure_pursuit_node");
    // Keep node alive until user termination.
    k_sleep(K_FOREVER);
  } catch (const std::exception & e) {
    std::cerr << e.what();
  } catch (...) {
    std::cerr << "Unknown error occurred";
  }
  return NULL;
}

static void net_if_cb(struct net_if * iface, void * user_data)
{
  auto ifs = reinterpret_cast<std::vector<struct net_if *> *>(user_data);
  ifs->push_back(iface);
}

static int setup_iface(
  struct net_if * iface, const char * addr, const char * gw, const char * netmask, uint16_t tag)
{
  struct in_addr inaddr;

  if (net_addr_pton(AF_INET, addr, &inaddr)) {
    std::printf("Invalid address: %s", addr);
    return 1;
  }
  if (!net_if_ipv4_addr_add(iface, &inaddr, NET_ADDR_MANUAL, 0)) {
    std::printf("Cannot add %s to interface %p", addr, iface);
    return 1;
  }

  if (net_addr_pton(AF_INET, gw, &inaddr)) {
    std::printf("Invalid address: %s", gw);
    return 1;
  }
  net_if_ipv4_set_gw(iface, &inaddr);

  if (net_addr_pton(AF_INET, netmask, &inaddr)) {
    std::printf("Invalid address: %s", netmask);
    return 1;
  }
  net_if_ipv4_set_netmask(iface, &inaddr);

#if defined(CONFIG_NET_VLAN)
  if (tag > 0) {
    int ret = net_eth_vlan_enable(iface, tag);
    if (ret < 0) {
      std::printf("Cannot set VLAN tag %d to interface %p", tag, iface);
      return 1;
    }
  }
#endif

  return 0;
}

#if defined(CONFIG_NET_DHCPV4)
static void handler(
  struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
  int i = 0;

  if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
    return;
  }

  for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
    char buf[NET_IPV4_ADDR_LEN];

    if (iface->config.ip.ipv4->unicast[i].addr_type != NET_ADDR_DHCP) {
      continue;
    }

    printf("  IP address: %s\n",
      net_addr_ntop(AF_INET,
          &iface->config.ip.ipv4->unicast[i].address.in_addr,
              buf, sizeof(buf)));
    printf("  Lease time: %u seconds\n",
       iface->config.dhcpv4.lease_time);
    printf("  Netmask:    %s\n",
      net_addr_ntop(AF_INET,
               &iface->config.ip.ipv4->netmask,
               buf, sizeof(buf)));
    printf("  Gateway:    %s\n",
      net_addr_ntop(AF_INET,
             &iface->config.ip.ipv4->gw,
             buf, sizeof(buf)));

    k_sem_give(&got_address);
  }
}
#endif  // CONFIG_NET_DHCPV4

int main(void)
{
  // Initialize network interfaces
  std::vector<struct net_if *> ifs {};
  net_if_foreach(net_if_cb, &ifs);
  if (ifs.size() >= 1 && sizeof(CONFIG_NET_IFACE1_ADDR) > 1) {
    int ret = setup_iface(
      ifs[0],
      CONFIG_NET_IFACE1_ADDR,
      CONFIG_NET_IFACE1_GW,
      CONFIG_NET_IFACE1_NETMASK,
      CONFIG_NET_IFACE1_VLAN);
    if (ret) {
      return 1;
    }
  }
  if (ifs.size() >= 2 && sizeof(CONFIG_NET_IFACE2_ADDR) > 1) {
    int ret = setup_iface(
      ifs[1],
      CONFIG_NET_IFACE2_ADDR,
      CONFIG_NET_IFACE2_GW,
      CONFIG_NET_IFACE2_NETMASK,
      CONFIG_NET_IFACE2_VLAN);
    if (ret) {
      return 1;
    }
  }

#if defined(CONFIG_NET_DHCPV4)
  if(ifs.size() >= 1) {
    std::printf("Requesting a DHCP lease...\n");
    net_mgmt_init_event_callback(&mgmt_cb, handler, NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&mgmt_cb);
    net_dhcpv4_start(ifs[0]);

    /* Wait for a lease. */
    if (k_sem_take(&got_address, K_SECONDS(10)) != 0) {
      std::printf("Did not get a DHCP lease\n");
    }
  }
#endif  // CONFIG_NET_DHCPV4

  pthread_attr_t main_attr {};
  pthread_t pthread_main {};
  void * retval = NULL;
  size_t main_stacksize = K_THREAD_STACK_SIZEOF(main_stack_area);

  pthread_attr_init(&main_attr);
  pthread_attr_setstack(&main_attr, &main_stack_area, main_stacksize);
  pthread_create(&pthread_main, &main_attr, &main_thread, NULL);
  pthread_join(pthread_main, &retval);

  return 0;
}
