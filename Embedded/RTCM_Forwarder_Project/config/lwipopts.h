// lwIP options
#pragma once

#define LWIP_IPV4                 1
#define LWIP_IPV6                 0

#define LWIP_TCP                  1
#define LWIP_UDP                  0
#define LWIP_RAW                  0

#define LWIP_SOCKET               1
#define LWIP_NETCONN              0

#define MEM_ALIGNMENT             4
#define MEM_SIZE                  1600

#define MEMP_NUM_TCP_PCB          4
#define MEMP_NUM_TCP_SEG          16
#define MEMP_NUM_SYS_TIMEOUT      10

#define TCP_MSS                   1460
#define TCP_SND_BUF               (2 * TCP_MSS)
#define TCP_WND                   (2 * TCP_MSS)

#define LWIP_NETIF_LINK_CALLBACK  1
#define LWIP_NETIF_STATUS_CALLBACK 1

#define LWIP_DHCP                 0
#define LWIP_AUTOIP               0

#define LWIP_STATS                0
#define LWIP_CHECKSUM_ON_COPY     1

#define CHECKSUM_GEN_IP           1
#define CHECKSUM_GEN_TCP          1
#define CHECKSUM_GEN_UDP          1
#define CHECKSUM_GEN_ICMP         1

#define CHECKSUM_CHECK_IP         1
#define CHECKSUM_CHECK_TCP        1
#define CHECKSUM_CHECK_UDP        1
#define CHECKSUM_CHECK_ICMP       1

