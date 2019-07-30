/*
 *  at_osdep.h -- Linux to Mac OS X definitions.
 *
 * Original Code Copyright (c) 2010 maolj <maolj@hotmail.com>.
 * Additionals Copyright (c) 2012 Hayley <tranquil.reticence@gmail.com>.
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 59
 * Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 * Linux to Mac OS X conversions. Some parts taken directly or heavily based on
 * the Linux kernel and are copyright of their respective authors. Other parts
 * such as the ether_crc_le routine heavily based on FreeBSD code, license to
 * follow.
 */

/*
 * Copyright (c) 1982, 1989, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 4. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#ifndef __AT_OS_DEP_H__
#define __AT_OS_DEP_H__

#include "ethtool.h"
#include "mii.h"
#include "netdevice.h"
#include "pci_regs.h"

typedef	unsigned short ushort;
typedef	unsigned int uint;

/******************************************************************************/
#pragma mark -
#pragma mark Debugging
#pragma mark -
/******************************************************************************/

#if defined(DEBUG)

// Levels 1 to 6 are used, in order of verbosity.
// 5 or above will usually produce a LOT of log output for every packet.
#define DEBUGLEVEL 3
#define DbgPrint(dbglvl,format,args...)                                        \
do                                                                             \
{                                                                              \
  if (dbglvl<=DEBUGLEVEL)                                                      \
  {                                                                            \
    IOLog("[AtherosL1cEthernet] " format, ##args);                             \
  }                                                                            \
} while (0)

#define dev_dbg(dev,format,args...)                                            \
  IOLog("[AtherosL1cEthernet] " format, ##args);

#else // Disable debugging.

#define DbgPrint(...)
#define dev_dbg(...)

#endif // Disable debugging.

#define ErrPrint(format,args...)                                               \
  IOLog("[AtherosL1cEthernet] Error: " format, ##args)

#define dev_err(dev,format,args...)                                            \
  IOLog("[AtherosL1cEthernet] Error: " format, ##args)

#define dev_warn(dev,format,args...)                                           \
  IOLog("[AtherosL1cEthernet] Warning: " format, ##args);

/******************************************************************************/
#pragma mark -
#pragma mark Bits and Bytes
#pragma mark -
/******************************************************************************/

#define HZ 1000

#define u8      UInt8
#define u16     UInt16
#define u32     UInt32
#define u64     UInt64
#define s32		  SInt32
#define __le16  SInt16
#define __le32  SInt32
#define __le64  SInt64

#define BITS_PER_LONG           LONG_BIT
#define BIT(nr)                 (1UL << (nr))
#define BIT_MASK(nr)            (1UL << ((nr) % BITS_PER_LONG))
#define BIT_WORD(nr)            ((nr) / BITS_PER_LONG)
#define BITS_PER_BYTE           8
#define BITS_TO_LONGS(bits)     (((bits)+BITS_PER_LONG-1)/BITS_PER_LONG)

#define swab16(x)               OSSwapInt16(x)
#define swab32(x)               OSSwapInt32(x)
#define swab64(x)               OSSwapInt64(x)

#define min_t(type,x,y) \
  ({ type __x = (x); type __y = (y); __x < __y ? __x: __y; })

static inline int atomic_dec_and_test(volatile SInt32 * addr)
{
  return ((OSDecrementAtomic(addr) == 1) ? 1 : 0);
}

static inline int atomic_inc_and_test(volatile SInt32 * addr)
{
  return ((OSIncrementAtomic(addr) == -1) ? 1 : 0);
}

#define atomic_inc(v) OSIncrementAtomic(v)
#define atomic_dec(v) OSDecrementAtomic(v)

static inline int
test_bit(int nr, const volatile unsigned long *addr)
{
  return (OSAddAtomic(0, addr) & (1 << nr)) != 0;
}

static inline void
set_bit(unsigned int nr, volatile unsigned long *addr)
{
  OSTestAndSet(nr, (volatile UInt8 *)addr);
}

static inline void
clear_bit(unsigned int nr, volatile unsigned long *addr)
{
  OSTestAndClear(nr, (volatile UInt8 *)addr);
}

static inline int
test_and_clear_bit(unsigned int nr, volatile unsigned long *addr)
{
  return !OSTestAndClear(nr, (volatile UInt8 *)addr);
}

static inline int
test_and_set_bit(unsigned int nr, volatile unsigned long *addr)
{
  return OSTestAndSet(nr, (volatile UInt8 *)addr);
}

#define cpu_to_le16(x) OSSwapHostToLittleInt16(x)
#define cpu_to_le32(x) OSSwapHostToLittleInt32(x)
#define cpu_to_le64(x) OSSwapHostToLittleInt64(x)
#define le16_to_cpu(x) OSSwapLittleToHostInt16(x)
#define le32_to_cpu(x) OSSwapLittleToHostInt32(x)
#define le64_to_cpu(x) OSSwapLittleToHostInt64(x)

// I don't think this is an issue for Intel like it may have
// been for PowerPC where writes can be reordered. Could probably
// just do nothing instead.

#define wmb() OSSynchronizeIO()

/******************************************************************************/
#pragma mark -
#pragma mark PCI
#pragma mark -
/******************************************************************************/

/* This defines the direction arg to the DMA mapping routines. */
#define PCI_DMA_BIDIRECTIONAL   0
#define PCI_DMA_TODEVICE        1
#define PCI_DMA_FROMDEVICE      2
#define PCI_DMA_NONE            3

typedef int pci_power_t;
#define PCI_D0          ((pci_power_t) 0)
#define PCI_D1          ((pci_power_t) 1)
#define PCI_D2          ((pci_power_t) 2)
#define PCI_D3hot       ((pci_power_t) 3)
#define PCI_D3cold      ((pci_power_t) 4)
#define PCI_UNKNOWN     ((pci_power_t) 5)
#define PCI_POWER_ERROR ((pci_power_t) -1)

static inline int
pci_read_config_dword(IOPCIDevice * pdev, int where, u32 * val)
{
  *val = (u32)pdev->extendedConfigRead32((IOByteCount)where);

  return 0;
}

static inline int
pci_read_config_word (IOPCIDevice * pdev, int where, u16 * val)
{
  *val = (u16)pdev->extendedConfigRead16((IOByteCount)where);

  return 0;
}

static inline int
pci_write_config_dword(IOPCIDevice * pdev, int where, u32 val)
{
  pdev->extendedConfigWrite32((IOByteCount)where, (UInt32)val);

  return 0;
}

static inline int
pci_write_config_word(IOPCIDevice * pdev, int where, u16 val)
{
  pdev->extendedConfigWrite16((IOByteCount)where, (UInt16)val);

  return 0;
}

static inline int
pci_enable_wake(IOPCIDevice * pdev, pci_power_t state, bool enable)
{
  UInt8 offset;
  UInt16 data;

  pdev->findPCICapability(kIOPCIPowerManagementCapability, &offset);
  DbgPrint (3, "kIOPCIPowerManagementCapability offset: %#x", (uint)offset);
  offset += 2; // Power Management Control/Status Register
  data = pdev->configRead16(offset);
  data |= (kPCIPMCSPMEStatus | kPCIPMCSPMEEnable);
  if (!enable)
  {
    data &= ~kPCIPMCSPMEEnable;
  }
  pdev->configWrite16(offset, data);

  return 0;
}

static inline int
pci_find_ext_capability(IOPCIDevice * pdev, int cap)
{
  IOByteCount pos = 0;

  pdev->extendedFindPCICapability(-cap, &pos);

  return (int) pos;
}

static inline int
pci_pcie_cap(IOPCIDevice *pdev)
{
  UInt8 pos = 0;
  pdev->findPCICapability(kIOPCIPCIExpressCapability, &pos);
  return (int) pos;
}

static inline int
pcie_get_readrq(IOPCIDevice *pdev)
{
	int ret, cap;
	u16 ctl;

	cap = pci_pcie_cap(pdev);
	if (!cap)
  {
		return -EINVAL;
  }

	ret = pci_read_config_word(pdev, cap + PCI_EXP_DEVCTL, &ctl);
	if (!ret)
  {
		ret = 128 << ((ctl & PCI_EXP_DEVCTL_READRQ) >> 12);
  }

	return ret;
}

static inline
bool is_power_of_2(unsigned long n)
{
  return (n != 0 && ((n & (n - 1)) == 0));
}

static inline int
pcie_set_readrq(IOPCIDevice *pdev, int rq)
{
	int cap, err = -EINVAL;
	u16 ctl, v;

	if (rq < 128 || rq > 4096 || !is_power_of_2(rq))
  {
		goto out;
  }

	cap = pci_pcie_cap(pdev);
	if (!cap)
  {
		goto out;
  }

	err = pci_read_config_word(pdev, cap + PCI_EXP_DEVCTL, &ctl);
	if (err)
  {
		goto out;
  }

	v = (ffs(rq) - 8) << 12;

	if ((ctl & PCI_EXP_DEVCTL_READRQ) != v)
  {
		ctl &= ~PCI_EXP_DEVCTL_READRQ;
		ctl |= v;
		err = pci_write_config_word(pdev, cap + PCI_EXP_DEVCTL, ctl);
	}

  out:
	return err;
}

/******************************************************************************/
#pragma mark -
#pragma mark Locks
#pragma mark -
/******************************************************************************/

#define spinlock_t  IOSimpleLock *
#define dma_addr_t  UInt64
#define atomic_t    volatile SInt32

static inline void
atomic_set(atomic_t *v, int i)
{
  OSCompareAndSwap(*v, i, v);
}

static inline SInt32
atomic_read(atomic_t *v)
{
  return OSAddAtomic(0, v);
}

#define spin_lock_init(slock)                           \
do                                                      \
{                                                       \
  if (*slock == NULL)                                   \
  {                                                     \
    *(slock) = IOSimpleLockAlloc();                     \
  }                                                     \
} while (0)

#define spin_lock(lock) (IOSimpleLockLock(*(lock)))

#define spin_unlock(lock) (IOSimpleLockUnlock(*(lock)))

#define spin_lock_irqsave(lock,flags)                   \
  ((flags) = IOSimpleLockLockDisableInterrupt(*(lock)))

#define spin_trylock_irqsave(lock,flags)                \
  ((flags) = IOSimpleLockLockDisableInterrupt(*(lock)))

#define spin_unlock_irqrestore(lock,flags)              \
  (IOSimpleLockUnlockEnableInterrupt(*(lock), (flags)))

#define usec_delay(x)           IODelay(x)
#define msec_delay(x)           IOSleep(x)
#define udelay(x)               IODelay(x)
#define mdelay(x)               IODelay(1000*(x))
#define msleep(x)               IOSleep(x)

enum
{
  GFP_KERNEL,
  GFP_ATOMIC,
};

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

/******************************************************************************/
#pragma mark -
#pragma mark Memory
#pragma mark -
/******************************************************************************/

static inline void *
kmalloc(size_t size, int flags)
{
  size_t *memory = (size_t *)IOMalloc(size + sizeof(size));
  if (memory == 0)
  {
    return NULL;
  }
  memory[0] = size;
  return ((void *)++memory);
}

static inline void *
kzalloc(size_t size, int flags)
{
  void *memory = kmalloc(size, flags);
  if (memory == NULL)
  {
    return memory;
  }
  memset(memory, 0, size);
  return memory;
}

static inline void *
kcalloc(size_t n, size_t size, int flags)
{
  return kzalloc(n * size, flags);
}

static inline void
kfree(void *p)
{
  if (p == NULL)
  {
    return;
  }
  size_t *memory = (size_t *)p;
  memory--;
  IOFree((void *)memory, *memory);
}

/******************************************************************************/
#pragma mark -
#pragma mark Read/Write Registers
#pragma mark -
/******************************************************************************/

OS_INLINE
void
_OSWriteInt8(
              volatile void               * base,
              uintptr_t                     byteOffset,
              uint16_t                      data
              )
{
  *(volatile uint8_t *)((uintptr_t)base + byteOffset) = data;
}

OS_INLINE
uint8_t
_OSReadInt8(
             const volatile void               * base,
             uintptr_t                     byteOffset
             )
{
  return *(volatile uint8_t *)((uintptr_t)base + byteOffset);
}

#define OSWriteLittleInt8(base, byteOffset, data) \
  _OSWriteInt8((base), (byteOffset), (data))
#define OSReadLittleInt8(base, byteOffset) \
  _OSReadInt8((base), (byteOffset))

#define	writel(val, reg) OSWriteLittleInt32((reg), 0, (val))
#define	writew(val, reg) OSWriteLittleInt16((reg), 0, (val))
#define	writeb(val, reg) OSWriteLittleInt8((reg), 0, (val))

#define	readl(reg) OSReadLittleInt32((reg), 0)
#define	readw(reg) OSReadLittleInt16((reg), 0)
#define	readb(reg) OSReadLittleInt8((reg), 0)

/******************************************************************************/
#pragma mark -
#pragma mark Ethernet
#pragma mark -
/******************************************************************************/

#define dev_kfree_skb(skb)      mbuf_freem_list(skb)
#define dev_kfree_skb_any(skb)  mbuf_freem_list(skb)
#define dev_kfree_skb_irq(skb)  freePacket(skb, kDelayFree)

// Can't find any other random number generator from the kernel
// that doesn't need this defined. Oh well, it's been there for
// ages anyway and randomised MAC addresses should be rarely needed.
// FIXME: Find a more suitable random number generator.

#define __APPLE_API_UNSTABLE
#ifdef __APPLE_API_UNSTABLE
#include <sys/random.h>
#endif

#define VLAN_HLEN   4 /* The additional bytes (on top of the Ethernet header) */
                      /* that VLAN requires. */
/**
 * eth_random_addr - Generate software assigned random Ethernet address
 * @addr: Pointer to a six-byte array containing the Ethernet address
 *
 * Generate a random Ethernet address (MAC) that is not multicast
 * and has the local assigned bit set.
 */
static inline void eth_random_addr (u8 * addr)
{
#ifdef __APPLE_API_UNSTABLE
  read_random(addr, ETH_ALEN);
  addr[0] &= 0xfe;		/* clear multicast bit */
  addr[0] |= 0x02;		/* set local assignment bit (IEEE802) */
#else
  addr[0] = 0x00;
  addr[1] = 0x13;
  addr[2] = 0x74;
  addr[3] = 0x00;
  addr[4] = 0x5c;
  addr[5] = 0x38;
#endif
}

static inline int
is_zero_ether_addr(const UInt8 * addr)
{
  return !(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5]);
}

static inline int
is_multicast_ether_addr(const UInt8 * addr)
{
  return (0x01 & addr[0]);
}

static inline int
is_valid_ether_addr(const UInt8 * addr)
{
  /* FF:FF:FF:FF:FF:FF is a multicast address so we don't need to
   * explicitly check for it here. */
  return !is_multicast_ether_addr(addr) && !is_zero_ether_addr(addr);
}

static inline uint32_t
ether_crc_le(size_t length, const uint8_t *data)
{
	static const uint32_t crctab[] =
  {
		0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
		0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
		0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
		0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c
	};
	size_t i;
	uint32_t crc;

	crc = 0xffffffff;

	for (i = 0; i < length; i++)
  {
		crc ^= data[i];
		crc = (crc >> 4) ^ crctab[crc & 0xf];
		crc = (crc >> 4) ^ crctab[crc & 0xf];
	}

	return (crc);
}

#define sk_buff __mbuf
#define net_device AtherosL1cEthernet

enum netdev_tx {
  __NETDEV_TX_MIN = INT_MIN,  /* make sure enum is signed */
  NETDEV_TX_OK = 0x00,        /* driver took care of packet */
  NETDEV_TX_BUSY = 0x10,      /* driver tx path was busy*/
  NETDEV_TX_LOCKED = 0x20,    /* driver tx lock was already taken */
};
typedef enum netdev_tx netdev_tx_t;

static inline bool
vlan_tx_tag_present(sk_buff *skb)
{
  u_int16_t vlan = 0;

  if (mbuf_get_vlan_tag(skb, &vlan) == ENXIO)
  {
    return false;
  }

  return true;
}

static inline u_int16_t
vlan_tx_tag_get(sk_buff *skb)
{
  u_int16_t vlan = 0;

  mbuf_get_vlan_tag(skb, &vlan);

  return vlan;
}

#define __vlan_hwaccel_put_tag(skb,vlan) mbuf_set_vlan_tag((skb),(vlan))

typedef u64 netdev_features_t;

#define SKB_GSO_TCPV4 MBUF_TSO_IPV4
#define SKB_GSO_TCPV6 MBUF_TSO_IPV6

static inline int
skb_is_gso(mbuf_t skb)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;

  mbuf_get_tso_requested(skb, &tsoreq, &value);

  return !!tsoreq; // Return exactly 1 or 0; is or is not GSO.
}

static inline int
gso_size(mbuf_t skb)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;

  mbuf_get_tso_requested(skb, &tsoreq, &value);

  return value;
}

static inline mbuf_tso_request_flags_t
gso_type(mbuf_t skb)
{
  mbuf_tso_request_flags_t tsoreq;
  u_int32_t value;

  mbuf_get_tso_requested(skb, &tsoreq, &value);

  return tsoreq;
}

#define skb_headlen(skb) ((int)mbuf_len(skb))

#define eth_hdr(skb) ((struct ether_header *)mbuf_data(skb))

#define ip_hdr(skb) ((struct ip *)(eth_hdr(skb) + 1))

#define ipv6_hdr(skb) ((struct ip6_hdr *)(eth_hdr(skb) + 1))

// #define tcp_hdr(skb) \
//  ((struct tcphdr *)((char *)ip_hdr(skb) + (ip_hdr(skb)->ip_hl << 2)))

static inline struct tcphdr
*tcp_hdr(const mbuf_t skb)
{
  char *hdr = NULL;
  struct ip6_ext *ext = NULL;

  if (ip_hdr(skb)->ip_v == 4)
  {
    hdr = (char *)ip_hdr(skb) + (ip_hdr(skb)->ip_hl << 2);
  }
  else if (ip_hdr(skb)->ip_v == 6)
  {
    hdr = (char *)ipv6_hdr(skb) + sizeof(struct ip6_hdr);

    if (unlikely(ipv6_hdr(skb)->ip6_nxt == IPPROTO_NONE))
    {
      ErrPrint("No TCP header found in IPv6 TSO packet!\n");
    }
    else if (ipv6_hdr(skb)->ip6_nxt != IPPROTO_TCP) // Extension headers.
    {
      DbgPrint(1, "IPv6 extension headers detected.\n");
      do
      {
        ext = (struct ip6_ext *)hdr;
        hdr += 8 + (ext->ip6e_len << 3);
        if (unlikely(ext->ip6e_nxt == IPPROTO_NONE))
        {
          ErrPrint("No TCP header found in IPv6 TSO packet!\n");
          break;
        }
      } while (ext->ip6e_nxt != IPPROTO_TCP);
    }
  }

  return (struct tcphdr *)hdr;
}

#define skb_network_offset(skb) \
  ((int)((char *)ip_hdr(skb) - (char *)mbuf_data(skb)))

#define skb_transport_offset(skb) \
  ((int)((char *)tcp_hdr(skb) - (char *)mbuf_data(skb)))

#define tcp_hdrlen(skb) ((int)(tcp_hdr(skb)->th_off << 2))

static inline u32
csum_tcpudp_nofold(u32 saddr, u32 daddr, u16 len, u16 proto, u32 sum)
{
  /**********************************************************/

#if defined(__i386__) || defined(__x86_64__)
  asm("addl %1, %0	;\n"
	    "adcl %2, %0	;\n"
	    "adcl %3, %0	;\n"
	    "adcl $0, %0	;\n"
	    : "=r" (sum)
	    : "g" (daddr), "g"(saddr),
      "g" ((len + proto) << 8), "0" (sum));

  return sum;

  /**********************************************************/

#else // Generic
  u64 s = sum;

  s += saddr;
	s += daddr;
#if BYTE_ORDER == BIG_ENDIAN
	s += proto + len;
#else // BYTE_ORDER == LITTLE_ENDIAN
	s += (proto + len) << 8;
#endif // BYTE_ORDER == LITTLE_ENDIAN
	s += (s >> 32);

  return s;
#endif // Generic

  /**********************************************************/
}

/*
 * Fold a partial checksum.
 */
static inline u16
csum_fold(u32 csum)
{
  /**********************************************************/

#if defined(__i386__) || defined(__x86_64__)
  asm("addl %1,%0\n"
	    "adcl $0xffff,%0"
	    : "=r" (csum)
	    : "r" ((u32)csum << 16),
      "0" ((u32)csum & 0xffff0000));

	return (u16)(~(u32)csum >> 16);

  /**********************************************************/

#else // Generic
	u32 sum = csum;

	sum = (sum & 0xffff) + (sum >> 16);
	sum = (sum & 0xffff) + (sum >> 16);

	return (u16)~sum;
#endif // Generic

  /**********************************************************/
}

/*
 * Computes the checksum of the TCP/UDP pseudo-header
 * returns a 16-bit checksum, already complemented.
 */
static inline u16
csum_tcpudp_magic(u32 saddr, u32 daddr, u16 len, u16 proto, u32 sum)
{
	return csum_fold(csum_tcpudp_nofold(saddr, daddr, len, proto, sum));
}

static inline u16
csum_ipv6_magic(const struct in6_addr *saddr,
                const struct in6_addr *daddr,
                u32 len, u16 proto, u32 sum)
{
	asm("addl 0(%1), %0	;\n"
	    "adcl 4(%1), %0	;\n"
	    "adcl 8(%1), %0	;\n"
	    "adcl 12(%1), %0	;\n"
	    "adcl 0(%2), %0	;\n"
	    "adcl 4(%2), %0	;\n"
	    "adcl 8(%2), %0	;\n"
	    "adcl 12(%2), %0	;\n"
	    "adcl %3, %0	;\n"
	    "adcl %4, %0	;\n"
	    "adcl $0, %0	;\n"
	    : "=&r" (sum)
	    : "r" (saddr), "r" (daddr),
      "r" (htonl(len)), "r" (htonl(proto)), "0" (sum)
	    : "memory");

	return csum_fold(sum);
}

/******************************************************************************/

#endif //__AT_OS_DEP_H__
