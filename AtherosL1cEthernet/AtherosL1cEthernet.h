/* AtherosL1cEthernet.h -- ATL1c driver definitions.
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
 * Driver for Atheros(R) AR8131/AR8132/AR8151/AR8152 PCI-E Ethernet.
 *
 * This driver is heavily based on the Atheros ATL1c Linux driver.
 */

#ifndef __AT_L1C_ETHERNET_H__
#define __AT_L1C_ETHERNET_H__

#include "at_main.h"

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Definitions & Helper Functions
#pragma mark -
//------------------------------------------------------------------------------

#define CLASS       AtherosL1cEthernet
#define super       IOEthernetController

//------------------------------------------------------------------------------

#define RELEASE(x)                                                             \
do                                                                             \
{                                                                              \
  if (x)                                                                       \
  {                                                                            \
    (x)->release();                                                            \
    (x) = 0;                                                                   \
  }                                                                            \
}                                                                              \
while(0)

#define pci_unmap_page(...)
#define pci_unmap_single(...)

#define NumPMStates                 2
#define PMStateOff                  0
#define PMStateOn                   kIOPMPowerOn

enum
{
  kPowerStateOff = 0,
  kPowerStateOn,
  kPowerStateCount
};

#define MBit                        1000000
#define kTransmitQueueCapacity      512

enum
{
  MEDIUM_INDEX_10HD = 0,
  MEDIUM_INDEX_10FD = 1,
  MEDIUM_INDEX_100HD = 2,
  MEDIUM_INDEX_100FD = 3,
  MEDIUM_INDEX_AUTO_FAST = 4,
  MEDIUM_INDEX_COUNT_FAST = 5,
  MEDIUM_INDEX_1000FD = 4,
  MEDIUM_INDEX_AUTO_GIGABIT = 5,
  MEDIUM_INDEX_COUNT_GIGABIT = 6,
};

enum at_speed_type
{
  TYPE_GIGABIT = 0,
  TYPE_FAST = 1
};

typedef struct
{
	const char *name;
  const char *vendor;
  const char *model;
	u16 pci_dev_id;
  enum at_speed_type type;
} AT_DEV_TABLE_ENTRY;

#define AT_VENDOR "Qualcomm Atheros"

#define AT_NAME_L1C     "AR8131 Gigabit Ethernet"
#define AT_NAME_L2C     "AR8132 Fast Ethernet"
#define AT_NAME_L2C_B   "AR8152 v1.1 Fast Ethernet"
#define AT_NAME_L2C_B2	"AR8152 v2.0 Fast Ethernet"
#define AT_NAME_L1D     "AR8151 v1.0 Gigabit Ethernet"
#define AT_NAME_L1D_2_0	"AR8151 v2.0 Gigabit Ethernet"

enum // Indices of at_dev_table[] below.
{
  AT_INDEX_UNKNOWN = 0,
  AT_INDEX_L1C     = 1,
  AT_INDEX_L2C     = 2,
  AT_INDEX_L2C_B   = 3,
  AT_INDEX_L2C_B2  = 4,
  AT_INDEX_L1D     = 5,
  AT_INDEX_L1D_2_0 = 6
};

const AT_DEV_TABLE_ENTRY at_dev_table[] =
{
  {
    AT_VENDOR " Ethernet",
    AT_VENDOR,
    "Unknown Model",
    PCI_DEVICE_ID_ATTANSIC_L1C,
    TYPE_FAST
  },
  {
    AT_VENDOR " " AT_NAME_L1C,
    AT_VENDOR,
    AT_NAME_L1C,
    PCI_DEVICE_ID_ATTANSIC_L1C,
    TYPE_GIGABIT
  },
  {
    AT_VENDOR " " AT_NAME_L2C,
    AT_VENDOR,
    AT_NAME_L2C,
    PCI_DEVICE_ID_ATTANSIC_L2C,
    TYPE_FAST
  },
  {
    AT_VENDOR " " AT_NAME_L2C_B,
    AT_VENDOR,
    AT_NAME_L2C_B,
    PCI_DEVICE_ID_ATHEROS_L2C_B,
    TYPE_FAST
  },
  {
    AT_VENDOR " " AT_NAME_L2C_B2,
    AT_VENDOR,
    AT_NAME_L2C_B2,
    PCI_DEVICE_ID_ATHEROS_L2C_B2,
    TYPE_FAST
  },
  {
    AT_VENDOR " " AT_NAME_L1D,
    AT_VENDOR,
    AT_NAME_L1D,
    PCI_DEVICE_ID_ATHEROS_L1D,
    TYPE_GIGABIT
  },
  {
    AT_VENDOR " " AT_NAME_L1D_2_0,
    AT_VENDOR,
    AT_NAME_L1D_2_0,
    PCI_DEVICE_ID_ATHEROS_L1D_2_0,
    TYPE_GIGABIT
  }
};

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark AtherosL1cEthernet
#pragma mark -
//------------------------------------------------------------------------------

class AtherosL1cEthernet : public IOEthernetController
{

  OSDeclareDefaultStructors (AtherosL1cEthernet)

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Public Methods
#pragma mark -
//------------------------------------------------------------------------------

public:
  bool init (OSDictionary * properties);
  void free ();
  bool start (IOService * provider);
  void stop (IOService * provider);

  bool configureInterface (IONetworkInterface * interface);
  const OSString *newVendorString () const;
  const OSString *newModelString () const;
  IOReturn selectMedium (const IONetworkMedium * medium);
  IOReturn enable (IONetworkInterface * interface);
  IOReturn disable (IONetworkInterface * interface);

  IOReturn getHardwareAddress (IOEthernetAddress * addr);
  IOReturn setHardwareAddress (const IOEthernetAddress * addr);
  IOReturn getMaxPacketSize (UInt32 *maxSize) const;
  IOReturn getMinPacketSize (UInt32 *minSize) const;
  IOReturn setMaxPacketSize (UInt32 maxSize);

  void getPacketBufferConstraints (IOPacketBufferConstraints *
                                           constraints) const;
  UInt32 outputPacket(mbuf_t m, void *param);

  IOReturn registerWithPolicyMaker(IOService * policyMaker);
  IOReturn setPowerState(unsigned long powerStateOrdinal,
                                  IOService * policyMaker);
  IOReturn enablePacketFilter(const OSSymbol * group, UInt32 aFilter,
                              UInt32 enabledFilters, IOOptionBits options = 0);
  IOReturn disablePacketFilter(const OSSymbol * group, UInt32 aFilter,
                               UInt32 enabledFilters, IOOptionBits options = 0);
  IOReturn getPacketFilters(const OSSymbol * group, UInt32 * filters) const;
  IOReturn setWakeOnMagicPacket(bool active);
  IOReturn setPromiscuousMode(bool enabled);
  IOReturn setMulticastMode(bool enabled);
  IOReturn setMulticastAllMode(bool enabled);
  IOReturn setMulticastList(IOEthernetAddress * addrs, UInt32 count);
  void systemWillShutdown(IOOptionBits specifier);
  UInt32 getFeatures() const;
  IOReturn getChecksumSupport(UInt32 * checksumMask,
                              UInt32   checksumFamily,
                              bool     isOutput );

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Private Methods
#pragma mark -
//------------------------------------------------------------------------------

private:
  void initPCIConfigSpace (IOPCIDevice * pci);
  bool OSAddNetworkMedium (UInt32 type, UInt32 bps,
                           UInt32 index, const char * name = 0);
  IOOutputQueue *createOutputQueue ();

  static IOReturn setPowerStateSleepAction(OSObject *owner,
                                           void *arg1, void *arg2,
                                           void *arg3, void *arg4);
  static IOReturn setPowerStateWakeAction(OSObject *owner,
                                          void *arg1, void *arg2,
                                          void *arg3, void *arg4);
  IOReturn setPowerStateSleep();
  IOReturn setPowerStateWake();

  // Hardware functions
  struct atl1c_adapter *netdev_priv(AtherosL1cEthernet *at);
  IOReturn atGetInterruptSource(IOService * provider);
  UInt32 atGetNicType();
  IOReturn atPrepareDMAMemory(IOBufferMemoryDescriptor **dbufs,
                              IODMACommand **dcoms,
                              IODMACommand::Segment64 *dsegs,
                              IOOptionBits direction,
                              unsigned int maxsize,
                              unsigned int num);
  IOReturn atPublishMediumDictionary();
  void atSetupDeviceInfo();
  void atTaskSchedule();
  static void atTimerFired(OSObject* owner, IOTimerEventSource* sender);
  int atl1c_alloc_rx_buffer(struct atl1c_adapter *adapter);
  int atl1c_change_mtu(int new_mtu);
  void atl1c_check_link_status (struct atl1c_adapter *adapter);
  void atl1c_clean_buffer (IOPCIDevice * pdev,
                           struct atl1c_buffer *buffer_info, int in_irq);
  void atl1c_clean_rx_irq (atl1c_adapter * adapter);
  bool atl1c_clean_tx_irq (atl1c_adapter * adapter,
                           atl1c_trans_queue type);
  void atl1c_clean_rx_ring(struct atl1c_adapter *adapter);
  void atl1c_clean_tx_ring(struct atl1c_adapter *adapter,
                            enum atl1c_trans_queue type);
  void atl1c_common_task();
  int atl1c_configure(struct atl1c_adapter *adapter);
  void atl1c_intr (OSObject * client, IOInterruptEventSource * src, int count);
  void atl1c_link_chg_event(struct atl1c_adapter *adapter);
  int atl1c_mdio_read (int phy_id, int reg_num);
  void atl1c_mdio_write (int phy_id, int reg_num, int val);
  int atl1c_probe();
  void atl1c_reset_dma_ring (struct atl1c_adapter *adapter);
  void atl1c_restore_vlan(struct atl1c_adapter *adapter);
  void atl1c_set_multi();
  int atl1c_tso_csum(struct atl1c_adapter *adapter,
                     struct sk_buff *skb,
                     struct atl1c_tpd_desc **tpd,
                     enum atl1c_trans_queue type);
  int atl1c_tx_map(struct atl1c_adapter *adapter, struct sk_buff *skb,
                   struct atl1c_tpd_desc *tpd,
                   enum atl1c_trans_queue type);
  void atl1c_tx_queue(struct atl1c_adapter *adapter,
                      struct sk_buff *skb, struct atl1c_tpd_desc *tpd,
                      enum atl1c_trans_queue type);
  void atl1c_vlan_mode(net_device *netdev, netdev_features_t features);
  UInt32 atl1c_xmit_frame(struct sk_buff *skb);

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Private Variables
#pragma mark -
//------------------------------------------------------------------------------

private:
  atl1c_adapter fAdapter;
  UInt16 fVendorID;
  UInt16 fDeviceID;
  UInt8 fDeviceTableIndex;
  IOMemoryMap *fRegMap;
  IOEthernetInterface *fNetif;
  IONetworkStats *fNetStats;
  IOEthernetStats *fEtherStats;
  IOWorkLoop *fWorkLoop;
  IOCommandGate *fCommandGate;
  IOTimerEventSource *fTimerEventSource;
  IOOutputQueue *fTransmitQueue;
  OSDictionary *fMediumDict;
  const IONetworkMedium *fMediumTable[MEDIUM_INDEX_COUNT_GIGABIT];
  IOInterruptEventSource *fInterruptSource;
  bool fLinkUp;
  UInt64 fLinkTimeout;
  bool fTSOEnabled; // Disabled by default for 8131/8132.

  IOEthernetAddress *fMCList;
  UInt8 fMCListLength;
  // The maximum multicase list capacity is somewhat arbitrary, but
  // 64 seems like a sane enough value. Beyond that there will definitely
  // be hash collisions, anyway.
#define MC_LIST_CAPACITY 64
  UInt32 fMCFlags;

  bool fMagicPacketSupported;
  bool fMagicPacketEnabled;

  UInt32 fCurrentPowerState;

  IOBufferMemoryDescriptor **fRXDMABuffers;
  IODMACommand **fRXDMACommands;
  IODMACommand::Segment64 *fRXDMASegments;

  IOBufferMemoryDescriptor **fTXDMABuffers;
  IODMACommand **fTXDMACommands;
  IODMACommand::Segment64 *fTXDMASegments;
  
  UInt32 fMTU;
};

#endif
