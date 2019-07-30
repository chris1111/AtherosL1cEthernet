/* AtherosL1cEthernet.cpp -- ATL1c driver implementations.
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

#include "AtherosL1cEthernet.h"

//------------------------------------------------------------------------------

OSDefineMetaClassAndStructors (AtherosL1cEthernet, IOEthernetController)

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark IOService Overrides
#pragma mark -
//------------------------------------------------------------------------------

bool
CLASS::init(OSDictionary * properties)
{
  DbgPrint (4, "init()\n");

  OSBoolean *tso;

  if (!super::init (properties))
  {
    ErrPrint ("Cannot init() superclass!\n");
    return false;
  }

  memset(&fAdapter, 0, sizeof (atl1c_adapter));

  /**********************************************************/
  
  fMCList = (IOEthernetAddress *)kzalloc(sizeof(IOEthernetAddress) *
                                         MC_LIST_CAPACITY, 0);
  if (fMCList == NULL)
  {
    ErrPrint ("Unable to allocate multicast list array!\n");
    return false;
  }
  DbgPrint(3, "Multicast list array allocated successfully.\n");
  fMCListLength = 0;
  fMCFlags = 0;

  /**********************************************************/

  fRXDMABuffers = (IOBufferMemoryDescriptor **)
                  kzalloc(sizeof(IOBufferMemoryDescriptor *) *
                          AT_RX_BUFF_COUNT, 0);
  if (fRXDMABuffers == NULL)
  {
    ErrPrint ("Unable to allocate RX DMA buffer array!\n");
    return false;
  }
  DbgPrint(3, "RX DMA buffer array allocated successfully.\n");

  fRXDMACommands = (IODMACommand **)
                   kzalloc(sizeof(IODMACommand *) * AT_RX_BUFF_COUNT, 0);
  if (fRXDMACommands == NULL)
  {
    ErrPrint ("Unable to allocate RX DMA command array!\n");
    return false;
  }
  DbgPrint(3, "RX DMA command array allocated successfully.\n");

  fRXDMASegments = (IODMACommand::Segment64 *)
                   kzalloc(sizeof(IODMACommand::Segment64) *
                           AT_RX_BUFF_COUNT, 0);

  if (fRXDMASegments == NULL)
  {
    ErrPrint ("Unable to allocate RX DMA segment array!\n");
    return false;
  }
  DbgPrint(3, "RX DMA segment array allocated successfully.\n");

  /**********************************************************/

  fTXDMABuffers = (IOBufferMemoryDescriptor **)
                  kzalloc(sizeof(IOBufferMemoryDescriptor *) *
                          AT_TX_BUFF_COUNT, 0);
  if (fTXDMABuffers == NULL)
  {
    ErrPrint ("Unable to allocate TX DMA buffer array!\n");
    return false;
  }
  DbgPrint(3, "TX DMA buffer array allocated successfully.\n");
  
  fTXDMACommands = (IODMACommand **)
                   kzalloc(sizeof(IODMACommand *) * AT_TX_BUFF_COUNT, 0);
  if (fTXDMACommands == NULL)
  {
    ErrPrint ("Unable to allocate TX DMA command array!\n");
    return false;
  }
  DbgPrint(3, "TX DMA command array allocated successfully.\n");

  fTXDMASegments = (IODMACommand::Segment64 *)
                   kzalloc(sizeof(IODMACommand::Segment64) *
                           AT_TX_BUFF_COUNT, 0);

  if (fTXDMASegments == NULL)
  {
    ErrPrint ("Unable to allocate TX DMA segment array!\n");
    return false;
  }
  DbgPrint(3, "TX DMA segment array allocated successfully.\n");

  /**********************************************************/

  fAdapter.pdev = NULL;
  fNetif = NULL;
  fRegMap = NULL;
  fAdapter.hw.hw_addr = NULL;
  fAdapter.hw.adapter = &fAdapter;
  fWorkLoop = NULL;
  fInterruptSource = NULL;
  fCommandGate = NULL;
  fTimerEventSource = NULL;
  fLinkUp = false;
  fLinkTimeout = 0;
  
  if ((tso = (OSBoolean *)getProperty("EnableTSO")) && tso->isTrue())
  {
		fTSOEnabled = true;
    DbgPrint(2, "TSO enabled.\n");
	}
  else
  {
		fTSOEnabled = false;
    DbgPrint(2, "TSO disabled.\n");
	}

  setProperty(kIOBuiltin, true);

  return true;
}

//------------------------------------------------------------------------------

void
CLASS::free()
{
  DbgPrint (4, "free()\n");

  if (fAdapter.mdio_lock)
  {
    IOSimpleLockFree(fAdapter.mdio_lock);
    fAdapter.mdio_lock = NULL;
  }

  if (fAdapter.tx_lock)
  {
    IOSimpleLockFree(fAdapter.tx_lock);
    fAdapter.tx_lock = NULL;
  }

  for (int i = 0; i < AT_TX_BUFF_COUNT; i++)
  {
      RELEASE (fTXDMACommands[i]);
      RELEASE (fTXDMABuffers[i]);
  }

  for (int i = 0; i < AT_RX_BUFF_COUNT; i++)
  {
      RELEASE (fRXDMACommands[i]);
      RELEASE (fRXDMABuffers[i]);
  }

  if (fMCList)
  {
    kfree(fMCList);
    fMCList = NULL;
  }

  if (fRXDMABuffers)
  {
    kfree(fRXDMABuffers);
    fRXDMABuffers = NULL;
  }

  if (fRXDMASegments)
  {
    kfree(fRXDMASegments);
    fRXDMASegments = NULL;
  }

  if (fTXDMABuffers)
  {
    kfree(fTXDMABuffers);
    fTXDMABuffers = NULL;
  }

  if (fTXDMASegments)
  {
    kfree(fRXDMASegments);
    fRXDMASegments = NULL;
  }

  RELEASE(fWorkLoop);
  RELEASE(fInterruptSource);
  RELEASE(fCommandGate);
  RELEASE(fTimerEventSource);
  RELEASE(fNetif);
  RELEASE(fAdapter.pdev);
  RELEASE(fRegMap);

  super::free ();
}

//------------------------------------------------------------------------------

void
CLASS::initPCIConfigSpace (IOPCIDevice * pci)
{
  DbgPrint(4, "initPCIConfigSpace()\n");

  atl1c_adapter *adapter = &fAdapter;
  UInt8 pmCapOffset;

  atl1c_reset_pcie(&adapter->hw, ATL1C_PCIE_L0S_L1_DISABLE);

  // PCI power management (wake from D3cold for magic packet).
  if (pci->findPCICapability(kIOPCIPowerManagementCapability, &pmCapOffset))
  {
    UInt16 pmc = pci->configRead16(pmCapOffset + 2);
    DbgPrint(3, "PCI register PMC = %#04x\n", pmc);

    if (pmc & kPCIPMCPMESupportFromD3Cold)
    {
      fMagicPacketSupported = true;
      DbgPrint(3, "PME# from D3cold state is supported.\n");
    }
  }

  if (pci->hasPCIPowerManagement(kPCIPMCD3Support))
  {
    pci->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
  }
}

//------------------------------------------------------------------------------

bool
CLASS::start (IOService * provider)
{
  DbgPrint (4, "start()\n");

  struct atl1c_adapter *adapter = &fAdapter;

  if (!super::start(provider))
  {
    ErrPrint ("Cannot start() IOEthernetController superclass!\n");
    return false;
  }
  PMinit();
  provider->joinPMtree(this);
  registerWithPolicyMaker(provider);

  adapter->pdev = OSDynamicCast(IOPCIDevice, provider);
  if (!adapter->pdev)
  {
    ErrPrint ("Unable to cast provider to IOPCIDevice!\n");
    return false;
  }

  adapter->pdev->retain ();
  adapter->pdev->open (this);

  if (atl1c_probe() != 0)
  {
    ErrPrint ("Cannot probe adapter!\n");
    stop (provider);
    return false;
  }

  atSetupDeviceInfo();

  /**********************************************************/
  /* Allocate RX/TX DMA buffers & descriptors.              */
  /**********************************************************/
  if (atl1c_setup_ring_resources (adapter))
  {
    ErrPrint ("Cannot allocate ring descriptors!\n");
    adapter->pdev->close (this);
    return kIOReturnError;
  }

  IOReturn ret;

  ret = atPrepareDMAMemory(&fRXDMABuffers[0], &fRXDMACommands[0],
                           &fRXDMASegments[0], kIODirectionOut,
                           RRS_PKT_SIZE_MASK + 1,
                           AT_RX_BUFF_COUNT);
  if (ret != kIOReturnSuccess)
  {
    ErrPrint("Error preparing RX DMA memory! Error code: %#x.\n", ret);
    return false;
  }
  DbgPrint(3, "RX DMA memory allocated successfully.\n");

  ret = atPrepareDMAMemory(&fTXDMABuffers[0], &fTXDMACommands[0],
                           &fTXDMASegments[0], kIODirectionOut,
                           TPD_MSS_MASK + 1, AT_TX_BUFF_COUNT);

  if (ret != kIOReturnSuccess)
  {
    ErrPrint("Error preparing TX DMA memory! Error code: %#x.\n", ret);
    return false;
  }
  DbgPrint(3, "TX DMA memory allocated successfully.\n");

  /**********************************************************/

  if (atPublishMediumDictionary() != kIOReturnSuccess)
  {
    ErrPrint("Unable to publish medium dictionary!\n");
    return false;
  }

  fWorkLoop = IOWorkLoop::workLoop();
  if (!fWorkLoop)
  {
    ErrPrint ("Unable to getWorkLoop()!\n");
    stop(provider);
    return false;
  }

  fCommandGate = getCommandGate();
  if (!fCommandGate)
  {
    ErrPrint ("Cannot create command gate!\n");
    stop(provider);
    return false;
  }
  fCommandGate->retain();

  fTimerEventSource = IOTimerEventSource::timerEventSource(this, atTimerFired);
  if (!fTimerEventSource)
  {
    ErrPrint("Cannot create timer event source!\n");
    stop(provider);
    return false;
  }
  if (fWorkLoop->addEventSource(fTimerEventSource) != kIOReturnSuccess)
  {
    ErrPrint("Cannot add timer event source to workloop!\n");
    stop(provider);
    return false;
  }

  fTransmitQueue = getOutputQueue();
  if (!fTransmitQueue)
  {
    ErrPrint ("Unable to getOutputQueue!\n");
    stop(provider);
    return false;
  }
  fTransmitQueue->retain();

  atGetInterruptSource(provider);
  // Add interrupt to workloop event sources.
  if (!fInterruptSource
      || fWorkLoop->addEventSource(fInterruptSource) != kIOReturnSuccess)
  {
    if (!fInterruptSource)
    {
      ErrPrint ("Cannot create interrupt source!\n");
    }
    else
    {
      ErrPrint ("Cannot attach interrupt source!\n");
    }
    stop (provider);
    return false;
  }
  fInterruptSource->enable();

  // Attaching dynamic link layer.
  if (!attachInterface(reinterpret_cast <IONetworkInterface**>(&fNetif)))
  {
    ErrPrint ("Failed to attach data link layer.\n");
    return false;
  }

  // fNetif->registerService();
  // Close provider, will be re-opened on demand when enable() is called.
  adapter->pdev->close(this);

  return true;
}

//------------------------------------------------------------------------------

void
CLASS::stop (IOService * provider)
{
  DbgPrint (4, "stop()\n");

  atl1c_adapter *adapter = &fAdapter;

  if (fNetif)
  {
    detachInterface(fNetif);
  }

  if (fInterruptSource && fWorkLoop)
  {
    fWorkLoop->removeEventSource(fInterruptSource);
  }

  if (fTimerEventSource && fWorkLoop)
  {
    fTimerEventSource->cancelTimeout();
    fWorkLoop->removeEventSource(fTimerEventSource);
  }

  for (int i = 0; i < AT_TX_BUFF_COUNT; i++)
  {
    if (fTXDMACommands[i])
    {
      fTXDMACommands[i]->clearMemoryDescriptor();
    }

    if (fTXDMABuffers[i])
    {
      fTXDMABuffers[i]->complete();
    }
  }
  
  for (int i = 0; i < AT_RX_BUFF_COUNT; i++)
  {
    if (fRXDMACommands[i])
    {
      fRXDMACommands[i]->clearMemoryDescriptor();
    }

    if (fRXDMABuffers[i])
    {
      fRXDMABuffers[i]->complete();
    }
  }

  atl1c_free_ring_resources(adapter);

  PMstop ();
  super::stop (provider);
}

//------------------------------------------------------------------------------

bool
CLASS::OSAddNetworkMedium (UInt32 type, UInt32 bps, UInt32 index,
                           const char * name)
{
  DbgPrint (6, "OSAddNetworkMedium()\n");

  IONetworkMedium *medium;

  medium = IONetworkMedium::medium (type, bps, 0, index, name);
  if (!medium)
  {
    DbgPrint (1, "Couldn't allocate medium.\n");
    return false;
  }
  if (!IONetworkMedium::addMedium (fMediumDict, medium))
  {
    DbgPrint (1, "Couldn't add medium.\n");
    return false;
  }
  fMediumTable[index] = medium;
  return true;
}

//------------------------------------------------------------------------------

IOOutputQueue *
CLASS::createOutputQueue()
{
  DbgPrint (4, "createOutputQueue()\n");
  // Sharing one event source with transmit/receive handles.
  return IOGatedOutputQueue::withTarget(this, getWorkLoop());
}

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark IOEthernetController Overrides
#pragma mark -
//------------------------------------------------------------------------------

IOReturn
CLASS::enable(IONetworkInterface *netif)
{
  DbgPrint(4, "enable()\n");

  atl1c_adapter *adapter = &fAdapter;
  atl1c_hw *hw = &adapter->hw;

  if (!adapter->pdev
      || (!adapter->pdev->isOpen () && (adapter->pdev->open (this)) == 0))
  {
    ErrPrint("Failed to open PCI device.\n");
    return kIOReturnError;
  }

  setLinkStatus(kIONetworkLinkValid);
  fLinkUp = false;

  hw->hibernate = true;
  if (atl1c_reset_mac(hw) != 0)
  {
    DbgPrint(1, "Reset MAC failed.\n");
  }

  if (atl1c_configure(adapter))
  {
    ErrPrint("Failed to configure adapter.\n");
    atl1c_clean_rx_ring(adapter);
    return kIOReturnError;
  }
  
  atl1c_check_link_status(adapter);
  clear_bit(__AT_DOWN, &adapter->flags);
  atl1c_irq_enable(adapter);
  
  fTransmitQueue->setCapacity(kTransmitQueueCapacity);
  fTransmitQueue->start();
  
  set_bit(ATL1C_WORK_EVENT_LINK_CHANGE, &adapter->work_event);
  clock_interval_to_deadline(AT_TRY_LINK_TIMEOUT, kMillisecondScale,
                             &fLinkTimeout);
  DbgPrint(3, "Link timeout initialised.\n");
  atTaskSchedule();
  
  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::disable(IONetworkInterface *netif)
{
  DbgPrint(4, "disable()\n");

  atl1c_adapter *adapter = &fAdapter;
  
  fTransmitQueue->stop();
  fTransmitQueue->setCapacity(0);
  fTransmitQueue->flush();

  set_bit(__AT_DOWN, &adapter->flags);
  setLinkStatus(0);

  atl1c_irq_disable(adapter);
  /* Disable ASPM if device inactive. */
  atl1c_disable_l0s_l1(&adapter->hw);
  /* Reset MAC to disable all RX/TX. */
  atl1c_reset_mac(&adapter->hw);

  adapter->link_speed = SPEED_0;
  adapter->link_duplex = -1;
  atl1c_reset_dma_ring(adapter);

  if (adapter->pdev && adapter->pdev->isOpen())
    adapter->pdev->close(this);

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

// Configure a newly instantiated IONetworkInterface object.

bool
CLASS::configureInterface (IONetworkInterface * netif)
{
  DbgPrint (4, "configureInterface()\n");

  IONetworkData *data;

  if (!super::configureInterface(netif))
    return false;

  // Get the generic network statistics structure.
  data = netif->getParameter (kIONetworkStatsKey);
  if (!data || !(fNetStats = (IONetworkStats *) data->getBuffer ()))
  {
    return false;
  }

  // Get the ethernet statistics structure.
  data = netif->getParameter (kIOEthernetStatsKey);
  if (!data || !(fEtherStats = (IOEthernetStats *) data->getBuffer ()))
  {
    return false;
  }

  return true;
}

//------------------------------------------------------------------------------

const OSString *
CLASS::newVendorString () const
{
  return OSString::withCString ("Atheros");
}

//------------------------------------------------------------------------------

const OSString *
CLASS::newModelString () const
{
  switch (fDeviceID)
  {
    case PCI_DEVICE_ID_ATTANSIC_L1C:
      return OSString::withCString ("AR8131 Gigabit Ethernet");
      break;
    case PCI_DEVICE_ID_ATTANSIC_L2C:
      return OSString::withCString ("AR8132 Fast Ethernet");
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B:
      return OSString::withCString ("AR8152 v1.1 Fast Ethernet");
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B2:
      return OSString::withCString ("AR8152 v2.0 Fast Ethernet");
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D:
      return OSString::withCString ("AR8151 v1.0 Gigabit Ethernet");
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D_2_0:
      return OSString::withCString ("AR8151 v2.0 Gigabit Ethernet");
      break;
    default:
      break;
  }

  return OSString::withCString ("L1c LAN");
}

//------------------------------------------------------------------------------

IOReturn
CLASS::selectMedium(const IONetworkMedium *medium)
{
  DbgPrint(1, "selectMedium()\n");
  
  struct atl1c_adapter *adapter = &fAdapter;
	struct atl1c_hw *hw = &adapter->hw;
	u16 autoneg_advertised;
  bool full_duplex;

  if (medium)
  {
    IOMediumType mType = medium->getType();
    UInt32 mTypeNet = mType & (kIOMediumNetworkTypeMask | kIOMediumSubTypeMask);

    if ((mType & kIOMediumCommonOptionsMask) & kIOMediumOptionFullDuplex)
    {
      full_duplex = true;
    }
    else
    {
      full_duplex = false;;
    }
    DbgPrint(2, "Selected medium: %s\n", medium->getName()->getCStringNoCopy());
   
    switch (mTypeNet)
    {
      case kIOMediumEthernetAuto:
        autoneg_advertised = ADVERTISED_Autoneg;
        break;
      case kIOMediumEthernet1000BaseT:
        autoneg_advertised = ADVERTISED_1000baseT_Full;
        break;
      case kIOMediumEthernet100BaseTX:
        autoneg_advertised = full_duplex ?
                             ADVERTISED_100baseT_Full :
                             ADVERTISED_100baseT_Half;
        break;
      case kIOMediumEthernet10BaseT:
        autoneg_advertised = full_duplex ?
                             ADVERTISED_10baseT_Full :
                             ADVERTISED_10baseT_Half;
        break;
      default:
        autoneg_advertised = ADVERTISED_Autoneg;
        break;
    }
    
    if (hw->autoneg_advertised != autoneg_advertised)
    {
      while (test_and_set_bit(__AT_RESETTING, &fAdapter.flags))
      {
        msleep(20);
      }
      hw->autoneg_advertised = autoneg_advertised;
      if (atl1c_restart_autoneg(hw) != 0)
      {
        DbgPrint(1, "Speed/duplex setting failed.\n");
        clear_bit(__AT_RESETTING, &adapter->flags);
        return kIOReturnError;
      }
      clear_bit(__AT_RESETTING, &adapter->flags);
    }
  }
  else
  {
    DbgPrint(1, "Selected medium is NULL.\n");
    return kIOReturnError;
  }
  
  setSelectedMedium(medium);
  
  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::getHardwareAddress(IOEthernetAddress *addr)
{
  DbgPrint (4, "getHardwareAddress()\n");

  IOSleep (1000);

  if (is_valid_ether_addr(fAdapter.hw.mac_addr))
  {
    memcpy(addr->bytes, fAdapter.hw.mac_addr, ETH_ALEN);
    return kIOReturnSuccess;
  }

  if (atl1c_read_mac_addr(&fAdapter.hw) == 0)
  {
    memcpy(addr->bytes, fAdapter.hw.mac_addr, ETH_ALEN);
    return kIOReturnSuccess;
  }
  else // A random MAC address has been generated.
  {
    DbgPrint(1, "Couldn't get device MAC address.\n");
    memcpy(fAdapter.hw.mac_addr, addr->bytes, ETH_ALEN);
    return kIOReturnSuccess;
  }
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setHardwareAddress (const IOEthernetAddress * addr)
{
  DbgPrint (4, "setHardwareAddress()\n");
  memcpy(fAdapter.hw.mac_addr, addr->bytes, ETH_ALEN);
  atl1c_hw_set_mac_addr(&fAdapter.hw, fAdapter.hw.mac_addr);
  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

void
CLASS::getPacketBufferConstraints(IOPacketBufferConstraints * constraints)
const
{
  DbgPrint (4, "getPacketBufferConstraints()\n");
  constraints->alignStart = kIOPacketBufferAlign8;
  constraints->alignLength = kIOPacketBufferAlign8;
}

//------------------------------------------------------------------------------

UInt32
CLASS::outputPacket(mbuf_t m, void *prm)
{
  DbgPrint (6, "outputPacket()\n");

  UInt32 ret;

  ret = atl1c_xmit_frame(m);

  if (ret == NETDEV_TX_BUSY)
  {
    return kIOReturnOutputStall;
  }

  OSSynchronizeIO();

  return kIOReturnOutputSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::registerWithPolicyMaker(IOService * policyMaker)
{
  IOReturn ior;
  DbgPrint (4, "registerWithPolicyMaker()\n");

  static IOPMPowerState powerStateArray[kPowerStateCount] = {
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {1, kIOPMDeviceUsable, kIOPMPowerOn, kIOPMPowerOn, 0, 0, 0, 0, 0, 0, 0, 0}
  };

  fCurrentPowerState = kPowerStateOn;

  ior = policyMaker->registerPowerDriver (this, powerStateArray,
                                          kPowerStateCount);

  return ior;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setPowerState(unsigned long powerStateOrdinal,
                     IOService * policyMaker)
{
  IOPCIDevice *pdev = fAdapter.pdev;

  if (!pdev)
  {
    ErrPrint("setPowerState() called with IOPCIDevice fAdapter.pdev NULL!\n");

    return IOPMAckImplied;
  }

  if (powerStateOrdinal == fCurrentPowerState)
  {
    DbgPrint (4, "setPowerState(fCurrentPowerState)\n");
    return IOPMAckImplied;
  }
  retain();
  switch (powerStateOrdinal)
  {
    case kPowerStateOff:
      DbgPrint (4, "setPowerState(kPowerStateOff)\n");
      if (fCommandGate)
      {
        fCommandGate->runAction(setPowerStateSleepAction);
      }
      break;

    case kPowerStateOn:
      DbgPrint (4, "setPowerState(kPowerStateOn)\n");
      if (fCommandGate)
      {
        fCommandGate->runAction(setPowerStateWakeAction);
      }
      break;
  }
  release();

  fCurrentPowerState = powerStateOrdinal;

  return IOPMAckImplied;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setPowerStateSleepAction(OSObject *owner,
                                void *arg1, void *arg2, void *arg3, void *arg4)
{
  if (owner)
  {
    AtherosL1cEthernet *atEth = OSDynamicCast(AtherosL1cEthernet, owner);

    if (atEth)
    {
      atEth->setPowerStateSleep();
    }
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setPowerStateWakeAction(OSObject *owner,
                               void *arg1, void *arg2, void *arg3, void *arg4)
{
  if (owner)
  {
    AtherosL1cEthernet *atEth = OSDynamicCast(AtherosL1cEthernet, owner);

    if (atEth)
    {
      atEth->setPowerStateWake();
    }
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setPowerStateSleep()
{
  DbgPrint(4, "setPowerStateSleep()\n");

  struct atl1c_adapter *adapter = &fAdapter;
  struct atl1c_hw *hw = &adapter->hw;
  IOPCIDevice *pdev = fAdapter.pdev;

  atl1c_disable_l0s_l1(hw);

  // This is already called automatically before setPowerState().
  // disable(fNetif);

  if (fMagicPacketEnabled)
  {
    if (atl1c_phy_to_ps_link(&adapter->hw) != 0)
    {
      DbgPrint(1, "PHY power saving failed.\n");
    }
    atl1c_power_saving(&(adapter->hw), AT_WUFC_MAG);
    if (pdev->hasPCIPowerManagement(kPCIPMCPMESupportFromD3Cold))
    {
      DbgPrint (3, "Enabling PME from D3cold.\n");
      pdev->enablePCIPowerManagement(kPCIPMCSPMEStatus |
                                     kPCIPMCSPMEEnable |
                                     kPCIPMCSPowerStateD3);
    }
  }
  else
  {
    atl1c_power_saving(&(adapter->hw), 0);
    if (pdev->hasPCIPowerManagement(kPCIPMCD3Support))
    {
      pdev->enablePCIPowerManagement(kPCIPMCSPowerStateD3);
    }
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setPowerStateWake()
{
  DbgPrint(4, "setPowerStateWake()\n");

  atl1c_adapter *adapter = &fAdapter;
  IOPCIDevice *pdev = adapter->pdev;

  AT_WRITE_REG(&adapter->hw, REG_WOL_CTRL, 0);
  // atl1c_reset_pcie(&adapter->hw, ATL1C_PCIE_L0S_L1_DISABLE);

  if (fCurrentPowerState == kPowerStateOff)
  {
    initPCIConfigSpace(pdev);
    atl1c_phy_reset(&adapter->hw);
    atl1c_reset_mac(&adapter->hw);
    atl1c_phy_init(&adapter->hw);
  }

  // This is already called automatically after setPowerState().
  // enable(fNetif);

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::enablePacketFilter(const OSSymbol * group, UInt32 aFilter,
                          UInt32 enabledFilters, IOOptionBits options)
{
  if (group == gIONetworkFilterGroup)
  {
    switch (aFilter)
    {
      case kIOPacketFilterMulticastAll:
        return setMulticastAllMode(true);
        break;
      case kIOPacketFilterPromiscuousAll:
        return kIOReturnUnsupported;
        break;
      default:
        break;
    }
  }
  return IOEthernetController::enablePacketFilter(group, aFilter,
                                                  enabledFilters, options);
}

//------------------------------------------------------------------------------

IOReturn
CLASS::disablePacketFilter(const OSSymbol * group, UInt32 aFilter,
                           UInt32 enabledFilters, IOOptionBits options)
{
  if (group == gIONetworkFilterGroup)
  {
    switch (aFilter)
    {
      case kIOPacketFilterMulticastAll:
        return setMulticastAllMode(false);
        break;
      case kIOPacketFilterPromiscuousAll:
        return kIOReturnUnsupported;
        break;
      default:
        break;
    }
  }
  return IOEthernetController::disablePacketFilter(group, aFilter,
                                                   enabledFilters, options);
}

//------------------------------------------------------------------------------

IOReturn
CLASS::getPacketFilters(const OSSymbol *group, UInt32 *filters) const
{
  if ((group == gIOEthernetWakeOnLANFilterGroup) && fMagicPacketSupported)
  {
    *filters = kIOEthernetWakeOnMagicPacket;
    DbgPrint(3, "kIOEthernetWakeOnMagicPacket added to filters.\n");

    return kIOReturnSuccess;
  }
  else if (group == gIONetworkFilterGroup)
  {
    IOEthernetController::getPacketFilters(group, filters);
    *filters |= kIOPacketFilterMulticastAll;
    DbgPrint(2, "Packet filters: %#x\n", (int)*filters);

    return kIOReturnSuccess;
  }

  return IOEthernetController::getPacketFilters(group, filters);
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setWakeOnMagicPacket(bool active)
{
  if (fMagicPacketSupported)
  {
    fMagicPacketEnabled = active;
    DbgPrint (2, "Wake on magic packet %s.\n",
              active ? "enabled" : "disabled");
    return kIOReturnSuccess;
  }
  return kIOReturnUnsupported;
}

//------------------------------------------------------------------------------

// Stop active DMA to allow proper shutdown.
void
CLASS::systemWillShutdown(IOOptionBits specifier)
{
  DbgPrint(4, "systemWillShutdown(%#x)\n", (uint)specifier);

  if ((kIOMessageSystemWillPowerOff | kIOMessageSystemWillRestart) & specifier)
  {
    disable(fNetif);
  }

  /* Must call super shutdown or system will stall. */
  super::systemWillShutdown(specifier);
}

//------------------------------------------------------------------------------

/**
 * Promiscuous mode set.
 **/

IOReturn
CLASS::setPromiscuousMode(bool enabled)
{
  DbgPrint(2, "%s promiscuous mode.\n", enabled ? "Enabling" : "Disabling");

  if (enabled)
  {
    fMCFlags |= IFF_PROMISC;
  }
  else
  {
    fMCFlags &= ~(IFF_PROMISC);
  }

  atl1c_set_multi();

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

/**
 * Multicast mode set.
 **/

IOReturn
CLASS::setMulticastMode(bool enabled)
{
  DbgPrint(2, "%s multicast mode.\n", enabled ? "Enabling" : "Disabling");

  if (!enabled)
  {
    // Clear the multicast list.
    fMCListLength = 0;
  }

  atl1c_set_multi();

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setMulticastAllMode (bool enabled)
{
  DbgPrint(2, "%s multicast-all mode.\n", enabled ? "Enabling" : "Disabling");

  if (enabled)
  {
    fMCFlags |= IFF_ALLMULTI;
  }
  else
  {
    fMCFlags &= ~(IFF_ALLMULTI);
  }

  atl1c_set_multi();

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::setMulticastList(IOEthernetAddress * addrs, UInt32 count)
{
  DbgPrint(6, "setMulticastList()\n");

  if (count <= MC_LIST_CAPACITY)
  {
    memcpy(&fMCList[0], addrs, count * sizeof(IOEthernetAddress));
    fMCListLength = count;
  }
  else
  {
    memcpy(&fMCList[0], addrs, MC_LIST_CAPACITY * sizeof(IOEthernetAddress));
    fMCListLength = MC_LIST_CAPACITY;
    ErrPrint("Excessively large multicast list has been truncated!\n");
  }

  atl1c_set_multi();

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::getMaxPacketSize (UInt32 *maxSize) const
{
  DbgPrint(4, "getMaxPacketSize()\n");

  const atl1c_adapter *adapter = &fAdapter;

  if (!maxSize)
  {
    return kIOReturnBadArgument;
  }
  *maxSize = adapter->hw.max_frame_size + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::getMinPacketSize (UInt32 *minSize) const
{
  return IOEthernetController::getMinPacketSize(minSize);
}


//------------------------------------------------------------------------------

IOReturn
CLASS::setMaxPacketSize (UInt32 maxSize)
{
  DbgPrint(4, "setMaxPacketSize(%u)\n", (unsigned int)maxSize);

  int err = atl1c_change_mtu(maxSize);

  if (err == -EINVAL)
  {
    return kIOReturnBadArgument;
  }
  else if (err == -ENOMEM)
  {
    return kIOReturnNoMemory;
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

UInt32
CLASS::getFeatures() const
{
  UInt32 features = kIONetworkFeatureHardwareVlan;

  if (fTSOEnabled)
  {
    features |= kIONetworkFeatureTSOIPv4 | kIONetworkFeatureTSOIPv6;
  }

	return features;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::getChecksumSupport(UInt32 *checksumMask, UInt32 checksumFamily,
                          bool isOutput)
{
  if (checksumFamily != kChecksumFamilyTCPIP)
  {
    return kIOReturnUnsupported;
  }

  if (isOutput)
  {
    *checksumMask = kChecksumTCPSum16;
    DbgPrint(2, "Enabled partial TX checksum offloading to hardware.\n");
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Atheros L1C Class Helper Methods
#pragma mark -
//------------------------------------------------------------------------------

struct atl1c_adapter *
CLASS::netdev_priv(AtherosL1cEthernet *at)
{
  return &at->fAdapter;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::atGetInterruptSource(IOService * provider)
{
  // Search for interrupt index of MSI type.
  int msi_index = -1;
  int intr_index = 0, intr_type = 0;
  IOReturn intr_ret;
  while (true)
  {
    intr_ret = provider->getInterruptType (intr_index, &intr_type);
    if (intr_ret != kIOReturnSuccess)
      break;
    if (intr_type & kIOInterruptTypePCIMessaged)
    {
      msi_index = intr_index;
      break;
    }
    intr_index++;
  }

  if (msi_index != -1)
  {
    DbgPrint (3, "MSI interrupt index: %d\n", msi_index);
    fInterruptSource =
    IOInterruptEventSource::interruptEventSource
    (this,
     OSMemberFunctionCast(IOInterruptEventSource::Action, this,
                          &CLASS::atl1c_intr),
     fAdapter.pdev,
     msi_index);
  }

  if (msi_index == -1 || fInterruptSource == NULL)
  {
    DbgPrint (1, "Warning: MSI index was not found or MSI "
              "interrupt could not be enabled.\n");
    fInterruptSource = IOInterruptEventSource::interruptEventSource
    (this,
     OSMemberFunctionCast(IOInterruptEventSource::Action,
                          this, &CLASS::atl1c_intr),
     fAdapter.pdev);
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

/*
 *  Driver	Model-name    vendor:device	Type
 *  atl1c 	AR8131        1969:1063     Gigabit Ethernet
 *  atl1c   AR8132        1969:1062     Fast Ethernet
 *  atl1c   AR8151(v1.0)  1969:1073     Gigabit Ethernet
 *  atl1c   AR8151(v2.0)  1969:1083     Gigabit Ethernet
 *  atl1c   AR8152(v1.1)  1969:2060     Fast Ethernet
 *  atl1c   AR8152(v2.0)  1969:2062     Fast Ethernet
 */
UInt32
CLASS::atGetNicType()
{
  return at_dev_table[fDeviceTableIndex].type;
}

//------------------------------------------------------------------------------

// Allocate and prepare memory segments for DMA transfers.
// dbufs, dcoms and dsegs are arrays of pointers of length num.
// Each allocated segment is contiguous of size maxsize and
// transfer direction is e.g. kIODirectionIn or kIODirectionOut.
IOReturn
CLASS::atPrepareDMAMemory(IOBufferMemoryDescriptor **dbufs,
                          IODMACommand **dcoms,
                          IODMACommand::Segment64 *dsegs,
                          IOOptionBits direction,
                          unsigned int maxsize,
                          unsigned int num)
{
  if (!dbufs || !dcoms || !dsegs || !maxsize || !num)
  {
    return kIOReturnBadArgument;
  }

  for (int i = 0; i < num; i++)
  {
    if (!dbufs[i])
    {
      dbufs[i] = IOBufferMemoryDescriptor::inTaskWithPhysicalMask
      (kernel_task, (direction | kIOMemoryPhysicallyContiguous),
       maxsize, AT_BUF_ADDR_MASK);
      if (!dbufs[i])
      {
        ErrPrint("Unable to allocate DMA buffer [%u]!\n", i);
        return kIOReturnNoMemory;
      }
    }
    if (dbufs[i]->prepare() != kIOReturnSuccess)
    {
      ErrPrint("Unable to prepare DMA buffer [%u]!\n", i);
      return kIOReturnCannotWire;
    }

    if (!dcoms[i])
    {
      dcoms[i] = IODMACommand::withSpecification(kIODMACommandOutputHost64,
                                                 32, maxsize,
                                                 IODMACommand::kMapped,
                                                 0, 8, 0, 0);
      if (!dcoms[i])
      {
        ErrPrint("Unable to create DMA command [%u]!\n", i);
        return kIOReturnError;
      }
    }
    if (dcoms[i]->setMemoryDescriptor(dbufs[i]) != kIOReturnSuccess)
    {
      ErrPrint("Unable to prepare DMA command [%u]!\n", i);
      return kIOReturnError;
    }

    UInt32 numSeg = 1;
    UInt64 offset = 0;
    if (dcoms[i]->gen64IOVMSegments(&offset, &dsegs[i],
                                    &numSeg) != kIOReturnSuccess)
    {
      ErrPrint("Unable to generate DMA segment [%u]!\n", i);
      return kIOReturnError;
    }
    DbgPrint(5, "DMA segment [%u]: addr=%#llx, len=%lld\n",
             i, dsegs[i].fIOVMAddr, dsegs[i].fLength);
  }


  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

IOReturn
CLASS::atPublishMediumDictionary()
{
  // Publish the ethernet medium dictionary items.
  if (atGetNicType () == TYPE_GIGABIT)
  {
    fMediumDict = OSDictionary::withCapacity (MEDIUM_INDEX_COUNT_GIGABIT + 1);
    OSAddNetworkMedium (kIOMediumEthernetAuto | kIOMediumOptionFlowControl,
                        0, MEDIUM_INDEX_AUTO_GIGABIT, "Autonegotiate Gigabit");
    OSAddNetworkMedium (kIOMediumEthernet1000BaseTX |
                        kIOMediumOptionFullDuplex | kIOMediumOptionFlowControl,
                        1000 * MBit, MEDIUM_INDEX_1000FD,
                        "1000Mb/s Full Duplex");
  }
  else //TYPE_FAST
  {
    fMediumDict = OSDictionary::withCapacity (MEDIUM_INDEX_COUNT_FAST + 1);
    OSAddNetworkMedium (kIOMediumEthernetAuto | kIOMediumOptionFlowControl,
                        0, MEDIUM_INDEX_AUTO_FAST, "Autonegotiate Fast");
  }
  
  OSAddNetworkMedium (kIOMediumEthernet10BaseT | kIOMediumOptionHalfDuplex |
                      kIOMediumOptionFlowControl,
                      10 * MBit, MEDIUM_INDEX_10HD, "10Mb/s Half Duplex");
  OSAddNetworkMedium (kIOMediumEthernet10BaseT | kIOMediumOptionFullDuplex |
                      kIOMediumOptionFlowControl,
                      10 * MBit, MEDIUM_INDEX_10FD, "10Mb/s Full Duplex");
  OSAddNetworkMedium (kIOMediumEthernet100BaseTX | kIOMediumOptionHalfDuplex |
                      kIOMediumOptionFlowControl,
                      100 * MBit, MEDIUM_INDEX_100HD, "100Mb/s Half Duplex");
  OSAddNetworkMedium (kIOMediumEthernet100BaseTX | kIOMediumOptionFullDuplex |
                      kIOMediumOptionFlowControl,
                      100 * MBit, MEDIUM_INDEX_100FD, "100Mb/s Full Duplex");

  if (!publishMediumDictionary(fMediumDict))
    return kIOReturnError;
  
  if (atGetNicType () == TYPE_GIGABIT)
  {
    setSelectedMedium(fMediumTable[MEDIUM_INDEX_AUTO_GIGABIT]);
  }
  else //TYPE_FAST
  {
    setSelectedMedium(fMediumTable[MEDIUM_INDEX_AUTO_FAST]);
  }

  return kIOReturnSuccess;
}

//------------------------------------------------------------------------------

// Setup some miscellaneous device information.
void
CLASS::atSetupDeviceInfo()
{
  // Get the device information table index.
  switch (fDeviceID)
  {
    case PCI_DEVICE_ID_ATTANSIC_L1C:
      fDeviceTableIndex = AT_INDEX_L1C;
      break;
    case PCI_DEVICE_ID_ATTANSIC_L2C:
      fDeviceTableIndex = AT_INDEX_L2C;
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B:
      fDeviceTableIndex = AT_INDEX_L2C_B;
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B2:
      fDeviceTableIndex = AT_INDEX_L2C_B2;
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D:
      fDeviceTableIndex = AT_INDEX_L1D;
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D_2_0:
      fDeviceTableIndex = AT_INDEX_L1D_2_0;
      break;
    default:
      fDeviceTableIndex = AT_INDEX_UNKNOWN;
      break;
  }

  // Set some friendly names in the system from the IOKit personality.
  // These properties are very poorly documented. :(
  setName(at_dev_table[fDeviceTableIndex].name);
  setProperty("name", at_dev_table[fDeviceTableIndex].name);
  // This property might not do anything in recent Mac OS X versions.
  setProperty("model", at_dev_table[fDeviceTableIndex].name);
}

//------------------------------------------------------------------------------

// Set the watchdog timer.
void
CLASS::atTaskSchedule()
{
	if (!test_bit(__AT_DOWN, &fAdapter.flags))
  {
    // Poll faster when waiting for link.
    if (test_bit(ATL1C_WORK_EVENT_LINK_CHANGE, &fAdapter.work_event))
    {
      fTimerEventSource->setTimeoutMS(100);
    }
    else
    {
      fTimerEventSource->setTimeoutMS(2000);
    }
	}
}

//------------------------------------------------------------------------------

void
CLASS::atTimerFired(OSObject *owner, IOTimerEventSource *sender)
{
  AtherosL1cEthernet *atEth = OSDynamicCast(AtherosL1cEthernet, owner);
  atEth->atl1c_common_task();
}

//------------------------------------------------------------------------------
#pragma mark -
#pragma mark Atheros L1C Linux-Based Methods
#pragma mark -
//------------------------------------------------------------------------------

int
CLASS::atl1c_alloc_rx_buffer(struct atl1c_adapter *adapter)
{
  DbgPrint (6, "atl1c_alloc_rx_buffer()\n");

  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  // IOPCIDevice *pdev = adapter->pdev;
  struct atl1c_buffer *buffer_info, *next_info;
  // struct sk_buff *skb;
  // void *vir_addr = NULL;
  u16 num_alloc = 0;
  u16 rfd_next_to_use, next_next;
  struct atl1c_rx_free_desc *rfd_desc;

  next_next = rfd_next_to_use = rfd_ring->next_to_use;
  if (++next_next == rfd_ring->count)
  {
    next_next = 0;
  }
  buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
  next_info = &rfd_ring->buffer_info[next_next];

  while (next_info->flags & ATL1C_BUFFER_FREE)
  {
    rfd_desc = ATL1C_RFD_DESC(rfd_ring, rfd_next_to_use);

    /*
     * Make buffer alignment 2 beyond a 16 byte boundary
     * this will result in a 16 byte aligned IP header after
     * the 14 byte MAC header is removed
     */
    // vir_addr = skb->data;
    ATL1C_SET_BUFFER_STATE(buffer_info, ATL1C_BUFFER_BUSY);

    buffer_info->length = adapter->rx_buffer_len;
		buffer_info->dma = fRXDMASegments[rfd_next_to_use].fIOVMAddr;

    ATL1C_SET_PCIMAP_TYPE(buffer_info, ATL1C_PCIMAP_SINGLE,
                          ATL1C_PCIMAP_FROMDEVICE);
    rfd_desc->buffer_addr = cpu_to_le64(buffer_info->dma);
    rfd_next_to_use = next_next;
    if (++next_next == rfd_ring->count)
      next_next = 0;
    buffer_info = &rfd_ring->buffer_info[rfd_next_to_use];
    next_info = &rfd_ring->buffer_info[next_next];
    num_alloc++;
  }

  if (num_alloc)
  {
    /* TODO: Update mailbox here. */
    wmb();
    rfd_ring->next_to_use = rfd_next_to_use;
    AT_WRITE_REG(&adapter->hw, REG_MB_RFD0_PROD_IDX,
                 rfd_ring->next_to_use & MB_RFDX_PROD_IDX_MASK);
    DbgPrint(5, "Total allocated space for RX descriptors: "
             "%d bytes (num_alloc=%d, adapter->rx_buffer_len=%d)\n",
             num_alloc * adapter->rx_buffer_len,
             num_alloc, adapter->rx_buffer_len);

  }

  return num_alloc;
}

//------------------------------------------------------------------------------

/**
 * atl1c_change_mtu - Change the Maximum Transfer Unit
 * @netdev: network interface device structure
 * @new_mtu: new value for maximum frame size
 *
 * Returns 0 on success, negative on failure
 */
int
CLASS::atl1c_change_mtu(int new_mtu)
{
  DbgPrint(4, "atl1c_change_mtu(%d)\n", new_mtu);

  struct atl1c_adapter *adapter = &fAdapter;
  struct atl1c_hw *hw = &adapter->hw;
  int old_mtu   = hw->max_frame_size;
  int max_frame = new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN;
  DbgPrint(2, "Changing MTU from %d to %d...\n", old_mtu, new_mtu);

  /* Fast Ethernet controller doesn't support jumbo packet */
  if (((hw->nic_type == athr_l2c ||
        hw->nic_type == athr_l2c_b ||
        hw->nic_type == athr_l2c_b2) && new_mtu > ETH_DATA_LEN) ||
      max_frame < ETH_ZLEN + ETH_FCS_LEN ||
      max_frame > MAX_JUMBO_FRAME_SIZE) {
    DbgPrint(1, "Invalid MTU setting!\n");
    return -EINVAL;
  }
  /* set MTU */
  if (old_mtu != new_mtu)
  {
    while (test_and_set_bit(__AT_RESETTING, &adapter->flags))
    {
      msleep(1);
    }
    adapter->hw.max_frame_size = new_mtu;
    atl1c_set_rxbufsize(adapter);
    disable(fNetif);
    // FIXME: Do something here?
    // netdev_update_features(netdev);
    enable(fNetif);
    clear_bit(__AT_RESETTING, &adapter->flags);
  }
  return 0;
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_check_link_status(struct atl1c_adapter *adapter)
{
  DbgPrint(4, "atl1c_check_link_status()\n");

  atl1c_hw *hw = &adapter->hw;
  int err;
  u16 speed, duplex, phy_data;
  u32 currentMediumIndex;
  IOInterruptState flags;
  UInt64 uptime;

  /* MII_BMSR must read twice. */
  spin_lock_irqsave(&adapter->mdio_lock,flags);
  atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
  atl1c_read_phy_reg(hw, MII_BMSR, &phy_data);
  spin_unlock_irqrestore(&adapter->mdio_lock,flags);
  
  if ((phy_data & BMSR_LSTATUS) == 0)
  {
    if (uptime < fLinkTimeout)
    {
      set_bit(ATL1C_WORK_EVENT_LINK_CHANGE, &adapter->work_event);
      DbgPrint(3, "Link timeout has not yet expired.\n");
    }
    // Only continue if link was up previously.
    if (!fLinkUp)
    {
      return;
    }
    DbgPrint(2, "Link is down.\n");
    setLinkStatus(kIONetworkLinkValid);
    fLinkUp = false;
    hw->hibernate = true;
    if (atl1c_reset_mac(hw) != 0)
    {
      DbgPrint(1, "Reset MAC failed.\n");
    }
    atl1c_set_aspm(hw, SPEED_0);
    atl1c_post_phy_linkchg(hw, SPEED_0);
    atl1c_reset_dma_ring(adapter);
    atl1c_configure(adapter);
    clock_get_uptime(&uptime);
  }
  else
  {
    hw->hibernate = false;
    spin_lock_irqsave(&adapter->mdio_lock,flags);
    err = atl1c_get_speed_and_duplex(&fAdapter.hw, &speed, &duplex);
    spin_unlock_irqrestore(&adapter->mdio_lock,flags);
    if (unlikely(err))
    {
      ErrPrint("Unable to get speed and duplex of link!\n");
      return;
    }
    
    // Link result is our setting.
    if (!fLinkUp ||
        adapter->link_speed != speed ||
        adapter->link_duplex != duplex)
    {
      adapter->link_speed  = speed;
      adapter->link_duplex = duplex;
      atl1c_set_aspm(hw, speed);
      atl1c_post_phy_linkchg(hw, speed);
      atl1c_start_mac(adapter);
      DbgPrint(2, "Link is up: %dMb/s %s\n",
               adapter->link_speed,
               adapter->link_duplex == FULL_DUPLEX ?
               "Full Duplex" : "Half Duplex");
    }
    
    if (atGetNicType() == TYPE_GIGABIT)
    {
      currentMediumIndex = MEDIUM_INDEX_AUTO_GIGABIT;
    }
    else // TYPE_FAST
    {
      currentMediumIndex = MEDIUM_INDEX_AUTO_FAST;
    }
    if (hw->autoneg_advertised != ADVERTISED_Autoneg)
    {
      if (speed == SPEED_10 && duplex == FULL_DUPLEX)
        currentMediumIndex = MEDIUM_INDEX_10FD;
      else if (speed == SPEED_100 && duplex == FULL_DUPLEX)
        currentMediumIndex = MEDIUM_INDEX_100FD;
      else if (speed == SPEED_1000 && duplex == FULL_DUPLEX)
        currentMediumIndex = MEDIUM_INDEX_1000FD;
      else if (speed == SPEED_10 && duplex == HALF_DUPLEX)
        currentMediumIndex = MEDIUM_INDEX_10HD;
      else if (speed == SPEED_100 && duplex == HALF_DUPLEX)
        currentMediumIndex = MEDIUM_INDEX_100HD;
    }
    
    setLinkStatus(kIONetworkLinkActive | kIONetworkLinkValid,
                  fMediumTable[currentMediumIndex], speed * MBit);
    fLinkUp = true;
  }
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_clean_buffer (IOPCIDevice * pdev, struct atl1c_buffer *buffer_info,
                           int in_irq)
{
  // u16 pci_driection;
  if (buffer_info->flags & ATL1C_BUFFER_FREE)
    return;
#if 0
  if (buffer_info->dma)
  {
    if (buffer_info->flags & ATL1C_PCIMAP_FROMDEVICE)
      pci_driection = PCI_DMA_FROMDEVICE;
    else
      pci_driection = PCI_DMA_TODEVICE;

    if (buffer_info->flags & ATL1C_PCIMAP_SINGLE)
      pci_unmap_single(pdev, buffer_info->dma,
                       buffer_info->length, pci_driection);
    else if (buffer_info->flags & ATL1C_PCIMAP_PAGE)
      pci_unmap_page(pdev, buffer_info->dma,
                     buffer_info->length, pci_driection);
  }
#endif
  if (buffer_info->skb)
  {
    if (in_irq)
      dev_kfree_skb_irq(buffer_info->skb);
    else
      dev_kfree_skb(buffer_info->skb);
  }
  buffer_info->skb = NULL;
  ATL1C_SET_BUFFER_STATE(buffer_info, ATL1C_BUFFER_FREE);
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_clean_rx_irq (struct atl1c_adapter *adapter)
{
  DbgPrint (6, "atl1c_clean_rx_irq()\n");

  u16 rfd_num, rfd_index;
  u16 count = 0;
  u16 length;
  // IOPCIDevice *pdev = adapter->pdev;
  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring;
  struct sk_buff *skb;
  struct atl1c_recv_ret_status *rrs;
  struct atl1c_buffer *buffer_info;
  u32 packet_size = 0;

  DbgPrint (5, "Cleaning RX descriptors: "
            "rfd_ring->next_to_clean=%d, rrd_ring->next_to_clean=%d\n",
            rfd_ring->next_to_clean, rrd_ring->next_to_clean);

  while (1)
  {
    rrs = ATL1C_RRD_DESC (rrd_ring, rrd_ring->next_to_clean);
    if (likely(RRS_RXD_IS_VALID(rrs->word3)))
    {
      rfd_num = (rrs->word0 >> RRS_RX_RFD_CNT_SHIFT) &
      RRS_RX_RFD_CNT_MASK;
      if (unlikely(rfd_num != 1))
        DbgPrint (2, "Multi-RFD not supported yet!\n");
      goto rrs_checked;
    }
    else
    {
      break;
    }
  rrs_checked:
    // Turn off RRS_RXD_IS_VALID
    atl1c_clean_rrd (rrd_ring, rrs, rfd_num);
    if (rrs->word3 & (RRS_RX_ERR_SUM | RRS_802_3_LEN_ERR))
    {
      atl1c_clean_rfd (rfd_ring, rrs, rfd_num);
      DbgPrint (1, "Wrong packet! RRS word3 is %x\n", (uint)rrs->word3);
      continue;
    }

    length = le16_to_cpu ((rrs->word3 >> RRS_PKT_SIZE_SHIFT) &
                          RRS_PKT_SIZE_MASK);
    /* Good Receive */
    if (likely(rfd_num == 1))
    {
      rfd_index = (rrs->word0 >> RRS_RX_RFD_INDEX_SHIFT) &
      RRS_RX_RFD_INDEX_MASK;
      DbgPrint(5, "RRS_PKT_SIZE=%d RRS_RX_RFD_INDEX=%d\n",
               length, rfd_index);
      buffer_info = &rfd_ring->buffer_info[rfd_index];
      // Remove the ethernet FCS since the NIC doesn't automatically.
      packet_size = length - kIOEthernetCRCSize;
      pci_unmap_single(pdev, buffer_info->dma,
                       buffer_info->length, PCI_DMA_FROMDEVICE);
      skb = allocatePacket(packet_size + 2);
      mbuf_adj(skb, ETHER_ALIGN);
      if (!skb)
      {
        DbgPrint(1, "Memory squeeze, deferring RX packet.\n");
        break;
      }
      fRXDMACommands[rfd_index]->synchronize(kIODirectionIn);
      mbuf_copyback(skb, 0, packet_size,
                    fRXDMABuffers[rfd_index]->getBytesNoCopy(), MBUF_DONTWAIT);
      DbgPrint(5, "RX mbuf data start address: %#llx\n",
               (addr64_t)mbuf_data(skb));
    }
    else
    {
      /* TODO: Multi RFD. */
      DbgPrint (2, "Multi RFD not supported yet!\n");
      break;
    }
    atl1c_clean_rfd (rfd_ring, rrs, rfd_num);
    if (rrs->word3 & RRS_VLAN_INS)
    {
			u16 vlan;

			AT_TAG_TO_VLAN(rrs->vlan_tag, vlan);
			vlan = le16_to_cpu(vlan);
      DbgPrint(5, "RX on VLAN #%d\n", vlan);
			__vlan_hwaccel_put_tag(skb, vlan);
		}

    fNetif->inputPacket(skb, packet_size,
                        IONetworkInterface::kInputOptionQueuePacket);
    count++;
  }

  if (count)
  {
    fNetif->flushInputQueue();
    fNetStats->inputPackets += count;

    atl1c_alloc_rx_buffer (adapter);
  }
}

//------------------------------------------------------------------------------

/**
 * atl1c_clean_rx_ring - Free rx-reservation skbs
 * @adapter: board private structure
 */
void
CLASS::atl1c_clean_rx_ring(struct atl1c_adapter *adapter)
{
  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring;
  struct atl1c_buffer *buffer_info;
  IOPCIDevice *pdev = adapter->pdev;
  int j;

  for (j = 0; j < rfd_ring->count; j++)
  {
    buffer_info = &rfd_ring->buffer_info[j];
    atl1c_clean_buffer(pdev, buffer_info, 0);
  }
  /* zero out the descriptor ring */
  memset(rfd_ring->desc, 0, rfd_ring->size);
  rfd_ring->next_to_clean = 0;
  rfd_ring->next_to_use = 0;
  rrd_ring->next_to_use = 0;
  rrd_ring->next_to_clean = 0;
}

//------------------------------------------------------------------------------

bool
CLASS::atl1c_clean_tx_irq(atl1c_adapter * adapter, atl1c_trans_queue type)
{
  DbgPrint (6, "atl1c_clean_tx_irq()\n");

  struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
  struct atl1c_buffer *buffer_info;
  IOPCIDevice *pdev = adapter->pdev;
  u16 next_to_clean = atomic_read(&tpd_ring->next_to_clean);
  u16 hw_next_to_clean;
  u16 reg;

  reg = type == atl1c_trans_high ? REG_TPD_PRI1_CIDX : REG_TPD_PRI0_CIDX;

  AT_READ_REGW(&adapter->hw, reg, &hw_next_to_clean);

  while (next_to_clean != hw_next_to_clean)
  {
    buffer_info = &tpd_ring->buffer_info[next_to_clean];
    DbgPrint(5, "Cleaning TX buffer: tpd_ring->next_to_clean=#%d\n",
             next_to_clean);
    atl1c_clean_buffer(pdev, buffer_info, 1);
    if (++next_to_clean == tpd_ring->count)
      next_to_clean = 0;
    atomic_set(&tpd_ring->next_to_clean, next_to_clean);
  }
  // Packet freeing delayed in atl1c_clean_buffer during IRQ.
  fNetStats->outputPackets += releaseFreePackets();
  fTransmitQueue->service(IOBasicOutputQueue::kServiceAsync);

  return true;
}

//------------------------------------------------------------------------------

/*
 * atl1c_clean_tx_ring - Free Tx-skb
 * @adapter: board private structure
 */
void
CLASS::atl1c_clean_tx_ring (struct atl1c_adapter *adapter,
                            enum atl1c_trans_queue type)
{
  DbgPrint(4, "atl1c_clean_tx_ring()\n");

  struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
  struct atl1c_buffer *buffer_info;
  IOPCIDevice *pdev = adapter->pdev;
  u16 index, ring_count;

  ring_count = tpd_ring->count;
  for (index = 0; index < ring_count; index++)
  {
    buffer_info = &tpd_ring->buffer_info[index];
    atl1c_clean_buffer(pdev, buffer_info, 0);
  }

  /* Zero out TX-buffers */
  memset (tpd_ring->desc, 0, sizeof (struct atl1c_tpd_desc) * ring_count);
  atomic_set (&tpd_ring->next_to_clean, 0);
  tpd_ring->next_to_use = 0;
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_common_task()
{
  DbgPrint(6, "atl1c_common_task()\n");

	struct atl1c_adapter *adapter = &fAdapter;

	if (test_bit(__AT_DOWN, &adapter->flags))
		return;

	if (test_and_clear_bit(ATL1C_WORK_EVENT_RESET, &adapter->work_event))
  {
    DbgPrint(3, "Common task handling reset event.\n");
		// netif_device_detach(netdev);
		disable(fNetif);
		enable(fNetif);
		// netif_device_attach(netdev);
	}

	if (test_and_clear_bit(ATL1C_WORK_EVENT_LINK_CHANGE, &adapter->work_event))
  {
    DbgPrint(3, "Common task handling link change event.\n");
		atl1c_irq_disable(adapter);
		atl1c_check_link_status(adapter);
		atl1c_irq_enable(adapter);
	}
  
  // Reset the timer.
  atTaskSchedule();
}

//------------------------------------------------------------------------------

int
CLASS::atl1c_configure(struct atl1c_adapter *adapter)
{
  DbgPrint (4, "atl1c_configure()\n");

  int num;

  atl1c_init_ring_ptrs(adapter);
  atl1c_set_multi();
  atl1c_restore_vlan(adapter);

  num = atl1c_alloc_rx_buffer(adapter);
  if (unlikely(num == 0))
    return -ENOMEM;

  if (atl1c_configure_mac(adapter))
    return -EIO;

  return 0;
}

//------------------------------------------------------------------------------

/**
 * atl1c_intr - Interrupt Handler
 **/
void
CLASS::atl1c_intr(OSObject *client, IOInterruptEventSource *src, int count)
{
  DbgPrint (6, "atl1c_intr()\n");

  atl1c_adapter *adapter = &fAdapter;
  atl1c_hw *hw = &adapter->hw;
  s32 max_ints = AT_MAX_INT_WORK;

  u32 status;
  u32 reg_data;

  do
  {
    AT_READ_REG (hw, REG_ISR, &reg_data);
    status = reg_data & hw->intr_mask;

    if (status == 0 || (status & ISR_DIS_INT) != 0) break;

    DbgPrint (5, "Interrupt generated: %#x\n", (uint)status);

    /* link event */
    if (status & ISR_GPHY)
      atl1c_clear_phy_int (adapter);

    /* Ack ISR */
    AT_WRITE_REG (hw, REG_ISR, status | ISR_DIS_INT);
    if (status & ISR_RX_PKT) atl1c_clean_rx_irq(adapter);
    if (status & ISR_TX_PKT) atl1c_clean_tx_irq(adapter, atl1c_trans_normal);

    /* check if PCIE PHY Link down */
    if (status & ISR_ERROR)
    {
      ErrPrint ("Hardware error! Interrupt status: 0x%x\n",
                (uint)(status & ISR_ERROR));
      set_bit(ATL1C_WORK_EVENT_RESET, &adapter->work_event);
      return;
    }

    if (status & ISR_OVER)
    {
      ErrPrint ("TX/RX overflow! Interrupt status: 0x%x\n",
                (uint)(status & ISR_OVER));
    }

    /* link event */
    if (status & (ISR_GPHY | ISR_MANUAL))
    {
      DbgPrint(2, "Link state change event detected.\n");
      atl1c_link_chg_event(adapter);
      break;
    }
  }
  while (--max_ints > 0);
  /* re-enable Interrupt */
  AT_WRITE_REG (&adapter->hw, REG_ISR, 0);
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_link_chg_event(struct atl1c_adapter *adapter)
{
	u16 phy_data;
	u16 link_up;

	spin_lock(&adapter->mdio_lock);
	atl1c_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	atl1c_read_phy_reg(&adapter->hw, MII_BMSR, &phy_data);
	spin_unlock(&adapter->mdio_lock);
	link_up = phy_data & BMSR_LSTATUS;
	// Notify upper layers that link is down ASAP.
	if (!link_up)
  {
      DbgPrint(1, "NIC link is down! Cable disconnected?\n");
      setLinkStatus(kIONetworkLinkValid);
      adapter->link_speed = SPEED_0;
	}
  set_bit(ATL1C_WORK_EVENT_LINK_CHANGE, &adapter->work_event);
  clock_interval_to_deadline(AT_TRY_LINK_TIMEOUT, kMillisecondScale,
                             &fLinkTimeout);
  DbgPrint(3, "Link timeout initialised.\n");
}

//------------------------------------------------------------------------------

/*
 *  caller should hold mdio_lock
 */
int
CLASS::atl1c_mdio_read (int phy_id, int reg_num)
{
  u16 result;

  atl1c_read_phy_reg (&fAdapter.hw, reg_num, &result);

  return result;
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_mdio_write (int phy_id, int reg_num, int val)
{
  atl1c_write_phy_reg (&fAdapter.hw, reg_num, val);
}

//------------------------------------------------------------------------------

/**
 * at_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in at_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * at_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static const u32 atl1c_default_msg = NETIF_MSG_DRV | NETIF_MSG_PROBE |
NETIF_MSG_LINK | NETIF_MSG_TIMER | NETIF_MSG_IFDOWN | NETIF_MSG_IFUP;
int
CLASS::atl1c_probe()
{
  DbgPrint (4, "atl1c_probe()\n");

  atl1c_adapter *adapter = &fAdapter;
  int err = 0;

  IOPCIDevice *pdev = fAdapter.pdev;
  pdev->setBusMasterEnable(true);
  pdev->setMemoryEnable(true);
  pdev->setIOEnable(false);
  fVendorID = pdev->configRead16(kIOPCIConfigVendorID);
  fDeviceID = pdev->configRead16(kIOPCIConfigDeviceID);

  DbgPrint (2, "Vendor ID: %#x Device ID: %#x\n", fVendorID, fDeviceID);
#ifdef __LP64__
  DbgPrint (3, "MMR0 address: %#010x\n",
            pdev->configRead32(kIOPCIConfigBaseAddress0));
#else // __ILP32__
  DbgPrint (3, "MMR0 address: %#010lx\n",
            pdev->configRead32(kIOPCIConfigBaseAddress0));
#endif // __ILP32__

  fRegMap = pdev->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
  if (fRegMap == NULL)
  {
    ErrPrint ("Unable to create mapping of physical PCI memory!\n");
    return -1;
  }

#ifdef __LP64__
  DbgPrint (2, "PCI memory mapped at bus address %#018llx, "
            "virtual address %#018llx, length %llu bytes.\n",
            fRegMap->getPhysicalAddress(),
            fRegMap->getVirtualAddress(),
            fRegMap->getLength());
#else // __ILP32__
  DbgPrint (2, "PCI memory mapped at bus address %#010lx, "
            "virtual address %#010x, length %lu bytes.\n",
            fRegMap->getPhysicalAddress(),
            fRegMap->getVirtualAddress(),
            fRegMap->getLength());
#endif // __ILP32__

  fRegMap->retain ();
  // FIXME: Replace DbgPrint with this debugging method instead.
  // adapter->msg_enable = netif_msg_init(-1, atl1c_default_msg);
  adapter->msg_enable = netif_msg_init(0, atl1c_default_msg);
  adapter->hw.hw_addr = reinterpret_cast <u8*>(fRegMap->getVirtualAddress());

  DbgPrint(3, "REG_MASTER_CTRL = %#010x\n",
           OSReadLittleInt32(adapter->hw.hw_addr, REG_MASTER_CTRL));

  /* setup the private structure */
  err = atl1c_sw_init (adapter);
  if (err)
  {
    ErrPrint ("Software structure initialisation failed!\n");
    return -1;
  }
  initPCIConfigSpace(pdev);

  /* Init GPHY as early as possible due to power saving issue  */
  atl1c_phy_reset(&adapter->hw);

  err = atl1c_reset_mac(&adapter->hw);
  if (err)
  {
    err = -EIO;
    return err;
  }

  /* reset the controller to
   * put the device in a known good starting state */
  err = atl1c_phy_init(&adapter->hw);
  if (err)
  {
    err = -EIO;
    return err;
  }

  /* copy the MAC address out of the EEPROM */
  atl1c_read_mac_addr(&adapter->hw);
  if (netif_msg_probe(adapter))
		dev_dbg(&pdev->dev, "MAC address: [%02x:%02x:%02x:%02x:%02x:%02x]\n",
            adapter->hw.mac_addr[0],
            adapter->hw.mac_addr[1],
            adapter->hw.mac_addr[2],
            adapter->hw.mac_addr[3],
            adapter->hw.mac_addr[4],
            adapter->hw.mac_addr[5]);

  atl1c_hw_set_mac_addr(&adapter->hw, adapter->hw.mac_addr);

  return 0;
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_reset_dma_ring (struct atl1c_adapter *adapter)
{
  DbgPrint (4, "atl1c_reset_dma_ring()\n");

  /* release tx-pending skbs and reset tx/rx ring index */
  atl1c_clean_tx_ring (adapter, atl1c_trans_normal);
  atl1c_clean_tx_ring (adapter, atl1c_trans_high);
  atl1c_clean_rx_ring (adapter);
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_restore_vlan(struct atl1c_adapter *adapter)
{

	DbgPrint(4, "atl1c_restore_vlan()\n");
	atl1c_vlan_mode(this, 0);
}

//------------------------------------------------------------------------------

/**
 * atl1c_set_multi - Multicast and Promiscuous mode set
 * @netdev: network interface device structure
 *
 * The set_multi entry point is called whenever the multicast address
 * list or the network interface flags are updated.  This routine is
 * responsible for configuring the hardware for proper multicast,
 * promiscuous mode, and all-multi behavior.
 */
void
CLASS::atl1c_set_multi()
{
  DbgPrint (6, "atl1c_set_multi()\n");

  struct atl1c_adapter *adapter = &fAdapter;
  struct atl1c_hw *hw = &adapter->hw;
  u32 mac_ctrl_data;
  u32 hash_value;
  u8 i;

  /* Check for Promiscuous and All Multicast modes */
  AT_READ_REG (hw, REG_MAC_CTRL, &mac_ctrl_data);

  if (fMCFlags & IFF_PROMISC)
  {
    mac_ctrl_data |= MAC_CTRL_PROMIS_EN;
  }
  else if (fMCFlags & IFF_ALLMULTI)
  {
    mac_ctrl_data |= MAC_CTRL_MC_ALL_EN;
    mac_ctrl_data &= ~MAC_CTRL_PROMIS_EN;
  }
  else
  {
    mac_ctrl_data &= ~(MAC_CTRL_PROMIS_EN | MAC_CTRL_MC_ALL_EN);
  }
  
  AT_WRITE_REG (hw, REG_MAC_CTRL, mac_ctrl_data);

  /* Clear the old settings from the multicast hash table. */
  AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 0, 0);
  AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0);
  AT_WRITE_FLUSH(&adapter->hw);

  for (i = 0; i < fMCListLength; i++)
  {
    hash_value = atl1c_hash_mc_addr(hw, &fMCList[i].bytes[0]);
    atl1c_hash_set(hw, hash_value);
  }
#ifdef DEBUG // Print the multicast address list.
#if DEBUGLEVEL > 4
  if (fMCListLength == 0)
  {
    DbgPrint(3, "Multicast address list is empty.\n");
  }
  else
  {
    DbgPrint(3, "Multicast list set to: ");
    for (i = 0; i < fMCListLength; i++)
    {
      IOLog("[");
      for (int j = 0; j < kIOEthernetAddressSize; j++)
      {
        IOLog("%02x", fMCList[i].bytes[j]);
        if (j != kIOEthernetAddressSize-1) IOLog(":");
      }
      IOLog("] ");
    }
    IOLog("\n");
  }
#endif // DEBUGLEVEL
#endif // DEBUG

#if 0
  AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 0, 0xffffffff);
  AT_WRITE_REG_ARRAY(hw, REG_RX_HASH_TABLE, 1, 0xffffffff);
  AT_WRITE_FLUSH(&adapter->hw);
#endif
}

//------------------------------------------------------------------------------

// Handle TCP Segment Offload as well as partial hardware TCP checksums.
int
CLASS::atl1c_tso_csum(struct atl1c_adapter *adapter,
                   struct sk_buff *skb,
                   struct atl1c_tpd_desc **tpd,
                   enum atl1c_trans_queue type)
{
	u8 hdr_len;
	u32 real_len;
	mbuf_tso_request_flags_t offload_type;
  // int err;

	if (skb_is_gso(skb))
  {
  // FIXME: Add an mbuf_pullup() call somewhere.
  // The following code probably isn't relevant to mbufs, but some code
  // should be added somewhere to use mbuf_pullup to ensure the packet
  // headers are all in the first mbuf of the chain. Generally it seems that
  // Mac OS X sends packets from the upper layer with the header already neatly
  // placed in the first mbuf, but just in case...
#if 0
		if (skb_header_cloned(skb))
    {
			err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
			if (unlikely(err))
				return -1;
		}
#endif
		offload_type = gso_type(skb);

		if (offload_type & SKB_GSO_TCPV4)
    {
      DbgPrint(5, "IPv4 TSO packet.\n");
			real_len = (((unsigned char *)ip_hdr(skb) -
                   (unsigned char *)mbuf_data(skb))
                  + ntohs(ip_hdr(skb)->ip_len));

			if (real_len < mbuf_pkthdr_len(skb))
      {
        mbuf_pkthdr_setlen(skb, real_len);
        mbuf_adj(skb, real_len - mbuf_pkthdr_len(skb));
      }

			hdr_len = (skb_transport_offset(skb) + tcp_hdrlen(skb));
			if (unlikely(mbuf_pkthdr_len(skb) == hdr_len))
      {
				/* Only checksum needed. */
				DbgPrint(1, "IPv4 TSO with zero data?\n");
				goto check_sum;
			}
      else
      {
				ip_hdr(skb)->ip_sum = 0;
				tcp_hdr(skb)->th_sum = ~csum_tcpudp_magic(ip_hdr(skb)->ip_src.s_addr,
                                                  ip_hdr(skb)->ip_dst.s_addr,
                                                  0, IPPROTO_TCP, 0);
				(*tpd)->word1 |= 1 << TPD_IPV4_PACKET_SHIFT;
			}
		}

		if (offload_type & SKB_GSO_TCPV6)
    {
      DbgPrint(5, "IPv6 TSO packet.\n");

			struct atl1c_tpd_ext_desc *etpd =
      *(struct atl1c_tpd_ext_desc **)(tpd);

			memset(etpd, 0, sizeof(struct atl1c_tpd_ext_desc));
			*tpd = atl1c_get_tpd(adapter, type);
			ipv6_hdr(skb)->ip6_plen = 0;
			/* check payload == 0 byte ? */
			hdr_len = (skb_transport_offset(skb) + tcp_hdrlen(skb));
			if (unlikely(mbuf_pkthdr_len(skb) == hdr_len))
      {
				/* Only checksum needed. */
				DbgPrint(1, "IPV6 TSO with zero data?\n");
				goto check_sum;
			}
      else
      {
				tcp_hdr(skb)->th_sum = ~csum_ipv6_magic(&ipv6_hdr(skb)->ip6_src,
                                                &ipv6_hdr(skb)->ip6_dst,
                                                0, IPPROTO_TCP, 0);
      }
			etpd->word1 |= 1 << TPD_LSO_EN_SHIFT;
			etpd->word1 |= 1 << TPD_LSO_VER_SHIFT;
			etpd->pkt_len = cpu_to_le32(mbuf_pkthdr_len(skb));
			(*tpd)->word1 |= 1 << TPD_LSO_VER_SHIFT;
		}

		(*tpd)->word1 |= 1 << TPD_LSO_EN_SHIFT;
		(*tpd)->word1 |= (skb_transport_offset(skb) & TPD_TCPHDR_OFFSET_MASK) <<
                     TPD_TCPHDR_OFFSET_SHIFT;
		(*tpd)->word1 |= (gso_size(skb) & TPD_MSS_MASK) << TPD_MSS_SHIFT;
		return 0;
	}

check_sum:
  UInt32 chk_demand;
	u16 cso, css;

  getChecksumDemand(skb, kChecksumFamilyTCPIP, &chk_demand,
                    &cso,  // Start offset.
                    &css); // Stuff offset.
	if (chk_demand & kChecksumTCPSum16)
	// if (likely(skb->ip_summed == CHECKSUM_PARTIAL))
  {
		// u8 css, cso;
		// cso = skb_checksum_start_offset(skb);

		if (unlikely(cso & 0x1))
    {
			ErrPrint("Payload offset should not be an even number!\n");
			return -1;
		}
    else
    {
			// css = cso + skb->csum_offset;
      
			(*tpd)->word1 |= ((cso >> 1) & TPD_PLOADOFFSET_MASK) <<
      TPD_PLOADOFFSET_SHIFT;
			(*tpd)->word1 |= ((css >> 1) & TPD_CCSUM_OFFSET_MASK) <<
      TPD_CCSUM_OFFSET_SHIFT;
			(*tpd)->word1 |= 1 << TPD_CCSUM_EN_SHIFT;
		}
	}

	return 0;
}

//------------------------------------------------------------------------------

int
CLASS::atl1c_tx_map(struct atl1c_adapter *adapter,
                  struct sk_buff *skb, struct atl1c_tpd_desc *tpd,
                  enum atl1c_trans_queue type)
{
	struct atl1c_tpd_desc *use_tpd = NULL;
	struct atl1c_buffer *buffer_info = NULL;
	u16 buf_len = mbuf_len(skb);
	u16 map_len = 0;
	u16 mapped_len = 0;
	u16 hdr_len = 0;
	// u16 nr_frags;
  sk_buff *frag = NULL;
	int tso;
  u16 *next_to_use = &adapter->tpd_ring[type].next_to_use;
  // Ensure no accidental writing beyond allocated DMA buffers!
  u16 max_len = TPD_MSS_MASK + 1;

	// nr_frags = atl1c_cal_tpd_req(skb) - 1;

	tso = (tpd->word1 >> TPD_LSO_EN_SHIFT) & TPD_LSO_EN_MASK;
	if (tso)
  {
		/* TSO */
		map_len = hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		use_tpd = tpd;

		buffer_info = atl1c_get_tx_buffer(adapter, use_tpd);
		buffer_info->length = map_len > max_len ? max_len : map_len;
    mbuf_copydata(skb, 0, buffer_info->length,
                  fTXDMABuffers[*next_to_use]->getBytesNoCopy(0, hdr_len));
    fTXDMACommands[*next_to_use]->synchronize(kIODirectionOut);
		buffer_info->dma = fTXDMASegments[*next_to_use].fIOVMAddr;
		ATL1C_SET_BUFFER_STATE(buffer_info, ATL1C_BUFFER_BUSY);
		ATL1C_SET_PCIMAP_TYPE(buffer_info, ATL1C_PCIMAP_SINGLE,
                          ATL1C_PCIMAP_TODEVICE);
		mapped_len += map_len;
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len = cpu_to_le16(buffer_info->length);
	}

	if (mapped_len < buf_len)
  {
		/* mapped_len == 0, means we should use the first tpd,
     which is given by caller  */
		if (mapped_len == 0)
    {
			use_tpd = tpd;
    }
		else
    {
			use_tpd = atl1c_get_tpd(adapter, type);
			memcpy(use_tpd, tpd, sizeof(struct atl1c_tpd_desc));
		}
		buffer_info = atl1c_get_tx_buffer(adapter, use_tpd);
		buffer_info->length = (buf_len - mapped_len) >
                          max_len ? max_len : (buf_len - mapped_len);
    mbuf_copydata(skb, mapped_len, buffer_info->length,
                  fTXDMABuffers[*next_to_use]->getBytesNoCopy
                  (0, buffer_info->length));
    fTXDMACommands[*next_to_use]->synchronize(kIODirectionOut);
		buffer_info->dma = fTXDMASegments[*next_to_use].fIOVMAddr;
		ATL1C_SET_BUFFER_STATE(buffer_info, ATL1C_BUFFER_BUSY);
		ATL1C_SET_PCIMAP_TYPE(buffer_info, ATL1C_PCIMAP_SINGLE,
                          ATL1C_PCIMAP_TODEVICE);
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len = cpu_to_le16(buffer_info->length);
	}

	for (frag = mbuf_next(skb); frag != NULL; frag = mbuf_next(frag))
  {
		use_tpd = atl1c_get_tpd(adapter, type);
		memcpy(use_tpd, tpd, sizeof(struct atl1c_tpd_desc));

		buffer_info = atl1c_get_tx_buffer(adapter, use_tpd);
		buffer_info->length = mbuf_len(frag) > max_len ? max_len : mbuf_len(frag);
    mbuf_copydata(frag, 0, buffer_info->length,
                  fTXDMABuffers[*next_to_use]->getBytesNoCopy
                  (0, buffer_info->length));
    fTXDMACommands[*next_to_use]->synchronize(kIODirectionOut);
		buffer_info->dma = fTXDMASegments[*next_to_use].fIOVMAddr;
		ATL1C_SET_BUFFER_STATE(buffer_info, ATL1C_BUFFER_BUSY);
		ATL1C_SET_PCIMAP_TYPE(buffer_info, ATL1C_PCIMAP_PAGE,
                          ATL1C_PCIMAP_TODEVICE);
		use_tpd->buffer_addr = cpu_to_le64(buffer_info->dma);
		use_tpd->buffer_len = cpu_to_le16(buffer_info->length);
	}
  
	/* The last tpd */
	use_tpd->word1 |= 1 << TPD_EOP_SHIFT;
	/* The last buffer info contain the skb address,
   so it will be free after unmap */
	buffer_info->skb = skb;

  return 0;
}

//------------------------------------------------------------------------------

void
CLASS::atl1c_tx_queue(struct atl1c_adapter *adapter, struct sk_buff *skb,
                      struct atl1c_tpd_desc *tpd, enum atl1c_trans_queue type)
{
  DbgPrint (6, "atl1c_tx_queue()\n");

  struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
  u16 reg;

  reg = type == atl1c_trans_high ? REG_TPD_PRI1_PIDX : REG_TPD_PRI0_PIDX;
  DbgPrint(5, "DMA TX to %s beginning...\n",
           (type == atl1c_trans_high ?
            "REG_TPD_PRI1_PIDX" : "REG_TPD_PRI0_PIDX"));
  AT_WRITE_REGW(&adapter->hw, reg, tpd_ring->next_to_use);
}

//------------------------------------------------------------------------------

#if 0
void __atl1c_vlan_mode(netdev_features_t features, u32 *mac_ctrl_data)
{
	if (features & NETIF_F_HW_VLAN_RX)
  {
		/* enable VLAN tag insert/strip */
		*mac_ctrl_data |= MAC_CTRL_RMV_VLAN;
	}
  else
  {
		/* disable VLAN tag insert/strip */
		*mac_ctrl_data &= ~MAC_CTRL_RMV_VLAN;
	}
}
#endif

//------------------------------------------------------------------------------

void
CLASS::atl1c_vlan_mode(net_device *netdev, netdev_features_t features)
{
  DbgPrint(4, "atl1c_vlan_mode()\n");

	struct atl1c_adapter *adapter = netdev_priv(netdev);
	u32 mac_ctrl_data = 0;

	atl1c_irq_disable(adapter);
	AT_READ_REG(&adapter->hw, REG_MAC_CTRL, &mac_ctrl_data);
	// __atl1c_vlan_mode(features, &mac_ctrl_data);
  mac_ctrl_data |= MAC_CTRL_RMV_VLAN;
	AT_WRITE_REG(&adapter->hw, REG_MAC_CTRL, mac_ctrl_data);
	atl1c_irq_enable(adapter);
}

//------------------------------------------------------------------------------

UInt32
CLASS::atl1c_xmit_frame(struct sk_buff *skb)
{
  DbgPrint (6, "atl1c_xmit_frame()\n");

  struct atl1c_adapter *adapter = &fAdapter;
  unsigned long flags;
  u16 tpd_req = 1;
  struct atl1c_tpd_desc *tpd = NULL;
  enum atl1c_trans_queue type = atl1c_trans_normal;

  if (test_bit(__AT_DOWN, &adapter->flags))
  {
    DbgPrint(1, "TX aborted, IF down!\n");
    dev_kfree_skb_any(skb);
    return NETDEV_TX_OK;
  }

  tpd_req = atl1c_cal_tpd_req(skb);
  if (!spin_trylock_irqsave(&adapter->tx_lock, flags))
  {
    DbgPrint(1, "TX locked!\n");
    return NETDEV_TX_LOCKED;
  }

  if (atl1c_tpd_avail(adapter, type) < tpd_req)
  {
    /* Not enough descriptors, just stop queue. */
    DbgPrint (1, "Not enough resources!\n");
    spin_unlock_irqrestore(&adapter->tx_lock, flags);
    return NETDEV_TX_BUSY;
  }
  tpd = atl1c_get_tpd(adapter, type);

  /* do TSO and check sum */
  if (atl1c_tso_csum(adapter, skb, &tpd, type) != 0)
  {
    spin_unlock_irqrestore(&adapter->tx_lock, flags);
    dev_kfree_skb_any(skb);
    return NETDEV_TX_OK;
  }

  if (unlikely(vlan_tx_tag_present(skb)))
  {
    u16 vlan = vlan_tx_tag_get(skb);
    DbgPrint(5, "TX on VLAN #%d\n", vlan);
    __le16 tag;

    vlan = cpu_to_le16(vlan);
    AT_VLAN_TO_TAG(vlan, tag);
    tpd->word1 |= 1 << TPD_INS_VTAG_SHIFT;
    tpd->vlan_tag = tag;
  }

  if (skb_network_offset(skb) != ETH_HLEN)
		tpd->word1 |= 1 << TPD_ETH_TYPE_SHIFT; /* Ethernet frame */

  if (atl1c_tx_map(adapter, skb, tpd, type) != 0)
  {
    DbgPrint(1, "TX physical address mapping failed!\n");
    spin_unlock_irqrestore(&adapter->tx_lock, flags);
    return NETDEV_TX_BUSY;
  }
  atl1c_tx_queue(adapter, skb, tpd, type);

  spin_unlock_irqrestore(&adapter->tx_lock, flags);

  return NETDEV_TX_OK;
}

//------------------------------------------------------------------------------
