/* at_main.cpp -- ATL1c adapter implementation.
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

#include "at_main.h"

#define ATL1C_DRV_VERSION "1.0.1.0-NAPI"
char atl1c_driver_name[] = "atl1c";
char atl1c_driver_version[] = ATL1C_DRV_VERSION;

#define roundup(x,n) (((x)+((n)-1))&(~((n)-1)))
/*
 * Copyright(c) 2007 Atheros Corporation. All rights reserved.
 *
 * Derived from Intel e1000 driver
 * Copyright(c) 1999 - 2005 Intel Corporation. All rights reserved.
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
 */

static const u16 atl1c_pay_load_size[] = {
  128, 256, 512, 1024, 2048, 4096,
};

static void
atl1c_pcie_patch (struct atl1c_hw *hw)
{
  u32 mst_data, data;

  /* pclk sel could switch to 25M */
  AT_READ_REG (hw, REG_MASTER_CTRL, &mst_data);
  mst_data &= ~MASTER_CTRL_CLK_SEL_DIS;
  AT_WRITE_REG (hw, REG_MASTER_CTRL, mst_data);

  /* WoL/PCIE related settings */
  if (hw->nic_type == athr_l1c || hw->nic_type == athr_l2c)
  {
    AT_READ_REG (hw, REG_PCIE_PHYMISC, &data);
    data |= PCIE_PHYMISC_FORCE_RCV_DET;
    AT_WRITE_REG (hw, REG_PCIE_PHYMISC, data);
  }
  else
  {				/* new dev set bit5 of MASTER */
    if (!(mst_data & MASTER_CTRL_WAKEN_25M))
      AT_WRITE_REG (hw, REG_MASTER_CTRL, mst_data | MASTER_CTRL_WAKEN_25M);
  }
  /* aspm/PCIE setting only for l2cb 1.0 */
  if (hw->nic_type == athr_l2c_b && hw->revision_id == L2CB_V10)
  {
    AT_READ_REG (hw, REG_PCIE_PHYMISC2, &data);
    data = FIELD_SETX (data, PCIE_PHYMISC2_CDR_BW,
                       L2CB1_PCIE_PHYMISC2_CDR_BW);
    data = FIELD_SETX (data, PCIE_PHYMISC2_L0S_TH,
                       L2CB1_PCIE_PHYMISC2_L0S_TH);
    AT_WRITE_REG (hw, REG_PCIE_PHYMISC2, data);
    /* extend L1 sync timer */
    AT_READ_REG (hw, REG_LINK_CTRL, &data);
    data |= LINK_CTRL_EXT_SYNC;
    AT_WRITE_REG (hw, REG_LINK_CTRL, data);
  }
  /* l2cb 1.x & l1d 1.x */
  if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d)
  {
    AT_READ_REG (hw, REG_PM_CTRL, &data);
    data |= PM_CTRL_L0S_BUFSRX_EN;
    AT_WRITE_REG (hw, REG_PM_CTRL, data);
    /* clear vendor msg */
    AT_READ_REG (hw, REG_DMA_DBG, &data);
    AT_WRITE_REG (hw, REG_DMA_DBG, data & ~DMA_DBG_VENDOR_MSG);
  }
}

/* FIXME: no need any more ? */
/*
 * atl1c_init_pcie - init PCIE module
 */
void
atl1c_reset_pcie (struct atl1c_hw *hw, u32 flag)
{
  DbgPrint (4, "atl1c_reset_pcie()\n");

  u32 data;
  u32 pci_cmd;
  IOPCIDevice *pdev = hw->adapter->pdev;
  IOByteCount pos;

  AT_READ_REG (hw, PCI_COMMAND, &pci_cmd);
  pci_cmd &= ~PCI_COMMAND_INTX_DISABLE;
  pci_cmd |= (PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_IO);
  AT_WRITE_REG (hw, PCI_COMMAND, pci_cmd);

  /*
   * Clear any PowerSaving Settings
   */
  // pci_enable_wake(pdev, PCI_D3hot, false);
  // pci_enable_wake(pdev, PCI_D3cold, false);
#define REG_PM_CTRLSTAT             	0x44
  AT_WRITE_REG (hw, REG_PM_CTRLSTAT, kPCIPMCSPMEStatus);
  /* wol sts read-clear */
  AT_READ_REG (hw, REG_WOL_CTRL, &data);
  AT_WRITE_REG (hw, REG_WOL_CTRL, 0);

  /*
   * Mask some pcie error bits
   */
  pos = pci_find_ext_capability (pdev, PCI_EXT_CAP_ID_ERR);
  pci_read_config_dword (pdev, pos + PCI_ERR_UNCOR_SEVER, &data);
  data &= ~(PCI_ERR_UNC_DLP | PCI_ERR_UNC_FCP);
  pci_write_config_dword (pdev, pos + PCI_ERR_UNCOR_SEVER, data);
  /* clear error status */
  pci_write_config_word (pdev, pci_pcie_cap (pdev) + PCI_EXP_DEVSTA,
                         PCI_EXP_DEVSTA_NFED |
                         PCI_EXP_DEVSTA_FED |
                         PCI_EXP_DEVSTA_CED | PCI_EXP_DEVSTA_URD);

  AT_READ_REG (hw, REG_LTSSM_ID_CTRL, &data);
  data &= ~LTSSM_ID_EN_WRO;
  AT_WRITE_REG (hw, REG_LTSSM_ID_CTRL, data);

  atl1c_pcie_patch (hw);
  if (flag & ATL1C_PCIE_L0S_L1_DISABLE)
    atl1c_disable_l0s_l1 (hw);

  msleep (5);
}

/*
 * atl1c_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 */
void
atl1c_irq_enable (atl1c_adapter * adapter)
{
  DbgPrint (4, "atl1c_irq_enable()\n");

  if (likely(atomic_dec_and_test(&adapter->irq_sem)))
  {
    AT_WRITE_REG (&adapter->hw, REG_ISR, 0x7FFFFFFF);
    AT_WRITE_REG (&adapter->hw, REG_IMR, adapter->hw.intr_mask);
    AT_WRITE_FLUSH (&adapter->hw);
  }
}

/*
 * atl1c_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 */
void
atl1c_irq_disable (atl1c_adapter * adapter)
{
  DbgPrint (4, "atl1c_irq_disable()\n");

  atomic_inc(&adapter->irq_sem);
  AT_WRITE_REG (&adapter->hw, REG_IMR, 0);
  AT_WRITE_REG (&adapter->hw, REG_ISR, ISR_DIS_INT);
  AT_WRITE_FLUSH (&adapter->hw);
}

/*
 * atl1c_irq_reset - reset interrupt confiure on the NIC
 * @adapter: board private structure
 */
static void
atl1c_irq_reset (atl1c_adapter * adapter)
{
  atl1c_irq_enable (adapter);
}

/*
 * atl1c_wait_until_idle - wait up to AT_HW_MAX_IDLE_DELAY reads
 * of the idle status register until the device is actually idle
 */
static u32
atl1c_wait_until_idle (struct atl1c_hw *hw, u32 modu_ctrl)
{
  int timeout;
  u32 data;

  for (timeout = 0; timeout < AT_HW_MAX_IDLE_DELAY; timeout++)
  {
    AT_READ_REG (hw, REG_IDLE_STATUS, &data);
    if ((data & modu_ctrl) == 0)
      return 0;
    msleep (1);
  }
  return data;
}

/*
 * atl1c_phy_config - Timer Call-back
 * @data: pointer to netdev cast into an unsigned long
 */
static void
atl1c_phy_config (unsigned long data)
{
	struct atl1c_adapter *adapter = (struct atl1c_adapter *) data;
	struct atl1c_hw *hw = &adapter->hw;
	unsigned long flags;

	spin_lock_irqsave(&adapter->mdio_lock, flags);
	atl1c_restart_autoneg(hw);
	spin_unlock_irqrestore(&adapter->mdio_lock, flags);
}

void
atl1c_set_rxbufsize (struct atl1c_adapter *adapter)
{
  int mtu = adapter->hw.max_frame_size;

  adapter->rx_buffer_len = mtu > AT_RX_BUF_SIZE ?
  roundup (mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN, 8) : AT_RX_BUF_SIZE;
}

/*
 * atl1c_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 */
static int
atl1c_alloc_queues (atl1c_adapter * adapter)
{
  // I get the distinct feeling this Linux code has a lot of
  // extra cleanup work still to be done. :p
  return 0;
}

static void
atl1c_set_mac_type (struct atl1c_hw *hw)
{
  switch (hw->device_id)
  {
    case PCI_DEVICE_ID_ATTANSIC_L2C:
      hw->nic_type = athr_l2c;
      break;
    case PCI_DEVICE_ID_ATTANSIC_L1C:
      hw->nic_type = athr_l1c;
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B:
      hw->nic_type = athr_l2c_b;
      break;
    case PCI_DEVICE_ID_ATHEROS_L2C_B2:
      hw->nic_type = athr_l2c_b2;
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D:
      hw->nic_type = athr_l1d;
      break;
    case PCI_DEVICE_ID_ATHEROS_L1D_2_0:
      hw->nic_type = athr_l1d_2;
      break;
    default:
      break;
  }
}

static int
atl1c_setup_mac_funcs (struct atl1c_hw *hw)
{
  u32 link_ctrl_data;

  atl1c_set_mac_type (hw);
  AT_READ_REG (hw, REG_LINK_CTRL, &link_ctrl_data);

  hw->ctrl_flags = ATL1C_INTR_MODRT_ENABLE | ATL1C_TXQ_MODE_ENHANCE;
  hw->ctrl_flags |= ATL1C_ASPM_L0S_SUPPORT | ATL1C_ASPM_L1_SUPPORT;
  hw->ctrl_flags |= ATL1C_ASPM_CTRL_MON;

  if (hw->nic_type == athr_l1c ||
      hw->nic_type == athr_l1d || hw->nic_type == athr_l1d_2)
    hw->link_cap_flags |= ATL1C_LINK_CAP_1000M;
  return 0;
}

struct atl1c_platform_patch
{
  u16 pci_did;
  u8 pci_revid;
  u16 subsystem_vid;
  u16 subsystem_did;
  u32 patch_flag;
#define ATL1C_LINK_PATCH    0x1
};
static const struct atl1c_platform_patch plats[] = {
  {0x2060, 0xC1, 0x1019, 0x8152, 0x1},
  {0x2060, 0xC1, 0x1019, 0x2060, 0x1},
  {0x2060, 0xC1, 0x1019, 0xE000, 0x1},
  {0x2062, 0xC0, 0x1019, 0x8152, 0x1},
  {0x2062, 0xC0, 0x1019, 0x2062, 0x1},
  {0x2062, 0xC0, 0x1458, 0xE000, 0x1},
  {0x2062, 0xC1, 0x1019, 0x8152, 0x1},
  {0x2062, 0xC1, 0x1019, 0x2062, 0x1},
  {0x2062, 0xC1, 0x1458, 0xE000, 0x1},
  {0x2062, 0xC1, 0x1565, 0x2802, 0x1},
  {0x2062, 0xC1, 0x1565, 0x2801, 0x1},
  {0x1073, 0xC0, 0x1019, 0x8151, 0x1},
  {0x1073, 0xC0, 0x1019, 0x1073, 0x1},
  {0x1073, 0xC0, 0x1458, 0xE000, 0x1},
  {0x1083, 0xC0, 0x1458, 0xE000, 0x1},
  {0x1083, 0xC0, 0x1019, 0x8151, 0x1},
  {0x1083, 0xC0, 0x1019, 0x1083, 0x1},
  {0x1083, 0xC0, 0x1462, 0x7680, 0x1},
  {0x1083, 0xC0, 0x1565, 0x2803, 0x1},
  {0, 0, 0, 0, 0},
};

static void
atl1c_patch_assign (struct atl1c_hw *hw)
{
  IOPCIDevice *pdev = hw->adapter->pdev;
  u32 misc_ctrl;
  int i = 0;

  hw->msi_lnkpatch = false;

  while (plats[i].pci_did != 0)
  {
    if (plats[i].pci_did == hw->device_id &&
        plats[i].pci_revid == hw->revision_id &&
	      plats[i].subsystem_vid == hw->subsystem_vendor_id &&
	      plats[i].subsystem_did == hw->subsystem_id)
    {
      if (plats[i].patch_flag & ATL1C_LINK_PATCH)
        hw->msi_lnkpatch = true;
    }
    i++;
  }

  if (hw->device_id == PCI_DEVICE_ID_ATHEROS_L2C_B2 &&
      hw->revision_id == L2CB_V21)
  {
    /* config acess mode */
    pci_write_config_dword (pdev, REG_PCIE_IND_ACC_ADDR,
                            REG_PCIE_DEV_MISC_CTRL);
    pci_read_config_dword (pdev, REG_PCIE_IND_ACC_DATA, &misc_ctrl);
    misc_ctrl &= ~0x100;
    pci_write_config_dword (pdev, REG_PCIE_IND_ACC_ADDR,
                            REG_PCIE_DEV_MISC_CTRL);
    pci_write_config_dword (pdev, REG_PCIE_IND_ACC_DATA, misc_ctrl);
  }
}

/*
 * atl1c_sw_init - Initialize general software structures (struct atl1c_adapter)
 * @adapter: board private structure to initialize
 *
 * atl1c_sw_init initializes the Adapter private data structure.
 * Fields are initialized based on PCI device information and
 * OS network device settings (MTU size).
 */
int
atl1c_sw_init(struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  IOPCIDevice *pdev = adapter->pdev;

  adapter->wol = 0;
  adapter->link_speed = SPEED_0;
  adapter->link_duplex = FULL_DUPLEX;
  adapter->tpd_ring[0].count = AT_TX_BUFF_COUNT;
  adapter->rfd_ring.count = AT_RX_BUFF_COUNT;

  hw->vendor_id = pdev->configRead16(kIOPCIConfigVendorID);
  hw->device_id = pdev->configRead16(kIOPCIConfigDeviceID);
  hw->subsystem_vendor_id = pdev->configRead16(kIOPCIConfigSubSystemVendorID);
  hw->subsystem_id = pdev->configRead16(kIOPCIConfigSubSystemID);
  hw->revision_id = pdev->configRead8(kIOPCIConfigRevisionID);
  /* Before link up, we assume hibernate is true. */
  hw->hibernate = true;
  hw->media_type = MEDIA_TYPE_AUTO_SENSOR;
  if (atl1c_setup_mac_funcs (hw) != 0)
  {
    ErrPrint ("Set mac function pointers failed!\n");
    return -1;
  }
  atl1c_patch_assign (hw);

  hw->intr_mask = IMR_NORMAL_MASK;
  hw->phy_configured = false;
  hw->preamble_len = 7;
  hw->max_frame_size = ETH_DATA_LEN;
  hw->autoneg_advertised = ADVERTISED_Autoneg;
  hw->indirect_tab = 0xE4E4E4E4;
  hw->base_cpu = 0;

  hw->ict = 50000;		/* 100ms */
  hw->smb_timer = 200000;	/* 400ms */
  hw->rx_imt = 200;
  hw->tx_imt = 1000;

  hw->tpd_burst = 5;
  hw->rfd_burst = 8;
  hw->dma_order = atl1c_dma_ord_out;
  hw->dmar_block = atl1c_dma_req_1024;

  if (atl1c_alloc_queues (adapter))
  {
    ErrPrint ("Unable to allocate memory for queues!\n");
    return -ENOMEM;
  }
  /* TODO: */
  atl1c_set_rxbufsize(adapter);
  atomic_set(&adapter->irq_sem, 1);
  spin_lock_init(&adapter->mdio_lock);
	spin_lock_init(&adapter->tx_lock);
  set_bit(__AT_DOWN, &adapter->flags);

  return 0;
}

/*
 * Read / Write Ptr Initialize:
 */
void
atl1c_init_ring_ptrs (struct atl1c_adapter *adapter)
{
  DbgPrint (4, "atl1c_init_ring_ptrs()\n");

  struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring;
  struct atl1c_buffer *buffer_info;
  int i, j;

  for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++)
  {
    tpd_ring[i].next_to_use = 0;
    atomic_set (&tpd_ring[i].next_to_clean, 0);
    buffer_info = tpd_ring[i].buffer_info;
    for (j = 0; j < tpd_ring->count; j++)
    {
      ATL1C_SET_BUFFER_STATE (&buffer_info[j], ATL1C_BUFFER_FREE);
    }
  }
  rfd_ring->next_to_use = 0;
  rfd_ring->next_to_clean = 0;
  rrd_ring->next_to_use = 0;
  rrd_ring->next_to_clean = 0;
  for (j = 0; j < rfd_ring->count; j++)
  {
    buffer_info = &rfd_ring->buffer_info[j];
    ATL1C_SET_BUFFER_STATE (buffer_info, ATL1C_BUFFER_FREE);
  }
}

/*
 * atl1c_free_ring_resources - Free Tx / RX descriptor Resources
 * @adapter: board private structure
 *
 * Free all transmit software resources
 */
void
atl1c_free_ring_resources (struct atl1c_adapter *adapter)
{

  if (adapter->ring_header.dma)
    adapter->ring_header.dma = NULL;

  if (adapter->ring_header.memDesc)
  {
    adapter->ring_header.memDesc->complete ();
    adapter->ring_header.memDesc->release ();
    adapter->ring_header.memDesc = NULL;
  }
  if (adapter->ring_header.desc)
  {
    adapter->ring_header.desc = NULL;
  }
  /* Note: just free tdp_ring.buffer_info,
   *  it contains rfd_ring.buffer_info, do not double free */
  if (adapter->tpd_ring[0].buffer_info)
  {
    kfree(adapter->tpd_ring[0].buffer_info);
    adapter->tpd_ring[0].buffer_info = NULL;
  }
}

/*
 * atl1c_setup_mem_resources - allocate Tx / RX descriptor resources
 * @adapter: board private structure
 *
 * Return 0 on success, negative on failure
 */
int
atl1c_setup_ring_resources (struct atl1c_adapter *adapter)
{
  struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;
  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring;
  struct atl1c_ring_header *ring_header = &adapter->ring_header;
  int size;
  int i;
  int count = 0;
  int rx_desc_count = 0;
  u32 offset = 0;

  rrd_ring->count = rfd_ring->count;
  for (i = 1; i < AT_MAX_TRANSMIT_QUEUE; i++)
    tpd_ring[i].count = tpd_ring[0].count;

  /* 2 tpd queue, one high priority queue,
   * another normal priority queue */
  size = sizeof (struct atl1c_buffer) * (tpd_ring->count * 2 +
                                         rfd_ring->count);
  tpd_ring->buffer_info = (atl1c_buffer *)kzalloc(size, GFP_KERNEL);
  if (!tpd_ring->buffer_info)
  {
    ErrPrint ("Unable to allocate %d bytes for descriptor TPD ring.", size);
    return -ENOMEM;
  }
  memset (tpd_ring->buffer_info, 0, size);
  DbgPrint(3, "TPD ring allocated, size=%d bytes, tpd_ring->count(x2)=%d, "
           "rfd_ring->count=%d.\n",
           size, tpd_ring->count * 2, rfd_ring->count);

  for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++)
  {
    tpd_ring[i].buffer_info = (struct atl1c_buffer *) (tpd_ring->buffer_info +
                                                       count);
    count += tpd_ring[i].count;
  }

  rfd_ring->buffer_info =
  (struct atl1c_buffer *) (tpd_ring->buffer_info + count);
  count += rfd_ring->count;
  rx_desc_count += rfd_ring->count;

  /*
   * real ring DMA buffer
   * each ring/block may need up to 8 bytes for alignment, hence the
   * additional bytes tacked onto the end.
   */
  ring_header->size = size =
  sizeof (struct atl1c_tpd_desc) * tpd_ring->count * 2 +
  sizeof (struct atl1c_rx_free_desc) * rx_desc_count +
  sizeof (struct atl1c_recv_ret_status) * rx_desc_count +
  8 * 4;

  ring_header->memDesc =
  IOBufferMemoryDescriptor::withOptions((kIODirectionInOut |
                                         kIOMemoryPhysicallyContiguous),
                                        ring_header->size,
                                        AT_BUF_ADDR_MASK);

  if (!ring_header->memDesc
      || (ring_header->memDesc->prepare () != kIOReturnSuccess))
  {
    IOSleep (1500);
    if (ring_header->memDesc)
    {
      ring_header->memDesc->release ();
      ring_header->memDesc = NULL;
    }
    DbgPrint (1, "Cannot allocate memory for descriptor ring header, size=%d\n",
              ring_header->size);
    return -ENOMEM;
  }
  DbgPrint(3, "Allocated memory for descriptor ring header: "
           "size=%d, page_size=%d\n",
           ring_header->size, PAGE_SIZE);
#ifdef __LP64__
  ring_header->dma = ring_header->memDesc->getPhysicalSegment(0, 0);
#else // __ILP32__
  ring_header->dma =
      ring_header->memDesc->getPhysicalSegment(0, 0, kIOMemoryMapperNone);
#endif // __ILP32__
  ring_header->desc = ring_header->memDesc->getBytesNoCopy();
  memset(ring_header->desc, 0, ring_header->size);

  /* init TPD ring */

  tpd_ring[0].dma = roundup(ring_header->dma, 8);
  offset = tpd_ring[0].dma - ring_header->dma;
  for (i = 0; i < AT_MAX_TRANSMIT_QUEUE; i++)
  {
    tpd_ring[i].dma = ring_header->dma + offset;
    tpd_ring[i].desc = (u8 *) ring_header->desc + offset;
    tpd_ring[i].size = sizeof (struct atl1c_tpd_desc) * tpd_ring[i].count;
    offset += roundup (tpd_ring[i].size, 8);
  }
  /* init RFD ring */
  rfd_ring->dma = ring_header->dma + offset;
  rfd_ring->desc = (u8 *) ring_header->desc + offset;
  rfd_ring->size = sizeof (struct atl1c_rx_free_desc) * rfd_ring->count;
  offset += roundup (rfd_ring->size, 8);

  /* init RRD ring */
  rrd_ring->dma = ring_header->dma + offset;
  rrd_ring->desc = (u8 *) ring_header->desc + offset;
  rrd_ring->size = sizeof (struct atl1c_recv_ret_status) * rrd_ring->count;
  offset += roundup (rrd_ring->size, 8);

  return 0;

}

static void
atl1c_configure_des_ring (struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  struct atl1c_rfd_ring *rfd_ring = &adapter->rfd_ring;
  struct atl1c_rrd_ring *rrd_ring = &adapter->rrd_ring;
  struct atl1c_tpd_ring *tpd_ring = (struct atl1c_tpd_ring *)
  adapter->tpd_ring;

  /* TPD */
  AT_WRITE_REG (hw, REG_TX_BASE_ADDR_HI,
                (u32) ((tpd_ring[atl1c_trans_normal].dma &
                        AT_DMA_HI_ADDR_MASK) >> 32));
  /* just enable normal priority TX queue */
  AT_WRITE_REG (hw, REG_TPD_PRI0_ADDR_LO,
                (u32) (tpd_ring[atl1c_trans_normal].dma &
                       AT_DMA_LO_ADDR_MASK));
  AT_WRITE_REG (hw, REG_TPD_PRI1_ADDR_LO,
                (u32) (tpd_ring[atl1c_trans_high].dma & AT_DMA_LO_ADDR_MASK));
  AT_WRITE_REG (hw, REG_TPD_RING_SIZE,
                (u32) (tpd_ring[0].count & TPD_RING_SIZE_MASK));


  /* RFD */
  AT_WRITE_REG (hw, REG_RX_BASE_ADDR_HI,
                (u32) ((rfd_ring->dma & AT_DMA_HI_ADDR_MASK) >> 32));
  AT_WRITE_REG (hw, REG_RFD0_HEAD_ADDR_LO,
                (u32) (rfd_ring->dma & AT_DMA_LO_ADDR_MASK));

  AT_WRITE_REG (hw, REG_RFD_RING_SIZE, rfd_ring->count & RFD_RING_SIZE_MASK);
  AT_WRITE_REG (hw, REG_RX_BUF_SIZE,
                adapter->rx_buffer_len & RX_BUF_SIZE_MASK);

  /* RRD */
  AT_WRITE_REG (hw, REG_RRD0_HEAD_ADDR_LO,
                (u32) (rrd_ring->dma & AT_DMA_LO_ADDR_MASK));
  AT_WRITE_REG (hw, REG_RRD_RING_SIZE,
                (rrd_ring->count & RRD_RING_SIZE_MASK));

  if (hw->nic_type == athr_l2c_b)
  {
    AT_WRITE_REG (hw, REG_SRAM_RXF_LEN, 0x02a0L);
    AT_WRITE_REG (hw, REG_SRAM_TXF_LEN, 0x0100L);
    AT_WRITE_REG (hw, REG_SRAM_RXF_ADDR, 0x029f0000L);
    AT_WRITE_REG (hw, REG_SRAM_RFD0_INFO, 0x02bf02a0L);
    AT_WRITE_REG (hw, REG_SRAM_TXF_ADDR, 0x03bf02c0L);
    AT_WRITE_REG (hw, REG_SRAM_TRD_ADDR, 0x03df03c0L);
    AT_WRITE_REG (hw, REG_TXF_WATER_MARK, 0);	/* TX watermark,
                                               * to enter l1 state. */
    AT_WRITE_REG (hw, REG_RXD_DMA_CTRL, 0);	  /* RXD threshold. */
  }
  /* Load all of base address above */
  AT_WRITE_REG (hw, REG_LOAD_PTR, 1);
}

static void
atl1c_configure_tx (struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  int max_pay_load;
  u16 tx_offload_thresh;
  u32 txq_ctrl_data;

  tx_offload_thresh = MAX_TSO_FRAME_SIZE;
  AT_WRITE_REG (hw, REG_TX_TSO_OFFLOAD_THRESH,
                (tx_offload_thresh >> 3) & TX_TSO_OFFLOAD_THRESH_MASK);
  max_pay_load = pcie_get_readrq(adapter->pdev) >> 8;
  hw->dmar_block =
  (atl1c_dma_req_block) min_t (u32, max_pay_load, hw->dmar_block);
  /*
   * if BIOS had changed the dma-read-max-length to an invalid value,
   * restore it to default value
   */
  if (hw->dmar_block < DEVICE_CTRL_MAXRRS_MIN)
  {
    pcie_set_readrq(adapter->pdev, 128 << DEVICE_CTRL_MAXRRS_MIN);
    hw->dmar_block = (atl1c_dma_req_block) DEVICE_CTRL_MAXRRS_MIN;
  }
  txq_ctrl_data =
  hw->nic_type == athr_l2c_b || hw->nic_type == athr_l2c_b2 ?
  L2CB_TXQ_CFGV : L1C_TXQ_CFGV;

  AT_WRITE_REG (hw, REG_TXQ_CTRL, txq_ctrl_data);
}

static void
atl1c_configure_rx (struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  u32 rxq_ctrl_data;

  rxq_ctrl_data = (hw->rfd_burst & RXQ_RFD_BURST_NUM_MASK) <<
  RXQ_RFD_BURST_NUM_SHIFT;

  if (hw->ctrl_flags & ATL1C_RX_IPV6_CHKSUM)
    rxq_ctrl_data |= IPV6_CHKSUM_CTRL_EN;

  /* aspm for gigabit */
  if (hw->nic_type != athr_l1d_2 && (hw->device_id & 1) != 0)
    rxq_ctrl_data = FIELD_SETX (rxq_ctrl_data, ASPM_THRUPUT_LIMIT,
                                ASPM_THRUPUT_LIMIT_100M);

  AT_WRITE_REG (hw, REG_RXQ_CTRL, rxq_ctrl_data);
}

static void
atl1c_configure_dma (struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  u32 dma_ctrl_data;

  dma_ctrl_data = FIELDX (DMA_CTRL_RORDER_MODE, DMA_CTRL_RORDER_MODE_OUT) |
  DMA_CTRL_RREQ_PRI_DATA |
  FIELDX (DMA_CTRL_RREQ_BLEN, hw->dmar_block) |
  FIELDX (DMA_CTRL_WDLY_CNT, DMA_CTRL_WDLY_CNT_DEF) |
  FIELDX (DMA_CTRL_RDLY_CNT, DMA_CTRL_RDLY_CNT_DEF);

  AT_WRITE_REG (hw, REG_DMA_CTRL, dma_ctrl_data);
}

/*
 * Stop the mac, transmit and receive units
 * hw - Struct containing variables accessed by shared code
 * return : 0  or  idle status (if error)
 */
int
atl1c_stop_mac (struct atl1c_hw *hw)
{
  u32 data;

  AT_READ_REG (hw, REG_RXQ_CTRL, &data);
  data &= ~RXQ_CTRL_EN;
  AT_WRITE_REG (hw, REG_RXQ_CTRL, data);

  AT_READ_REG (hw, REG_TXQ_CTRL, &data);
  data &= ~TXQ_CTRL_EN;
  AT_WRITE_REG (hw, REG_TXQ_CTRL, data);

  atl1c_wait_until_idle (hw, IDLE_STATUS_RXQ_BUSY | IDLE_STATUS_TXQ_BUSY);

  AT_READ_REG (hw, REG_MAC_CTRL, &data);
  data &= ~(MAC_CTRL_TX_EN | MAC_CTRL_RX_EN);
  AT_WRITE_REG (hw, REG_MAC_CTRL, data);

  return (int) atl1c_wait_until_idle (hw,
                                      IDLE_STATUS_TXMAC_BUSY |
                                      IDLE_STATUS_RXMAC_BUSY);
}

void
atl1c_start_mac (struct atl1c_adapter *adapter)
{
  struct atl1c_hw *hw = &adapter->hw;
  u32 mac, txq, rxq;

  hw->mac_duplex = adapter->link_duplex == FULL_DUPLEX ? true : false;
  hw->mac_speed = adapter->link_speed == SPEED_1000 ?
  atl1c_mac_speed_1000 : atl1c_mac_speed_10_100;

  AT_READ_REG (hw, REG_TXQ_CTRL, &txq);
  AT_READ_REG (hw, REG_RXQ_CTRL, &rxq);
  AT_READ_REG (hw, REG_MAC_CTRL, &mac);

  txq |= TXQ_CTRL_EN;
  rxq |= RXQ_CTRL_EN;
  mac |= MAC_CTRL_TX_EN | MAC_CTRL_TX_FLOW |
  MAC_CTRL_RX_EN | MAC_CTRL_RX_FLOW |
  MAC_CTRL_ADD_CRC | MAC_CTRL_PAD |
  MAC_CTRL_BC_EN | MAC_CTRL_SINGLE_PAUSE_EN | MAC_CTRL_HASH_ALG_CRC32;
  // FIXME: This has been added to fix Bonjour maybe.
  if (hw->nic_type == athr_l1d    ||
      hw->nic_type == athr_l2c_b2 ||
      hw->nic_type == athr_l1d_2)
  {
		mac |= (MAC_CTRL_SPEED_MODE_SW | MAC_CTRL_HASH_ALG_CRC32);
	}
  else
  {
    mac &= ~(MAC_CTRL_SPEED_MODE_SW | MAC_CTRL_HASH_ALG_CRC32);
  }
  
  if (hw->mac_duplex)
    mac |= MAC_CTRL_DUPLX;
  else
    mac &= ~MAC_CTRL_DUPLX;
  mac = FIELD_SETX (mac, MAC_CTRL_SPEED, hw->mac_speed);
  mac = FIELD_SETX (mac, MAC_CTRL_PRMLEN, hw->preamble_len);

  AT_WRITE_REG (hw, REG_TXQ_CTRL, txq);
  AT_WRITE_REG (hw, REG_RXQ_CTRL, rxq);
  AT_WRITE_REG (hw, REG_MAC_CTRL, mac);
}

/*
 * Reset the transmit and receive units; mask and clear all interrupts.
 * hw - Struct containing variables accessed by shared code
 * return : 0  or  idle status (if error)
 */
int
atl1c_reset_mac (struct atl1c_hw *hw)
{
  DbgPrint (4, "atl1c_reset_mac()\n");

  u32 ctrl_data = 0;

  atl1c_stop_mac (hw);
  /*
   * Issue Soft Reset to the MAC.  This will reset the chip's
   * transmit, receive, DMA.  It will not effect
   * the current PCI configuration.  The global reset bit is self-
   * clearing, and should clear within a microsecond.
   */
  AT_READ_REG (hw, REG_MASTER_CTRL, &ctrl_data);
  ctrl_data |= MASTER_CTRL_OOB_DIS;
  AT_WRITE_REG (hw, REG_MASTER_CTRL, ctrl_data | MASTER_CTRL_SOFT_RST);

  AT_WRITE_FLUSH (hw);
  msleep (10);
  /* Wait at least 10ms for All module to be Idle */

  if (atl1c_wait_until_idle (hw, IDLE_STATUS_MASK))
  {
    ErrPrint ("MAC state machine can't be idle since"
              " disabled for 10ms!\n");
    return -1;
  }
  AT_WRITE_REG (hw, REG_MASTER_CTRL, ctrl_data);

  /* driver control speed/duplex */
  AT_READ_REG (hw, REG_MAC_CTRL, &ctrl_data);
  AT_WRITE_REG (hw, REG_MAC_CTRL, ctrl_data | MAC_CTRL_SPEED_MODE_SW);

  /* clk switch setting */
  AT_READ_REG (hw, REG_SERDES, &ctrl_data);
  switch (hw->nic_type)
  {
    case athr_l2c_b:
      ctrl_data &= ~(SERDES_PHY_CLK_SLOWDOWN | SERDES_MAC_CLK_SLOWDOWN);
      AT_WRITE_REG (hw, REG_SERDES, ctrl_data);
      break;
    case athr_l2c_b2:
    case athr_l1d_2:
      ctrl_data |= SERDES_PHY_CLK_SLOWDOWN | SERDES_MAC_CLK_SLOWDOWN;
      AT_WRITE_REG (hw, REG_SERDES, ctrl_data);
      break;
    default:
      break;
  }

  return 0;
}

void
atl1c_disable_l0s_l1 (struct atl1c_hw *hw)
{
  DbgPrint (4, "atl1c_disable_l0s_l1()\n");

  u16 ctrl_flags = hw->ctrl_flags;

  hw->ctrl_flags &= ~(ATL1C_ASPM_L0S_SUPPORT | ATL1C_ASPM_L1_SUPPORT);
  atl1c_set_aspm (hw, SPEED_0);
  hw->ctrl_flags = ctrl_flags;
}

/*
 * Set ASPM state.
 * Enable/disable L0s/L1 depend on link state.
 */
void
atl1c_set_aspm (struct atl1c_hw *hw, u16 link_speed)
{
  u32 pm_ctrl_data;
  u32 link_l1_timer;

  AT_READ_REG (hw, REG_PM_CTRL, &pm_ctrl_data);
  pm_ctrl_data &= ~(PM_CTRL_ASPM_L1_EN |
                    PM_CTRL_ASPM_L0S_EN | PM_CTRL_MAC_ASPM_CHK);
  /* L1 timer */
  if (hw->nic_type == athr_l2c_b2 || hw->nic_type == athr_l1d_2)
  {
    pm_ctrl_data &= ~PMCTRL_TXL1_AFTER_L0S;
    link_l1_timer =
    link_speed == SPEED_1000 || link_speed == SPEED_100 ?
    L1D_PMCTRL_L1_ENTRY_TM_16US : 1;
    pm_ctrl_data = FIELD_SETX (pm_ctrl_data,
                               L1D_PMCTRL_L1_ENTRY_TM, link_l1_timer);
  }
  else
  {
    link_l1_timer = hw->nic_type == athr_l2c_b ?
    L2CB1_PM_CTRL_L1_ENTRY_TM : L1C_PM_CTRL_L1_ENTRY_TM;
    if (link_speed != SPEED_1000 && link_speed != SPEED_100)
      link_l1_timer = 1;
    pm_ctrl_data = FIELD_SETX (pm_ctrl_data,
                               PM_CTRL_L1_ENTRY_TIMER, link_l1_timer);
  }

  /* L0S/L1 enable */
  if ((hw->ctrl_flags & ATL1C_ASPM_L0S_SUPPORT) && link_speed != SPEED_0)
    pm_ctrl_data |= PM_CTRL_ASPM_L0S_EN | PM_CTRL_MAC_ASPM_CHK;
  if (hw->ctrl_flags & ATL1C_ASPM_L1_SUPPORT)
    pm_ctrl_data |= PM_CTRL_ASPM_L1_EN | PM_CTRL_MAC_ASPM_CHK;

  /* l2cb & l1d & l2cb2 & l1d2 */
  if (hw->nic_type == athr_l2c_b || hw->nic_type == athr_l1d ||
      hw->nic_type == athr_l2c_b2 || hw->nic_type == athr_l1d_2)
  {
    pm_ctrl_data = FIELD_SETX (pm_ctrl_data,
                               PM_CTRL_PM_REQ_TIMER, PM_CTRL_PM_REQ_TO_DEF);
    pm_ctrl_data |= PM_CTRL_RCVR_WT_TIMER |
    PM_CTRL_SERDES_PD_EX_L1 | PM_CTRL_CLK_SWH_L1;
    pm_ctrl_data &= ~(PM_CTRL_SERDES_L1_EN |
                      PM_CTRL_SERDES_PLL_L1_EN |
                      PM_CTRL_SERDES_BUFS_RX_L1_EN |
                      PM_CTRL_SA_DLY_EN | PM_CTRL_HOTRST);
    /* disable l0s if link down or l2cb */
    if (link_speed == SPEED_0 || hw->nic_type == athr_l2c_b)
      pm_ctrl_data &= ~PM_CTRL_ASPM_L0S_EN;
  }
  else
  {				/* l1c */
    pm_ctrl_data = FIELD_SETX (pm_ctrl_data, PM_CTRL_L1_ENTRY_TIMER, 0);
    if (link_speed != SPEED_0)
    {
      pm_ctrl_data |= PM_CTRL_SERDES_L1_EN |
	    PM_CTRL_SERDES_PLL_L1_EN | PM_CTRL_SERDES_BUFS_RX_L1_EN;
      pm_ctrl_data &= ~(PM_CTRL_SERDES_PD_EX_L1 |
                        PM_CTRL_CLK_SWH_L1 |
                        PM_CTRL_ASPM_L0S_EN | PM_CTRL_ASPM_L1_EN);
    }
    else
    {			/* link down */
      pm_ctrl_data |= PM_CTRL_CLK_SWH_L1;
      pm_ctrl_data &= ~(PM_CTRL_SERDES_L1_EN |
                        PM_CTRL_SERDES_PLL_L1_EN |
                        PM_CTRL_SERDES_BUFS_RX_L1_EN |
                        PM_CTRL_ASPM_L0S_EN);
    }
  }
  AT_WRITE_REG (hw, REG_PM_CTRL, pm_ctrl_data);

  return;
}

/*
 * atl1c_configure - Configure Transmit&Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx /Rx unit of the MAC after a reset.
 */

int
atl1c_configure_mac(struct atl1c_adapter *adapter)
{
  DbgPrint (4, "atl1c_configure_mac()\n");

	struct atl1c_hw *hw = &adapter->hw;
	u32 master_ctrl_data = 0;
	u32 intr_modrt_data;
	u32 data;

	AT_READ_REG(hw, REG_MASTER_CTRL, &master_ctrl_data);
	master_ctrl_data &= ~(MASTER_CTRL_TX_ITIMER_EN |
                        MASTER_CTRL_RX_ITIMER_EN |
                        MASTER_CTRL_INT_RDCLR);
	/* clear interrupt status */
	AT_WRITE_REG(hw, REG_ISR, 0xFFFFFFFF);
	/*  Clear any WOL status */
	AT_WRITE_REG(hw, REG_WOL_CTRL, 0);
	/* set Interrupt Clear Timer
	 * HW will enable self to assert interrupt event to system after
	 * waiting x-time for software to notify it accept interrupt.
	 */

	data = CLK_GATING_EN_ALL;
	if (hw->ctrl_flags & ATL1C_CLK_GATING_EN) {
		if (hw->nic_type == athr_l2c_b)
			data &= ~CLK_GATING_RXMAC_EN;
	} else
		data = 0;
	AT_WRITE_REG(hw, REG_CLK_GATING_CTRL, data);

	AT_WRITE_REG(hw, REG_INT_RETRIG_TIMER,
               hw->ict & INT_RETRIG_TIMER_MASK);

	atl1c_configure_des_ring(adapter);

	if (hw->ctrl_flags & ATL1C_INTR_MODRT_ENABLE) {
		intr_modrt_data = (hw->tx_imt & IRQ_MODRT_TIMER_MASK) <<
    IRQ_MODRT_TX_TIMER_SHIFT;
		intr_modrt_data |= (hw->rx_imt & IRQ_MODRT_TIMER_MASK) <<
    IRQ_MODRT_RX_TIMER_SHIFT;
		AT_WRITE_REG(hw, REG_IRQ_MODRT_TIMER_INIT, intr_modrt_data);
		master_ctrl_data |=
    MASTER_CTRL_TX_ITIMER_EN | MASTER_CTRL_RX_ITIMER_EN;
	}

	if (hw->ctrl_flags & ATL1C_INTR_CLEAR_ON_READ)
		master_ctrl_data |= MASTER_CTRL_INT_RDCLR;

	master_ctrl_data |= MASTER_CTRL_SA_TIMER_EN;
	AT_WRITE_REG(hw, REG_MASTER_CTRL, master_ctrl_data);

	AT_WRITE_REG(hw, REG_SMB_STAT_TIMER,
               hw->smb_timer & SMB_STAT_TIMER_MASK);

	/* set MTU */
	AT_WRITE_REG(hw, REG_MTU, hw->max_frame_size + ETH_HLEN +
               VLAN_HLEN + ETH_FCS_LEN);

	atl1c_configure_tx(adapter);
	atl1c_configure_rx(adapter);
	atl1c_configure_dma(adapter);

	return 0;
}

static void
atl1c_update_hw_stats (struct atl1c_adapter *adapter)
{
  u16 hw_reg_addr = 0;
  unsigned long *stats_item = NULL;
  u32 data;

  /* update rx status */
  hw_reg_addr = REG_MAC_RX_STATUS_BIN;
  stats_item = &adapter->hw_stats.rx_ok;
  while (hw_reg_addr <= REG_MAC_RX_STATUS_END)
  {
    AT_READ_REG (&adapter->hw, hw_reg_addr, &data);
    *stats_item += data;
    stats_item++;
    hw_reg_addr += 4;
  }
  /* update tx status */
  hw_reg_addr = REG_MAC_TX_STATUS_BIN;
  stats_item = &adapter->hw_stats.tx_ok;
  while (hw_reg_addr <= REG_MAC_TX_STATUS_END)
  {
    AT_READ_REG (&adapter->hw, hw_reg_addr, &data);
    *stats_item += data;
    stats_item++;
    hw_reg_addr += 4;
  }
}

void
atl1c_clear_phy_int (atl1c_adapter * adapter)
{
  u16 phy_data;

  IOSimpleLockLock(adapter->mdio_lock);
  atl1c_read_phy_reg (&adapter->hw, MII_ISR, &phy_data);
  IOSimpleLockUnlock(adapter->mdio_lock);
}

void
atl1c_clean_rrd (struct atl1c_rrd_ring *rrd_ring,
                 struct atl1c_recv_ret_status *rrs, u16 num)
{
  u16 i;
  /* the relationship between rrd and rfd is one map one */
  for (i = 0; i < num; i++, rrs = ATL1C_RRD_DESC (rrd_ring,
                                                  rrd_ring->next_to_clean))
  {
    rrs->word3 &= ~RRS_RXD_UPDATED;
    if (++rrd_ring->next_to_clean == rrd_ring->count)
      rrd_ring->next_to_clean = 0;
  }
  return;
}

void
atl1c_clean_rfd (struct atl1c_rfd_ring *rfd_ring,
                 struct atl1c_recv_ret_status *rrs, u16 num)
{
  u16 i;
	u16 rfd_index;
	struct atl1c_buffer *buffer_info = rfd_ring->buffer_info;

	rfd_index = (rrs->word0 >> RRS_RX_RFD_INDEX_SHIFT) & RRS_RX_RFD_INDEX_MASK;
	for (i = 0; i < num; i++)
  {
    buffer_info[rfd_index].skb = NULL;
    ATL1C_SET_BUFFER_STATE(&buffer_info[rfd_index], ATL1C_BUFFER_FREE);
    if (++rfd_index == rfd_ring->count)
      rfd_index = 0;
  }
	rfd_ring->next_to_clean = rfd_index;
}

u16
atl1c_tpd_avail (struct atl1c_adapter * adapter, enum atl1c_trans_queue type)
{
  struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
  u16 next_to_use = 0;
  u16 next_to_clean = 0;

  next_to_clean = atomic_read (&tpd_ring->next_to_clean);
  next_to_use = tpd_ring->next_to_use;

  return (u16) (next_to_clean > next_to_use) ?
  (next_to_clean - next_to_use - 1) :
  (tpd_ring->count + next_to_clean - next_to_use - 1);
}

/*
 * get next usable tpd
 * Note: should call atl1c_tdp_avail to make sure
 * there is enough tpd to use
 */
struct atl1c_tpd_desc *
atl1c_get_tpd (struct atl1c_adapter *adapter, enum atl1c_trans_queue type)
{
  struct atl1c_tpd_ring *tpd_ring = &adapter->tpd_ring[type];
  struct atl1c_tpd_desc *tpd_desc;
  u16 next_to_use = 0;

  next_to_use = tpd_ring->next_to_use;
  if (++tpd_ring->next_to_use == tpd_ring->count)
    tpd_ring->next_to_use = 0;
  tpd_desc = ATL1C_TPD_DESC (tpd_ring, next_to_use);
  memset (tpd_desc, 0, sizeof (struct atl1c_tpd_desc));
  return tpd_desc;
}

struct atl1c_buffer *
atl1c_get_tx_buffer (struct atl1c_adapter *adapter,
                     struct atl1c_tpd_desc *tpd)
{
  struct atl1c_tpd_ring *tpd_ring = adapter->tpd_ring;

  DbgPrint (5, "atl1c_get_tx_buffer() buffer_info[%d]\n",
            (int)(tpd - (struct atl1c_tpd_desc *) tpd_ring->desc));
  
  return &tpd_ring->buffer_info[tpd -
                                (struct atl1c_tpd_desc *) tpd_ring->desc];
}

/* Calculate the transmit packet descript needed. */
u16 atl1c_cal_tpd_req(struct sk_buff *skb)
{
  struct sk_buff *next_skb = skb;
	u16 tpd_req = 0;
	u16 proto_hdr_len = 0;
  
  while (next_skb != NULL)
  {
    tpd_req++;
    next_skb = mbuf_next(next_skb);
  }

	if (skb_is_gso(skb))
  {
		proto_hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		if (proto_hdr_len < skb_headlen(skb))
			tpd_req++;
		if (gso_type(skb) & SKB_GSO_TCPV6)
			tpd_req++;
	}

	return tpd_req;
}
