/* at_main.h -- ATL1c adapter definitions
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
 * This driver is heavily based on Atheros ATL1c Linux driver.
 */

#ifndef __AT_MAIN_H__
#define __AT_MAIN_H__

#include "at.h"

void atl1c_disable_l0s_l1 (struct atl1c_hw * hw);
int atl1c_setup_ring_resources (struct atl1c_adapter * adapter);
void atl1c_free_ring_resources (struct atl1c_adapter * adapter);
void atl1c_clear_phy_int (struct atl1c_adapter * adapter);

void atl1c_clean_rrd (struct atl1c_rrd_ring * rrd_ring,
                      struct atl1c_recv_ret_status * rrs, u16 num);
void atl1c_clean_rfd (struct atl1c_rfd_ring * rfd_ring,
                      struct atl1c_recv_ret_status * rrs, u16 num);
void atl1c_init_ring_ptrs (struct atl1c_adapter * adapter);
int atl1c_configure_mac(struct atl1c_adapter *adapter);
void atl1c_irq_enable (struct atl1c_adapter * adapter);
void atl1c_irq_disable (struct atl1c_adapter * adapter);
int atl1c_reset_mac (struct atl1c_hw * hw);
void atl1c_reset_dma_ring (struct atl1c_adapter *adapter);
int atl1c_sw_init (struct atl1c_adapter * adapter);
void atl1c_reset_pcie (struct atl1c_hw *hw, u32 flag);
int atl1c_stop_mac (struct atl1c_hw * hw);
void atl1c_set_aspm (struct atl1c_hw *hw, u16 link_speed);
void atl1c_start_mac (struct atl1c_adapter *adapter);
struct atl1c_tpd_desc *atl1c_get_tpd (struct atl1c_adapter * adapter,
                                      enum atl1c_trans_queue type);
struct atl1c_buffer *atl1c_get_tx_buffer (struct atl1c_adapter * adapter,
                                          struct atl1c_tpd_desc * tpd);
u16 atl1c_tpd_avail (struct atl1c_adapter * adapter,
                     enum atl1c_trans_queue type);
u16 atl1c_cal_tpd_req(struct sk_buff *skb);
void atl1c_set_rxbufsize (struct atl1c_adapter *adapter);

#endif //__AT_MAIN_H__
