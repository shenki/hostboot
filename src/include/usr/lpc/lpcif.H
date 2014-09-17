/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/include/usr/lpc/lpcif.H $                                 */
/*                                                                        */
/* OpenPOWER HostBoot Project                                             */
/*                                                                        */
/* Contributors Listed Below - COPYRIGHT 2014                             */
/* [+] <joel@jms.id.au                                                    */
/*                                                                        */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/*                                                                        */
/* IBM_PROLOG_END_TAG                                                     */
#ifndef __LPCIF_H
#define __LPCIF_H

#include <errl/errlentry.H>

//LPC slave is AST2400 instead of DPSS
#define LPC_SLAVE_AST2400


namespace LPC
{

enum TransType {
  TRANS_IO,
  TRANS_MEM,
  TRANS_FW,
  TRANS_REG  // LPCHC Register space
};

// LPC I/O space (64K) access functions.
errlHndl_t inb(uint32_t i_addr, uint8_t *o_data);
errlHndl_t inw(uint32_t i_addr, uint16_t *o_data);
errlHndl_t inl(uint32_t i_addr, uint32_t *o_data);
errlHndl_t outb(uint32_t i_addr, uint8_t i_data);
errlHndl_t outw(uint32_t i_addr, uint16_t i_data);
errlHndl_t outl(uint32_t i_addr, uint32_t i_data);

// LPC memory space access functions.
errlHndl_t readb(uint32_t i_addr, uint8_t *o_data);
errlHndl_t readw(uint32_t i_addr, uint16_t *o_data);
errlHndl_t readl(uint32_t i_addr, uint32_t *o_data);
errlHndl_t writeb(uint32_t i_addr, uint8_t i_data);
errlHndl_t writew(uint32_t i_addr, uint16_t i_data);
errlHndl_t writel(uint32_t i_addr, uint32_t i_data);

// LPC FWH space access functions.
errlHndl_t fw_readb(uint32_t i_addr, uint8_t *o_data);
errlHndl_t fw_readw(uint32_t i_addr, uint16_t *o_data);
errlHndl_t fw_readl(uint32_t i_addr, uint32_t *o_data);
errlHndl_t fw_writeb(uint32_t i_addr, uint8_t i_data);
errlHndl_t fw_writew(uint32_t i_addr, uint16_t i_data);
errlHndl_t fw_writel(uint32_t i_addr, uint32_t i_data);

};  // namespace LPC

#endif  // end __LPCIF_H
