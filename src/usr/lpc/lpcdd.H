/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/lpc/lpcdd.H $                                         */
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
#ifndef __LPC_LPCDD_H
#define __LPC_LPCDD_H

#include <limits.h>
#include <sys/sync.h>
#include <stdint.h>
#include <errl/errlentry.H>
#include <lpc/lpcif.H>

/** @file lpcdd.H
 *  @brief Provides the interfaces to the LPC Device Driver
 */

/**
 *  @brief  LPC Device Driver Class
 *     Provides access to the LPC bus
 */
class LpcDD
{
  public:
    /**
     * @brief Performs a LPC Read Operation
     *
     * @parm i_trans    LPC transaction type
     * @parm i_address  LPC address
     * @parm o_buffer   Buffer to read data into
     * @parm io_buflen  Input: Number of bytes to read,
     *       Output: Number of bytes actually read
     *
     * @return Error from operation
     */
    errlHndl_t readLPC(LPC::TransType i_type,
                       uint32_t i_address,
                       void* o_buffer,
                       size_t& io_buflen);

    /**
     * @brief Performs a LPC Write Operation
     *
     * @parm i_trans    LPC transaction type
     * @parm i_address  LPC address
     * @parm i_buffer  Buffer to write data from
     * @parm io_buflen  Input: Number of bytes to write,
     *       Output: Number of bytes actually written
     *
     * @return Error from operation
     */
    errlHndl_t writeLPC(LPC::TransType i_type,
                        uint32_t i_address,
                        const void* i_buffer,
                        size_t& io_buflen);

    /**
     * @brief Constructor
     */
    LpcDD();


    /**
     * @brief Destructor
     */
    ~LpcDD();

  protected:
    /**
     * @brief LPC HC Registers
     *    These are offsets within the LPC Host Controller Register Space
     */
    enum LpcRegAddr {
        LPC_REG_BAR0 = 0x00, /**< BAR0 : OPB register */
        LPC_REG_BAR1 = 0x04, /**< BAR1 : LPC I/O space */
        LPC_REG_BAR2 = 0x08, /**< BAR2 : LPC Memory space */
        LPC_REG_BAR3 = 0x0C, /**< BAR3 : LPC Firmware space */
    };


    /**
     * @brief Some general constants
     *
     */
    enum {
        LPCHC_FW_SPACE  = 0xF0000000, /**< LPC Host Controller FW Space */
        LPCHC_MEM_SPACE = 0xE0000000, /**< LPC Host Controller Mem Space */
        LPCHC_IO_SPACE  = 0xD0010000, /**< LPC Host Controller I/O Space */
        LPCHC_REG_SPACE = 0xC0012000, /**< LPC Host Ctlr Register Space */

        ECCB_NON_FW_RESET_REG = 0x000B0001, /**< ECCB Reset Reg (non-FW) */
        ECCB_CTL_REG  = 0x000B0020, /**< ECCB Control Reg (FW) */
        ECCB_STAT_REG = 0x000B0022, /**< ECCB Status Reg (FW) */
        ECCB_DATA_REG = 0x000B0023, /**< ECCB Data Reg (FW) */

        // Default Values to set for all operations
        // 1101.xxxx.0000.000x.0000.00xx.x000.0000.<address>
        LPC_CTL_REG_DEFAULT = 0xD000000000000000,

        LPC_STAT_REG_ERROR_MASK = 0xFC0000000007F700, /**< Error Bits */

        ECCB_RESET_LPC_FAST_RESET = 1ULL << 62,

        ECCB_POLL_TIME_NS = 400000, /**< max time from Manfred Walz is 400ms */
        ECCB_POLL_INCR_NS = 10, /**< minimum increment during poll */

        LPCHC_SYNC_CYCLE_COUNTER_REG = 0xC001202C,
        LPCHC_SYNC_CYCLE_COUNTER_INFINITE = 0xFF000000
    };

    /**
     * @brief  ECCB Control Register Layout
     */
    union ControlReg_t
    {
        uint64_t data64;
        struct
        {
            // unused sections should be set to zero
            uint64_t magic1      : 4;  /**< 0:3 = b1101 per spec */
            uint64_t data_len    : 4;  /**< 4:7 = b0100 means 4 byte */
            uint64_t unused1     : 7;  /**< 8:14 */
            uint64_t read_op     : 1;  /**< 15 = set for read operation */
            uint64_t unused2     : 7;  /**< 16:22 */
            uint64_t addr_len    : 3;  /**< 23:25 = b100 means 4 byte */
            uint64_t unused3     : 6;  /**< 26:31 */
            uint64_t address     : 32; /**< 32:63 = LPC Address */
        };

        ControlReg_t() : data64(LPC_CTL_REG_DEFAULT) {};
    };

    /**
     * @brief  ECCB Status Register Layout
     */
    union StatusReg_t
    {
        uint64_t data64;
        struct
        {
            uint64_t pib_errors  : 6;  /**< 0:5  */
            uint64_t read_data   : 32; /**< 6:37 */
            uint64_t unused1     : 6;  /**< 38:43 */
            uint64_t busy        : 1;  /**< 44 = Operation Busy */
            uint64_t errors1     : 7;  /**< 45:51 */
            uint64_t op_done     : 1;  /**< 52 */
            uint64_t errors2     : 3;  /**< 53:55 */
            uint64_t unused2     : 8;  /**< 56:63 */
        };
        StatusReg_t() : data64(0) {};
    };

    errlHndl_t checkAddr(LPC::TransType i_type, uint32_t i_addr,
                         uint32_t *o_addr);

    errlHndl_t readReg(uint32_t reg, uint64_t *val);
    errlHndl_t writeReg(uint32_t reg, uint64_t val);

    errlHndl_t pollComplete(const ControlReg_t &i_ctrl,
                            StatusReg_t *o_stat);

    // brs make global so pnordd can see it. Not proud, this is a travesty.
#if 0
  private:
    /**
     * @brief Mutex to prevent concurrent LPC accesses
     */
    static mutex_t cv_mutex;
#endif
};


#endif
