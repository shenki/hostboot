/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/lpc/lpcdd.C $                                         */
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
/**
 *  @file lpcdd.C
 *
 *  @brief Implementation of the LPC Device Driver
 */

#include <sys/mmio.h>
#include <sys/task.h>
#include <sys/sync.h>
#include <string.h>
#include <devicefw/driverif.H>
#include <trace/interface.H>
#include <errl/errlentry.H>
#include <targeting/common/targetservice.H>
#include <errl/errlmanager.H>
#include "lpcdd.H"
#include <sys/time.h>
#include <lpc/lpc_reasoncodes.H>
#include <initservice/initserviceif.H>
#include <kernel/console.H>

// Set to enable LPC tracing.
// brs - beware this traces in functions that are called by trace
// so it deadlocks when you're CONSOLE::printf tracing.
#define LPC_TRACING 0
#if LPC_TRACING
trace_desc_t* g_trac_lpc;
TRAC_INIT( & g_trac_lpc, "LPC", 2*KILOBYTE, TRACE::BUFFER_SLOW);

#define LPC_TRACDCOMP(des,printf_string,args...) \
    TRACDCOMP(des,printf_string,##args)
#define LPC_TRACFCOMP(des,printf_string,args...) \
    TRACFCOMP(des,printf_string,##args)
#else
#define LPC_TRACDCOMP(des,printf_string,args...) do {} while(0)
#define LPC_TRACFCOMP(des,printf_string,args...) do {} while(0)
#endif

namespace LPC
{
/**
 * @brief Performs an LPC Read Operation
 *
 * @param[in]   i_opType        Operation type, see DeviceFW::OperationType
 *                              in driverif.H
 * @param[in]   i_target        LPC target
 * @param[in/out] io_buffer     Read: Pointer to output data storage
 *                              Write: Pointer to input data storage
 * @param[in/out] io_buflen     Input: size of io_buffer (in bytes)
 *                              Output:
 *                                  Read: Size of output data
 *                                  Write: Size of data written
 * @param[in]   i_accessType    DeviceFW::AccessType enum (usrif.H)
 * @param[in]   i_args          This is an argument list for DD framework.
 * @return  errlHndl_t
 */
errlHndl_t lpcRead(DeviceFW::OperationType i_opType,
                   TARGETING::Target* i_target,
                   void* io_buffer,
                   size_t& io_buflen,
                   int64_t i_accessType, va_list i_args)
{
    LPC::TransType l_type = static_cast<LPC::TransType>(
        va_arg(i_args,uint64_t) );
    uint64_t l_addr = va_arg(i_args,uint64_t);

    assert( io_buflen == sizeof(uint8_t) ||
        io_buflen == sizeof(uint16_t) ||
        io_buflen == sizeof(uint32_t) );

    return Singleton<LpcDD>::instance().readLPC( l_type,
                                                 l_addr,
                                                 io_buffer, io_buflen );
}

/**
 * @brief Performs an PNOR Write Operation
 * This function performs a PNOR Write operation. It follows a pre-defined
 * prototype functions in order to be registered with the device-driver
 * framework.
 *
 * @param[in]   i_opType        Operation type, see DeviceFW::OperationType
 *                              in driverif.H
 * @param[in]   i_target        LPC target
 * @param[in/out] io_buffer     Read: Pointer to output data storage
 *                              Write: Pointer to input data storage
 * @param[in/out] io_buflen     Input: size of io_buffer (in bytes)
 *                              Output:
 *                                  Read: Size of output data
 *                                  Write: Size of data written
 * @param[in]   i_accessType    DeviceFW::AccessType enum (usrif.H)
 * @param[in]   i_args          This is an argument list for DD framework.
 * @return  errlHndl_t
 */
errlHndl_t lpcWrite(DeviceFW::OperationType i_opType,
                    TARGETING::Target* i_target,
                    void* io_buffer,
                    size_t& io_buflen,
                    int64_t i_accessType, va_list i_args)
{
    LPC::TransType l_type = static_cast<LPC::TransType>(
        va_arg(i_args,uint64_t) );
    uint64_t l_addr = va_arg(i_args,uint64_t);

    assert( io_buflen == sizeof(uint8_t) ||
        io_buflen == sizeof(uint16_t) ||
        io_buflen == sizeof(uint32_t) );

    return Singleton<LpcDD>::instance().writeLPC( l_type,
                                                  l_addr,
                                                  io_buffer, io_buflen );
}

// Register LPC access functions to DD framework
DEVICE_REGISTER_ROUTE( DeviceFW::READ,
                       DeviceFW::LPC,
                       TARGETING::TYPE_PROC,
                       lpcRead );
DEVICE_REGISTER_ROUTE( DeviceFW::WRITE,
                       DeviceFW::LPC,
                       TARGETING::TYPE_PROC,
                       lpcWrite );

template<typename T>
static errlHndl_t lpc_in(LPC::TransType i_trans_type, uint32_t i_addr,
                         T *o_data)
{
    size_t len = sizeof(*o_data);
    return deviceOp( DeviceFW::READ,
                     TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                     o_data,
                     len,
                     DEVICE_LPC_ADDRESS(i_trans_type, i_addr));
                 }

errlHndl_t inb(uint32_t i_addr, uint8_t *o_data)
{
    return lpc_in( LPC::TRANS_IO, i_addr, o_data );
}
errlHndl_t inw(uint32_t i_addr, uint16_t *o_data)
{
    return lpc_in( LPC::TRANS_IO, i_addr, o_data );
}
errlHndl_t inl(uint32_t i_addr, uint32_t *o_data)
{
    return lpc_in( LPC::TRANS_IO, i_addr, o_data );
}

errlHndl_t readb(uint32_t i_addr, uint8_t *o_data)
{
    return lpc_in( LPC::TRANS_MEM, i_addr, o_data );
}
errlHndl_t readw(uint32_t i_addr, uint16_t *o_data)
{
    return lpc_in( LPC::TRANS_MEM, i_addr, o_data );
}
errlHndl_t readl(uint32_t i_addr, uint32_t *o_data)
{
    return lpc_in( LPC::TRANS_MEM, i_addr, o_data );
}

errlHndl_t fw_readb(uint32_t i_addr, uint8_t *o_data)
{
    return lpc_in( LPC::TRANS_FW, i_addr, o_data );
}
errlHndl_t fw_readw(uint32_t i_addr, uint16_t *o_data)
{
    return lpc_in( LPC::TRANS_FW, i_addr, o_data );
}
errlHndl_t fw_readl(uint32_t i_addr, uint32_t *o_data)
{
    return lpc_in( LPC::TRANS_FW, i_addr, o_data );
}

template<typename T>
static errlHndl_t lpc_out(LPC::TransType i_trans_type, uint32_t i_addr,
                          T i_data)
{
    size_t len = sizeof(i_data);
    return deviceOp( DeviceFW::WRITE,
                     TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                     &i_data,
                     len,
                     DEVICE_LPC_ADDRESS(i_trans_type, i_addr));
                 }

errlHndl_t outb(uint32_t i_addr, uint8_t i_data)
{
    return lpc_out( LPC::TRANS_IO, i_addr, i_data );
}
errlHndl_t outw(uint32_t i_addr, uint16_t i_data)
{
    return lpc_out( LPC::TRANS_IO, i_addr, i_data );
}
errlHndl_t outl(uint32_t i_addr, uint32_t i_data)
{
    return lpc_out( LPC::TRANS_IO, i_addr, i_data );
}

errlHndl_t writeb(uint32_t i_addr, uint8_t i_data)
{
    return lpc_out( LPC::TRANS_MEM, i_addr, i_data );
}
errlHndl_t writew(uint32_t i_addr, uint16_t i_data)
{
    return lpc_out( LPC::TRANS_MEM, i_addr, i_data );
}
errlHndl_t writel(uint32_t i_addr, uint32_t i_data)
{
    return lpc_out( LPC::TRANS_MEM, i_addr, i_data );
}

errlHndl_t fw_writeb(uint32_t i_addr, uint8_t i_data)
{
    return lpc_out( LPC::TRANS_FW, i_addr, i_data );
}
errlHndl_t fw_writew(uint32_t i_addr, uint16_t i_data)
{
    return lpc_out( LPC::TRANS_FW, i_addr, i_data );
}
errlHndl_t fw_writel(uint32_t i_addr, uint32_t i_data)
{
    return lpc_out( LPC::TRANS_FW, i_addr, i_data );
}

mutex_t cv_mutex = MUTEX_INITIALIZER;

}; //namespace LPC


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// brs move the mutex into the namespace

/*
 * RAII mutex locking class.  Locks mutex when constructed, unlocks upon
 * destruction.
 */
namespace {
class MutexLock
{
public:
    MutexLock(mutex_t *i_mutex, Spinlock *i_spinlock) :
        iv_mutex( i_mutex ), iv_spinlock( i_spinlock )
    {
        if( iv_mutex )
        {
            mutex_lock( iv_mutex );
        }
        if( iv_spinlock )
        {
            iv_spinlock->lock();
        }
    }
    ~MutexLock()
    {
        if( iv_spinlock )
        {
            iv_spinlock->unlock();
        }
        if( iv_mutex )
        {
            mutex_unlock( iv_mutex );
        }
    }

private:
    mutex_t *iv_mutex;
    Spinlock *iv_spinlock;
};
}

LpcDD::LpcDD()
{
    // Reset LPC bus via LPC_FAST_RESET bit before starting to make sure we
    // are in clean state.
    writeReg( ECCB_NON_FW_RESET_REG, ECCB_RESET_LPC_FAST_RESET );
    writeReg( ECCB_NON_FW_RESET_REG, 0 );
    TimeManager::simpleDelay(0, ECCB_POLL_TIME_NS);

    // Clear related FIR register PBAMFIR.
    writeReg( 0x01010C00, 0 );

    // Set LPC HC Sync Cycle Counter Value Register to PNOR desired value of
    // 0xFF aka infinite timeout.  The LPC2SPI MMIO space accesses take
    // longer than the maximum value available for this register (~7us).
    uint32_t lpchc_sync_val = LPCHC_SYNC_CYCLE_COUNTER_INFINITE;
    size_t reg_size = sizeof(lpchc_sync_val);
    writeLPC(LPC::TRANS_REG, LPCHC_SYNC_CYCLE_COUNTER_REG,
             &lpchc_sync_val, reg_size);
}

LpcDD::~LpcDD()
{
}

errlHndl_t LpcDD::checkAddr(LPC::TransType i_type, uint32_t i_addr,
                            uint32_t *o_addr)
{
    bool invalid_address = false;
    switch ( i_type )
    {
        case LPC::TRANS_IO:
            if( i_addr >= 0x10000 )
            {
                invalid_address = true;
                break;
            }
            *o_addr = i_addr + LPCHC_IO_SPACE;
            break;
        case LPC::TRANS_MEM:
            if( i_addr >= 0x10000000 )
            {
                invalid_address = true;
                break;
            }
            *o_addr = i_addr + LPCHC_MEM_SPACE;
            break;
        case LPC::TRANS_FW:
            if( i_addr < LPCHC_FW_SPACE )
            {
                invalid_address = true;
                break;
            }
            *o_addr = i_addr;
            break;
        case LPC::TRANS_REG:
            if( i_addr < LPCHC_REG_SPACE )
            {
                invalid_address = true;
                break;
            }
            *o_addr = i_addr;
            break;
    }

    if( invalid_address )
    {
        /*@
         * @errortype
         * @moduleid     LPC::MOD_LPCDD_ADDR
         * @reasoncode   LPC::RC_LPC_INVALID_ADDR
         * @userdata1[0:31]   LPC Address
         * @userdata1[32:63]  LPC Transaction Type
         * @devdesc      LpcDD> LPC invalid address
         */
        return new ERRORLOG::ErrlEntry( ERRORLOG::ERRL_SEV_UNRECOVERABLE,
                                        LPC::MOD_LPCDD_ADDR,
                                        LPC::RC_LPC_INVALID_ADDR,
                                        TWO_UINT32_TO_UINT64(
                                            i_addr, i_type),
                                        0 );
    }

    return NULL;
}

/**
 * @brief Read an address from LPC space
 */
errlHndl_t LpcDD::readLPC(LPC::TransType i_type, uint32_t i_addr,
                          void* o_buffer,
                          size_t& io_buflen)
{
    errlHndl_t l_err = NULL;

    uint32_t l_addr = 0;
    l_err = checkAddr( i_type, i_addr, &l_addr );
    if( l_err )
    {
        return l_err;
    }

    Spinlock *lpc_spinlock = NULL;
#ifdef HOSTBOOT_SERIAL_CONSOLE
    // Lock out the kernel from printing anything while performing the
    // LPC op as it will use the same set of registers for printk.
    lpc_spinlock = &g_printk_lpc_spinlock;
#endif
    MutexLock ml( &LPC::cv_mutex, lpc_spinlock );

    // Execute command.
    ControlReg_t eccb_cmd;
    eccb_cmd.data_len = io_buflen;
    eccb_cmd.read_op = 1;
    eccb_cmd.addr_len = sizeof(l_addr);
    eccb_cmd.address = l_addr;
    l_err = writeReg( ECCB_CTL_REG, eccb_cmd.data64 );
    if( l_err )
    {
        return l_err;
    }

    StatusReg_t eccb_stat;
    l_err = pollComplete( eccb_cmd, &eccb_stat );
    if( l_err )
    {
        return l_err;
    }

    // Copy data out to caller's buffer.
    switch ( io_buflen )
    {
        case 1:
            *reinterpret_cast<uint8_t*>( o_buffer ) =
                eccb_stat.read_data >> 24;
            break;
        case 2:
            *reinterpret_cast<uint16_t*>( o_buffer ) =
                eccb_stat.read_data >> 16;
            break;
        case 4:
            *reinterpret_cast<uint32_t*>( o_buffer ) =
                eccb_stat.read_data;
            break;
        default:
            assert( false );
            break;
    }

    LPC_TRACDCOMP( g_trac_lpc, "readLPC> %08X[%d] = %08X", l_addr, io_buflen,
                   eccb_stat.read_data >> (8 * (4 - io_buflen)) );

    return l_err;
}

/**
 * @brief Write an address from LPC space
 */
errlHndl_t LpcDD::writeLPC(LPC::TransType i_type, uint32_t i_addr,
                           const void* i_buffer,
                           size_t& io_buflen)
{
    errlHndl_t l_err = NULL;

    uint32_t l_addr = 0;
    l_err = checkAddr( i_type, i_addr, &l_addr );
    if( l_err )
    {
        return l_err;
    }

    uint64_t eccb_data = 0;
    // Left-justify user data into data register.
    switch ( io_buflen )
    {
        case 1:
            eccb_data = static_cast<uint64_t>(
                *reinterpret_cast<const uint8_t*>( i_buffer ) ) << 56;
            break;
        case 2:
            eccb_data = static_cast<uint64_t>(
                *reinterpret_cast<const uint16_t*>( i_buffer ) ) << 48;
            break;
        case 4:
            eccb_data = static_cast<uint64_t>(
                *reinterpret_cast<const uint32_t*>( i_buffer ) ) << 32;
            break;
        default:
            assert( false );
            break;
    }

    // brs - deadlocks when lpc tracing is on
    LPC_TRACFCOMP(g_trac_lpc, "writeLPC> %08X[%d] = %08X", l_addr, io_buflen,
                  eccb_data >> (32 + 8 * (4 - io_buflen)));

    Spinlock *lpc_spinlock = NULL;
#ifdef HOSTBOOT_SERIAL_CONSOLE
    // Lock out the kernel from printing anything while performing the
    // LPC op as it will use the same set of registers for printk.
    lpc_spinlock = &g_printk_lpc_spinlock;
#endif
    MutexLock ml( &LPC::cv_mutex, lpc_spinlock );

    l_err = writeReg( ECCB_DATA_REG, eccb_data );
    if( l_err )
    {
        return l_err;
    }

    // Execute command.
    ControlReg_t eccb_cmd;
    eccb_cmd.data_len = io_buflen;
    eccb_cmd.read_op = 0;
    eccb_cmd.addr_len = sizeof(l_addr);
    eccb_cmd.address = l_addr;
    l_err = writeReg( ECCB_CTL_REG, eccb_cmd.data64 );
    if( l_err )
    {
        return l_err;
    }

    StatusReg_t eccb_stat;
    l_err = pollComplete( eccb_cmd, &eccb_stat );
    if( l_err )
    {
        return l_err;
    }

    return l_err;
}

errlHndl_t LpcDD::readReg(uint32_t i_reg, uint64_t *o_val)
{
  // always read/write 64 bits to SCOM
  size_t scom_size = sizeof(uint64_t);
  return deviceOp( DeviceFW::READ,
                   TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                   o_val,
                   scom_size,
                   DEVICE_SCOM_ADDRESS(i_reg) );
}

errlHndl_t LpcDD::writeReg(uint32_t i_reg, uint64_t i_val)
{
  // always read/write 64 bits to SCOM
  size_t scom_size = sizeof(uint64_t);
  return deviceOp( DeviceFW::WRITE,
                   TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                   &i_val,
                   scom_size,
                   DEVICE_SCOM_ADDRESS(i_reg) );
}

errlHndl_t LpcDD::pollComplete(const ControlReg_t &i_ctrl, StatusReg_t *o_stat)
{
    errlHndl_t err = NULL;
    uint64_t poll_time = 0;
    uint64_t loop = 0;
    do
    {
        err = readReg( ECCB_STAT_REG, &o_stat->data64 );
        LPC_TRACDCOMP( g_trac_lpc, "writeLPC> Poll on ECCB Status, "
                       "poll_time=0x%.16x, stat=0x%.16x",
                       poll_time,
                       o_stat->data64 );
        if( err )
        {
            return err;
        }

        if( o_stat->op_done )
        {
            break;
        }

        // want to start out incrementing by small numbers then get bigger
        //  to avoid a really tight loop in an error case so we'll increase
        //  the wait each time through
        //TODO tmp remove for VPO, need better polling strategy -- RTC43738
        //nanosleep( 0, ECCB_POLL_INCR_NS*(++loop) );
        poll_time += ECCB_POLL_INCR_NS * loop++;
    } while ( poll_time < ECCB_POLL_TIME_NS );

    // Check for errors or timeout.
    if( (o_stat->data64 & LPC_STAT_REG_ERROR_MASK) || (!o_stat->op_done) )
    {
        LPC_TRACFCOMP( g_trac_lpc, "LpcDD::pollComplete> LPC error or timeout: "
                       "addr=0x%.8X, status=0x%.16X",
                       i_ctrl.address, o_stat->data64 );

        if( i_ctrl.read_op )
        {
            /*@
             * @errortype
             * @moduleid     LPC::MOD_LPCDD_READLPC
             * @reasoncode   LPC::RC_LPC_ERROR
             * @userdata1[0:31]   LPC Address
             * @userdata1[32:63]  Total poll time (ns)
             * @userdata2    ECCB Status Register
             * @devdesc      LpcDD::pollComplete> LPC error or timeout
             */
            err = new ERRORLOG::ErrlEntry( ERRORLOG::ERRL_SEV_UNRECOVERABLE,
                                           LPC::MOD_LPCDD_READLPC,
                                           LPC::RC_LPC_ERROR,
                                           TWO_UINT32_TO_UINT64(
                                               i_ctrl.address, poll_time),
                                           o_stat->data64 );
        }
        else
        {
            /*@
             * @errortype
             * @moduleid     LPC::MOD_LPCDD_WRITELPC
             * @reasoncode   LPC::RC_LPC_ERROR
             * @userdata1[0:31]   LPC Address
             * @userdata1[32:63]  Total poll time (ns)
             * @userdata2    ECCB Status Register
             * @devdesc      LpcDD::pollComplete> LPC error or timeout
             */
            err = new ERRORLOG::ErrlEntry( ERRORLOG::ERRL_SEV_UNRECOVERABLE,
                                           LPC::MOD_LPCDD_WRITELPC,
                                           LPC::RC_LPC_ERROR,
                                           TWO_UINT32_TO_UINT64(
                                               i_ctrl.address, poll_time),
                                           o_stat->data64 );
        }
        err->collectTrace( "XSCOM" );
        //@todo (RTC:37744) - Any cleanup or recovery needed?
    }

    return err;
}
