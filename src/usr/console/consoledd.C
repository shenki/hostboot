/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/console/consoledd.C $                                 */
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
 *  @file consoledd.C
 *
 *  @brief Implementation of the console Device Driver
 */

#include "consoledd.H"
#include <console/consoleif.H>

#include <devicefw/driverif.H>
#include <errl/errlentry.H>
#include <initservice/taskargs.H>
#include <kernel/console.H>
#include <lpc/lpcif.H>
#include <stdarg.h>
#include <stdio.h>
#include <sys/sync.h>
#include <sys/time.h>
#include <targeting/common/targetservice.H>

namespace CONSOLE
{

/**
 * @brief Performs an console Operation
 *
 * @param[in]   i_opType        Operation type, see DeviceFW::OperationType
 *                              in driverif.H
 * @param[in]   i_target        always SENTINEL value
 * @param[in/out] io_buffer     Read: Pointer to output data storage
 *                              Write: Pointer to input data storage
 * @param[in/out] io_buflen     Input: size of io_buffer (in bytes)
 *                              Output:
 *                                  Read: Size of output data
 *                                  Write: Size of data written
 * @param[in]   i_accessType    DeviceFW::AccessType enum (usrif.H)
 * @param[in]   i_args          This is an argument list for DD framework.
 *
 * @return  errlHndl_t
 */
errlHndl_t consoleOp(DeviceFW::OperationType i_opType,
                        TARGETING::Target* i_target,
                        void* io_buffer,
                        size_t& io_buflen,
                        int64_t i_accessType,
                        va_list i_args)
{
    switch (i_opType)
    {
        case DeviceFW::WRITE:
            return Singleton<ConsoleDD>::instance().writeConsole(
                reinterpret_cast<const char *>(io_buffer),
                io_buflen );
        case DeviceFW::READ:
            assert(io_buflen == sizeof(char));
            return Singleton<ConsoleDD>::instance().readConsole(
                va_arg(i_args,uint64_t),
                reinterpret_cast<char*>(io_buffer) );
        default:
            assert(0);
            return NULL;
    }
}

// Register console functions to DD framework.
DEVICE_REGISTER_ROUTE( DeviceFW::WRITE,
                       DeviceFW::CONSOLE,
                       TARGETING::TYPE_PROC,
                       consoleOp );
DEVICE_REGISTER_ROUTE( DeviceFW::READ,
                       DeviceFW::CONSOLE,
                       TARGETING::TYPE_PROC,
                       consoleOp );

errlHndl_t printf(const char* str, ...)
{
    va_list args;
    va_start( args, str );

    char buf[CONSOLE_MAX_STRING_LENGTH];
    // TODO: use vsnprintf
    size_t len = vsprintf( buf, str, args ) + sizeof('\0');
    va_end( args );

    return deviceOp( DeviceFW::WRITE,
                     TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                     buf,
                     len,
                     DeviceFW::CONSOLE );
}

errlHndl_t getChar(uint32_t i_timeout_ms, char *c)
{
    size_t l_size = sizeof(*c);
    return deviceOp( DeviceFW::READ,
                     TARGETING::MASTER_PROCESSOR_CHIP_TARGET_SENTINEL,
                     c,
                     l_size,
                     DeviceFW::CONSOLE,
                     i_timeout_ms );
}

}  // namespace CONSOLE


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/*
 * RAII mutex locking class.  Locks mutex when constructed, unlocks upon
 * destruction.
 */
class MutexLock {
 public:
  MutexLock(mutex_t *i_mutex) : iv_mutex(i_mutex) { mutex_lock(iv_mutex); }
  ~MutexLock() { mutex_unlock(iv_mutex); }

 private:
  mutex_t *iv_mutex;
};

class ConsoleUart : public ConsoleInterface
{
public:
    static const int UART_CLOCK = 1843200;

    ConsoleUart() : iv_clock(UART_CLOCK)
    {
        // brs hack for now
#if 0
        // Read console attributes from system object.
        TARGETING::Target *sys;
        TARGETING::targetService().getTopLevelTarget(sys);

        iv_base_addr = sys->getAttr<TARGETING::ATTR_CONSOLE_UART_BASE>();
        iv_baud = sys->getAttr<TARGETING::ATTR_CONSOLE_UART_BAUD_RATE>();
#else
        iv_base_addr = 0x3f8;
        iv_baud = 115200;
#endif
        iv_uart_working = init();
    }

    errlHndl_t getChar(uint32_t i_timeout_ms, char *c)
    {
        errlHndl_t l_err = NULL;
        if (!iv_uart_working) {
            return NULL;
        }

        uint8_t lsr = 0;
        uint32_t time_elapsed_ms = 0;
        do {
            if( (l_err = readReg( UART_LSR, &lsr )) != NULL )
            {
                return l_err;
            }
            // Check for data ready in receive buffer.
            if( lsr & LSR_DR )
            {
                uint8_t tmp;
                if ( (l_err = readReg ( UART_RBR , &tmp )) != NULL )
                {
                    return l_err;
                }
                *c = static_cast<char>(tmp);
                return l_err;
            }
            nanosleep( 0, 1e6 );
            time_elapsed_ms += 1;
        } while (time_elapsed_ms < i_timeout_ms);

        // No character ready within timeout.
        *c = 0;
        return l_err;
    }

    errlHndl_t putChar(const char c)
    {
        errlHndl_t l_err = NULL;
        if (!iv_uart_working) {
            return NULL;
        }

        // Serial port newline conversion LF -> CR/LF.
        if (c == '\n') {
            l_err = putChar('\r');
            if (l_err)
            {
                return l_err;
            }
        }

        // Wait for FIFO empty.
        uint8_t lsr = 0;
        int timeout = 0;
        do
        {
            if( (l_err = readReg( UART_LSR, &lsr )) != NULL )
            {
                return l_err;
            }
            // Wait for idle.
            if( lsr == LSR_BAD || (lsr & LSR_THRE) != 0 )
            {
                break;
            }
            TimeManager::simpleDelay( 0, 100 );
        } while ( timeout++ < 10000 );

        if ( lsr == LSR_BAD || timeout >= 10000 ) {
            iv_uart_working = false;
            return NULL;
        }

        return writeReg( UART_THR, c );
    }

    // Initialize the UART.
    // Returns: true on success, false if UART not found/working
    bool init()
    {
        printk( "Initializing console UART @ %04x %d,8N1... ",
                iv_base_addr, iv_baud );

#ifdef LPC_SLAVE_AST2400
         LPC::outb( 0x2e, 0xA5 );  //unlock
         LPC::outb( 0x2e, 0xA5 );
         LPC::outb( 0x2e, 0x07 );
         LPC::outb( 0x2f, 0x02 );
         LPC::outb( 0x2e, 0x30 );
         LPC::outb( 0x2f, 0x00 );
         LPC::outb( 0x2e, 0x60 );
         LPC::outb( 0x2f, 0x03 );
         LPC::outb( 0x2e, 0x61 );
         LPC::outb( 0x2f, 0xF8 ); //set to 3F8
         LPC::outb( 0x2e, 0x70 );
         LPC::outb( 0x2f, 0x04 );
         LPC::outb( 0x2e, 0x30 );
         LPC::outb( 0x2f, 0x01 );
         LPC::outb( 0x2e, 0xAA );  //lock
 #endif



        // Clear line control reg.
        errlHndl_t l_err;
        if( (l_err = writeReg( UART_LCR, 0x00 )) != NULL )
        {
            delete l_err;
            return false;
        }

        // Do a simple existence test with the IRQ register and
        // disable interrupts.
        uint8_t ier[2] = { };
        static const uint8_t enable_ier = 0x0f;
        static const uint8_t disable_ier = 0x00;
        if( (l_err = writeReg( UART_IER, enable_ier )) != NULL ||
            (l_err = readReg( UART_IER, &ier[0] )) != NULL ||
            (l_err = writeReg( UART_IER, disable_ier )) != NULL ||
            (l_err = readReg( UART_IER, &ier[1] )) != NULL )
        {
            delete l_err;
            printk( "error\n" );
            return false;
        }

        // If these don't match then we don't have a UART.
        if( ier[0] != enable_ier || ier[1] != disable_ier )
        {
            printk( "not found\n" );
            return false;
        }

        // Set Baud rate.
        int dll = (iv_clock / 16) / iv_baud;
        writeReg( UART_LCR, LCR_DLAB );
        writeReg( UART_DLL, dll & 0xff );
        writeReg( UART_DLM, dll >> 8 );

        // 8N1.
        writeReg( UART_LCR, 0x03 );
        // Enable RTS/DTR.
        writeReg( UART_MCR, 0x03 );
        // Clear and enable FIFOs.
        writeReg( UART_FCR, 0x07 );

        printk( "done\n" );
        return true;
    }

private:
    errlHndl_t readReg(uint8_t reg, uint8_t *val)
    {
        return LPC::inb( iv_base_addr + reg, val );
    }
    errlHndl_t writeReg(uint8_t reg, uint8_t val)
    {
        return LPC::outb( iv_base_addr + reg, val );
    }

    /* UART reg defs */
    enum
    {
        UART_RBR = 0,
        UART_THR = 0,
        UART_DLL = 0,
        UART_IER = 1,
        UART_DLM = 1,
        UART_FCR = 2,
        UART_IIR = 2,
        UART_LCR = 3,
        UART_MCR = 4,
        UART_LSR = 5,
        UART_MSR = 6,
        UART_SCR = 7,
    };

    /* Line Status Register (LSR) bit definitions */
    enum
    {
        LSR_DR = 0x01,    /**<  Data ready */
        LSR_OE = 0x02,    /**<  Overrun */
        LSR_PE = 0x04,    /**<  Parity error */
        LSR_FE = 0x08,    /**<  Framing error */
        LSR_BI = 0x10,    /**<  Break */
        LSR_THRE = 0x20,  /**<  Xmit holding register empty */
        LSR_TEMT = 0x40,  /**<  Xmitter empty */
        LSR_ERR = 0x80,   /**<  Error */

        LSR_BAD = 0xff,   /**<  Invalid value for LSR */

        LCR_DLAB = 0x80,  /**<  DLL access */
    };

    uint32_t iv_base_addr;
    uint32_t iv_baud;
    uint32_t iv_clock;
    bool iv_uart_working;
};

// Console equivalent of /dev/null for unsupported systems.
class ConsoleNull : public ConsoleInterface
{
public:
    ConsoleNull()
    {
    }
    errlHndl_t getChar(uint32_t i_timeout_ms, char *c)
    {
        return NULL;
    }
    errlHndl_t putChar(const char c)
    {
        return NULL;
    }
    bool init()
    {
        return true;
    }
};

// This is the task entry point.  It is responsible for actually intializing
// the console after targeting can tell us what to use for our console
// output.
static void consoleEntryPoint(errlHndl_t &io_taskRetErrl)
{
    printk("in console entry point\n");
    io_taskRetErrl = Singleton<ConsoleDD>::instance().init();
}
TASK_ENTRY_MACRO( consoleEntryPoint );

mutex_t ConsoleDD::cv_mutex = MUTEX_INITIALIZER;

ConsoleDD::ConsoleDD()
    : iv_console( NULL )
{
}

ConsoleDD::~ConsoleDD()
{
    delete iv_console;
}

errlHndl_t ConsoleDD::init()
{
    MutexLock ml( &cv_mutex );

    // brs hack for now
#if 0
    TARGETING::Target *sys = NULL;
    TARGETING::targetService().getTopLevelTarget( sys );
    if( sys == NULL )
    {
        return NULL;
    }

    TARGETING::ATTR_CONSOLE_KIND_type console_kind =
        TARGETING::CONSOLE_KIND_NONE;

    sys->tryGetAttr<TARGETING::ATTR_CONSOLE_KIND>( console_kind );
#else
    TARGETING::ATTR_CONSOLE_KIND_type console_kind =
        TARGETING::CONSOLE_KIND_UART;
#endif
    switch ( console_kind )
    {
        case TARGETING::CONSOLE_KIND_UART:
            iv_console = new ConsoleUart;
            TRACE_TO_CONSOLE = 1;
            break;
        default:
            iv_console = new ConsoleNull;
            break;
    }

    return NULL;
}

errlHndl_t ConsoleDD::writeConsole(const char *i_buffer, size_t i_buflen)
{
    MutexLock ml( &cv_mutex );
    // Send console output to the kernel before targetting is initialized.
    if( iv_console == NULL )
    {
        // TODO: Check string buffer length.
        printk( "%s", i_buffer );
        return NULL;
    }

    for ( size_t i = 0; i < i_buflen; ++i )
    {
        errlHndl_t l_err = iv_console->putChar( i_buffer[i] );
        if( l_err != NULL )
        {
            return l_err;
        }
    }

    return NULL;
}

errlHndl_t ConsoleDD::readConsole(uint32_t i_timeout_ms, char *o_data)
{
    MutexLock ml( &cv_mutex );

    // No reading before targeting is initialized.
    if( iv_console == NULL )
    {
        *o_data = 0;
        return NULL;
    }

    return iv_console->getChar( i_timeout_ms, o_data );
}
