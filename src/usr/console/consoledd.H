/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/console/consoledd.H $                                 */
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
#ifndef __CONSOLE_CONSOLEDD_H
#define __CONSOLE_CONSOLEDD_H

#include <errl/errlentry.H>
#include <limits.h>
#include <stdint.h>
#include <sys/sync.h>

/**
 *   @brief Abstract console character interface.
 */
class ConsoleInterface
{
public:
    virtual ~ConsoleInterface()
    {
    }

    // Read char from console.
    virtual errlHndl_t getChar(uint32_t i_timeout_ms, char *c) = 0;
    // Write char to console.
    virtual errlHndl_t putChar(const char c) = 0;
};

/**
 *  @brief  Console Device Driver Class
 *     Provides access to the system console.
 */
class ConsoleDD
{
public:
    ConsoleDD();
    ~ConsoleDD();

    /**
     * @brief Initialize the console.
     *
     * @return Error from operation
     */
    errlHndl_t init();

    /**
     * @brief Writes a buffer to the console.
     *
     * @parm i_buffer  Buffer to write data from
     * @parm i_buflen  Input: Number of bytes to write,
     *
     * @return Error from operation
     */
    errlHndl_t writeConsole(const char *i_buffer, size_t i_buflen);

    /**
     * @brief Reads a character from the console.
     *
     * @parm i_timeout_ms  Time to wait to get character.
     * @parm o_data        Read character or '\0' on timeout.
     *
     * @return Error from operation
     */
    errlHndl_t readConsole(uint32_t i_timeout_ms, char *o_data);

private:
    ConsoleInterface *iv_console;

    /**
     * @brief Mutex to prevent concurrent accesses.
     */
    static mutex_t cv_mutex;
};


#endif
