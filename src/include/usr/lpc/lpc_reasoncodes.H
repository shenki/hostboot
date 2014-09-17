/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/include/usr/lpc/lpc_reasoncodes.H $                       */
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
#ifndef __LPC_REASONCODES_H
#define __LPC_REASONCODES_H

#include <hbotcompid.H>

namespace LPC
{
    enum LPCModuleId
    {
        MOD_LPCDD_ADDR                  = 0x00,
        MOD_LPCDD_READLPC               = 0x01,
        MOD_LPCDD_WRITELPC              = 0x02,
    };

    enum LPCReasonCode
    {
        RC_LPC_INVALID_ADDR          = LPC_COMP_ID | 0x01,
        RC_LPC_ERROR                 = LPC_COMP_ID | 0x02,
    };
};

#endif
