/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/console/cpuDisp.C $                                   */
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
 *  @file cpuDisp.C
 *
 *  @brief Print out detected CPU configuration.
 */

#include <assert.h>
#include <console/consoleif.H>
#include <targeting/common/targetservice.H>
#include <targeting/common/utilFilter.H>

using namespace TARGETING;

namespace CONSOLE
{

static void printProcessor(TARGETING::Target *i_processor) {
    // Get bitmask and count of enabled threads.
    TARGETING::Target* sys = NULL;
    TARGETING::targetService().getTopLevelTarget(sys);
    assert( sys != NULL );
    uint64_t en_threads = sys->getAttr<ATTR_ENABLED_THREADS>() >> 56;
    int thread_count = 0;
    for (int i = 0; i < 8; ++i) {
        if (en_threads & (1 << i)) {
            thread_count++;
        }
    }

    // Processor.
    const char *model = i_processor->getAttrAsString<ATTR_MODEL>();
    uint32_t node_id = i_processor->getAttr<ATTR_FABRIC_CHIP_ID>();
    EntityPath phys_path = i_processor->getAttr<ATTR_PHYS_PATH>();
    uint8_t ec = i_processor->getAttr<ATTR_EC>();

    CONSOLE::printf("Processor %s #%d DD%02x.%02x [%s]\n",
                    model,
                    node_id,
                    (ec >> 4) & 0xf,
                    ec & 0xf,
                    phys_path.toString());

    // EXecution units and cores.
    TargetHandleList exs;
    getChildChiplets(exs, i_processor, TYPE_EX, true);
    for (TargetHandleList::iterator it = exs.begin(); it != exs.end(); it++) {
        Target *ex = *it;
        int unit_id = ex->getAttr<ATTR_CHIP_UNIT>();
        int cpu_id = node_id << 7 | unit_id << 3;
        CONSOLE::printf("  EX #%d %d threads (CPUs %d-%d) [%s]\n",
                        unit_id,
                        thread_count,
                        cpu_id,
                        cpu_id + thread_count - 1,
                        ex->getAttr<ATTR_PHYS_PATH>().toString());
    }

    // Centaurs attached to Memory Controller Synchronous.
    TargetHandleList mcses;
    getChildChiplets(mcses, i_processor, TYPE_MCS, true);
    for (TargetHandleList::iterator it = mcses.begin();
        it != mcses.end(); it++) {
        Target *mcs = *it;

        uint32_t mcs_id = mcs->getAttr<ATTR_CHIP_UNIT>();

        TargetHandleList membufs;
        getChildAffinityTargets(membufs, mcs, CLASS_CHIP, TYPE_MEMBUF, false);
        for (TargetHandleList::iterator it = membufs.begin();
            it != membufs.end(); it++) {
            Target *membuf = *it;

            HwasState hwasState = membuf->getAttr<ATTR_HWAS_STATE>();
            if (!hwasState.present) {
                continue;
            }
            CONSOLE::printf(
                "  Centaur MCS%d [%s]\n",
                mcs_id,
                membuf->getAttr<ATTR_AFFINITY_PATH>().toString());
        }
    }
}

void cpuDisplay() {
    CONSOLE::printf("=====================CPU DISPLAY=====================\n");

    TARGETING::TargetHandleList procs;
    getAllChips(procs, TARGETING::TYPE_PROC, false);
    for (TargetHandleList::iterator it = procs.begin();
        it != procs.end(); ++it) {
        HwasState hwasState = (*it)->getAttr<ATTR_HWAS_STATE>();
        if (!hwasState.present) {
            continue;
        }
        printProcessor(*it);
    }
    CONSOLE::printf("=====================================================\n");
}

}  // namespace CONSOLE
