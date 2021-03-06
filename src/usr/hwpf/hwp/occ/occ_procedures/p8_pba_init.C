/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/hwpf/hwp/occ/occ_procedures/p8_pba_init.C $           */
/*                                                                        */
/* OpenPOWER HostBoot Project                                             */
/*                                                                        */
/* Contributors Listed Below - COPYRIGHT 2013,2014                        */
/* [+] International Business Machines Corp.                              */
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
// $Id: p8_pba_init.C,v 1.19 2014/04/08 05:45:40 stillgs Exp $
// $Source: /afs/awd/projects/eclipz/KnowledgeBase/.cvsroot/eclipz/chips/p8/working/procedures/ipl/fapi/p8_pba_init.C,v $
//------------------------------------------------------------------------------
// *! (C) Copyright International Business Machines Corp. 2011
// *! All Rights Reserved -- Property of IBM
// *! ***  ***
//------------------------------------------------------------------------------
// *! OWNER NAME: Klaus P. Gungl         Email: kgungl@de.ibm.com
// *! BACKUP NAME: Greg Still            Email: stillgs@us.ibm.com
// *!
// *!
/// \file p8_pba_init.C
/// \brief Initialize PBA registers for modes PM_INIT, PM_RESET and PM_CONFIG
// *!
// *! Functional description: setup the PBA registers depending on mode
// *!   calling parameters:
// *! :   Target i_target  // target according to calling conventions
// *!     uint64_t mode    // mode according to power_up spec: PM_CONFIG, PM_INIT, PM_RESET
// *!
// *! high level flow:
// *!  if (mode == PM_CONFIG) {
// *!      rc =  pba_init_config(i_target);
// *!  } else if {mode == PM_INIT) {
// *!      rc =  pba_init_init(i_target);
// *!  } else if (mode == PM_RESET) {
// *!      rc =  pba_init_reset(i_target);
// *!  } else {
// *!      FAPI_SET_HWP_ERROR(rc,RC_PMPROC_PBA_INIT_INCORRECT_MODE);
// *!  }
// *!
// *! buildfapiprcd -e "../../xml/error_info/p8_pba_init_errors.xml" p8_pba_init.C
//------------------------------------------------------------------------------


// ----------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------
#include <fapi.H>
#include "p8_scom_addresses.H"
#include "p8_pba_init.H"
#include "p8_pm.H"

// get the constants from here
#include "pgp_pba.h"
#include "pgp_common.h"

extern "C" {

using namespace fapi;

// ----------------------------------------------------------------------
// Constant definitions
// ----------------------------------------------------------------------


// ----------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------
// mandated: do not use global variables

// ----------------------------------------------------------------------
// local Function definitions / prototypes
// ----------------------------------------------------------
fapi::ReturnCode pba_init_config ( const Target& i_target );
fapi::ReturnCode pba_init_init ( const Target& i_target );
fapi::ReturnCode pba_init_reset ( const Target& i_target );

fapi::ReturnCode pba_slave_setup_init ( const Target& i_target );
fapi::ReturnCode pba_slave_setup_reset ( const Target& i_target );
fapi::ReturnCode pba_slave_reset(const Target& i_target);

fapi::ReturnCode pba_bc_stop(const Target& i_target);


// **********************************************************************************************
// ----------------------------------------------- p8_pba_init --------------------------------
// function:
// set the pba registers depending on "mode", no default mode
// returns: fapi return codes
fapi::ReturnCode
p8_pba_init(const Target& i_target,
                uint64_t mode
             )
{
    fapi::ReturnCode rc;
    // calling the selected function from here

    if (mode == PM_CONFIG)
    {
       rc =  pba_init_config(i_target);
    }
    else if (mode == PM_INIT)
    {
       rc =  pba_init_init(i_target);
    }
    else if (mode == PM_RESET)
    {
       rc =  pba_init_reset(i_target);
    }
    else
    {
      FAPI_ERR("Unknown mode passed to p8_pba_init. Mode %08llx ", mode);
      const uint64_t& PM_MODE = mode;
      FAPI_SET_HWP_ERROR(rc, RC_PMPROC_PBA_INIT_INCORRECT_MODE);
    } // endif


    return rc;
}

// **********************************************************************************************
 // ******************************************************** mode = PM_RESET ********************

fapi::ReturnCode
pba_init_reset(const Target& i_target)
{

    fapi::ReturnCode    rc;
    uint32_t            l_rc = 0;
    ecmdDataBufferBase  data(64);
    uint64_t            address;

    //--------------------------------------------------------------------------
    const int MAX_PBA_RESET_REGS = 19; //Number of regs
    uint64_t ary_pba_reset_regs[MAX_PBA_RESET_REGS] =
    {
        PBA_MODE_0x00064000            ,
        PBA_BCDE_STAT_0x00064012       ,
        PBA_BCDE_PBADR_0x00064013      ,
        PBA_BCDE_OCIBAR_0x00064014     ,
        PBA_BCUE_CTL_0x00064015        ,
        PBA_BCUE_SET_0x00064016        ,
        PBA_BCUE_STAT_0x00064017       ,
        PBA_BCUE_PBADR_0x00064018      ,
        PBA_BCUE_OCIBAR_0x00064019     ,
        PBAXSHBR0_00064026             ,
        PBAXSHCS0_00064027             ,
        PBAXSHBR1_0006402A             ,
        PBAXSHBR1_0006402B             ,
        PBA_SLVCTL0_0x00064004         ,
        PBA_SLVCTL1_0x00064005         ,
//      PBA_SLVCTL2_0x00064006         ,  // this is only touched by SLW init
        PBA_SLVCTL3_0x00064007         ,
        PBA_FIR_0x02010840             ,
        PBA_CONFIG_0x0201084B          ,
        PBA_ERR_RPT0_0x0201084C
//      PBAXCFG_00064021                  // Takes more than write of 0
                                            // and should be done last of
                                            // after err_rpt clearing
    };

    FAPI_INF("pba_init_reset start ...");
    do
    {
        // Stop the  BCDE and BCUE        
        rc = pba_bc_stop(i_target);
        if (!rc.ok())
        {
            FAPI_ERR("pba_bc_stop detected an error");
            break;
        }
      
        // Reset each slave and wait for completion.
        rc =  pba_slave_reset(i_target);
        if (rc)
        {
             FAPI_ERR("pba_slave_reset failed.");
             break;
        }

        // For reset phase, write these with 0x0

        // Clear buffer to create 0 write data
        l_rc = data.flushTo0();
        if (l_rc)
        {
            rc.setEcmdError(l_rc);
            break;
        }

        for (int i = 0; i < MAX_PBA_RESET_REGS; i++)
        {
            FAPI_INF("\tResetting PBA register addr=0x%08llX with 0, Target = %s",
                            ary_pba_reset_regs[i],
                            i_target.toEcmdString());

            rc = fapiPutScom(i_target, ary_pba_reset_regs[i], data);
            if (!rc.ok())
            {
                FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            ary_pba_reset_regs[i],
                            i_target.toEcmdString());
                break;
            }
        }
        if(!rc.ok())
        {
            break;
        }

        // Perform non-zero reset operations

        // Reset PBAX errors via Configuration Register
        address = PBAXCFG_00064021;

        l_rc |= data.flushTo0();
        l_rc |= data.setBit(2);      // Bit 2: PBAXCFG_SND_RESET
        l_rc |= data.setBit(3);      // Bit 3: PBAXCFG_RCV_RESET
        if (l_rc)
        {
            rc.setEcmdError(l_rc);
            break;
        }
        FAPI_INF("\tResetting PBAX errors via PBAX config register addr=0x%08llX, value=0x%16llX, Target = %s",
                            address,
                            data.getDoubleWord(0),
                            i_target.toEcmdString());
        rc = fapiPutScom(i_target, address, data);
        if (!rc.ok())
        {
            FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            address,
                            i_target.toEcmdString());
            break;
        }
    } while(0);
    FAPI_INF("pba_init_reset end ...");
    return rc;

 } // endif (mode == PM_RESET)



// ***********************************************************************************************
// ************************************************************ mode = PM_INIT *******************
// call pba_slave_setup
fapi::ReturnCode
pba_init_init(const Target& i_target)
{

    fapi::ReturnCode    rc;
    uint32_t            l_rc = 0;
    ecmdDataBufferBase  data(64);


    // PBAX defaults
    uint8_t ATTR_PM_PBAX_RCV_RESERV_TIMEOUT_value = 0 ;
    uint8_t ATTR_PM_PBAX_SND_RETRY_COUNT_OVERCOMMIT_ENABLE_value = 0 ;
    uint8_t ATTR_PM_PBAX_SND_RETRY_THRESHOLD_value = 0 ;
    uint8_t ATTR_PM_PBAX_SND_RESERV_TIMEOUT_value = 0 ;


    pbaxcfg_t pbaxcfg_setup ;
    pbaxcfg_setup.value = 0;
    FAPI_INF("pba_init_init start ...");
    do
    {


        l_rc =  data.setDoubleWord(0, 0x0);
        if (l_rc)
        {
            rc.setEcmdError(l_rc);
            break;
        }

        // This register is cleared as there are no chicken switches that need
        // to be disabled.  All other bits are set by OCC Firmware.
        FAPI_INF("flushing PBA_CONFIG register ");

        rc = fapiPutScom(i_target, PBA_CONFIG_0x0201084B  , data);
        if (rc)
        {
            FAPI_ERR("fapiPutScom( PBA_CONFIG_0x0201084B ) failed. With rc = 0x%x", (uint32_t)rc);
            break;
        }

        // Clear the PBA FIR only
        // data still 0
        FAPI_INF("flushing PBA_FIR register ");
        rc = fapiPutScom(i_target, PBA_FIR_0x02010840   , data);
        if (rc)
        {
            FAPI_ERR("fapiPutScom( PBA_FIR_0x02010840 ) failed. With rc = 0x%x", (uint32_t)rc);
            break;
        }

        // The following registers are ROX, hence need not be touched:
        // PBA_RBUFVAL0_0x02010850
        // PBA_RBUFVAL1_0x02010851
        // PBA_RBUFVAL2_0x02010852
        // PBA_RBUFVAL3_0x02010853
        // PBA_RBUFVAL4_0x02010854
        // PBA_RBUFVAL5_0x02010855
        // PBA_WBUFVAL0_0x02010858
        // PBA_WBUFVAL1_0x02010859

        // These PowerBus Overcommit regs are read-only, therefore no action required:
        // PBA_PBOCR0_0x00064020
        // PBA_PBOCR1_0x00064021
        // PBA_PBOCR2_0x00064022
        // PBA_PBOCR3_0x00064023
        // PBA_PBOCR4_0x00064024
        // PBA_PBOCR5_0x0006402

        // The PBA BARs and their associated Masks are done outside of this FAPI
        // set.  Thus, during a reset, the BARS/MASKS are retained. This applies
        // to:
        // PBA_BAR0_0x02013F00
        // PBA_BARMSK0_0x02013F04
        // PBA_BAR1_0x02013F01
        // PBA_BARMSK1_0x02013F05
        // PBA_BAR2_0x02013F02
        // PBA_BAR3_0x02013F03
        // PBA_TRUSTMODE_0x02013F08


        // Per SW223235 and other testing, the PBAX hardware timeout mechanism
        // does not allow for durations that can cover all system topologies.
        // Thus, these mechanisms are being disabled.  The original attributes
        // for PBAX were to enable and control these settings but, given then
        // will not be used for P8, are no longer needed. Therefore, the attribute
        // use calls have been removed in favor of simple procedure disablement.

        // 20:24, ATTR_PM_PBAX_RCV_RESERV_TIMEOUT_value
        // 27; ATTR_PM_PBAX_SND_RETRY_COUNT_OVERCOMMIT_ENABLE_value
        // 28:35; ATTR_PM_PBAX_SND_RETRY_THRESHOLD_value
        // 36:40  ATTR_PM_PBAX_SND_RESERV_TIMEOUT_value
        pbaxcfg_setup.fields.ATTR_PM_PBAX_RCV_RESERV_TIMEOUT = ATTR_PM_PBAX_RCV_RESERV_TIMEOUT_value;
        pbaxcfg_setup.fields.ATTR_PM_PBAX_SND_RETRY_COUNT_OVERCOMMIT_ENABLE = ATTR_PM_PBAX_SND_RETRY_COUNT_OVERCOMMIT_ENABLE_value;
        pbaxcfg_setup.fields.ATTR_PM_PBAX_SND_RETRY_THRESHOLD = ATTR_PM_PBAX_SND_RETRY_THRESHOLD_value;
        pbaxcfg_setup.fields.ATTR_PM_PBAX_SND_RESERV_TIMEOUT = ATTR_PM_PBAX_SND_RESERV_TIMEOUT_value;

        // put the attribute values into PBAXCFG
        l_rc = data.setDoubleWord(0, pbaxcfg_setup.value);
        if (l_rc)
        {
            rc.setEcmdError(l_rc);
            break;
        }

        rc = fapiPutScom(i_target, PBAXCFG_00064021   , data);
        if (rc)
        {
            FAPI_ERR("fapiPutScom(PBAXCFG_00064021) failed. With rc = 0x%x", (uint32_t)rc);
            break;
        }

        // last step: pba slave setup for init
        rc = pba_slave_setup_init (i_target);
        if (rc)
        {
            FAPI_ERR("fapi pba_slave_setup_init failed. With rc = 0x%x", (uint32_t)rc);
            break;
        }
    } while(0);
    FAPI_INF("pba_init_init end ...");
    return rc;

} // end PM_INIT


// *************************************************************************************************
// ************************************************************* mode = PM_CONFIG ******************
//
/// Configuration:  perform translation of any Platform Attributes into
/// Feature Attributes that are applied during Initalization of PBAX
fapi::ReturnCode
pba_init_config(const Target& i_target)
{
    fapi::ReturnCode rc;

    FAPI_INF("mode = PM_CONFIG...");

 return rc;
};


// ************************************************************************************************
// **************************************************** pba_slave_setup ***************************
/// PgP PBA Setup
///
/// The PBA is 'set up' twice. The first set up is via scan-0 settings or
/// SBE-IPL code to enable the Host Boot image to be injected into the cache
/// of the IPL core.
///
/// This procedure documents the second setup that will normally be done by a
/// FAPI procedure prior to releasing the OCC from reset.  This setup serves
/// both for the initial boot of OCC as well as for the OCC (GPE), SLW and FSP
/// runtime.  This procedure documents how the PBA should be set up the second
/// time.
///
/// PBA slave 0 is reserved to the GPE engines, which must be able to switch
/// modes to read from mainstore or Centaur inband SCOM spaces, and write
/// Centaur inband SCOMs and also write into mainstore using IMA to support
/// the Power Proxy Trace application.
///
/// PBA slave 1 is dedicated to the 405 ICU/DCU. This PBA slave is used for
/// the initial boot, and for the initial runtime code that manages the OCC
/// applet table.  Once OCC has initialzied applets, OCC FW will remove all
/// TLB mappings associated with mainstore, effectively disabling this slave.
///
/// PBA Slave 2 is dedicated to the PORE-SLW.  Since 24x7 performance
/// monitoring code now runs on PORE-SLW, the PORE-SLW is no longer a
/// read-only master, but instead serves as a generic read-write master like
/// all of the others.
///
/// PBA Slave 3 is mapped to the OCB in order to service FSP direct read/write
/// of mainstore.
///
/// The design of PBA allows read buffers to be dedicated to a particular
/// slave, and requires that a slave have a dedicated read buffer in order to
/// do aggressive prefetching. In the end there is little reason to partition
/// the resources.  The PORE-SLW will be running multiple applications that
/// read and write main memory so we want to allow PORE-SLW access to all
/// resources.  It also isn't clear that aggressive prefetching provides any
/// performacne boost. Another wrinkle is that the BCDE claims read buffer C
/// whenever it runs, and the 405 I + D sides never run after the initial OCC
/// startup. For these reasons all slaves are programmed to share all
/// resources.
///
/// \bug Need to disable slave prefetch for now for all shared buffers until a
/// mode bit gets added too the PBA logic.  Dealt with using
/// ATTR_PROC_EC_PBA_PREFETCH_ENABLE.
///
/// \bug The dis_slvmatch_order bit is going away

fapi::ReturnCode
pba_slave_setup_init(const Target& i_target)
{
    pba_mode_t          pm;
    pba_slvctln_t       ps;
    fapi::ReturnCode    rc;
    uint32_t            l_rc = 0;
    ecmdDataBufferBase  data(64);
    
    uint8_t             ec_allows_pba_prefetch_enable;

    do
    {       
        rc = FAPI_ATTR_GET(ATTR_PROC_EC_PBA_PREFETCH_ENABLE,
                                   &i_target,
                                   ec_allows_pba_prefetch_enable);
        if(rc)
        {
     	    FAPI_ERR("Error querying Chip EC feature: "
                     "ATTR_PROC_EC_PBA_PREFETCH_ENABLE");
            break;
        }

        FAPI_INF("PBA prefetch is %sallowed",
                 (ec_allows_pba_prefetch_enable ? "" : "NOT "));

        // Set the PBA_MODECTL register. It's not yet clear how PBA BCE
        // transaction size will affect performance - for now we go with the
        // largest size.  The HTM marker space is enabled and configured. Slave
        // fairness is enabled. The setting 'dis_slvmatch_order' ensures that PBA
        // will correctly flush write data before allowing a read of the same
        // address from a different master on a different slave.  The second write
        // buffer is enabled.
        // prepare the value to be set:
        pm.value = 0;
        pm.fields.pba_region = PBA_OCI_REGION;
        pm.fields.bcde_ocitrans = PBA_BCE_OCI_TRANSACTION_64_BYTES;
        pm.fields.bcue_ocitrans = PBA_BCE_OCI_TRANSACTION_64_BYTES;
        pm.fields.en_marker_ack = 1;
        pm.fields.oci_marker_space = (PBA_OCI_MARKER_BASE >> 16) & 0x7;
        pm.fields.en_slave_fairness = 1;
        pm.fields.dis_slvmatch_order = 1;
        pm.fields.en_second_wrbuf = 1;

        l_rc = data.setDoubleWord(0, pm.value);
        if (l_rc)
        {
           rc.setEcmdError(l_rc);
           return rc;
        }

        // write the prepared value
        rc = fapiPutScom(i_target, PBA_MODE_0x00064000 , data);
        if (rc)
        {
           FAPI_ERR("fapiPutScom( PBA_MODE_0x00064000 ) failed. With rc = 0x%x", (uint32_t)rc);
           break;
        }

        // Slave 0 (PORE-GPE).  This is a read/write slave. We only do 'static'
        // setup here. Dynamic setup will be done by each GPE program that needs
        // to access mainstore, before issuing any trasactions targeting the PBA
        // bridge.

        //   pba_slave_reset(PBA_SLAVE_PORE_GPE);
        ps.value = 0;
        ps.fields.enable = 1;
        ps.fields.mid_match_value = OCI_MASTER_ID_PORE_GPE;
        ps.fields.mid_care_mask = 0x7;
        ps.fields.buf_alloc_a = 1;
        ps.fields.buf_alloc_b = 1;
        ps.fields.buf_alloc_c = 1;
        ps.fields.buf_alloc_w = 1;

        // Turn off buffer sharing for EC that don't allow prefetch
        // GPEs only use buffer A
        if (!ec_allows_pba_prefetch_enable)
        {
            ps.fields.buf_alloc_b = 0;
            ps.fields.buf_alloc_c = 0;
        }

        l_rc = data.setDoubleWord(0, ps.value);
        if (l_rc)
        {
           FAPI_ERR("data.setDoubleWord ( ) failed. With rc = 0x%x", (uint32_t)l_rc);   
           rc.setEcmdError(l_rc);
           break;
        } // end if

        rc = fapiPutScom(i_target, PBA_SLVCTL0_0x00064004 , data);
        if (rc)
        {
           FAPI_ERR("fapiPutScom( PBA_SLVCTL0_0x00064004 ) failed. With rc = 0x%x", (uint32_t)rc);
           break;
        }

        // Slave 1 (405 ICU/DCU).  This is a read/write slave.  Write gethering is
        // allowed, but with the shortest possible timeout. This slave is
        // effectively disabled soon after IPL.

        //    pba_slave_reset(PBA_SLAVE_OCC);
        ps.value = 0;
        ps.fields.enable = 1;
        ps.fields.mid_match_value = OCI_MASTER_ID_OCC_ICU & OCI_MASTER_ID_OCC_DCU;
        ps.fields.mid_care_mask = OCI_MASTER_ID_OCC_ICU & OCI_MASTER_ID_OCC_DCU;
        ps.fields.read_ttype = PBA_READ_TTYPE_CL_RD_NC;
        ps.fields.read_prefetch_ctl = PBA_READ_PREFETCH_NONE;
        ps.fields.write_ttype = PBA_WRITE_TTYPE_DMA_PR_WR;
        ps.fields.wr_gather_timeout = PBA_WRITE_GATHER_TIMEOUT_2_PULSES;
        ps.fields.buf_alloc_a = 1;
        ps.fields.buf_alloc_b = 1;
        ps.fields.buf_alloc_c = 1;
        ps.fields.buf_alloc_w = 1;

        // Turn off buffer sharing for EC that don't allow prefetch
        // All non-GPEs can share buffer B
        if (!ec_allows_pba_prefetch_enable)
        {
            ps.fields.buf_alloc_a = 0;
            ps.fields.buf_alloc_c = 0;
        }

        l_rc = data.setDoubleWord(0, ps.value);
        if (l_rc)
        {
           FAPI_ERR("data.setDoubleWord ( ) failed. With rc = 0x%x", (uint32_t)l_rc);
           rc.setEcmdError(l_rc);
           break;
        } // end if

        rc = fapiPutScom(i_target, PBA_SLVCTL1_0x00064005 , data);
        if (rc)
        {
           FAPI_ERR("fapiPutScom( PBA_SLVCTL1_0x00064005 ) failed. With rc = 0x%x", (uint32_t)rc);
           break;
        }

    /*  Removed as this is done by p8_set_port_bar.C for the SLW-used path
        through the PBA

        // Slave 2 (PORE-SLW).  This is a read/write slave. Write gathering is
        // allowed, but with the shortest possible timeout.  The slave is set up
        // to allow normal reads and writes at initialization.  The 24x7 code may
        // reprogram this slave for IMA writes using special code sequences that
        // restore normal DMA writes after each IMA sequence.

        //   pba_slave_reset(PBA_SLAVE_PORE_SLW);
        ps.value = 0;
        ps.fields.enable = 1;
        ps.fields.mid_match_value = OCI_MASTER_ID_PORE_SLW;
        ps.fields.mid_care_mask = 0x7;
        ps.fields.read_ttype = PBA_READ_TTYPE_CL_RD_NC;
        ps.fields.read_prefetch_ctl = PBA_READ_PREFETCH_NONE;
        ps.fields.write_ttype = PBA_WRITE_TTYPE_DMA_PR_WR;
        ps.fields.wr_gather_timeout = PBA_WRITE_GATHER_TIMEOUT_2_PULSES;
        ps.fields.buf_alloc_a = 1;
        ps.fields.buf_alloc_b = 1;
        ps.fields.buf_alloc_c = 1;
        ps.fields.buf_alloc_w = 1;
        l_rc = data.setDoubleWord(0, ps.value);
        if (l_rc)
        {
           FAPI_ERR("data.setDoubleWord ( ) failed. With rc = 0x%x", (uint32_t)l_rc);
           rc.setEcmdError(l_rc);
           break;
        } // end if

        rc = fapiPutScom(i_target, PBA_SLVCTL2_0x00064006 , data);
        if (rc)
        {
           FAPI_ERR("fapiPutScom( PBA_SLVCTL2_0x00064006 ) failed. With rc = 0x%x", (uint32_t)rc);
           break;
        }
    */

        // Slave 3 (OCB).  This is a read/write slave. Write gathering is
        // allowed, but with the shortest possible timeout.

        //   pba_slave_reset(PBA_SLAVE_OCB);
        ps.value = 0;
        ps.fields.enable = 1;
        ps.fields.mid_match_value = OCI_MASTER_ID_OCB;
        ps.fields.mid_care_mask = 0x7;
        ps.fields.read_ttype = PBA_READ_TTYPE_CL_RD_NC;
        ps.fields.read_prefetch_ctl = PBA_READ_PREFETCH_NONE;
        ps.fields.write_ttype = PBA_WRITE_TTYPE_DMA_PR_WR;
        ps.fields.wr_gather_timeout = PBA_WRITE_GATHER_TIMEOUT_2_PULSES;
        ps.fields.buf_alloc_a = 1;
        ps.fields.buf_alloc_b = 1;
        ps.fields.buf_alloc_c = 1;
        ps.fields.buf_alloc_w = 1;

        // Turn off buffer sharing for EC that don't allow prefetch
        // All non-GPEs can share buffer B
        if (!ec_allows_pba_prefetch_enable)
        {
            ps.fields.buf_alloc_a = 0;
            ps.fields.buf_alloc_c = 0;
        }

        l_rc = data.setDoubleWord(0, ps.value);
        if (l_rc)
        {
           FAPI_ERR("data.setDoubleWord ( ) failed. With rc = 0x%x", (uint32_t)l_rc);   
           rc.setEcmdError(l_rc); 
           break;
        } // end if

        rc = fapiPutScom(i_target, PBA_SLVCTL3_0x00064007 , data);
        if (rc)
        {
           FAPI_ERR("fapiPutScom( PBA_SLVCTL3_0x00064007 ) failed. With rc = 0x%x", (uint32_t)rc); 
           break;
        }
    } while(0);

    return rc;
}  // end pba_slave_setup_init



// ************************************************************************************************
// **************************************************** pba_slave_reset ***************************
// Walk each slave to hit the respective reset and then poll for completion
fapi::ReturnCode
pba_slave_reset(const Target& i_target)
{
    fapi::ReturnCode    rc;
    ecmdDataBufferBase  data(64);
    bool                poll_failure = false;
    uint32_t            p;
    
    uint8_t             ec_has_pba_slvrest_bug = 0;
    uint8_t             attr_mpipl = 0;

    do
    {
        rc = FAPI_ATTR_GET(ATTR_CHIP_EC_FEATURE_HW_BUG_PBASLVRESET,
                                   &i_target,
                                   ec_has_pba_slvrest_bug);
        if(rc)
        {
     	    FAPI_ERR("Error querying Chip EC feature: "
                     "ATTR_CHIP_EC_FEATURE_HW_BUG_PBASLVRESET");
            break;
        }
        
        rc = FAPI_ATTR_GET(ATTR_IS_MPIPL, NULL, attr_mpipl);
        if(rc)
        {
     	    FAPI_ERR("Error querying attribute ATTR_IS_MPIPL");
            break;
        }

        for (int s=0; s<= 3; s++)
        {

            // Skip Slave 2 has this is handled in p8_set_pore_bars.C as part
            // of the SLW setup
            if (s == 2)
            {
                continue;
            }

            FAPI_INF("Reseting PBA Slave %x", s);
            poll_failure = true;
            for (p=0; p<MAX_PBA_RESET_POLLS; p++)
            {

                // Set the reset for the selected slave
                data.setDoubleWord(0, PBA_SLVRESETs[s]);

                rc = fapiPutScom(i_target, PBA_SLVRST_0x00064001 , data);
                if (rc)
                {
                     FAPI_ERR("fapiPutScom( PBA_SLVRST_0x00064001 ) failed. With rc = 0x%x", (uint32_t)rc);
                     break ;
                }
                
                // Due to HW228485, skip the check of the in-progress bits for MPIPL 
                // (after the PBA channels have been used at runtime) as they
                // are unreliable in Murano 1.x.
                if (attr_mpipl && ec_has_pba_slvrest_bug)
                {
                   FAPI_INF("PBA Reset Polling being skipped due to MPIPL on a chip with PBA reset bug");
                   poll_failure = false;
                   continue;
                }
                
                // Read the reset register to check for reset completion
                rc = fapiGetScom(i_target, PBA_SLVRST_0x00064001 , data);
                if (rc)
                {
                     FAPI_ERR("fapiGetPutScom( PBA_SLVRST_0x00064001 ) failed. With rc = 0x%x", (uint32_t)rc);
                     break;
                }
                FAPI_DBG("Slave %x reset poll data = 0x%016llX", s, data.getDoubleWord(0));

                // If slave reset in progress, wait and then poll
                if (data.isBitClear(4+s))
                {
                    FAPI_INF("PBA Reset complete for Slave %d", s);
                    poll_failure = false;
                    break;
                }
                else
                {
                    rc = fapiDelay(PBA_RESET_POLL_DELAY*1000, 200000);   // In microseconds
                    if (rc)
                    {
                         FAPI_ERR("fapiDelay failed. With rc = 0x%x", (uint32_t)rc);
                         break;
                    }
                }
            }

            // Error exit from above loop
            if (!rc.ok())
            {
                 break;
            }

            if (poll_failure)
            {
                 FAPI_ERR("PBA Slave Reset Timout");
                 const fapi::Target & CHIP = i_target;
                 const uint64_t& POLLCOUNT = (uint64_t)p;
                 const uint64_t& SLAVENUM  = (uint64_t)s;
                 const uint64_t& PBASLVREG = data.getDoubleWord(0);
                 FAPI_SET_HWP_ERROR(rc, RC_PMPROC_PBA_SLAVE_RESET_TIMEOUT);
                 break;
            }

            // Check if the slave is still actually busy.  Consider whether this should be polled
            if (data.isBitSet(8+s))
            {
                FAPI_ERR("Slave %x still busy after reset", s);
                const fapi::Target & CHIP = i_target;
                const uint64_t& POLLCOUNT = (uint64_t)p;
                const uint64_t& SLAVENUM  = (uint64_t)s;
                const uint64_t& PBASLVREG = data.getDoubleWord(0);
                FAPI_SET_HWP_ERROR(rc, RC_PMPROC_PBA_SLAVE_BUSY_AFTER_RESET);
                break;
            }
        }
    } while(0);

    return rc;

}  // end pba_slave_setup_reset

// ************************************************************************************************
// **************************************************** pba_bc_stop *******************************
// Stop the BCDE and BCUE and then poll for respective completion
fapi::ReturnCode
pba_bc_stop(const Target& i_target)
{
    fapi::ReturnCode    rc;
    uint32_t            e_rc = 0;
    ecmdDataBufferBase  data(64);
    uint64_t            address = 0;
    bool                bcde_stop_complete = false;
    bool                bcue_stop_complete = false;
    uint32_t            p;
    
    
    do
    {
       
        FAPI_INF("Stop the  BCDE and BCUE");
        address = PBA_BCDE_CTL_0x00064010;

        e_rc |= data.flushTo0();
        e_rc |= data.setBit(0);      // Bit 0: BCDE_CTL_STOP
        if (e_rc)
        {
            rc.setEcmdError(e_rc);
            break;
        }
        FAPI_INF("\tStopping BCDE addr=0x%08llX, value=0x%16llX, Target = %s",
                            address,
                            data.getDoubleWord(0),
                            i_target.toEcmdString());
        rc = fapiPutScom(i_target, address, data);
        if (!rc.ok())
        {
            FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            address,
                            i_target.toEcmdString());
            break;
        }

        address = PBA_BCUE_CTL_0x00064015;

        e_rc |= data.flushTo0();
        e_rc |= data.setBit(0);      // Bit 0: BCUE_CTL_STOP
        if (e_rc)
        {
            rc.setEcmdError(e_rc);
             break;
        }
        FAPI_INF("\tStopping BCUE addr=0x%08llX, value=0x%16llX, Target = %s",
                            address,
                            data.getDoubleWord(0),
                            i_target.toEcmdString());
        rc = fapiPutScom(i_target, address, data);
        if (!rc.ok())
        {
            FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            address,
                            i_target.toEcmdString());
            break;
        }
        
        // Stopped for the BC engines is defined as the Running bit is clear
        //
        // The Stopped, Done and Error bits may also be on if the BCE happened
        // stop before the requested transfer was complete.  However, done of 
        // these indicators are relevent as the OCC is being reset anyway.
                
        for (p=0; p<MAX_PBA_BC_STOP_POLLS; p++)
        {
        
               
            // Read the BCDE Status register to check for a stopped condition          
            rc = fapiGetScom(i_target, PBA_BCDE_STAT_0x00064012 , data);
            if (rc)
            {
                 FAPI_ERR("fapiGetPutScom( PBA_BCDE_STAT_0x00064012 ) failed. With rc = 0x%x", (uint32_t)rc);
                 break;
            }
            FAPI_DBG("BCDE Status poll data = 0x%016llX", data.getDoubleWord(0));
            
           
            if (data.isBitClear(PBA_BC_STAT_RUNNING))
            {
                FAPI_INF("BCDE Running Bit is clear to indicate the stop condition");
                bcde_stop_complete = true;
            }
            
            
            // Read the BCUE Status register to check for stop condition
            rc = fapiGetScom(i_target, PBA_BCUE_STAT_0x00064017 , data);
            if (rc)
            {
                 FAPI_ERR("fapiGetPutScom( PBA_BCUE_STAT_0x00064017 ) failed. With rc = 0x%x", (uint32_t)rc);
                 break;
            }
            FAPI_DBG("BCUE Status poll data = 0x%016llX", data.getDoubleWord(0));
            
            if (data.isBitClear(PBA_BC_STAT_RUNNING))
            {
                FAPI_INF("BCUE Running Bit is clear to indicate the stop condition");
                bcue_stop_complete = true;
            }                        
                                    
            // If both engines are stopped , exit polling loop 
            if (bcde_stop_complete && bcue_stop_complete)
            {                
                break;         
            }                   
            else
            {                
                rc = fapiDelay(MAX_PBA_BC_STOP_POLLS*1000, 2000000);   // In microseconds
                if (rc)
                {
                     FAPI_ERR("fapiDelay failed. With rc = 0x%x", (uint32_t)rc);
                     break;
                }
            }
        } // polling loop

        // Error exit from above loop
        if (!rc.ok())
        {
             break;
        }

        if (!bcde_stop_complete)
        {
             FAPI_ERR("PBA BCDE Stop Timout");
             const fapi::Target & CHIP = i_target;
             const uint32_t     & POLLCOUNT = MAX_PBA_BC_STOP_POLLS;
             const uint32_t     & POLLVALUE = MAX_PBA_BC_STOP_POLLS;
	         FAPI_SET_HWP_ERROR(rc, RC_PROCPM_PBA_BCDE_STOP_TIMEOUT);
             break;
        }
        if (!bcue_stop_complete)
        {
             FAPI_ERR("PBA BCDE Stop Timout");
             const fapi::Target & CHIP = i_target;
             const uint32_t     & POLLCOUNT = MAX_PBA_BC_STOP_POLLS;
             const uint32_t     & POLLVALUE = MAX_PBA_BC_STOP_POLLS;
	         FAPI_SET_HWP_ERROR(rc, RC_PROCPM_PBA_BCUE_STOP_TIMEOUT);
             break;
        }
        
        FAPI_INF("Clear the  BCDE and BCUE stop bits via control register clearing");
        
        e_rc |= data.flushTo0();
        if (e_rc)
        {
            rc.setEcmdError(e_rc);
            break;
        }
        
        address = PBA_BCDE_CTL_0x00064010;
        FAPI_INF("\tClear BCDE stop bit addr=0x%08llX, value=0x%16llX, Target = %s",
                            address,
                            data.getDoubleWord(0),
                            i_target.toEcmdString());
        rc = fapiPutScom(i_target, address, data);
        if (!rc.ok())
        {
            FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            address,
                            i_target.toEcmdString());
            break;
        }

        address = PBA_BCUE_CTL_0x00064015;        
        FAPI_INF("\tClear BCUE stop bit addr=0x%08llX, value=0x%16llX, Target = %s",
                            address,
                            data.getDoubleWord(0),
                            i_target.toEcmdString());
        rc = fapiPutScom(i_target, address, data);
        if (!rc.ok())
        {
            FAPI_ERR("fapiPutScom(addr=0x%08llX) failed, Target = %s",
                            address,
                            i_target.toEcmdString());
            break;
        }
                
    } while(0);

    return rc;

}  // end pba_bcde_stop



} //end extern C

