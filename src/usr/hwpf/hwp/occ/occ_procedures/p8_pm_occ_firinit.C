/* IBM_PROLOG_BEGIN_TAG                                                   */
/* This is an automatically generated prolog.                             */
/*                                                                        */
/* $Source: src/usr/hwpf/hwp/occ/occ_procedures/p8_pm_occ_firinit.C $     */
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

// $Id: p8_pm_occ_firinit.C,v 1.22 2014/07/09 14:49:32 daviddu Exp $
// $Source: /afs/awd/projects/eclipz/KnowledgeBase/.cvsroot/eclipz/chips/p8/working/procedures/ipl/fapi/p8_pm_occ_firinit.C,v $
//------------------------------------------------------------------------------
// *! (C) Copyright International Business Machines Corp. 2011
// *! All Rights Reserved -- Property of IBM
// *! ***  ***
//------------------------------------------------------------------------------
// *! OWNER NAME: Jim Yacynych         Email: jimyac@us.ibm.com
// *!
/// \file p8_pm_occ_firinit.C
/// \brief Configures the OCC LFIR Mask and Action

/// \todo
///
/// Procedure Prereq:
///   o System clocks are running
/// \endverbatim
///
/// buildfapiprcd p8_pm_occ_firinit.C
//------------------------------------------------------------------------------

// ----------------------------------------------------------------------
// Includes
// ----------------------------------------------------------------------
#include <fapi.H>
#include "p8_scom_addresses.H"
#include "p8_pm_occ_firinit.H"

extern "C" {

using namespace fapi;

// ----------------------------------------------------------------------
// Constant definitions
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Macro definitions
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Global variables
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Function prototypes
// ----------------------------------------------------------------------

// ----------------------------------------------------------------------
// Function definitions
// ----------------------------------------------------------------------

/// \param[in]  i_target            => Chip Target

/// \retval FAPI_RC_SUCCESS
/// \retval ERROR
fapi::ReturnCode
p8_pm_occ_firinit(const fapi::Target& i_target , uint32_t mode)
{
    fapi::ReturnCode    rc;
    ecmdDataBufferBase  fir(64);
    ecmdDataBufferBase  action_0(64);
    ecmdDataBufferBase  action_1(64);
    ecmdDataBufferBase  mask(64);
    ecmdDataBufferBase  data(64);
    uint32_t            e_rc = 0;
    uint8_t             ce_fir_disable = 0;;
    uint64_t            attr_pm_occ_lfir_mask;
    uint8_t             attr_pm_firinit_done_once_flag;

    FAPI_DBG("Executing p8_pm_occ_firinit  ....");
    
    do
    {
       if (mode == PM_RESET)
       {

            // -----------
            // SW260003
            // -----------
            rc = FAPI_ATTR_GET(ATTR_PM_FIRINIT_DONE_ONCE_FLAG, &i_target, attr_pm_firinit_done_once_flag);
            if (!rc.ok()) {
              FAPI_ERR("fapiGetAttribute of ATTR_PM_FIRINIT_DONE_ONCE_FLAG failed.");
              break;
            }
            if (attr_pm_firinit_done_once_flag == 1) { 
                rc = fapiGetScom(i_target, OCC_LFIR_MASK_0x01010803, data );
                if (!rc.ok())
                {
                        FAPI_ERR("fapiGetScom(OCC_LFIR_MASK_0x01010803) failed.");
                        break;
                }
                attr_pm_occ_lfir_mask = data.getDoubleWord(0);
                rc = FAPI_ATTR_SET(ATTR_PM_OCC_LFIR_MASK, &i_target, attr_pm_occ_lfir_mask);
                if (!rc.ok()) {
                  FAPI_ERR("fapiSetAttribute of ATTR_PM_OCC_LFIR_MASK failed");
                  break;
                }
            }
 
            e_rc  = mask.flushTo1();
            if (e_rc)
            {
                rc.setEcmdError(e_rc);
                break;
            }

            // ------------
            // OCC_FIR_MASK
            // ------------
            rc = fapiPutScom(i_target, OCC_LFIR_MASK_0x01010803, mask );
            if (!rc.ok())
            {
	            FAPI_ERR("fapiPutScom(OCC_LFIR_MASK_0x01010803) failed.");
	            break;
            }
        }
        else
        {

            // Read attributes to determine mask modifications        
            rc = FAPI_ATTR_GET(ATTR_CHIP_EC_FEATURE_OCC_CE_FIR_DISABLE,
               &i_target,
               ce_fir_disable);
            if(rc)
            {
     		    FAPI_ERR("Error querying Chip EC feature: "
                         "ATTR_CHIP_EC_FEATURE_OCC_CE_FIR_DISABLE");
                break;
            }
           
            FAPI_INF("OCC Correctable error FIRs are %s",
                     (ce_fir_disable ? "MASKED" : "ENABLED"));
                          
            // Clear the FIR
            e_rc |= fir.flushTo0();
            
            // make action default be RECOV_ATTN - "01"
            e_rc |= action_0.flushTo0();
            e_rc |= action_1.flushTo1();

            // make mask default be unmasked - "0"
            e_rc |= mask.flushTo0()    ;
            if (e_rc)
            {
                rc.setEcmdError(e_rc);
                break;
            }

            // ------------------------------------------------------------------------
            // set the action and mask for the OCC LFIR bits using the following macros
            // ------------------------------------------------------------------------
            //   Action0/1 Setting Macros - 4 possible settings
            //     SET_CHECK_STOP(b) - sets action0/1 to "00" for LFIR bit b
            //     SET_RECOV_ATTN(b) - sets action0/1 to "01" for LFIR bit b
            //     SET_RECOV_INTR(b) - sets action0/1 to "10" for LFIR bit b
            //     SET_MALF_ALERT(b) - sets action0/1 to "11" for LFIR bit b
            //
            //  Mask Setting Macro
            //   SET_FIR_MASKED(b) - sets mask to '1' for LFIR bit b
            // ------------------------------------------------------------------------

            SET_MALF_ALERT(0);  SET_FIR_MASKED(0);        //  0 = occ_fw0
            SET_MALF_ALERT(1);  SET_FIR_MASKED(1);        //  1 = occ_fw1
            SET_MALF_ALERT(2);  SET_FIR_MASKED(2);        //  2 = occ_fw2
            SET_MALF_ALERT(3);  SET_FIR_MASKED(3);        //  3 = occ_fw3
            SET_MALF_ALERT(4);  SET_FIR_MASKED(4);        //  4 = pmc_pore_sw_malf
            SET_MALF_ALERT(5);  SET_FIR_MASKED(5);        //  5 = pmc_occ_hb_malf

            SET_RECOV_ATTN(6);  SET_FIR_MASKED(6);        //  6 = pore_gpe0_fatal_err
            SET_RECOV_ATTN(7);  SET_FIR_MASKED(7);        //  7 = pore_gpe1_fatal_err
            SET_RECOV_INTR(8);                            //  8 = ocb_error
            SET_RECOV_INTR(9);                            //  9 = srt_ue
            SET_RECOV_ATTN(10);                           //  10 = srt_ce
            if (ce_fir_disable)
            {
                SET_FIR_MASKED(10);
            }
            SET_RECOV_INTR(11);                           //  11 = srt_read_error
            SET_RECOV_INTR(12);                           //  12 = srt_write_error
            SET_RECOV_INTR(13);                           //  13 = srt_dataout_perr
            SET_RECOV_INTR(14);                           //  14 = srt_oci_write_data_parity
            SET_RECOV_INTR(15);                           //  15 = srt_oci_be_parity_err
            SET_RECOV_INTR(16);                           //  16 = srt_oci_addr_parity_err
            SET_RECOV_ATTN(17); SET_FIR_MASKED(17);       //  17 = pore_sw_error_err                        
            SET_RECOV_ATTN(18); SET_FIR_MASKED(18);       //  18 = pore_gpe0_error_err                      
            SET_RECOV_ATTN(19); SET_FIR_MASKED(19);       //  19 = pore_gpe1_error_err                      
            SET_RECOV_ATTN(20); SET_FIR_MASKED(20);       //  20 = external_trap                            
            SET_RECOV_ATTN(21); SET_FIR_MASKED(21);       //  21 = ppc405_core_reset                        
            SET_RECOV_ATTN(22); SET_FIR_MASKED(22);       //  22 = ppc405_chip_reset                        
            SET_RECOV_ATTN(23); SET_FIR_MASKED(23);       //  23 = ppc405_system_reset                      
            SET_RECOV_ATTN(24); SET_FIR_MASKED(24);       //  24 = ppc405_dbgmsrwe                          
            SET_RECOV_ATTN(25); SET_FIR_MASKED(25);       //  25 = ppc405_dbgstopack                        
            SET_RECOV_ATTN(26);                           //  26 = ocb_db_oci_timeout
            SET_RECOV_ATTN(27);                           //  27 = ocb_db_oci_read_data_parity
            SET_RECOV_ATTN(28);                           //  28 = ocb_db_oci_slave_error
            SET_RECOV_ATTN(29);                           //  29 = ocb_pib_addr_parity_err
            SET_RECOV_ATTN(30);                           //  30 = ocb_db_pib_data_parity_err
            SET_RECOV_ATTN(31); SET_FIR_MASKED(31);       //  31 = ocb_idc0_error
            SET_RECOV_ATTN(32); SET_FIR_MASKED(32);       //  32 = ocb_idc1_error
            SET_RECOV_ATTN(33); SET_FIR_MASKED(33);       //  33 = ocb_idc2_error
            SET_RECOV_ATTN(34); SET_FIR_MASKED(34);       //  34 = ocb_idc3_error
            SET_RECOV_INTR(35);                           //  35 = srt_fsm_err
            SET_RECOV_ATTN(36); SET_FIR_MASKED(36);       //  36 = jtagacc_err
            SET_RECOV_ATTN(37); SET_FIR_MASKED(37);       //  37 = spare_err_37
            SET_RECOV_INTR(38);                           //  38 = c405_ecc_ue
            SET_RECOV_ATTN(39);                           //  39 = c405_ecc_ce
            if (ce_fir_disable)
            {
                SET_FIR_MASKED(39);
            }
            SET_RECOV_ATTN(40); SET_FIR_MASKED(40);       //  40 = c405_oci_machinecheck
            SET_RECOV_ATTN(41); SET_FIR_MASKED(41);       //  41 = sram_spare_direct_error0
            SET_RECOV_ATTN(42); SET_FIR_MASKED(42);       //  42 = sram_spare_direct_error1
            SET_RECOV_ATTN(43); SET_FIR_MASKED(43);       //  43 = sram_spare_direct_error2
            SET_RECOV_ATTN(44); SET_FIR_MASKED(44);       //  44 = sram_spare_direct_error3
            SET_RECOV_ATTN(45); SET_FIR_MASKED(45);       //  45 = slw_ocislv_err (SW250823)
            SET_RECOV_INTR(46);                           //  46 = gpe_ocislv_err
            SET_RECOV_INTR(47);                           //  47 = ocb_ocislv_err
            SET_RECOV_ATTN(48); SET_FIR_MASKED(48);       //  48 = c405icu_m_timeout
            SET_RECOV_ATTN(49); SET_FIR_MASKED(49);       //  49 = c405dcu_m_timeout
            SET_RECOV_ATTN(50);                           //  50 = occ_complex_fault_safe         
            SET_RECOV_ATTN(51); SET_FIR_MASKED(51);       //  51 = spare_fir                   
            SET_RECOV_ATTN(52); SET_FIR_MASKED(52);       //  52 = spare_fir                   
            SET_RECOV_ATTN(53); SET_FIR_MASKED(53);       //  53 = spare_fir                   
            SET_RECOV_ATTN(54); SET_FIR_MASKED(54);       //  54 = spare_fir                   
            SET_RECOV_ATTN(55); SET_FIR_MASKED(55);       //  55 = spare_fir                   
            SET_RECOV_ATTN(56); SET_FIR_MASKED(56);       //  56 = spare_fir                   
            SET_RECOV_ATTN(57); SET_FIR_MASKED(57);       //  57 = spare_fir                   
            SET_RECOV_ATTN(58); SET_FIR_MASKED(58);       //  58 = spare_fir                   
            SET_RECOV_ATTN(59); SET_FIR_MASKED(59);       //  59 = spare_fir                   
            SET_RECOV_ATTN(60); SET_FIR_MASKED(60);       //  60 = spare_fir                   
            SET_RECOV_ATTN(61); SET_FIR_MASKED(61);       //  61 = spare_fir                   
            SET_RECOV_ATTN(62);                           //  62 = fir_parity_err_dup
            SET_RECOV_ATTN(63);                           //  63 = fir_parity_err

            if (e_rc)
            {
                rc.setEcmdError(e_rc);
                break;
            }
            
            // ---------------
            // OCC_FIR - cleared
            // ---------------
            rc = fapiPutScom(i_target, OCC_LFIR_0x01010800, fir);
            if (!rc.ok())
            {
                FAPI_ERR("fapiPutScom(OCC_LFIR_0x01010800) failed.");
                 break;
            }
            

            FAPI_DBG(" action_0  => 0x%16llx ",  action_0.getDoubleWord(0));
            FAPI_DBG(" action_1  => 0x%16llx ",  action_1.getDoubleWord(0));
            FAPI_DBG(" mask      => 0x%16llx ",  mask.getDoubleWord(0));

            // ---------------
            // OCC_FIR_ACTION0
            // ---------------
            rc = fapiPutScom(i_target, OCC_LFIR_ACT0_0x01010806, action_0 );
            if (!rc.ok())
            {
                FAPI_ERR("fapiPutScom(OCC_LFIR_ACT0_0x01010806) failed.");
                break;
            }

            // ----------------
            // OCC_FIR_ACTION1
            // ----------------
            rc = fapiPutScom(i_target, OCC_LFIR_ACT1_0x01010807, action_1 );
            if (!rc.ok())
            {
                FAPI_ERR("fapiPutScom(OCC_LFIR_ACT1_0x01010807) failed.");
                    break;
            }

            // -----------
            // SW260003
            // -----------
            rc = FAPI_ATTR_GET(ATTR_PM_FIRINIT_DONE_ONCE_FLAG, &i_target, attr_pm_firinit_done_once_flag);
            if (!rc.ok()) {
              FAPI_ERR("fapiGetAttribute of ATTR_PM_FIRINIT_DONE_ONCE_FLAG failed.");
              break;
            }
            if (attr_pm_firinit_done_once_flag) {
                rc = FAPI_ATTR_GET(ATTR_PM_OCC_LFIR_MASK, &i_target, attr_pm_occ_lfir_mask);
                if (!rc.ok()) {
                  FAPI_ERR("fapiGetAttribute of ATTR_PM_OCC_LFIR_MASK failed.");
                  break;
                }
                e_rc |= data.setDoubleWord(0, attr_pm_occ_lfir_mask);
                e_rc |= mask.setOr(data, 0, 64);
                if (e_rc)
                {
                    rc.setEcmdError(e_rc);
                    break;
                }
            }

            // ------------
            // OCC_FIR_MASK
            // ------------
            rc = fapiPutScom(i_target, OCC_LFIR_MASK_0x01010803, mask );
            if (!rc.ok())
            {
                FAPI_ERR("fapiPutScom(OCC_LFIR_MASK_0x01010803) failed.");
                break;
            }
        }
    } while(0);
    return rc ;
} // end p8_pm_occ_firinit

} //end extern C
