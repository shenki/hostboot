# IBM_PROLOG_BEGIN_TAG
# This is an automatically generated prolog.
#
# $Source: src/usr/hwpf/hwp/core_activate/makefile $
#
# OpenPOWER HostBoot Project
#
# Contributors Listed Below - COPYRIGHT 2012,2014
# [+] International Business Machines Corp.
#
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
# implied. See the License for the specific language governing
# permissions and limitations under the License.
#
# IBM_PROLOG_END_TAG
ROOTPATH = ../../../../..

MODULE = core_activate

##      support for Targeting and fapi
EXTRAINCDIR += ${ROOTPATH}/src/include/usr/ecmddatabuffer
EXTRAINCDIR += ${ROOTPATH}/src/include/usr/hwpf/fapi
EXTRAINCDIR += ${ROOTPATH}/src/include/usr/hwpf/plat
EXTRAINCDIR += ${ROOTPATH}/src/include/usr/hwpf/hwp

## pointer to common HWP files
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/include

##  NOTE: add the base istep dir here.
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate

##  Include sub dirs
##  NOTE: add a new EXTRAINCDIR when you add a new HWP
##  EXAMPLE:
##  EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/<HWP_dir>
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_prep_master_winkle
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_stop_deadman_timer
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_switch_cfsim
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_post_winkle
##  @todo   RTC Story 51709 and Story 51711 : revisit this later -
##  proc_set_pore_bar is used by multiple isteps, so it should be in its own
##  "utility" library
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/build_winkle_images/p8_set_pore_bar
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/switch_rec_attn
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_check_slw_done
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/build_winkle_images/p8_block_wakeup_intr
EXTRAINCDIR += ${ROOTPATH}/src/usr/hwpf/hwp/activate_powerbus/proc_pcie_slot_power
EXTRAINCDIR += ${ROOTPATH}/src/include/usr/hwpf/hwp/utility_procedures


##  NOTE: add new object files when you add a new HWP
OBJS += core_activate.o
OBJS += proc_prep_master_winkle.o
OBJS += proc_stop_deadman_timer.o
OBJS += proc_switch_cfsim.o
OBJS += proc_switch_rec_attn.o
OBJS += cen_switch_rec_attn.o
OBJS += proc_post_winkle.o
OBJS += proc_check_slw_done.o



##  NOTE: add a new directory onto the vpaths when you add a new HWP
##  EXAMPLE:
#   VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/<HWP_dir>
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_prep_master_winkle
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_stop_deadman_timer
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_switch_cfsim
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/switch_rec_attn
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_post_winkle
VPATH += ${ROOTPATH}/src/usr/hwpf/hwp/core_activate/proc_check_slw_done

include ${ROOTPATH}/config.mk

