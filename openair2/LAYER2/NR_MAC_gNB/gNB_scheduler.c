/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file gNB_scheduler.c
 * \brief gNB scheduler top level function operates on per subframe basis
 * \author  Navid Nikaein and Raymond Knopp, WEI-TAI CHEN
 * \date 2010 - 2014, 2018
 * \email: navid.nikaein@eurecom.fr, kroempa@gmail.com
 * \version 0.5
 * \company Eurecom, NTUST
 * @ingroup _mac

 */

#include "assertions.h"

#include "NR_MAC_COMMON/nr_mac_extern.h"
#include "NR_MAC_gNB/mac_proto.h"

#include "common/utils/LOG/log.h"
#include "common/utils/nr/nr_common.h"
#include "common/utils/LOG/vcd_signal_dumper.h"
#include "UTIL/OPT/opt.h"
#include "OCG.h"
#include "OCG_extern.h"

#include "RRC/NR/nr_rrc_extern.h"

//#include "LAYER2/MAC/pre_processor.c"
#include "pdcp.h"

#include "intertask_interface.h"

#include "executables/softmodem-common.h"
#include "nfapi/oai_integration/vendor_ext.h"
#include "executables/nr-softmodem.h"

#include <errno.h>
#include <string.h>

const uint8_t nr_rv_round_map[4] = { 0, 2, 3, 1 };
uint16_t nr_pdcch_order_table[6] = { 31, 31, 511, 2047, 2047, 8191 };
extern int num_delay;//add_yjn
uint8_t vnf_first_sched_entry = 1;

/* 
  name: get_last_ul_tti_req_ind
  input: current scheduler frame,slot
  function: used to get last ul_tti_req index
*/
//yjn两帧为一组（偶数帧，奇数帧）共40个时隙；该函数实现当前时隙的上一时隙（转化成组内时隙号后的）；
//若为偶数帧0时隙，则返回上一组的第39时隙；若为偶数帧，则返回上一时隙；若为奇数帧，则返回当前组的上一时隙；
int get_last_ul_tti_req_ind(gNB_MAC_INST * gNB, frame_t frame, sub_frame_t slot)       //add_yjn
{
  NR_ServingCellConfigCommon_t *scc = gNB->common_channels->ServingCellConfigCommon;
  const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing] + num_delay;     //  if 30khz, 40个时隙
  
  int index = 0;
  if (frame % 2 == 0 && slot == 0)
    index = num_slots - 1;  
  else if (frame % 2 == 0) 
    index = slot - 1;
  else if (frame % 2)
    index = slot + num_delay - 1;
  LOG_D(MAC,"drop_ul_tti index = %d \n", index);
  return index;
}

/* 
  name: get_future_ul_tti_req_ind
  input: current scheduler frame,slot
  function: used to get future ul_tti_req index
*/
//函数返回当前时隙的组内时隙号
int get_future_ul_tti_req_ind(frame_t frame, sub_frame_t slot)//add_yjn
{
  int index = frame % 2 ? slot + num_delay : slot;
  LOG_D(MAC,"drop_ul_tti index = %d \n", index);
  return index;  
}

void clear_nr_nfapi_information(gNB_MAC_INST * gNB,
                                int CC_idP,
                                frame_t frameP,
                                sub_frame_t slotP){  //调度的时隙为当前时隙加6个时隙，后续称为调度时隙
  //add_yjn
  NR_ServingCellConfigCommon_t *scc = gNB->common_channels->ServingCellConfigCommon;
  const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing] + num_delay;
  LOG_D(NR_MAC,"[yjn] clear_nr_nfapi_information && UL_tti_req_ahead_initialization\n");
  UL_tti_req_ahead_initialization(gNB, scc, num_slots, CC_idP);

  nfapi_nr_dl_tti_request_t    *DL_req = &gNB->DL_req[0];
  nfapi_nr_dl_tti_pdcch_pdu_rel15_t **pdcch = (nfapi_nr_dl_tti_pdcch_pdu_rel15_t **)gNB->pdcch_pdu_idx[CC_idP];
  int last_ind = get_last_ul_tti_req_ind(gNB, frameP, slotP);//add_yjn
  nfapi_nr_ul_tti_request_t    *future_ul_tti_req =
      &gNB->UL_tti_req_ahead[CC_idP][last_ind];           //add_yjn   //当前组内时隙的上一时隙的future_ul_tti_req的pdu被清空
  nfapi_nr_ul_dci_request_t    *UL_dci_req = &gNB->UL_dci_req[0];
  nfapi_nr_tx_data_request_t   *TX_req = &gNB->TX_req[0];

  gNB->pdu_index[CC_idP] = 0;

  DL_req[CC_idP].SFN                                   = frameP;
  DL_req[CC_idP].Slot                                  = slotP;
  DL_req[CC_idP].dl_tti_request_body.nPDUs             = 0;
  DL_req[CC_idP].dl_tti_request_body.nGroup            = 0;
  //DL_req[CC_idP].dl_tti_request_body.transmission_power_pcfich           = 6000;
  memset(pdcch, 0, sizeof(*pdcch) * MAX_NUM_CORESET);

  UL_dci_req[CC_idP].SFN                         = frameP;
  UL_dci_req[CC_idP].Slot                        = slotP;
  UL_dci_req[CC_idP].numPdus                     = 0;

  /* advance last round's future UL_tti_req to be ahead of current frame/slot */
  future_ul_tti_req->SFN = (slotP == 0 ? frameP + 1: frameP + 2) % 1024;  //add_yjn  //上一时隙
  LOG_D(NR_MAC, "In %s: UL_tti_req_ahead SFN.slot = %d.%d for slot %d \n", __FUNCTION__, future_ul_tti_req->SFN, future_ul_tti_req->Slot, (slotP + num_slots - 1) % num_slots);
  /* future_ul_tti_req->Slot is fixed! */
  future_ul_tti_req->n_pdus = 0;
  future_ul_tti_req->n_ulsch = 0;
  future_ul_tti_req->n_ulcch = 0;
  future_ul_tti_req->n_group = 0;
  /* UL_tti_req is a simple pointer into the current UL_tti_req_ahead, i.e.,
   * it walks over UL_tti_req_ahead in a circular fashion */  //gNB->UL_tti_req[CC_idP] = &gNB->UL_tti_req_ahead[CC_idP][slotP]; //add_yjn
   
  /*
  *  Since the ul_tti_req_ahead array is now extended from 20 to 40, 
  *  but the slot is always looped 0-20, special processing is done here
  */
  int future_ind = get_future_ul_tti_req_ind(frameP, slotP);  //add_yjn
  gNB->UL_tti_req[CC_idP] = &gNB->UL_tti_req_ahead[CC_idP][future_ind];  //add_yjn

  TX_req[CC_idP].Number_of_PDUs                  = 0;
}

// void clear_nr_nfapi_information(gNB_MAC_INST * gNB,
//                                 int CC_idP,
//                                 frame_t frameP,
//                                 sub_frame_t slotP){
//   NR_ServingCellConfigCommon_t *scc = gNB->common_channels->ServingCellConfigCommon;
//   const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing];

//   UL_tti_req_ahead_initialization(gNB, scc, num_slots, CC_idP);

//   nfapi_nr_dl_tti_request_t    *DL_req = &gNB->DL_req[0];
//   nfapi_nr_dl_tti_pdcch_pdu_rel15_t **pdcch = (nfapi_nr_dl_tti_pdcch_pdu_rel15_t **)gNB->pdcch_pdu_idx[CC_idP];
//   nfapi_nr_ul_tti_request_t    *future_ul_tti_req =
//       &gNB->UL_tti_req_ahead[CC_idP][(slotP + num_slots - 1) % num_slots];
//   nfapi_nr_ul_dci_request_t    *UL_dci_req = &gNB->UL_dci_req[0];
//   nfapi_nr_tx_data_request_t   *TX_req = &gNB->TX_req[0];

//   gNB->pdu_index[CC_idP] = 0;

//   DL_req[CC_idP].SFN                                   = frameP;
//   DL_req[CC_idP].Slot                                  = slotP;
//   DL_req[CC_idP].dl_tti_request_body.nPDUs             = 0;
//   DL_req[CC_idP].dl_tti_request_body.nGroup            = 0;
//   //DL_req[CC_idP].dl_tti_request_body.transmission_power_pcfich           = 6000;
//   memset(pdcch, 0, sizeof(*pdcch) * MAX_NUM_CORESET);

//   UL_dci_req[CC_idP].SFN                         = frameP;
//   UL_dci_req[CC_idP].Slot                        = slotP;
//   UL_dci_req[CC_idP].numPdus                     = 0;

//   /* advance last round's future UL_tti_req to be ahead of current frame/slot */
//   future_ul_tti_req->SFN = (slotP == 0 ? frameP : frameP + 1) % 1024;
//   LOG_D(NR_MAC, "In %s: UL_tti_req_ahead SFN.slot = %d.%d for slot %d \n", __FUNCTION__, future_ul_tti_req->SFN, future_ul_tti_req->Slot, (slotP + num_slots - 1) % num_slots);
//   /* future_ul_tti_req->Slot is fixed! */
//   future_ul_tti_req->n_pdus = 0;
//   future_ul_tti_req->n_ulsch = 0;
//   future_ul_tti_req->n_ulcch = 0;
//   future_ul_tti_req->n_group = 0;

//   /* UL_tti_req is a simple pointer into the current UL_tti_req_ahead, i.e.,
//    * it walks over UL_tti_req_ahead in a circular fashion */
//   gNB->UL_tti_req[CC_idP] = &gNB->UL_tti_req_ahead[CC_idP][slotP];

//   TX_req[CC_idP].Number_of_PDUs                  = 0;

// }

bool is_xlsch_in_slot(uint64_t bitmap, sub_frame_t slot) {
  if (slot>=64) return false; //quickfix for FR2 where there are more than 64 slots (bitmap to be removed)
  return (bitmap >> slot) & 0x01;
}

void gNB_dlsch_ulsch_scheduler(module_id_t module_idP,
                               frame_t frame,
                               sub_frame_t slot){

  protocol_ctxt_t   ctxt={0};
  PROTOCOL_CTXT_SET_BY_MODULE_ID(&ctxt, module_idP, ENB_FLAG_YES, NOT_A_RNTI, frame, slot,module_idP);

  gNB_MAC_INST *gNB = RC.nrmac[module_idP];
  NR_COMMON_channels_t *cc = gNB->common_channels;
  NR_ServingCellConfigCommon_t        *scc     = cc->ServingCellConfigCommon;

  if (slot==0 && (*scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0]>=257)) {
    //FR2
    const NR_TDD_UL_DL_Pattern_t *tdd = &scc->tdd_UL_DL_ConfigurationCommon->pattern1;
    AssertFatal(tdd,"Dynamic TDD not handled yet\n");
    const int nb_periods_per_frame = get_nb_periods_per_frame(tdd->dl_UL_TransmissionPeriodicity);
    // re-initialization of tdd_beam_association at beginning of frame
    for (int i=0; i<nb_periods_per_frame; i++)
      gNB->tdd_beam_association[i] = -1;
  }

  start_meas(&RC.nrmac[module_idP]->eNB_scheduler);
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_DLSCH_ULSCH_SCHEDULER,VCD_FUNCTION_IN);
  LOG_D(NR_MAC,"[yjn] NR_UL_indication in gNB_dlsch_ulsch_scheduler\n");
  pdcp_run(&ctxt);
  /* send tick to RLC and RRC every ms */
  if ((slot & ((1 << *scc->ssbSubcarrierSpacing) - 1)) == 0) {
    void nr_rlc_tick(int frame, int subframe);
    void nr_pdcp_tick(int frame, int subframe);
    nr_rlc_tick(frame, slot >> *scc->ssbSubcarrierSpacing);
    nr_pdcp_tick(frame, slot >> *scc->ssbSubcarrierSpacing);
    nr_rrc_trigger(&ctxt, 0 /*CC_id*/, frame, slot >> *scc->ssbSubcarrierSpacing);
  }

  for (int CC_id = 0; CC_id < MAX_NUM_CCs; CC_id++) {
    //mbsfn_status[CC_id] = 0;

    // clear vrb_maps
    memset(cc[CC_id].vrb_map, 0, sizeof(uint16_t) * MAX_BWP_SIZE);
    // clear last scheduled slot's content (only)!
    const int num_slots = nr_slots_per_frame[*scc->ssbSubcarrierSpacing] + num_delay;
    // const int last_slot = (slot + num_slots - 1) % num_slots;
    // uint16_t *vrb_map_UL = cc[CC_id].vrb_map_UL;
    // memcpy(&vrb_map_UL[last_slot * MAX_BWP_SIZE], &RC.nrmac[module_idP]->ulprbbl, sizeof(uint16_t) * MAX_BWP_SIZE);
    LOG_D(NR_MAC,"[yjn] frame = %d, slot = %d\n",frame, slot);
    const int last_slot = get_last_ul_tti_req_ind(gNB, frame, slot);  //add_yjn
    uint16_t *vrb_map_UL = cc[CC_id].vrb_map_UL;
    memset(&vrb_map_UL[last_slot * MAX_BWP_SIZE], 0, sizeof(uint16_t) * MAX_BWP_SIZE); //清空组内时隙号的上一时隙号的vrb_map_UL； //add_yjn
    
    clear_nr_nfapi_information(RC.nrmac[module_idP], CC_id, frame, slot);

    /*VNF first entry into scheduler. Since frame numbers for future_ul_tti_req of some future slots 
    will not be set before we encounter them, set them here */

    if (NFAPI_MODE == NFAPI_MODE_VNF){  //VNF模式用不到，可以不改吧
      if(vnf_first_sched_entry == 1)
      {
        for (int i = 0; i<num_slots; i++){
          if(i < slot)
            gNB->UL_tti_req_ahead[CC_id][i].SFN = (frame + 1) % 1024;
          else
            gNB->UL_tti_req_ahead[CC_id][i].SFN = frame;
        }
        vnf_first_sched_entry = 0;
      }
    }
  }


  if ((slot == 0) && (frame & 127) == 0) {
    char stats_output[16384];
    stats_output[0] = '\0';
    dump_mac_stats(RC.nrmac[module_idP], stats_output, sizeof(stats_output), true);
    LOG_I(NR_MAC, "Frame.Slot %d.%d\n%s\n", frame, slot, stats_output);
  }

  nr_mac_update_timers(module_idP, frame, slot);//add_yjn_未知新加

  schedule_nr_bwp_switch(module_idP, frame, slot);//add_yjn_未知新加

  // This schedules MIB
  schedule_nr_mib(module_idP, frame, slot);

  // This schedules SIB1
  if ( get_softmodem_params()->sa == 1 )
    schedule_nr_sib1(module_idP, frame, slot);


  // This schedule PRACH if we are not in phy_test mode
  if (get_softmodem_params()->phy_test == 0) {
    /* we need to make sure that resources for PRACH are free. To avoid that
       e.g. PUSCH has already been scheduled, make sure we schedule before
       anything else: below, we simply assume an advance one frame (minus one
       slot, because otherwise we would allocate the current slot in
       UL_tti_req_ahead), but be aware that, e.g., K2 is allowed to be larger
       (schedule_nr_prach will assert if resources are not free). */
    const sub_frame_t n_slots_ahead = nr_slots_per_frame[*scc->ssbSubcarrierSpacing] - 1;
     const frame_t f = (frame + (slot + n_slots_ahead + num_delay) / nr_slots_per_frame[*scc->ssbSubcarrierSpacing]) % 1024;//add_yjn_test
    const sub_frame_t s = (slot + n_slots_ahead+ num_delay) % nr_slots_per_frame[*scc->ssbSubcarrierSpacing];//add_yjn_test
    schedule_nr_prach(module_idP, f, s);
  }

  // Schedule CSI-RS transmission
  nr_csirs_scheduling(module_idP, frame, slot, nr_slots_per_frame[*scc->ssbSubcarrierSpacing]);

  // Schedule CSI measurement reporting: check in slot 0 for the whole frame
  if (slot == 0)
    nr_csi_meas_reporting(module_idP, frame, slot);

  // Schedule SRS: check in slot 0 for the whole frame
  if (slot == 0)
    nr_schedule_srs(module_idP, frame);//add_yjn_未知新加

  // This schedule RA procedure if not in phy_test mode
  // Otherwise already consider 5G already connected
  if (get_softmodem_params()->phy_test == 0) {
    nr_schedule_RA(module_idP, frame, slot);
  }

  // This schedules the DCI for Uplink and subsequently PUSCH
  nr_schedule_ulsch(module_idP, frame, slot);

  // This schedules the DCI for Downlink and PDSCH
  start_meas(&gNB->schedule_dlsch);
  nr_schedule_ue_spec(module_idP, frame, slot); 
  stop_meas(&gNB->schedule_dlsch);

  nr_schedule_pucch(RC.nrmac[module_idP], frame, slot);

  // This schedule SR after PUCCH for multiplexing
  nr_sr_reporting(RC.nrmac[module_idP], frame, slot);

  stop_meas(&RC.nrmac[module_idP]->eNB_scheduler);
  
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_gNB_DLSCH_ULSCH_SCHEDULER,VCD_FUNCTION_OUT);
}
