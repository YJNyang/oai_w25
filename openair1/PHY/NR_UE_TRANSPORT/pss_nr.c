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

/**********************************************************************
*
* FILENAME    :  pss_nr.c
*
* MODULE      :  synchronisation signal
*
* DESCRIPTION :  generation of pss
*                3GPP TS 38.211 7.4.2.2 Primary synchronisation signal
*
************************************************************************/

#include <stdio.h>
#include <assert.h>
#include <errno.h>
#include <math.h>
#include <nr-uesoftmodem.h>

#include "PHY/defs_nr_UE.h"

#include "PHY/NR_REFSIG/ss_pbch_nr.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

#define DEFINE_VARIABLES_PSS_NR_H
#include "PHY/NR_REFSIG/pss_nr.h"
#undef DEFINE_VARIABLES_PSS_NR_H

#include "PHY/NR_REFSIG/sss_nr.h"
#include "PHY/NR_UE_TRANSPORT/cic_filter_nr.h"
#include<stdio.h>
#include<immintrin.h>
#include<stdint.h>
#include<math.h>
#include<stdlib.h>
#include<time.h>
//#define DBG_PSS_NR
int  pss_corr_shift;

void pre_cfo_compensation_simd(int32_t* rxdata, int32_t* output,int start, int end, double off_angle) {
    int nb_samples_within_simd = 8;
    float phase_re = (float)cos(nb_samples_within_simd * off_angle);
    float phase_im = (float)sin(nb_samples_within_simd * off_angle);
    __m256 base_phase_re = _mm256_set1_ps(phase_re);
    __m256 base_phase_im = _mm256_set1_ps(phase_im);
    __m256 real_phase_re = _mm256_setr_ps(cos(start * off_angle), cos((start + 1) * off_angle), cos((start + 2) * off_angle), cos((start + 3) * off_angle),
        cos((start + 4) * off_angle), cos((start + 5) * off_angle), cos((start + 6) * off_angle), cos((start + 7) * off_angle));
    __m256 real_phase_im = _mm256_setr_ps(sin(start * off_angle), sin((start + 1) * off_angle), sin((start + 2) * off_angle), sin((start + 3) * off_angle),
        sin((start + 4) * off_angle), sin((start + 5) * off_angle), sin((start + 6) * off_angle), sin((start + 7) * off_angle));

    __m256 real_phase_re_tem;
    __m256 real_phase_im_tem;
    for (int n = start; n < end; n += 8) {

        __m256 rx_re = _mm256_setr_ps((float)((short*)rxdata)[2 * n], (float)(((short*)rxdata))[2 * n + 2],
                                              (float)(((short*)rxdata))[2 * n + 4], (float)(((short*)rxdata))[2 * n + 6],
                                              (float)(((short*)rxdata))[2 * n + 8], (float)(((short*)rxdata))[2 * n + 10],
                                              (float)(((short*)rxdata))[2 * n + 12], (float)(((short*)rxdata))[2 * n + 14]);
        __m256 rx_im = _mm256_setr_ps((float)(((short*)rxdata))[2 * n + 1], (float)(((short*)rxdata))[2 * n + 3],
                                              (float)(((short*)rxdata))[2 * n + 5], (float)(((short*)rxdata))[2 * n + 7],
                                              (float)(((short*)rxdata))[2 * n + 9], (float)(((short*)rxdata))[2 * n + 11],
                                              (float)(((short*)rxdata))[2 * n + 13], (float)(((short*)rxdata))[2 * n + 15]);

        __m256 data_re = _mm256_fmsub_ps(rx_re, real_phase_re, _mm256_mul_ps(rx_im, real_phase_im)); 
        __m256 data_im = _mm256_fmadd_ps(rx_im, real_phase_re, _mm256_mul_ps(rx_re, real_phase_im)); 

        __m256i data_re_int = _mm256_cvtps_epi32(data_re);
        __m256i data_im_int = _mm256_cvtps_epi32(data_im);
        __m256i data = _mm256_blend_epi16(data_re_int, _mm256_slli_epi32(data_im_int, 16), 0b10101010);
        _mm256_store_si256((__m256i*)(output+start) + (n - start) / 8, data);

        real_phase_re_tem = real_phase_re;
        real_phase_im_tem = real_phase_im;
        real_phase_re = _mm256_fmsub_ps(real_phase_re_tem, base_phase_re, _mm256_mul_ps(real_phase_im_tem, base_phase_im));
        real_phase_im = _mm256_fmadd_ps(real_phase_im_tem, base_phase_re, _mm256_mul_ps(real_phase_re_tem, base_phase_im));

    }
}

void generate_dpss_nr(NR_DL_FRAME_PARMS *fp)
{
      char buf[1000];
      FILE *fpRead;

      if(NULL == getcwd(buf, sizeof(buf))){
        LOG_E(PHY,"Failed to find current path.\n");
        assert(0);
      }

      fpRead  = fopen("./dpssSeq.txt","r");
      if(NULL ==  fpRead){
        LOG_E(PHY,"Failed to open dpssSeq files.\n");
        assert(0);
      }

      int i=0;
      int n=0;
      for (i = 0; i < NUMBER_DPSS_SEQUENCE; i++){ 
          for (n = 0; n < fp->ofdm_symbol_size; n++){
                if(fscanf(fpRead, "%hd\n,", &dpss_seq_nr[i][2*n])==EOF){
                    LOG_E(PHY,"Failed to read dpssSeq files.\n");
                    assert(0);
                }
                // printf("%d+j%d\n",dpss_seq_nr[i][2*n],dpss_seq_nr[i][2*n+1]);
          }
           printf("DPSS %d loaded\n",i);
      }

      //  dpss_equ_seq generate
      int16_t real1,real2,imag1,imag2;
      for (int pss_idx = 0; pss_idx < NUMBER_PSS_SEQUENCE; pss_idx++){
          for (int dpss_idx=0; dpss_idx<NUMBER_DPSS_SEQUENCE; dpss_idx++){
          // dpss-pss sequence generate
          for(int cnt=0; cnt<fp->ofdm_symbol_size; cnt++){
            real1=dpss_seq_nr[dpss_idx][2*cnt];
            real2=primary_synchro_time_nr[pss_idx][2*cnt]>>1;
            imag1=dpss_seq_nr[dpss_idx][2*cnt+1];
            imag2=primary_synchro_time_nr[pss_idx][2*cnt+1]>>1;

            dpss_equ_seq[pss_idx*NUMBER_DPSS_SEQUENCE+dpss_idx][2*cnt]=  real1*real2-imag1*imag2;
            dpss_equ_seq[pss_idx*NUMBER_DPSS_SEQUENCE+dpss_idx][2*cnt+1]= real1*imag2+real2*imag1;
          }
        }
      }
      LOG_I(PHY,"dpss_equ_seq generated\n");
      
      // primary_synchro_time_nr_fo generate
      for (int pss_idx = 0; pss_idx < NUMBER_PSS_SEQUENCE; pss_idx++){
          for (int fo_idx=0; fo_idx<(NUMBER_DPSS_SEQUENCE+1); fo_idx++){

            double fo_angle = -2*M_PI*(fo_idx-4)/fp->ofdm_symbol_size;  //搜索区间：[-4,4]整数
          // dpss-pss sequence generate
          for(int n=0; n<fp->ofdm_symbol_size; n++){
              real1 = (double)(primary_synchro_time_nr[pss_idx][2*n]);
              imag1 = (double)(primary_synchro_time_nr[pss_idx][2*n+1]);
              primary_synchro_time_nr_fo[pss_idx*(NUMBER_DPSS_SEQUENCE+1)+fo_idx][2*n] = (short)(round(real1*cos(n*fo_angle) - imag1*sin(n*fo_angle))); //TODO: primary_synchro_time_nr_fo初始化
              primary_synchro_time_nr_fo[pss_idx*(NUMBER_DPSS_SEQUENCE+1)+fo_idx][2*n+1] = (short)(round(real1*sin(n*fo_angle) + imag1*cos(n*fo_angle)));
          }
        }
      }
      LOG_I(PHY,"primary_synchro_time_nr_fo generated\n");
}

/*******************************************************************
*
* NAME :         generate_pss_nr
*
* PARAMETERS :   N_ID_2 : element 2 of physical layer cell identity
*                value : { 0, 1, 2}
*
* RETURN :       generate binary pss sequence (this is a m-sequence)
*
* DESCRIPTION :  3GPP TS 38.211 7.4.2.2 Primary synchronisation signal
*                Sequence generation
*
*********************************************************************/

void generate_pss_nr(NR_DL_FRAME_PARMS *fp,int N_ID_2)
{
  AssertFatal(fp->ofdm_symbol_size > 127,"Illegal ofdm_symbol_size %d\n",fp->ofdm_symbol_size);
  AssertFatal(N_ID_2>=0 && N_ID_2 <=2,"Illegal N_ID_2 %d\n",N_ID_2);
  int16_t d_pss[LENGTH_PSS_NR];
  int16_t x[LENGTH_PSS_NR];
  int16_t *primary_synchro_time = primary_synchro_time_nr[N_ID_2];
  unsigned int length = fp->ofdm_symbol_size;
  unsigned int size = length * IQ_SIZE; /* i & q */
  int16_t *primary_synchro = primary_synchro_nr[N_ID_2]; /* pss in complex with alternatively i then q */
  int16_t *primary_synchro2 = primary_synchro_nr2[N_ID_2]; /* pss in complex with alternatively i then q */


  #define INITIAL_PSS_NR    (7)
  const int x_initial[INITIAL_PSS_NR] = {0, 1, 1 , 0, 1, 1, 1};

  assert(N_ID_2 < NUMBER_PSS_SEQUENCE);
  assert(size <= SYNCF_TMP_SIZE);
  assert(size <= SYNC_TMP_SIZE);

  bzero(synchroF_tmp, size);
  bzero(synchro_tmp, size);

  for (int i=0; i < INITIAL_PSS_NR; i++) {
    x[i] = x_initial[i];
  }

  for (int i=0; i < (LENGTH_PSS_NR - INITIAL_PSS_NR); i++) {
    x[i+INITIAL_PSS_NR] = (x[i + 4] + x[i])%(2);
  }

  for (int n=0; n < LENGTH_PSS_NR; n++) {
    int m = (n + 43*N_ID_2)%(LENGTH_PSS_NR);
    d_pss[n] = 1 - 2*x[m];
  }

  /* PSS is directly mapped to subcarrier without modulation 38.211 */
  for (int i=0; i < LENGTH_PSS_NR; i++) {
#if 1
    primary_synchro[2*i] = (d_pss[i] * SHRT_MAX)>>SCALING_PSS_NR; /* Maximum value for type short int ie int16_t */
    primary_synchro[2*i+1] = 0;
    primary_synchro2[i] = d_pss[i];
#else
    primary_synchro[2*i] = d_pss[i] * AMP;
    primary_synchro[2*i+1] = 0;
    primary_synchro2[i] = d_pss[i];
#endif
  }

#ifdef DBG_PSS_NR

  if (N_ID_2 == 0) {
    char output_file[255];
    char sequence_name[255];
    sprintf(output_file, "pss_seq_%d_%u.m", N_ID_2, length);
    sprintf(sequence_name, "pss_seq_%d_%u", N_ID_2, length);
    printf("file %s sequence %s\n", output_file, sequence_name);

    LOG_M(output_file, sequence_name, primary_synchro, LENGTH_PSS_NR, 1, 1);
  }

#endif

  /* call of IDFT should be done with ordered input as below
  *
  *                n input samples
  *  <------------------------------------------------>
  *  0                                                n
  *  are written into input buffer for IFFT
  *   -------------------------------------------------
  *  |xxxxxxx                       N/2       xxxxxxxx|
  *  --------------------------------------------------
  *  ^      ^                 ^               ^          ^
  *  |      |                 |               |          |
  * n/2    end of            n=0            start of    n/2-1
  *         pss                               pss
  *
  *                   Frequencies
  *      positives                   negatives
  * 0                 (+N/2)(-N/2)
  * |-----------------------><-------------------------|
  *
  * sample 0 is for continuous frequency which is used here
  */

  unsigned int  k = fp->first_carrier_offset + fp->ssb_start_subcarrier + 56; //and
  if (k>= fp->ofdm_symbol_size) k-=fp->ofdm_symbol_size;


  for (int i=0; i < LENGTH_PSS_NR; i++) {
    synchroF_tmp[2*k] = primary_synchro[2*i];
    synchroF_tmp[2*k+1] = primary_synchro[2*i+1];

    k++;

    if (k == length) k=0;
    
  }

  /* IFFT will give temporal signal of Pss */

 
 
  idft((int16_t)get_idft(length),
  	   synchroF_tmp,          /* complex input */
       synchro_tmp,           /* complex output */
       1);                 /* scaling factor */

  /* then get final pss in time */
  for (unsigned int i=0; i<length; i++) {
    ((int32_t *)primary_synchro_time)[i] = ((int32_t *)synchro_tmp)[i];
  }

#ifdef DBG_PSS_NR

  if (N_ID_2 == 0) {
    char output_file[255];
    char sequence_name[255];
    sprintf(output_file, "%s%d_%u%s","pss_seq_t_", N_ID_2, length, ".m");
    sprintf(sequence_name, "%s%d_%u","pss_seq_t_", N_ID_2, length);

    printf("file %s sequence %s\n", output_file, sequence_name);

    LOG_M(output_file, sequence_name, primary_synchro_time, length, 1, 1);
    sprintf(output_file, "%s%d_%u%s","pss_seq_f_", N_ID_2, length, ".m");
    sprintf(sequence_name, "%s%d_%u","pss_seq_f_", N_ID_2, length);
    LOG_M(output_file, sequence_name, synchroF_tmp, length, 1, 1);
  }

#endif



#if 0

/* it allows checking that process of idft on a signal and then dft gives same signal with limited errors */

  if ((N_ID_2 == 0) && (length == 256)) {

    LOG_M("pss_f00.m","pss_f00",synchro_tmp,length,1,1);


    bzero(synchroF_tmp, size);

  

    /* get pss in the time domain by applying an inverse FFT */
    dft((int16_t)get_dft(length),
    	synchro_tmp,           /* complex input */
        synchroF_tmp,          /* complex output */
        1);                 /* scaling factor */

    if ((N_ID_2 == 0) && (length == 256)) {
      LOG_M("pss_f_0.m","pss_f_0",synchroF_tmp,length,1,1);
    }

    /* check Pss */
    k = length - (LENGTH_PSS_NR/2);

#define LIMIT_ERROR_FFT   (10)

    for (int i=0; i < LENGTH_PSS_NR; i++) {
      if (abs(synchroF_tmp[2*k] - primary_synchro[2*i]) > LIMIT_ERROR_FFT) {
      printf("Pss Error[%d] Compute %d Reference %d \n", k, synchroF_tmp[2*k], primary_synchro[2*i]);
      }
    
      if (abs(synchroF_tmp[2*k+1] - primary_synchro[2*i+1]) > LIMIT_ERROR_FFT) {
        printf("Pss Error[%d] Compute %d Reference %d\n", (2*k+1), synchroF_tmp[2*k+1], primary_synchro[2*i+1]);
      }

      k++;

      if (k >= length) {
        k-=length;
      }
    }
  }
#endif
}

/*******************************************************************
*
* NAME :         init_context_pss_nr
*
* PARAMETERS :   structure NR_DL_FRAME_PARMS give frame parameters
*
* RETURN :       generate binary pss sequences (this is a m-sequence)
*
* DESCRIPTION :  3GPP TS 38.211 7.4.2.2 Primary synchronisation signal
*                Sequence generation
*
*********************************************************************/

void init_context_pss_nr(NR_DL_FRAME_PARMS *frame_parms_ue)
{
  int ofdm_symbol_size = frame_parms_ue->ofdm_symbol_size;
  int sizePss = LENGTH_PSS_NR * IQ_SIZE;  /* complex value i & q signed 16 bits */
  int size = ofdm_symbol_size * IQ_SIZE; /* i and q samples signed 16 bits */
  int16_t *p = NULL;

  AssertFatal(ofdm_symbol_size > 127, "illegal ofdm_symbol_size %d\n",ofdm_symbol_size);
  for (int i = 0; i < NUMBER_PSS_SEQUENCE; i++) {

    p = malloc16(sizePss); /* pss in complex with alternatively i then q */
    if (p != NULL) {
      primary_synchro_nr[i] = p;
      bzero( primary_synchro_nr[i], sizePss);
    }
    else {
      LOG_E(PHY,"Fatal memory allocation problem \n");
      assert(0);
    }
    p = malloc(LENGTH_PSS_NR*2);
    if (p != NULL) {
      primary_synchro_nr2[i] = p;
      bzero( primary_synchro_nr2[i],LENGTH_PSS_NR*2);
    }
    p = malloc16(size);
    if (p != NULL) {
      primary_synchro_time_nr[i] = p;
      bzero( primary_synchro_time_nr[i], size);
    }
    else {
      LOG_E(PHY,"Fatal memory allocation problem \n");
     assert(0);
    }

    generate_pss_nr(frame_parms_ue,i);
  }
  // dpss_seq memory initialize
  for (int dpss_idx = 0; dpss_idx < NUMBER_DPSS_SEQUENCE; dpss_idx++){
    printf("DPSS %d initializing\n",dpss_idx);
    p = malloc16(size);
    if (p != NULL) {
      dpss_seq_nr[dpss_idx] = p;
      bzero( dpss_seq_nr[dpss_idx], size);
    }
    else {
      LOG_E(PHY,"Fatal memory allocation problem \n");
      assert(0);
    }
  }
  for (int i = 0; i < NUMBER_DPSS_SEQUENCE*NUMBER_PSS_SEQUENCE; i++){
      p = malloc16(size);
      if (p != NULL) {
        dpss_equ_seq[i] = p;
        bzero( dpss_equ_seq[i], size);
      }
      else {
        LOG_E(PHY,"Fatal memory allocation problem \n");
        assert(0);
      }
  }
  for (int i = 0; i < NUMBER_PSS_SEQUENCE*(NUMBER_DPSS_SEQUENCE+1); i++){
    p = malloc16(size);
    if (p != NULL) {
      primary_synchro_time_nr_fo[i] = p;
      bzero( primary_synchro_time_nr_fo[i], size);
    }
    else {
      LOG_E(PHY,"Fatal memory allocation problem \n");
      assert(0);
    }
  }
  generate_dpss_nr(frame_parms_ue); // load dpss seq
}

/*******************************************************************
*
* NAME :         free_context_pss_nr
*
* PARAMETERS :   none
*
* RETURN :       none
*
* DESCRIPTION :  free context related to pss
*
*********************************************************************/

void free_context_pss_nr(void)
{
  for (int i = 0; i < NUMBER_PSS_SEQUENCE; i++) {
    free_and_zero(primary_synchro_nr[i]);
    free_and_zero(primary_synchro_nr2[i]);
    free_and_zero(primary_synchro_time_nr[i]);
  }
  for (int i = 0; i < NUMBER_DPSS_SEQUENCE; i++) {
    free_and_zero(dpss_seq_nr[i]);
  }
  for (int i = 0; i < NUMBER_PSS_SEQUENCE*NUMBER_DPSS_SEQUENCE; i++) {
    free_and_zero(dpss_equ_seq[i]);
  }
  for (int i = 0; i < NUMBER_PSS_SEQUENCE*(NUMBER_DPSS_SEQUENCE+1); i++) {
    free_and_zero(primary_synchro_time_nr_fo[i]);
  }
}

/*******************************************************************
*
* NAME :         init_context_synchro_nr
*
* PARAMETERS :   none
*
* RETURN :       generate context for pss and sss
*
* DESCRIPTION :  initialise contexts and buffers for synchronisation
*
*********************************************************************/

void init_context_synchro_nr(NR_DL_FRAME_PARMS *frame_parms_ue)
{
#ifndef STATIC_SYNC_BUFFER

  /* initialise global buffers for synchronisation */
  synchroF_tmp = malloc16(SYNCF_TMP_SIZE);
  if (synchroF_tmp == NULL) {
    LOG_E(PHY,"Fatal memory allocation problem \n");
    assert(0);
  }

  synchro_tmp = malloc16(SYNC_TMP_SIZE);
  if (synchro_tmp == NULL) {
    LOG_E(PHY,"Fatal memory allocation problem \n");
    assert(0);
  }

#endif

  init_context_pss_nr(frame_parms_ue);

  init_context_sss_nr(AMP);
}

/*******************************************************************
*
* NAME :         free_context_synchro_nr
*
* PARAMETERS :   none
*
* RETURN :       free context for pss and sss
*
* DESCRIPTION :  deallocate memory of synchronisation
*
*********************************************************************/

void free_context_synchro_nr(void)
{
#ifndef STATIC_SYNC_BUFFER

  if (synchroF_tmp != NULL) {
    free(synchroF_tmp);
    synchroF_tmp = NULL;
  }
  else {
    LOG_E(PHY,"Fatal memory deallocation problem \n");
    assert(0);
  }

  if (synchro_tmp != NULL) {
    free(synchro_tmp);
    synchro_tmp = NULL;
  }
  else {
    LOG_E(PHY,"Fatal memory deallocation problem \n");
    assert(0);
  }

#endif

  free_context_pss_nr();
}

/*******************************************************************
*
* NAME :         set_frame_context_pss_nr
*
* PARAMETERS :   configuration for UE with new FFT size
*
* RETURN :       0 if OK else error
*
* DESCRIPTION :  initialisation of UE contexts
*
*********************************************************************/

void set_frame_context_pss_nr(NR_DL_FRAME_PARMS *frame_parms_ue, int rate_change)
{
  /* set new value according to rate_change */
  frame_parms_ue->ofdm_symbol_size = (frame_parms_ue->ofdm_symbol_size / rate_change);
  frame_parms_ue->samples_per_subframe = (frame_parms_ue->samples_per_subframe / rate_change);

  free_context_pss_nr();

  /* pss reference have to be rebuild with new parameters ie ofdm symbol size */
  init_context_synchro_nr(frame_parms_ue);

#ifdef SYNCHRO_DECIMAT
  set_pss_nr(frame_parms_ue->ofdm_symbol_size);
#endif
}

/*******************************************************************
*
* NAME :         restore_frame_context_pss_nr
*
* PARAMETERS :   configuration for UE and eNB with new FFT size
*
* RETURN :       0 if OK else error
*
* DESCRIPTION :  initialisation of UE and eNode contexts
*
*********************************************************************/

void restore_frame_context_pss_nr(NR_DL_FRAME_PARMS *frame_parms_ue, int rate_change)
{
  frame_parms_ue->ofdm_symbol_size = frame_parms_ue->ofdm_symbol_size * rate_change;
  frame_parms_ue->samples_per_subframe = frame_parms_ue->samples_per_subframe * rate_change;

  free_context_pss_nr();

  /* pss reference have to be rebuild with new parameters ie ofdm symbol size */
  init_context_synchro_nr(frame_parms_ue);
#ifdef SYNCHRO_DECIMAT
  set_pss_nr(frame_parms_ue->ofdm_symbol_size);
#endif
}

/********************************************************************
*
* NAME :         decimation_synchro_nr
*
* INPUT :        UE context
*                for first and second pss sequence
*                - position of pss in the received UE buffer
*                - number of pss sequence
*
* RETURN :      0 if OK else error
*
* DESCRIPTION :  detect pss sequences in the received UE buffer
*
********************************************************************/

void decimation_synchro_nr(PHY_VARS_NR_UE *PHY_vars_UE, int rate_change, int **rxdata)
{
  NR_DL_FRAME_PARMS *frame_parms = &(PHY_vars_UE->frame_parms);
  int samples_for_frame = 2*frame_parms->samples_per_frame;

#if TEST_SYNCHRO_TIMING_PSS

  opp_enabled = 1;

  start_meas(&generic_time[TIME_RATE_CHANGE]);

#endif

/* build with cic filter does not work properly. Performances are significantly deteriorated */
#ifdef CIC_DECIMATOR

  cic_decimator((int16_t *)&(PHY_vars_UE->common_vars.rxdata[0][0]), (int16_t *)&(rxdata[0][0]),
                            samples_for_frame, rate_change, CIC_FILTER_STAGE_NUMBER, 0, FIR_RATE_CHANGE);
#else

  fir_decimator((int16_t *)&(PHY_vars_UE->common_vars.rxdata[0][0]), (int16_t *)&(rxdata[0][0]),
                            samples_for_frame, rate_change, 0);

#endif

  set_frame_context_pss_nr(frame_parms, rate_change);

#if TEST_SYNCHRO_TIMING_PSS

  stop_meas(&generic_time[TIME_RATE_CHANGE]);

  printf("Rate change execution duration %5.2f \n", generic_time[TIME_RATE_CHANGE].p_time/(cpuf*1000.0));

#endif
}

/*******************************************************************
*
* NAME :         pss_synchro_nr
*
* PARAMETERS :   int rate_change
*
* RETURN :       position of detected pss
*
* DESCRIPTION :  pss search can be done with sampling decimation.*
*
*********************************************************************/

int pss_synchro_nr(PHY_VARS_NR_UE *PHY_vars_UE, int is, int rate_change)
{
  NR_DL_FRAME_PARMS *frame_parms = &(PHY_vars_UE->frame_parms);
  int synchro_position;
  int **rxdata = NULL;
  int fo_flag = PHY_vars_UE->UE_fo_compensation;  // flag to enable freq offset estimation and compensation

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SYNCHRO_NR, VCD_FUNCTION_IN);
#ifdef DBG_PSS_NR

  LOG_M("rxdata0_rand.m","rxd0_rand", &PHY_vars_UE->common_vars.rxdata[0][0], frame_parms->samples_per_frame, 1, 1);

#endif

  if (rate_change != 1) {

    rxdata = (int32_t**)malloc16(frame_parms->nb_antennas_rx*sizeof(int32_t*));

    for (int aa=0; aa < frame_parms->nb_antennas_rx; aa++) {
      rxdata[aa] = (int32_t*) malloc16_clear( (frame_parms->samples_per_frame+8192)*sizeof(int32_t));
    }
#ifdef SYNCHRO_DECIMAT

    decimation_synchro_nr(PHY_vars_UE, rate_change, rxdata);

#endif
  }
  else {

    rxdata = PHY_vars_UE->common_vars.rxdata;
  }

#ifdef DBG_PSS_NR

  LOG_M("rxdata0_des.m","rxd0_des", &rxdata[0][0], frame_parms->samples_per_frame,1,1);

#endif

#if TEST_SYNCHRO_TIMING_PSS

  opp_enabled = 1;

  start_meas(&generic_time[TIME_PSS]);

#endif

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SEARCH_TIME_NR, VCD_FUNCTION_IN);
  synchro_position = pss_search_time_nr(rxdata,
                                        frame_parms,
					fo_flag,
                                        is,
                                        (int *)&PHY_vars_UE->common_vars.eNb_id,
					(int *)&PHY_vars_UE->common_vars.freq_offset);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SEARCH_TIME_NR, VCD_FUNCTION_OUT);

#if TEST_SYNCHRO_TIMING_PSS

  stop_meas(&generic_time[TIME_PSS]);

  int duration_ms = generic_time[TIME_PSS].p_time/(cpuf*1000.0);

  #ifndef NR_UNIT_TEST

    printf("PSS execution duration %4d microseconds \n", duration_ms);

  #endif

#endif

#ifdef SYNCHRO_DECIMAT

  if (rate_change != 1) {

    if (rxdata[0] != NULL) {

      for (int aa=0;aa<frame_parms->nb_antennas_rx;aa++) {
        free(rxdata[aa]);
      }

      free(rxdata);
    }

    restore_frame_context_pss_nr(frame_parms, rate_change);  
  }
#endif

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SYNCHRO_NR, VCD_FUNCTION_OUT);
  return synchro_position;
}


int pss_track_synchro_nr(PHY_VARS_NR_UE *PHY_vars_UE, int position,int length, int rate_change)
{
  NR_DL_FRAME_PARMS *frame_parms = &(PHY_vars_UE->frame_parms);
  int synchro_position;
  int **rxdata = NULL;

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SYNCHRO_NR, VCD_FUNCTION_IN);

  if (rate_change != 1) {

    rxdata = (int32_t**)malloc16(frame_parms->nb_antennas_rx*sizeof(int32_t*));

    for (int aa=0; aa < frame_parms->nb_antennas_rx; aa++) {
      rxdata[aa] = (int32_t*) malloc16_clear( (frame_parms->samples_per_frame+8192)*sizeof(int32_t));
    }
#ifdef SYNCHRO_DECIMAT

    decimation_synchro_nr(PHY_vars_UE, rate_change, rxdata);

#endif
  }
  else {

    rxdata = PHY_vars_UE->common_vars.rxdata;
  }

#if TEST_SYNCHRO_TIMING_PSS

  opp_enabled = 1;

  start_meas(&generic_time[TIME_PSS]);

#endif

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SEARCH_TIME_NR, VCD_FUNCTION_IN);
  synchro_position = pss_search_time_track_nr(rxdata,
                                        frame_parms,
					                              position,
                                        length,
                                        PHY_vars_UE->common_vars.eNb_id,
					(int *)&PHY_vars_UE->track_sync_fo);

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SEARCH_TIME_NR, VCD_FUNCTION_OUT);

#if TEST_SYNCHRO_TIMING_PSS

  stop_meas(&generic_time[TIME_PSS]);

  // int duration_ms = generic_time[TIME_PSS].p_time/(cpuf*1000.0);

  #ifndef NR_UNIT_TEST
    // LOG_I(PHY,"[TRACK SYNC] PSS SYNC execution duration %4d microseconds \n", duration_ms);

  #endif

#endif

#ifdef SYNCHRO_DECIMAT

  if (rate_change != 1) {

    if (rxdata[0] != NULL) {

      for (int aa=0;aa<frame_parms->nb_antennas_rx;aa++) {
        free(rxdata[aa]);
      }

      free(rxdata);
    }

    restore_frame_context_pss_nr(frame_parms, rate_change);  
  }
#endif

  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_PSS_SYNCHRO_NR, VCD_FUNCTION_OUT);
  return synchro_position;
}



static inline int abs32(int x)
{
  return (((int)((short*)&x)[0])*((int)((short*)&x)[0]) + ((int)((short*)&x)[1])*((int)((short*)&x)[1]));
}

static inline int64_t abs64(int64_t x)
{
  return (((int64_t)((int32_t*)&x)[0])*((int64_t)((int32_t*)&x)[0]) + ((int64_t)((int32_t*)&x)[1])*((int64_t)((int32_t*)&x)[1]));
}

static inline double angle64(int64_t x)
{

  double re=((int32_t*)&x)[0];
  double im=((int32_t*)&x)[1];
  return (atan2(im,re));
}


/*******************************************************************
*
* NAME :         pss_search_time_nr
*
* PARAMETERS :   received buffer
*                frame parameters
*
* RETURN :       position of detected pss
*
* DESCRIPTION :  Synchronisation on pss sequence is based on a time domain correlation between received samples and pss sequence
*                A maximum likelihood detector finds the timing offset (position) that corresponds to the maximum correlation
*                Length of received buffer should be a minimum of 2 frames (see TS 38.213 4.1 Cell search)
*                Search pss in the received buffer is done each 4 samples which ensures a memory alignment to 128 bits (32 bits x 4).
*                This is required by SIMD (single instruction Multiple Data) Extensions of Intel processors
*                Correlation computation is based on a a dot product which is realized thank to SIMS extensions
*
*                                    (x frames)
*     <--------------------------------------------------------------------------->
*
*
*     -----------------------------------------------------------------------------
*     |                      Received UE data buffer                              |
*     ----------------------------------------------------------------------------
*                -------------
*     <--------->|    pss    |
*      position  -------------
*                ^
*                |
*            peak position
*            given by maximum of correlation result
*            position matches beginning of first ofdm symbol of pss sequence
*
*     Remark: memory position should be aligned on a multiple of 4 due to I & Q samples of int16
*             An OFDM symbol is composed of x number of received samples depending of Rf front end sample rate.
*
*     I & Q storage in memory
*
*             First samples       Second  samples
*     ------------------------- -------------------------  ...
*     |     I1     |     Q1    |     I2     |     Q2    |
*     ---------------------------------------------------  ...
*     ^    16  bits   16 bits  ^
*     |                        |
*     ---------------------------------------------------  ...
*     |         sample 1       |    sample   2          |
*    ----------------------------------------------------  ...
*     ^
*
*********************************************************************/

#define DOT_PRODUCT_SCALING_SHIFT    (17)
int pss_search_time_nr(int **rxdata, ///rx data in time domain
                       NR_DL_FRAME_PARMS *frame_parms,
		       int fo_flag,
                       int is, 
                       int *eNB_id,
		       int *f_off)
{
  unsigned int n, ar, peak_position, pss_source;
  int64_t peak_value;
  int64_t result;
  int64_t avg[NUMBER_PSS_SEQUENCE]={0};
  int64_t ifo_peak_value=0;

  // performing the correlation on a frame length plus one symbol for the first of the two frame
  // to take into account the possibility of PSS in between the two frames 
  unsigned int length;
  LOG_I(PHY,"[UE_SYNC] =====Start UE Sync in frame %d=====\n",is);
 // ===========================================  参数初始化   ===========================================
  /* 相关运算长度计算 */
    if (is==0)
      length = frame_parms->samples_per_frame + (2*frame_parms->ofdm_symbol_size); // 为什么plus one symbol却2*frame_parms->ofdm_symbol_size
    else
      length = frame_parms->samples_per_frame;


  AssertFatal(length>0,"illegal length %d\n",length);
  peak_value = 0;
  peak_position = 0;
  pss_source = 0;
  /* PSS本地序列最大值计算 */
  int maxval=0;
  for (int i=0;i<2*(frame_parms->ofdm_symbol_size);i++) {
    maxval = max(maxval,primary_synchro_time_nr[0][i]);
    maxval = max(maxval,-primary_synchro_time_nr[0][i]);
    maxval = max(maxval,primary_synchro_time_nr[1][i]);
    maxval = max(maxval,-primary_synchro_time_nr[1][i]);
    maxval = max(maxval,primary_synchro_time_nr[2][i]);
    maxval = max(maxval,-primary_synchro_time_nr[2][i]);
  }
  pss_corr_shift = log2_approx(maxval);//*(frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples)*2);  //可以优化

  // ===========================================  DPSS序列定时估计   ===========================================
  for (int pss_index = 0; pss_index < NUMBER_PSS_SEQUENCE; pss_index++) { 
    LOG_I(PHY,"[UE_SYNC] STO Est (pssIdx%d)\n",pss_index);
     for (n=0; n < length; n+=4) {
      int64_t dpss_corr_ue=0;
      for (int dpss_index=0; dpss_index<NUMBER_DPSS_SEQUENCE; dpss_index++){
        // dpss-pss sequence generate
        for (ar=0; ar<frame_parms->nb_antennas_rx; ar++) {
        /* perform correlation of rx data and pss sequence ie it is a dot product */
        // dpss_equ_seq[pss_index*dpss_index]
          result  = dot_product64((short*)dpss_equ_seq[pss_index*NUMBER_DPSS_SEQUENCE+dpss_index],
                              (short*)&(rxdata[ar][n+is*frame_parms->samples_per_frame]),
                              frame_parms->ofdm_symbol_size,
                              pss_corr_shift);  
          dpss_corr_ue += abs64(result)>>3; // ant and dpss accumulate
        }
      }

      avg[pss_index]+=dpss_corr_ue/(length/4);
      if (dpss_corr_ue > peak_value) {
        peak_value = dpss_corr_ue;
        peak_position = n;
        pss_source = pss_index;
      }
     }
  }

// 定时估计结果：峰值位置 peak_position ；相关峰值 peak_value；相关运算累加avg[pss_index]；小区识别号2 pss_source
// ===========================================  SNR计算   ===========================================


  // avg[pss_source*NUMBER_DPSS_SEQUENCE+dpss_source]/=(length/4);
  
  uint16_t syncSNR=dB_fixed64(peak_value)-dB_fixed64(avg[pss_source]);

  LOG_I(PHY,"[UE_SYNC] nr_synchro_time: Sync source = %d, Peak found at pos %d, val = %llu (%d dB) avg %d dB\n", pss_source, peak_position, (unsigned long long)peak_value, dB_fixed64(peak_value),dB_fixed64(avg[pss_source]));
  LOG_I(PHY,"[UE_SYNC] SyncSNR=%d\n",syncSNR);
  if (syncSNR < 8) {/* SNR */
    LOG_I(PHY,"[UE_SYNC] Sync SNR is too low, can't find PSS Position.\n");
    return(-1);
  }

// ===========================================  频偏est   ===========================================
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PSS FFO EST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

     int64_t result1,result2;
     double ffo_pss=0;
	  // Computing cross-correlation at peak on half the symbol size for first half of data
	  result1  = dot_product64((short*)primary_synchro_time_nr[pss_source], 
				  (short*) &(rxdata[0][peak_position+is*frame_parms->samples_per_frame]), 
				  frame_parms->ofdm_symbol_size>>1, 
				  pss_corr_shift);
	  // Computing cross-correlation at peak on half the symbol size for data shifted by half symbol size 
	  // as it is real and complex it is necessary to shift by a value equal to symbol size to obtain such shift
	  result2  = dot_product64((short*)primary_synchro_time_nr[pss_source]+(frame_parms->ofdm_symbol_size), 
				  (short*) &(rxdata[0][peak_position+is*frame_parms->samples_per_frame])+frame_parms->ofdm_symbol_size, 
				  frame_parms->ofdm_symbol_size>>1, 
				  pss_corr_shift);

	  int64_t re1,re2,im1,im2;
	  re1=((int*) &result1)[0];
	  re2=((int*) &result2)[0];
	  im1=((int*) &result1)[1];
	  im2=((int*) &result2)[1];

 	  // estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	  ffo_pss=-atan2(re1*im2-re2*im1,re1*re2+im1*im2)/M_PI;
    LOG_I(PHY,"[UE_SYNC_FFO_PSS]  FFO_PSS Est res:  ffo_pss=%f \n",ffo_pss);

  
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> CP FFO EST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

      double ffo_cp=0;
      int64_t ffo_cp_corr = 0;
      int64_t ffo_cp_est = 0;
      int offsetUnit = frame_parms->nb_prefix_samples + frame_parms->ofdm_symbol_size;
      int PosStart = peak_position +is*frame_parms->samples_per_frame - frame_parms->nb_prefix_samples; 
      int cp_corr_shift = frame_parms->ffo_corr_shift >0 ? frame_parms->ffo_corr_shift : pss_corr_shift;
          for (int i = -1; i < 4; i++){
              ffo_cp_corr = dot_product64((short*)&(rxdata[0][PosStart+i*offsetUnit]), 
                                        (short*)&(rxdata[0][PosStart+i*offsetUnit+frame_parms->ofdm_symbol_size]), // 修改：调用前完成数据截取
                                        frame_parms->nb_prefix_samples,
                                        cp_corr_shift);
            ((int32_t*) &ffo_cp_est)[0] = ((int32_t*) &ffo_cp_est)[0]+((int32_t*) &ffo_cp_corr)[0];
            ((int32_t*) &ffo_cp_est)[1] = ((int32_t*) &ffo_cp_est)[1]+((int32_t*) &ffo_cp_corr)[1];
            //LOG_I(PHY,"[UE_SYNC_FFO_CP]  FFO_CP Corr:  ffo_cp_corr=%d+j%d\t \n",((int32_t*) &ffo_cp_est)[0],((int32_t*) &ffo_cp_est)[1]);
            //  PosStart += offsetUnit;
          }
          int32_t re3,im3;
          re3=((int32_t*) &ffo_cp_est)[0];
          im3=((int32_t*) &ffo_cp_est)[1];
          ffo_cp=-atan2(im3,re3)/2/M_PI; // ffo=-angle()/2/pi
          LOG_I(PHY,"[UE_SYNC_FFO_CP]  FFO_CP Est res:  ffo_cp=%f  (cp_corr_shift: %d)\n",ffo_cp,cp_corr_shift);
    
    
      
    
    //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PSS IFO EST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

       int ifo_pss=-4;
       int ifo_corr_shift = 15;
      /* IFO search */
      //LOG_I(PHY,"[UE_SYNC_IFO] Start IFO Est\n");
      for (int fo_idx = -4; fo_idx < 5 ; fo_idx++){
        int64_t ifo_result=0;
        /* 计算频偏估计等效序列 */
       double fo_angle = -2*M_PI*(fo_idx+ffo_cp)/frame_parms->ofdm_symbol_size;  //搜索区间：[-4,4]整数
        pre_cfo_compensation_simd((int32_t *)primary_synchro_time_nr[pss_source],(int32_t *)primary_synchro_time_nr_fo[0],0,frame_parms->ofdm_symbol_size,fo_angle);
        // int ifo_est=fo_idx-4;
        /* 频偏估计等效序列相关运算 */
        result  = dot_product64((short*)primary_synchro_time_nr_fo[0], 
                                    (short*)&(rxdata[0][peak_position+is*frame_parms->samples_per_frame]), // 修改：调用前完成数据截取
                                    frame_parms->ofdm_symbol_size,
                                    ifo_corr_shift); 
        ifo_result = abs64(result);
        LOG_I(PHY,"[UE_SYNC_IFO]  IFO %d cor result: ifo_result = %llu\n",fo_idx,(unsigned long long)ifo_result);
        if (ifo_result > ifo_peak_value) {
        ifo_pss = fo_idx;
        ifo_peak_value = ifo_result;
        }
      } // 输出ifo_pss
       LOG_I(PHY,"[UE_SYNC_IFO]  IFO Est result: IFO = %d (ifo_corr_shift: %d)\n",ifo_pss,ifo_corr_shift);

// ===========================================  输出量计算   =========================================== 

  // computing absolute value of frequency offset
  *f_off = (ffo_cp+ifo_pss)*frame_parms->subcarrier_spacing;  
  *eNB_id = pss_source; /* N_id2 */
LOG_I(PHY,"[UE_SYNC_CFO]pss_cfo=%f( %dHZ)] , carrier Freq=%f\n",ffo_cp+ifo_pss,*f_off,(double)(frame_parms->dl_CarrierFreq)-(double)(*f_off));
#ifdef DBG_PSS_NR

  static int debug_cnt = 0;

  if (debug_cnt == 0) {
    if (is)
      LOG_M("rxdata1.m","rxd0",rxdata[frame_parms->samples_per_frame],length,1,1); 
    else
      LOG_M("rxdata0.m","rxd0",rxdata[0],length,1,1);
  } else {
    debug_cnt++;
  }

#endif

  return(peak_position);
}


int pss_search_time_track_nr(int **rxdata, ///rx data in time domain
                       NR_DL_FRAME_PARMS *frame_parms,
                       int position,
                       int length,
                       int eNB_id,
		                   int *f_off)
{
   unsigned int n, ar, peak_position;
  int64_t peak_value;
  int64_t result;
  int64_t avg=0;
  int winPos = position-(length>>1);
  peak_value = 0;
  peak_position = 0;
  AssertFatal(length>0,"illegal length %d\n",length);

// ===========================================  PSS定时估计   ===========================================
  for (n=0; n < length; n+=4) { //

    int64_t pss_corr_ue=0;
    /* calculate dot product of primary_synchro_time_nr and rxdata[ar][n]
      * (ar=0..nb_ant_rx) and store the sum in temp[n]; */
    for (ar=0; ar<frame_parms->nb_antennas_rx; ar++) {
      /* perform correlation of rx data and pss sequence ie it is a dot product */
      result  = dot_product64((short*)primary_synchro_time_nr[eNB_id],
                              (short*)&(rxdata[ar][n+winPos]),
                              frame_parms->ofdm_symbol_size,
                              pss_corr_shift);
      pss_corr_ue += abs64(result);
    }
    /* calculate the absolute value of sync_corr[n] */
    avg+=pss_corr_ue;
    if (pss_corr_ue > peak_value) {
      peak_value = pss_corr_ue;
      peak_position = n;  
    }
  }

  // ===========================================  SNR计算   ===========================================
  avg /= length>>2;
  uint16_t trackSNR=dB_fixed64(peak_value)-dB_fixed64(avg);
  if (trackSNR < 8) {/* SNR */
    LOG_I(PHY,"[TRACK SYNC] Sync SNR is too low, can't find PSS Position.\n");
    return(-1);
  }
  /* add the start offset in frame */
  peak_position += winPos; // add_zjw

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> PSS FFO EST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

     int64_t result1,result2;
     double ffo_pss=0;
	  // Computing cross-correlation at peak on half the symbol size for first half of data
	  result1  = dot_product64((short*)primary_synchro_time_nr[eNB_id], 
				  (short*) &(rxdata[0][peak_position]), 
				  frame_parms->ofdm_symbol_size>>1, 
				  pss_corr_shift);
	  // Computing cross-correlation at peak on half the symbol size for data shifted by half symbol size 
	  // as it is real and complex it is necessary to shift by a value equal to symbol size to obtain such shift
	  result2  = dot_product64((short*)primary_synchro_time_nr[eNB_id]+(frame_parms->ofdm_symbol_size), 
				  (short*) &(rxdata[0][peak_position])+frame_parms->ofdm_symbol_size, 
				  frame_parms->ofdm_symbol_size>>1, 
				  pss_corr_shift);

	  int64_t re1,re2,im1,im2;
	  re1=((int*) &result1)[0];
	  re2=((int*) &result2)[0];
	  im1=((int*) &result1)[1];
	  im2=((int*) &result2)[1];

 	  // estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	  ffo_pss=-atan2(re1*im2-re2*im1,re1*re2+im1*im2)/M_PI;
    LOG_D(PHY,"[UE_SYNC_FFO_PSS]  FFO_PSS Est res:  ffo_pss=%f \n",ffo_pss);

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> CP FFO EST <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<//

    // double ffo_cp=0;
    // int64_t ffo_cp_corr = 0;
    // int64_t ffo_cp_est = 0;
    // int offsetUnit = frame_parms->nb_prefix_samples + frame_parms->ofdm_symbol_size;
    // int PosStart = peak_position - frame_parms->nb_prefix_samples;  
    // int cp_corr_shift = frame_parms->ffo_corr_shift >0 ? frame_parms->ffo_corr_shift : pss_corr_shift;
    //     for (int i = 0; i < 2; i++){
    //         ffo_cp_corr = dot_product64((short*)&(rxdata[0][PosStart+i*offsetUnit]), 
    //                                   (short*)&(rxdata[0][PosStart+i*offsetUnit+frame_parms->ofdm_symbol_size]), // 修改：调用前完成数据截取
    //                                   frame_parms->nb_prefix_samples,
    //                                   cp_corr_shift);
    //       ((int32_t*) &ffo_cp_est)[0] = ((int32_t*) &ffo_cp_est)[0]+((int32_t*) &ffo_cp_corr)[0];
    //       ((int32_t*) &ffo_cp_est)[1] = ((int32_t*) &ffo_cp_est)[1]+((int32_t*) &ffo_cp_corr)[1];
    //     }
    //     int32_t re3,im3;
    //     re3=((int32_t*) &ffo_cp_est)[0];
    //     im3=((int32_t*) &ffo_cp_est)[1];
    //     ffo_cp=-atan2(im3,re3)/2/M_PI; // ffo=-angle()/2/pi

  // ===========================================  输出量计算   =========================================== 
    *f_off += ffo_pss*frame_parms->subcarrier_spacing;  
    LOG_I(PHY,"[TRC SYNC] SNR=%d, sync_pos=%d(%d), ffo_cp=%f, cfo_track=%d \n",trackSNR,peak_position,length,ffo_pss*frame_parms->subcarrier_spacing,*f_off);
    return(peak_position);
}

