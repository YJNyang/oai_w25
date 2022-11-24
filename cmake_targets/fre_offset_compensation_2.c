#include<stdio.h>
#include<immintrin.h>
#include<stdint.h>
#include<math.h>
#include<stdlib.h>
#include<time.h>
#define M_PI 3.1415926575
//rxdata is single antennas_rx;   start:stamples start
void fre_offset_compensation_simd_test(int32_t* rxdata, int start, int end, int samples_per_subframe, double freq_offset) {
    int nb_samples_within_simd = 8;
    float s_time = 1 / (1.0e3 * samples_per_subframe);  // sampling time
    float off_angle = 2 * M_PI * s_time * (freq_offset);  // offset rotation angle compensation per sample
    float phase_re = cos(nb_samples_within_simd * off_angle);
    float phase_im = sin(nb_samples_within_simd * off_angle);
    __m256 base_phase_re = _mm256_set1_ps(phase_re);
    __m256 base_phase_im = _mm256_set1_ps(phase_im);
    int nb_samples_start = 0;
    int nb_samples_end = 0;
    if (((int64_t)rxdata) % 256)
    {
        float re, im;
        if (((((int64_t)rxdata) % 256) % 32))
        {
            printf("Error, Addresses are not aligned in 32 bits!\n");
            return 0;
        }
        nb_samples_start = nb_samples_within_simd - ((((int64_t)rxdata) % 256) / 32);
        printf("nb_samples_start = %d\n", nb_samples_start);
        for (int i = start; i < start + nb_samples_start; i++)
        {
            re = (float)(((short*)rxdata)[2 * i]);
            im = (float)(((short*)rxdata)[2 * i + 1]);
            ((short*)rxdata)[2 * i] = (short)(round(re * cos(i * off_angle) - im * sin(i * off_angle)));
            ((short*)rxdata)[2 * i + 1] = (short)(round(im * cos(i * off_angle) + re * sin(i * off_angle)));
        }
        //((int32_t*)rxdata) += nb_samples_start;
    }

    __m256 real_phase_re = _mm256_setr_ps(cos((start + nb_samples_start) * off_angle), cos((start + nb_samples_start + 1) * off_angle), cos((start + nb_samples_start + 2) * off_angle), cos((start + nb_samples_start + 3) * off_angle),
        cos((start + nb_samples_start + 4) * off_angle), cos((start + nb_samples_start + 5) * off_angle), cos((start + nb_samples_start + 6) * off_angle), cos((start + nb_samples_start + 7) * off_angle));
    __m256 real_phase_im = _mm256_setr_ps(sin((start + nb_samples_start) * off_angle), sin((start + nb_samples_start + 1) * off_angle), sin((start + nb_samples_start + 2) * off_angle), sin((start + nb_samples_start + 3) * off_angle),
        sin((start + nb_samples_start + 4) * off_angle), sin((start + nb_samples_start + 5) * off_angle), sin((start + nb_samples_start + 6) * off_angle), sin((start + nb_samples_start + 7) * off_angle));

    __m256 real_phase_re_tem;
    __m256 real_phase_im_tem;
    __m256i* addr_store = (__m256i*)((int32_t*)rxdata + nb_samples_start);
    nb_samples_end = (end - start - nb_samples_start) % 8;  //remain
    printf("nb_samples_end = %d\n", nb_samples_end);
    int end_simd = end - nb_samples_end;
    for (int n = start + nb_samples_start; n < end_simd; n += 8) {

        __m256 rx_re = _mm256_setr_ps((float)(((short*)rxdata)[2 * n]), (float)(((short*)rxdata)[2 * n + 2]),
            (float)(((short*)rxdata)[2 * n + 4]), (float)(((short*)rxdata)[2 * n + 6]),
            (float)(((short*)rxdata)[2 * n + 8]), (float)(((short*)rxdata)[2 * n + 10]),
            (float)(((short*)rxdata)[2 * n + 12]), (float)(((short*)rxdata)[2 * n + 14]));
        __m256 rx_im = _mm256_setr_ps((float)(((short*)rxdata)[2 * n + 1]), (float)(((short*)rxdata)[2 * n + 3]),
            (float)(((short*)rxdata)[2 * n + 5]), (float)(((short*)rxdata)[2 * n + 7]),
            (float)(((short*)rxdata)[2 * n + 9]), (float)(((short*)rxdata)[2 * n + 11]),
            (float)(((short*)rxdata)[2 * n + 13]), (float)(((short*)rxdata)[2 * n + 15]));

        __m256 data_re = _mm256_fmsub_ps(rx_re, real_phase_re, _mm256_mul_ps(rx_im, real_phase_im)); 
        __m256 data_im = _mm256_fmadd_ps(rx_im, real_phase_re, _mm256_mul_ps(rx_re, real_phase_im)); //

        __m256i data_re_int = _mm256_cvtps_epi32(data_re);
        __m256i data_im_int = _mm256_cvtps_epi32(data_im);
        __m256i data = _mm256_blend_epi16(data_re_int, _mm256_slli_epi32(data_im_int, 16), 0b10101010);
        _mm256_store_si256((((__m256i*)(addr_store)) + ((n - start) / 8)), data);

        real_phase_re_tem = real_phase_re;
        real_phase_im_tem = real_phase_im;

        real_phase_re = _mm256_fmsub_ps(real_phase_re_tem, base_phase_re, _mm256_mul_ps(real_phase_im_tem, base_phase_im));
        real_phase_im = _mm256_fmadd_ps(real_phase_im_tem, base_phase_re, _mm256_mul_ps(real_phase_re_tem, base_phase_im));

    }
    if (nb_samples_end)
    {
        for (int i = end_simd; i < end; i++)
        {
            float re, im;
            re = (float)(((short*)rxdata)[2 * i]);
            im = (float)(((short*)rxdata)[2 * i + 1]);
            ((short*)rxdata)[2 * i] = (short)(round(re * cos((i)*off_angle) - im * sin((i)*off_angle)));
            ((short*)rxdata)[2 * i + 1] = (short)(round(im * cos((i)*off_angle) + re * sin((i)*off_angle)));
        }
    }
}