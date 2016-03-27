/*
 * filter.h
 *
 *  Created on: Jul 23, 2015
 *      Author: Pablo
 */

#ifndef FILTER_H_
#define FILTER_H_
int16_t Filter2ndOrder(int16_t NewValue,
                       float *Ins,
                       float *Outs,
                       const float *ACoeffs,
                       const float *BCoeffs);

int16_t Filter1stOrder(int16_t NewValue,
                       float *Ins,
                       float *Outs,
                       const float *ACoeffs,
                       const float *BCoeffs);

extern const float Butter2_3_ACoeffs[2];
extern const float Butter2_3_BCoeffs[3];
extern const float Butter2_2_ACoeffs[2];
extern const float Butter2_2_BCoeffs[3];
extern const float Butter2_15_ACoeffs[2];
extern const float Butter2_15_BCoeffs[3];
extern const float Butter2_1_ACoeffs[2];
extern const float Butter2_1_BCoeffs[3];
extern const float Butter2_01_ACoeffs[2];
extern const float Butter2_01_BCoeffs[3];
extern const float Butter2_03_ACoeffs[2];
extern const float Butter2_03_BCoeffs[3];
extern const float Butter2_04_ACoeffs[2];
extern const float Butter2_04_BCoeffs[3];
extern const float Butter2_16_ACoeffs[2];
extern const float Butter2_16_BCoeffs[3];
extern const float Butter2_05_ACoeffs[2];
extern const float Butter2_05_BCoeffs[3];
extern const float Butter1_04_ACoeffs[1];
extern const float Butter1_04_BCoeffs[2];
extern const float Butter1_03_ACoeffs[1];
extern const float Butter1_03_BCoeffs[2];


#endif /* FILTER_H_ */
