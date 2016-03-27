/*
 * filter.h
 *
 *  Created on: Jul 23, 2015
 *      Author: Pablo
 *  This file is part of ChibiFlight.

    ChibiFlight is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ChibiFlight is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ChibiFlight.  If not, see <http://www.gnu.org/licenses/>.
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
