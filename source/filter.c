/*
 * filter.c
 *
 *  Created on: Jan 17, 2016
 *      Author: Pablo
 *
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

#include "ch.h"
#include "hal.h"

const float Butter2_3_ACoeffs[2]={0.74779f,-0.27221f};
const float Butter2_3_BCoeffs[3]={0.13111f,0.26221f,0.13111f};

const float Butter2_2_ACoeffs[2]={1.14298f,-0.41280f};
const float Butter2_2_BCoeffs[3]={0.067455f,0.134911f,0.067455f};

const float Butter2_15_ACoeffs[2]={1.34897f,-0.51398f};
const float Butter2_15_BCoeffs[3]={0.041254f,0.082507f,0.041254f};

const float Butter2_1_ACoeffs[2]={1.56102f,-0.64135f};
const float Butter2_1_BCoeffs[3]={0.020083f,0.040167f,0.020083f};

const float Butter2_01_ACoeffs[2]={1.95558f,-0.95654f};
const float Butter2_01_BCoeffs[3]={0.00024136f,0.00048272f,0.00024136f};

const float Butter2_03_ACoeffs[2]={1.86689f,-0.87521f};
const float Butter2_03_BCoeffs[3]={0.0020806f,0.0041611f,0.0020806f};

const float Butter2_04_ACoeffs[2]={1.82269f,-0.83718f};
const float Butter2_04_BCoeffs[3]={0.0036217f,0.0072434f,0.0036217f};

const float Butter2_05_ACoeffs[2]={1.77863f,-0.80080f};
const float Butter2_05_BCoeffs[3]={0.0055427f,0.0110854f,0.0055427f};

const float Butter2_16_ACoeffs[2]={1.30729f,-0.49181f};
const float Butter2_16_BCoeffs[3]={0.046132f,0.092264f,0.046132f};


const float Butter1_04_ACoeffs[1]={0.88162f};
const float Butter1_04_BCoeffs[2]={0.059191f,0.059191f};

const float Butter1_03_ACoeffs[1]={0.90993f};
const float Butter1_03_BCoeffs[2]={0.045035f,0.045035f};

int16_t Filter2ndOrder(int16_t NewValue,
                       float *Ins,
                       float *Outs,
                       const float *ACoeffs,
                       const float *BCoeffs)
{
  float Xn;
  float Acc;

  Xn = (float)NewValue;
  Acc = (BCoeffs[0] * Xn) +
      (BCoeffs[1] * Ins[0]) +
      (BCoeffs[2] * Ins[1]) +
      (ACoeffs[0] * Outs[0]) +
      (ACoeffs[1] * Outs[1]);
  Ins[1]=Ins[0];
  Ins[0]=Xn;
  Outs[1]=Outs[0];
  Outs[0]=Acc;
  return (int16_t) (lrintf(Acc));
}

int16_t Filter1stOrder(int16_t NewValue,
                       float *Ins,
                       float *Outs,
                       const float *ACoeffs,
                       const float *BCoeffs)
{
  float Xn;
  float acc;

  Xn = (float)NewValue;
  acc = (BCoeffs[0] * Xn) +
      (BCoeffs[1] * Ins[0]) +
      (ACoeffs[0] * Outs[0]);

  Ins[0]=Xn;
  Outs[0]=acc;
  return (int16_t) (lrintf(acc));
}

