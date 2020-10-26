//
//  openCLNR.cpp
//  OpenCL Example1
//
//  Created by Rasmusson, Jim on 18/03/13.
//
//  Copyright (c) 2013, Sony Mobile Communications AB
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of Sony Mobile Communications AB nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#define __CL_ENABLE_EXCEPTIONS

#include "openCLNR.h"
#include "hassproc.h"

float LutHassLambda2[67][16] = {
        // 350
        {
                (float) 0.536334,
                (float) -0.145395,
                (float) -0.036945,
                (float) 0.013983,
                (float) 0.000010,
                (float) 0.006555,
                (float) 0.027848,
                (float) -0.028205,
                (float) 0.027093,
                (float) -0.002534,
                (float) 0.015493,
                (float) -0.016844,
                (float) -0.005681,
                (float) -0.007270,
                (float) -0.054821,
                (float) -0.006873,
        },
        // 360
        {
                (float) 1.072588,
                (float) -0.290000,
                (float) -0.073900,
                (float) 0.028000,
                (float) 0.000021,
                (float) 0.013100,
                (float) 0.055600,
                (float) -0.056400,
                (float) 0.054000,
                (float) -0.005080,
                (float) 0.031000,
                (float) -0.033600,
                (float) -0.011400,
                (float) -0.014600,
                (float) -0.109000,
                (float) -0.013700,
        },
        // 370
        {
                (float) 1.608683,
                (float) -0.436000,
                (float) -0.111000,
                (float) 0.042100,
                (float) 0.000047,
                (float) 0.019600,
                (float) 0.083400,
                (float) -0.084600,
                (float) 0.081000,
                (float) -0.007550,
                (float) 0.046500,
                (float) -0.050500,
                (float) -0.017000,
                (float) -0.022000,
                (float) -0.164000,
                (float) -0.020500,
        },
        // 380
        {
                (float) 2.142117,
                (float) -0.579595,
                (float) -0.148000,
                (float) 0.056000,
                (float) 0.000265,
                (float) 0.025900,
                (float) 0.111000,
                (float) -0.112000,
                (float) 0.106000,
                (float) -0.010500,
                (float) 0.063200,
                (float) -0.065500,
                (float) -0.021900,
                (float) -0.030100,
                (float) -0.219000,
                (float) -0.027800,
        },
        // 390
        {
                (float) 2.663831,
                (float) -0.720064,
                (float) -0.184000,
                (float) 0.072300,
                (float) 0.000342,
                (float) 0.030700,
                (float) 0.141000,
                (float) -0.139000,
                (float) 0.122000,
                (float) -0.020500,
                (float) 0.079500,
                (float) -0.069100,
                (float) -0.016000,
                (float) -0.036600,
                (float) -0.274000,
                (float) -0.042900,
        },
        // 400
        {
                (float) 3.158692,
                (float) -0.848894,
                (float) -0.228000,
                (float) 0.098500,
                (float) -0.000073,
                (float) 0.028700,
                (float) 0.170000,
                (float) -0.157000,
                (float) 0.124000,
                (float) -0.044900,
                (float) 0.082600,
                (float) -0.058800,
                (float) 0.009850,
                (float) -0.031000,
                (float) -0.320000,
                (float) -0.074300,
        },
        // 410
        {
                (float) 3.622487,
                (float) -0.979710,
                (float) -0.263000,
                (float) 0.131000,
                (float) -0.002490,
                (float) 0.017900,
                (float) 0.192000,
                (float) -0.161000,
                (float) 0.115000,
                (float) -0.080600,
                (float) 0.070300,
                (float) -0.048100,
                (float) 0.050800,
                (float) -0.008060,
                (float) -0.345000,
                (float) -0.120000,
        },
        // 420
        {
                (float) 4.037697,
                (float) -1.097416,
                (float) -0.294000,
                (float) 0.178000,
                (float) -0.008020,
                (float) -0.002180,
                (float) 0.205000,
                (float) -0.152000,
                (float) 0.106000,
                (float) -0.120000,
                (float) 0.040900,
                (float) -0.048500,
                (float) 0.092600,
                (float) 0.025100,
                (float) -0.340000,
                (float) -0.172000,
        },
        // 430
        {
                (float) 4.344775,
                (float) -1.114737,
                (float) -0.376000,
                (float) 0.240000,
                (float) 0.004330,
                (float) -0.036900,
                (float) 0.206000,
                (float) -0.126000,
                (float) 0.105000,
                (float) -0.141000,
                (float) -0.001420,
                (float) -0.062000,
                (float) 0.106000,
                (float) 0.054400,
                (float) -0.314000,
                (float) -0.222000,
        },
        // 440
        {
                (float) 4.447664,
                (float) -0.933628,
                (float) -0.558000,
                (float) 0.299000,
                (float) 0.062800,
                (float) -0.087400,
                (float) 0.184000,
                (float) -0.087000,
                (float) 0.110000,
                (float) -0.145000,
                (float) -0.053200,
                (float) -0.092600,
                (float) 0.089400,
                (float) 0.070300,
                (float) -0.262000,
                (float) -0.237000,
        },
        // 450
        {
                (float) 4.280420,
                (float) -0.527435,
                (float) -0.805434,
                (float) 0.302000,
                (float) 0.164000,
                (float) -0.124000,
                (float) 0.124000,
                (float) -0.048500,
                (float) 0.112000,
                (float) -0.139000,
                (float) -0.106000,
                (float) -0.135000,
                (float) 0.045700,
                (float) 0.073900,
                (float) -0.180000,
                (float) -0.195000,
        },
        // 460
        {
                (float) 3.808005,
                (float) 0.070711,
                (float) -1.000446,
                (float) 0.164000,
                (float) 0.287000,
                (float) -0.106000,
                (float) 0.032900,
                (float) -0.024600,
                (float) 0.108000,
                (float) -0.125000,
                (float) -0.145000,
                (float) -0.172000,
                (float) -0.009690,
                (float) 0.065500,
                (float) -0.088200,
                (float) -0.108000,
        },
        // 470
        {
                (float) 3.068396,
                (float) 0.754000,
                (float) -1.033855,
                (float) -0.146000,
                (float) 0.387000,
                (float) -0.014900,
                (float) -0.065500,
                (float) -0.033500,
                (float) 0.080200,
                (float) -0.118000,
                (float) -0.174000,
                (float) -0.191000,
                (float) -0.057600,
                (float) 0.056800,
                (float) 0.017300,
                (float) 0.017900,
        },
        // 480
        {
                (float) 2.125233,
                (float) 1.444180,
                (float) -0.895333,
                (float) -0.540000,
                (float) 0.396000,
                (float) 0.137000,
                (float) -0.135000,
                (float) -0.074700,
                (float) 0.022300,
                (float) -0.128000,
                (float) -0.194000,
                (float) -0.182000,
                (float) -0.077100,
                (float) 0.062000,
                (float) 0.126000,
                (float) 0.148000,
        },
        // 490
        {
                (float) 1.044621,
                (float) 2.077801,
                (float) -0.621902,
                (float) -0.874874,
                (float) 0.242000,
                (float) 0.308000,
                (float) -0.142000,
                (float) -0.133000,
                (float) -0.064800,
                (float) -0.155000,
                (float) -0.207000,
                (float) -0.155000,
                (float) -0.067100,
                (float) 0.085000,
                (float) 0.230000,
                (float) 0.266000,
        },
        // 500
        {
                (float) -0.042506,
                (float) 2.521932,
                (float) -0.203792,
                (float) -1.050540,
                (float) -0.091000,
                (float) 0.434000,
                (float) -0.071100,
                (float) -0.182000,
                (float) -0.174000,
                (float) -0.190000,
                (float) -0.202000,
                (float) -0.120000,
                (float) -0.051200,
                (float) 0.111000,
                (float) 0.325000,
                (float) 0.368000,
        },
        // 510
        {
                (float) -0.975000,
                (float) 2.637533,
                (float) 0.382000,
                (float) -1.061306,
                (float) -0.511000,
                (float) 0.451202,
                (float) 0.052800,
                (float) -0.188000,
                (float) -0.286000,
                (float) -0.225000,
                (float) -0.178000,
                (float) -0.080200,
                (float) -0.031800,
                (float) 0.133000,
                (float) 0.387000,
                (float) 0.430000,
        },
        // 520
        {
                (float) -1.652262,
                (float) 2.413879,
                (float) 1.050000,
                (float) -0.907211,
                (float) -0.882263,
                (float) 0.307872,
                (float) 0.185000,
                (float) -0.128000,
                (float) -0.366000,
                (float) -0.250000,
                (float) -0.133000,
                (float) -0.032900,
                (float) -0.020000,
                (float) 0.128000,
                (float) 0.392000,
                (float) 0.437497,
        },
        // 530
        {
                (float) -2.043439,
                (float) 1.916794,
                (float) 1.692067,
                (float) -0.602079,
                (float) -1.113823,
                (float) 0.012400,
                (float) 0.278000,
                (float) -0.001040,
                (float) -0.378000,
                (float) -0.252000,
                (float) -0.080200,
                (float) 0.017200,
                (float) -0.021100,
                (float) 0.084200,
                (float) 0.336000,
                (float) 0.381762,
        },
        // 540
        {
                (float) -2.157411,
                (float) 1.232920,
                (float) 2.191694,
                (float) -0.133080,
                (float) -1.196571,
                (float) -0.363000,
                (float) 0.284000,
                (float) 0.166000,
                (float) -0.306000,
                (float) -0.228000,
                (float) -0.040900,
                (float) 0.056400,
                (float) -0.024900,
                (float) 0.025700,
                (float) 0.227000,
                (float) 0.266000,
        },
        // 550
        {
                (float) -2.045624,
                (float) 0.495337,
                (float) 2.430126,
                (float) 0.468000,
                (float) -1.120656,
                (float) -0.712000,
                (float) 0.163000,
                (float) 0.327000,
                (float) -0.155000,
                (float) -0.172000,
                (float) -0.017200,
                (float) 0.075500,
                (float) -0.028600,
                (float) -0.033600,
                (float) 0.084600,
                (float) 0.109000,
        },
        // 560
        {
                (float) -1.783634,
                (float) -0.179559,
                (float) 2.363427,
                (float) 1.099641,
                (float) -0.869551,
                (float) -0.955596,
                (float) -0.081400,
                (float) 0.425000,
                (float) 0.046900,
                (float) -0.087000,
                (float) -0.005760,
                (float) 0.063200,
                (float) -0.033500,
                (float) -0.082200,
                (float) -0.056800,
                (float) -0.048100,
        },
        // 570
        {
                (float) -1.449424,
                (float) -0.707000,
                (float) 2.008917,
                (float) 1.638159,
                (float) -0.422838,
                (float) -1.077633,
                (float) -0.387000,
                (float) 0.418587,
                (float) 0.254000,
                (float) 0.014300,
                (float) -0.010400,
                (float) 0.027200,
                (float) -0.035300,
                (float) -0.104000,
                (float) -0.167000,
                (float) -0.175000,
        },
        // 580
        {
                (float) -1.105401,
                (float) -1.061226,
                (float) 1.473378,
                (float) 1.966053,
                (float) 0.172409,
                (float) -1.060551,
                (float) -0.665000,
                (float) 0.284832,
                (float) 0.414000,
                (float) 0.126000,
                (float) -0.026300,
                (float) -0.022200,
                (float) -0.037500,
                (float) -0.102000,
                (float) -0.228000,
                (float) -0.250000,
        },
        // 590
        {
                (float) -0.779970,
                (float) -1.266170,
                (float) 0.870266,
                (float) 2.060600,
                (float) 0.804000,
                (float) -0.890605,
                (float) -0.847583,
                (float) 0.044100,
                (float) 0.484254,
                (float) 0.236000,
                (float) -0.037200,
                (float) -0.073900,
                (float) -0.045300,
                (float) -0.091800,
                (float) -0.236000,
                (float) -0.272000,
        },
        // 600
        {
                (float) -0.486240,
                (float) -1.345701,
                (float) 0.263380,
                (float) 1.956599,
                (float) 1.374700,
                (float) -0.573437,
                (float) -0.912613,
                (float) -0.250000,
                (float) 0.435749,
                (float) 0.330000,
                (float) -0.027400,
                (float) -0.114000,
                (float) -0.056800,
                (float) -0.084200,
                (float) -0.206000,
                (float) -0.248000,
        },
        // 610
        {
                (float) -0.235969,
                (float) -1.312371,
                (float) -0.305000,
                (float) 1.682731,
                (float) 1.832814,
                (float) -0.130300,
                (float) -0.871100,
                (float) -0.522000,
                (float) 0.261791,
                (float) 0.380000,
                (float) 0.010300,
                (float) -0.132000,
                (float) -0.069900,
                (float) -0.079100,
                (float) -0.157000,
                (float) -0.194000,
        },
        // 620
        {
                (float) -0.040100,
                (float) -1.179727,
                (float) -0.786000,
                (float) 1.256199,
                (float) 2.133417,
                (float) 0.418000,
                (float) -0.724712,
                (float) -0.722884,
                (float) -0.007590,
                (float) 0.360000,
                (float) 0.062800,
                (float) -0.122000,
                (float) -0.078700,
                (float) -0.075100,
                (float) -0.106000,
                (float) -0.123000,
        },
        // 630
        {
                (float) 0.098900,
                (float) -0.976810,
                (float) -1.129951,
                (float) 0.728645,
                (float) 2.214457,
                (float) 1.023964,
                (float) -0.446792,
                (float) -0.834434,
                (float) -0.316000,
                (float) 0.261000,
                (float) 0.110000,
                (float) -0.089800,
                (float) -0.087800,
                (float) -0.073100,
                (float) -0.059200,
                (float) -0.060400,
        },
        // 640
        {
                (float) 0.186000,
                (float) -0.737106,
                (float) -1.315986,
                (float) 0.171614,
                (float) 2.062070,
                (float) 1.594660,
                (float) -0.024987,
                (float) -0.841942,
                (float) -0.590000,
                (float) 0.077900,
                (float) 0.132000,
                (float) -0.039400,
                (float) -0.099300,
                (float) -0.089400,
                (float) -0.024900,
                (float) -0.011200,
        },
        // 650
        {
                (float) 0.226000,
                (float) -0.488464,
                (float) -1.350150,
                (float) -0.352000,
                (float) 1.706368,
                (float) 2.042207,
                (float) 0.517000,
                (float) -0.735279,
                (float) -0.779493,
                (float) -0.163000,
                (float) 0.104000,
                (float) 0.011600,
                (float) -0.106000,
                (float) -0.115000,
                (float) -0.012900,
                (float) 0.016600,
        },
        // 660
        {
                (float) 0.228000,
                (float) -0.253449,
                (float) -1.258185,
                (float) -0.783000,
                (float) 1.191009,
                (float) 2.304793,
                (float) 1.124906,
                (float) -0.491762,
                (float) -0.869471,
                (float) -0.428000,
                (float) 0.016900,
                (float) 0.039100,
                (float) -0.097700,
                (float) -0.144000,
                (float) -0.017000,
                (float) 0.012500,
        },
        // 670
        {
                (float) 0.203000,
                (float) -0.050500,
                (float) -1.080533,
                (float) -1.081526,
                (float) 0.587738,
                (float) 2.329502,
                (float) 1.729568,
                (float) -0.094944,
                (float) -0.860295,
                (float) -0.675691,
                (float) -0.135000,
                (float) 0.017700,
                (float) -0.081800,
                (float) -0.164000,
                (float) -0.032900,
                (float) -0.001520,
        },
        // 680
        {
                (float) 0.160000,
                (float) 0.114000,
                (float) -0.862321,
                (float) -1.237766,
                (float) -0.007826,
                (float) 2.110297,
                (float) 2.231539,
                (float) 0.449000,
                (float) -0.741118,
                (float) -0.864546,
                (float) -0.334000,
                (float) -0.061600,
                (float) -0.077900,
                (float) -0.182000,
                (float) -0.045300,
                (float) -0.026400,
        },
        // 690
        {
                (float) 0.108000,
                (float) 0.240000,
                (float) -0.639540,
                (float) -1.275863,
                (float) -0.522000,
                (float) 1.711890,
                (float) 2.539292,
                (float) 1.074097,
                (float) -0.490490,
                (float) -0.960244,
                (float) -0.548000,
                (float) -0.203000,
                (float) -0.107000,
                (float) -0.209000,
                (float) -0.053600,
                (float) -0.040900,
        },
        // 700
        {
                (float) 0.046900,
                (float) 0.331000,
                (float) -0.433008,
                (float) -1.229344,
                (float) -0.918930,
                (float) 1.219056,
                (float) 2.624543,
                (float) 1.687061,
                (float) -0.110040,
                (float) -0.945387,
                (float) -0.738695,
                (float) -0.392000,
                (float) -0.176000,
                (float) -0.240000,
                (float) -0.054800,
                (float) -0.058400,
        },
        // 710
        {
                (float) -0.022100,
                (float) 0.390000,
                (float) -0.254640,
                (float) -1.127647,
                (float) -1.189222,
                (float) 0.699327,
                (float) 2.500798,
                (float) 2.210683,
                (float) 0.374000,
                (float) -0.821999,
                (float) -0.876066,
                (float) -0.598000,
                (float) -0.283000,
                (float) -0.277000,
                (float) -0.056000,
                (float) -0.080200,
        },
        // 720
        {
                (float) -0.102000,
                (float) 0.417000,
                (float) -0.112000,
                (float) -0.996275,
                (float) -1.342324,
                (float) 0.208161,
                (float) 2.201983,
                (float) 2.585453,
                (float) 0.914000,
                (float) -0.600609,
                (float) -0.953054,
                (float) -0.795900,
                (float) -0.409000,
                (float) -0.306000,
                (float) -0.052000,
                (float) -0.098500,
        },
        // 730
        {
                (float) -0.195000,
                (float) 0.409371,
                (float) -0.007470,
                (float) -0.853144,
                (float) -1.395953,
                (float) -0.219000,
                (float) 1.777437,
                (float) 2.768111,
                (float) 1.448947,
                (float) -0.289599,
                (float) -0.962826,
                (float) -0.961237,
                (float) -0.542372,
                (float) -0.323000,
                (float) -0.040100,
                (float) -0.095700,
        },
        // 740
        {
                (float) -0.300000,
                (float) 0.367063,
                (float) 0.058000,
                (float) -0.712278,
                (float) -1.374064,
                (float) -0.563000,
                (float) 1.289330,
                (float) 2.747851,
                (float) 1.902810,
                (float) 0.089400,
                (float) -0.897597,
                (float) -1.068099,
                (float) -0.665958,
                (float) -0.332000,
                (float) -0.012400,
                (float) -0.059200,
        },
        // 750
        {
                (float) -0.415000,
                (float) 0.295000,
                (float) 0.090600,
                (float) -0.582296,
                (float) -1.300413,
                (float) -0.824105,
                (float) 0.795701,
                (float) 2.550852,
                (float) 2.225222,
                (float) 0.501000,
                (float) -0.753274,
                (float) -1.098251,
                (float) -0.760624,
                (float) -0.334000,
                (float) 0.024000,
                (float) -0.001510,
        },
        // 760
        {
                (float) -0.536572,
                (float) 0.194000,
                (float) 0.094100,
                (float) -0.467887,
                (float) -1.192837,
                (float) -1.009305,
                (float) 0.328132,
                (float) 2.211557,
                (float) 2.388137,
                (float) 0.903278,
                (float) -0.538360,
                (float) -1.047362,
                (float) -0.809248,
                (float) -0.317000,
                (float) 0.070300,
                (float) 0.079100,
        },
        // 770
        {
                (float) -0.663098,
                (float) 0.066700,
                (float) 0.070700,
                (float) -0.369844,
                (float) -1.064881,
                (float) -1.127488,
                (float) -0.101000,
                (float) 1.777834,
                (float) 2.388375,
                (float) 1.253657,
                (float) -0.268544,
                (float) -0.918930,
                (float) -0.801104,
                (float) -0.277000,
                (float) 0.128000,
                (float) 0.187000,
        },
        // 780
        {
                (float) -0.786842,
                (float) -0.075100,
                (float) 0.030600,
                (float) -0.286000,
                (float) -0.927312,
                (float) -1.195737,
                (float) -0.480000,
                (float) 1.302916,
                (float) 2.259426,
                (float) 1.528597,
                (float) 0.034700,
                (float) -0.716012,
                (float) -0.735279,
                (float) -0.221000,
                (float) 0.186000,
                (float) 0.292000,
        },
        // 790
        {
                (float) -0.908919,
                (float) -0.233000,
                (float) -0.025600,
                (float) -0.218000,
                (float) -0.788670,
                (float) -1.225888,
                (float) -0.796774,
                (float) 0.810638,
                (float) 2.002243,
                (float) 1.694172,
                (float) 0.353000,
                (float) -0.437894,
                (float) -0.602913,
                (float) -0.129000,
                (float) 0.244000,
                (float) 0.397000,
        },
        // 800
        {
                (float) -1.022653,
                (float) -0.396000,
                (float) -0.093400,
                (float) -0.160000,
                (float) -0.652451,
                (float) -1.223624,
                (float) -1.058604,
                (float) 0.324557,
                (float) 1.650594,
                (float) 1.771438,
                (float) 0.672000,
                (float) -0.111231,
                (float) -0.417276,
                (float) -0.017700,
                (float) 0.299000,
                (float) 0.497000,
        },
        // 810
        {
                (float) -1.119424,
                (float) -0.550000,
                (float) -0.158000,
                (float) -0.110000,
                (float) -0.521715,
                (float) -1.201894,
                (float) -1.277849,
                (float) -0.120000,
                (float) 1.284126,
                (float) 1.788759,
                (float) 0.964535,
                (float) 0.234000,
                (float) -0.195847,
                (float) 0.097300,
                (float) 0.329000,
                (float) 0.552264,
        },
        // 820
        {
                (float) -1.206502,
                (float) -0.703498,
                (float) -0.228000,
                (float) -0.069900,
                (float) -0.399996,
                (float) -1.164989,
                (float) -1.450179,
                (float) -0.525000,
                (float) 0.879284,
                (float) 1.706169,
                (float) 1.210356,
                (float) 0.607000,
                (float) 0.075100,
                (float) 0.242000,
                (float) 0.364000,
                (float) 0.594730,
        },
        // 830
        {
                (float) -1.269865,
                (float) -0.832646,
                (float) -0.287000,
                (float) -0.032600,
                (float) -0.284434,
                (float) -1.110248,
                (float) -1.583338,
                (float) -0.885997,
                (float) 0.487948,
                (float) 1.618376,
                (float) 1.450417,
                (float) 0.973711,
                (float) 0.347000,
                (float) 0.357000,
                (float) 0.358000,
                (float) 0.570537,
        },
        // 840
        {
                (float) -1.315032,
                (float) -0.944394,
                (float) -0.338000,
                (float) -0.001300,
                (float) -0.180000,
                (float) -1.049746,
                (float) -1.683010,
                (float) -1.194386,
                (float) 0.134669,
                (float) 1.501226,
                (float) 1.629420,
                (float) 1.307763,
                (float) 0.629000,
                (float) 0.481000,
                (float) 0.342000,
                (float) 0.512459,
        },
        // 850
        {
                (float) -1.346574,
                (float) -1.045177,
                (float) -0.388000,
                (float) 0.025000,
                (float) -0.084200,
                (float) -0.977406,
                (float) -1.744624,
                (float) -1.462215,
                (float) -0.228000,
                (float) 1.339146,
                (float) 1.781251,
                (float) 1.637365,
                (float) 0.930371,
                (float) 0.608237,
                (float) 0.321000,
                (float) 0.424228,
        },
        // 860
        {
                (float) -1.353169,
                (float) -1.114617,
                (float) -0.421000,
                (float) 0.049300,
                (float) 0.001690,
                (float) -0.901450,
                (float) -1.778112,
                (float) -1.677130,
                (float) -0.525000,
                (float) 1.207098,
                (float) 1.905790,
                (float) 1.906425,
                (float) 1.192956,
                (float) 0.705207,
                (float) 0.273000,
                (float) 0.291585,
        },
        // 870
        {
                (float) -1.355354,
                (float) -1.187593,
                (float) -0.460657,
                (float) 0.064800,
                (float) 0.074700,
                (float) -0.822913,
                (float) -1.781767,
                (float) -1.853472,
                (float) -0.834235,
                (float) 1.003466,
                (float) 1.955606,
                (float) 2.150260,
                (float) 1.479853,
                (float) 0.843292,
                (float) 0.269000,
                (float) 0.189000,
        },
        // 880
        {
                (float) -1.331717,
                (float) -1.224776,
                (float) -0.482506,
                (float) 0.079800,
                (float) 0.138000,
                (float) -0.742032,
                (float) -1.759878,
                (float) -1.978448,
                (float) -1.072270,
                (float) 0.850006,
                (float) 1.997437,
                (float) 2.327674,
                (float) 1.703428,
                (float) 0.933191,
                (float) 0.232000,
                (float) 0.048100,
        },
        // 890
        {
                (float) -1.308875,
                (float) -1.270818,
                (float) -0.515676,
                (float) 0.085800,
                (float) 0.187000,
                (float) -0.663773,
                (float) -1.714949,
                (float) -2.069181,
                (float) -1.318290,
                (float) 0.623690,
                (float) 1.947065,
                (float) 2.460834,
                (float) 1.942417,
                (float) 1.079699,
                (float) 0.264000,
                (float) -0.029200,
        },
        // 900
        {
                (float) -1.259933,
                (float) -1.278286,
                (float) -0.528627,
                (float) 0.091800,
                (float) 0.226000,
                (float) -0.585236,
                (float) -1.646780,
                (float) -2.107556,
                (float) -1.488235,
                (float) 0.456406,
                (float) 1.903406,
                (float) 2.530314,
                (float) 2.094605,
                (float) 1.157759,
                (float) 0.261000,
                (float) -0.132000,
        },
        // 910
        {
                (float) -1.213137,
                (float) -1.293342,
                (float) -0.551548,
                (float) 0.089000,
                (float) 0.249000,
                (float) -0.512459,
                (float) -1.560337,
                (float) -2.114110,
                (float) -1.654844,
                (float) 0.228024,
                (float) 1.769134,
                (float) 2.549700,
                (float) 2.246634,
                (float) 1.284484,
                (float) 0.327000,
                (float) -0.158000,
        },
        // 920
        {
                (float) -1.140677,
                (float) -1.268196,
                (float) -0.551946,
                (float) 0.086600,
                (float) 0.263000,
                (float) -0.441747,
                (float) -1.454906,
                (float) -2.068466,
                (float) -1.736043,
                (float) 0.066700,
                (float) 1.651229,
                (float) 2.503738,
                (float) 2.293471,
                (float) 1.327864,
                (float) 0.353000,
                (float) -0.194000,
        },
        // 930
        {
                (float) -1.068655,
                (float) -1.244083,
                (float) -0.556911,
                (float) 0.077100,
                (float) 0.262000,
                (float) -0.377789,
                (float) -1.336365,
                (float) -1.989849,
                (float) -1.798213,
                (float) -0.131000,
                (float) 1.455541,
                (float) 2.404146,
                (float) 2.325569,
                (float) 1.402150,
                (float) 0.431000,
                (float) -0.163000,
        },
        // 940
        {
                (float) -0.974823,
                (float) -1.182429,
                (float) -0.540068,
                (float) 0.067900,
                (float) 0.253000,
                (float) -0.318000,
                (float) -1.203960,
                (float) -1.864197,
                (float) -1.775133,
                (float) -0.262000,
                (float) 1.277969,
                (float) 2.240000,
                (float) 2.248144,
                (float) 1.396708,
                (float) 0.476030,
                (float) -0.132000,
        },
        // 950
        {
                (float) -0.873762,
                (float) -1.103057,
                (float) -0.514644,
                (float) 0.055600,
                (float) 0.233000,
                (float) -0.263000,
                (float) -1.059399,
                (float) -1.699178,
                (float) -1.707798,
                (float) -0.381000,
                (float) 1.062895,
                (float) 2.032395,
                (float) 2.128928,
                (float) 1.374263,
                (float) 0.519450,
                (float) -0.080200,
        },
        // 960
        {
                (float) -0.758002,
                (float) -0.994924,
                (float) -0.473091,
                (float) 0.043300,
                (float) 0.206000,
                (float) -0.213000,
                (float) -0.906178,
                (float) -1.497889,
                (float) -1.570070,
                (float) -0.447000,
                (float) 0.852946,
                (float) 1.768101,
                (float) 1.923865,
                (float) 1.294892,
                (float) 0.541816,
                (float) -0.019100,
        },
        // 970
        {
                (float) -0.625716,
                (float) -0.846192,
                (float) -0.407583,
                (float) 0.033200,
                (float) 0.174000,
                (float) -0.164000,
                (float) -0.738655,
                (float) -1.251074,
                (float) -1.361193,
                (float) -0.450407,
                (float) 0.660118,
                (float) 1.481601,
                (float) 1.656950,
                (float) 1.136824,
                (float) 0.494979,
                (float) -0.003510,
        },
        // 980
        {
                (float) -0.484691,
                (float) -0.676723,
                (float) -0.330913,
                (float) 0.022500,
                (float) 0.135000,
                (float) -0.120000,
                (float) -0.565492,
                (float) -0.977167,
                (float) -1.100396,
                (float) -0.413423,
                (float) 0.463278,
                (float) 1.145563,
                (float) 1.328182,
                (float) 0.943639,
                (float) 0.436662,
                (float) 0.025300,
        },
        // 990
        {
                (float) -0.330119,
                (float) -0.473369,
                (float) -0.233000,
                (float) 0.014100,
                (float) 0.093800,
                (float) -0.077100,
                (float) -0.382954,
                (float) -0.673863,
                (float) -0.777666,
                (float) -0.315818,
                (float) 0.297544,
                (float) 0.790894,
                (float) 0.935376,
                (float) 0.672552,
                (float) 0.319393,
                (float) 0.023500,
        },
        // 1000
        {
                (float) -0.169000,
                (float) -0.247092,
                (float) -0.123000,
                (float) 0.006400,
                (float) 0.047700,
                (float) -0.037900,
                (float) -0.194000,
                (float) -0.344817,
                (float) -0.408378,
                (float) -0.178000,
                (float) 0.136000,
                (float) 0.402419,
                (float) 0.492755,
                (float) 0.362296,
                (float) 0.176000,
                (float) 0.017000,
        },
        // None
        {
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
                (float) 0.125,
        },
};

inline std::string loadProgram(std::string input)
{
	std::ifstream stream(input.c_str());
	if (!stream.is_open()) {
		LOGE("Cannot open input file\n");
		exit(1);
	}
	return std::string( std::istreambuf_iterator<char>(stream),
						(std::istreambuf_iterator<char>()));
}

void openCLNR (unsigned char* bufIn, unsigned char* bufOut, int* info)
{

	LOGI("\n\nStart openCLNR (i.e., OpenCL on the GPU)");
    int xOffset=3;
    int yOffset=7;
    unsigned short blackLevel=60;


	int width = info[0];
	int height = info[1];
	int hass_iwidth=width;
    int hass_iheight=height;
	unsigned int imageSize = width * height * 4 * sizeof(cl_uchar);

	cl_int err = CL_SUCCESS;
	try {

        std::vector<cl::Platform> platforms;
        cl::Platform::get(&platforms);
        if (platforms.size() == 0) {
            std::cout << "Platform size 0\n";
            return;
        }

        cl_context_properties properties[] =
                {CL_CONTEXT_PLATFORM, (cl_context_properties) (platforms[0])(), 0};
        cl::Context context(CL_DEVICE_TYPE_GPU, properties);

        std::vector<cl::Device> devices = context.getInfo<CL_CONTEXT_DEVICES>();
        cl::CommandQueue queue(context, devices[0], 0, &err);

        std::string kernelSource = loadProgram(
                "/data/data/com.example.android.camera2basic/app_execdir/bilateralKernel.cl");

        cl::Program::Sources source(1, std::make_pair(kernelSource.c_str(),
                                                      kernelSource.length() + 1));
        cl::Program program(context, source);
        const char *options = "-cl-fast-relaxed-math";
        program.build(devices, options);

        //PreKernel;
        size_t globalSize[2] = {(size_t) (width), (size_t) height};
        size_t *localSize = NULL;

        cl::Kernel prekernel(program, "PreKernel", &err);
        cl::Buffer bufferIn = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR,
                                         imageSize, (void *) &bufIn[0], &err);
        cl::Buffer bufferOut = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR,
                                          imageSize, (void *) &bufOut[0], &err);

        int hass_channel = 16, hass_spectrum = 4;


        int d_recon_coef_sz = hass_channel * hass_spectrum * sizeof(float);
        float *hass_recon_coef = new float[hass_channel * hass_spectrum];

        for (int c = 0; c < hass_channel; c++) {
            for (int s = 0; s < hass_spectrum; s++) {
                //HASS RGB mode
                switch(s){
                    case 0: hass_recon_coef[c+hass_channel*s] = LutHassLambda2[WAVELENGTH_600][c]; break;
                    case 1: hass_recon_coef[c+hass_channel*s] = LutHassLambda2[WAVELENGTH_530][c]; break;
                    case 2: hass_recon_coef[c+hass_channel*s] = LutHassLambda2[WAVELENGTH_460][c]; break;
                }
            }
        }

        cl::Buffer d_recon_coef = cl::Buffer(context, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, d_recon_coef_sz, (void *)&hass_recon_coef[0], &err);
        cl::Buffer d_sat_map;

		prekernel.setArg(0,bufferIn);
		prekernel.setArg(1,bufferOut);
        prekernel.setArg(2,sizeof(xOffset),((void *)(&xOffset)));

        prekernel.setArg(3,sizeof(yOffset),((void *)(&yOffset)));

        prekernel.setArg(4,sizeof(blackLevel) ,((void *)(&blackLevel)));

        prekernel.setArg(5,sizeof(hass_iwidth),((void *)(&hass_iwidth)));

		prekernel.setArg(6,sizeof(hass_iheight),((void *)(&hass_iheight)));
		queue.enqueueNDRangeKernel(prekernel, cl::NullRange, cl::NDRange(
				width, height), cl::NullRange, 0, NULL);

        //Noise reduction
		cl::Kernel kernel(program, "bilateralFilterKernel", &err);

		kernel.setArg(0,bufferIn);
		kernel.setArg(1,bufferOut);
		kernel.setArg(2,width);
		kernel.setArg(3,height);

		cl::Event event;

		clock_t startTimer1, stopTimer1;
		startTimer1=clock();

// 		one time
		queue.enqueueNDRangeKernel(	kernel,
				cl::NullRange,
				cl::NDRange(width,height),
				cl::NullRange,
				NULL,
				&event);

//		swap in and out buffer pointers and run a 2nd time
		kernel.setArg(0,bufferOut);
		kernel.setArg(1,bufferIn);
		queue.enqueueNDRangeKernel(	kernel,
				cl::NullRange,
				cl::NDRange(width,height),
				cl::NullRange,
				NULL,
				&event);

//		swap in and out buffer pointers and run a 3rd time
		kernel.setArg(0,bufferIn);
		kernel.setArg(1,bufferOut);
		queue.enqueueNDRangeKernel(	kernel,
				cl::NullRange,
				cl::NDRange(width,height),
				cl::NullRange,
				NULL,
				&event);


        //Reduction
        int hass_raw_input_bit =10, hass_sat_th;
        hass_sat_th = (1 << hass_raw_input_bit) - 1 -blackLevel -1;
        float hass_sat_val = (float)hass_sat_th;

        cl::Kernel recon_kernel(program, "ReconKernel", &err);
        recon_kernel.setArg(0, bufferOut);
        recon_kernel.setArg(1, bufferIn);
        recon_kernel.setArg(2, d_recon_coef);
        recon_kernel.setArg(3, d_sat_map);
        recon_kernel.setArg(4, hass_sat_val);
        recon_kernel.setArg(5, width);
        recon_kernel.setArg(6, height);

		cl_int dimSiz   = 2;
        queue.enqueueNDRangeKernel(recon_kernel,dimSiz,cl::NDRange(width,height),cl::NDRange(128,8), NULL, &event);

        //		swap in and out buffer pointers and run a 3rd time
        kernel.setArg(0,bufferIn);
        kernel.setArg(1,bufferOut);
        queue.enqueueNDRangeKernel(	kernel,
                                       cl::NullRange,
                                       cl::NDRange(width,height),
                                       cl::NullRange,
                                       NULL,
                                       &event);

		queue.finish();

		stopTimer1 = clock();
		double elapse = 1000.0* (double)(stopTimer1 - startTimer1)/(double)CLOCKS_PER_SEC;
		info[2] = (int)elapse;
		LOGI("OpenCL code on the GPU took %g ms\n\n", 1000.0* (double)(stopTimer1 - startTimer1)/(double)CLOCKS_PER_SEC) ;

		queue.enqueueReadBuffer(bufferOut, CL_TRUE, 0, imageSize, bufOut);
	}
	catch (cl::Error err) {
		LOGE("ERROR: %s\n", err.what());
	}
	return;
}
