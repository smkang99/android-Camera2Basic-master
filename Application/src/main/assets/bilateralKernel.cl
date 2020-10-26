//
//  clKernelsExample.cl
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

#define sigmaDomain 3.0f
#define sigmaRange  0.2f
#define oneover255 0.00392156862745f
#define filterWidth 2

#ifndef VECN
#define VECN 16
#endif

#if (VECN == 16)
#define FLOATN float16
#define UCHARN uchar16
#define USHORTN ushort16
#define CONV_FLOATN convert_float16
#else
#if (VECN == 8)
    #define FLOATN float8
    #define UCHARN uchar8
    #define USHORTN ushort8
    #define CONV_FLOATN convert_float8
  #else
    #define FLOATN float4
    #define UCHARN uchar4
    #define USHORTN ushort4
    #define CONV_FLOATN convert_float4
  #endif
#endif

#ifndef VECM
#define VECM 4
#endif

#if (VECM == 16)
#define FLOATM float16
  #define CONV_FLOATM convert_float16
#else
#if (VECM == 8)
#define FLOATM float8
    #define CONV_FLOATM convert_float8
#else
#define FLOATM float4
#define CONV_FLOATM convert_float4
#endif
#endif

#define MAX_LOCAL_MEMORY_SIZE 32768  // 32KB

__kernel void bilateralFilterKernel(__global uchar4 *srcBuffer,
                                    __global uchar4 *dstBuffer, 
                                    const int width, const int height)
{

    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;
    float4  sum4 = (float4)0.0f;
    
	if ( (x >= filterWidth) && (x < (width - filterWidth)) &&     //avoid reading outside of buffer
         (y >= filterWidth) && (y < (height - filterWidth)) )
	{
		float4 centerPixel = oneover255 * convert_float4(srcBuffer[centerIndex]);  
		float normalizeCoeff = 0.0f;
	    
		for (int yy=-filterWidth; yy<=filterWidth; yy++)
		{
			for (int xx=-filterWidth; xx<=filterWidth; xx++)
			{
				int thisIndex = (y+yy) * width + (x+xx); 
				float4 currentPixel = oneover255 * convert_float4(srcBuffer[thisIndex]);
                float domainDistance = fast_distance((float)(xx), (float)(yy));

                float domainWeight = exp(-0.5f * pow((domainDistance/sigmaDomain),2.0f));

                float diffx = currentPixel.x - centerPixel.x;
                float diffy = currentPixel.y - centerPixel.y;
                float diffz = currentPixel.z - centerPixel.z;
                
                float rangeDistance = sqrt(diffx*diffx + diffy*diffy + diffz*diffz);
//				float rangeDistance = fast_distance(currentPixel.xyz, centerPixel.xyz);
				float rangeWeight = exp(-0.5f * pow((rangeDistance/sigmaRange),2.0f));
	            
	            float totalWeight = domainWeight * rangeWeight;
	            normalizeCoeff += totalWeight;
				sum4 += totalWeight * currentPixel;
			}
		}
		sum4 /= normalizeCoeff;
        sum4.w = 1.0f; // set alpha to fully opaque
	}
    
	dstBuffer[centerIndex] = convert_uchar4_sat_rte(255.0f * sum4);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Recon Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////


inline float DotMatrixVector(FLOATN m, FLOATN v)
{
#if (VECN==16)
    return (
        dot(m.s0123, v.s0123)+
        dot(m.s4567, v.s4567)+
        dot(m.s89AB, v.s89AB)+
        dot(m.sCDEF, v.sCDEF)
    );
#else
#if (VECN==8)
    return (
        dot(m.s0123, v.s0123)+
        dot(m.s4567, v.s4567)
    );
#else // VECN==4
    return (
            dot(m, v)
    );
#endif
#endif
}

__kernel void ReconKernel(    __global FLOATN *srcBuffer,
__global FLOATM *dstBuffer,
__constant FLOATN* const lambda __attribute__((max_constant_size (1024))),
__global uchar   *mapBuffer,
const float      sat_val,
const int width, const int height)
{
int x = get_global_id(0);
int y = get_global_id(1);
int centerIndex = y * width + x;

dstBuffer[centerIndex] = mapBuffer[centerIndex] ? (FLOATM) sat_val : (FLOATM)

#if (VECM == 16)
(
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 0]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 1]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 2]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 3]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 4]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 5]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 6]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 7]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 8]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 9]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[10]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[11]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[12]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[13]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[14]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[15])
    );
#else
#if (VECM == 8)
(
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 0]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 1]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 2]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 3]),
    DotMatrixVector(srcBuffer[centerIndex], lambda[ 4]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 5]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 6]),
        DotMatrixVector(srcBuffer[centerIndex], lambda[ 7])
    );
#else // VECM==4
(
DotMatrixVector(srcBuffer[centerIndex], lambda[0]
),
DotMatrixVector(srcBuffer[centerIndex], lambda[1]
),
DotMatrixVector(srcBuffer[centerIndex], lambda[2]
),
DotMatrixVector(srcBuffer[centerIndex], lambda[3]
)
);
#endif
#endif
}

__kernel void PreKernel (     __global uchar *srcBuffer,
__global ushort *dstBuffer,
const int dx, const int dy,
const ushort blacklevel,
const int width, const int height)
{
int x = get_global_id(0);
int y = get_global_id(1);
int inputIndex = y * width + x;
int xx = ((x+dx)>=width)  ? width-1  : x+dx;
int yy = ((y+dy)>=height) ? height-1 : y+dy;
int centerIndex = yy * width + xx;

ushort p = (ushort)srcBuffer[inputIndex];
p = (p < blacklevel) ? 0 : p-blacklevel;
dstBuffer[centerIndex] = p;
}

