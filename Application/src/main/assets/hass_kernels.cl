
////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Vector Length Definition
//
////////////////////////////////////////////////////////////////////////////////////////////////////

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

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Shrink Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef REP_UNIT
  #define REP_UNIT 8
#endif

#ifndef SHRINK_SIZE
  #define SHRINK_SIZE REP_UNIT
#endif

#ifndef SHRINK_TAP
  #define SHRINK_TAP SHRINK_SIZE
#endif


__kernel void ShrinkKernel(__global ushort  *srcBuffer,
                           __global FLOATN  *dstBuffer,
                           __constant float* const coeff __attribute__((max_constant_size (32))),
                           __constant UCHARN* const color __attribute__((max_constant_size (1024))),
                           const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int widthShrink = width/SHRINK_SIZE;
    int heightShrink = height/SHRINK_SIZE;
    int centerIndex = y * widthShrink + x;
    FLOATN  sum = (FLOATN) 0.0f;
    FLOATN normalizeCoeff = (FLOATN)0.0f;

    {
        for (int yy=-(SHRINK_TAP-1); yy<=(SHRINK_TAP-1); yy++)
        {
                int yOrg = y * SHRINK_SIZE + yy;
                if(yOrg < 0 || yOrg >= height ) continue;

                for (int xx=-(SHRINK_TAP-1); xx<=(SHRINK_TAP-1); xx++)
                {
                        int xOrg = x * SHRINK_SIZE + xx;
                        if(xOrg < 0 || xOrg >= width ) continue;

                        int thisIndex = yOrg * width + xOrg;

                        USHORTN thisPixel = srcBuffer[thisIndex];

                        FLOATN  currentPixel = CONV_FLOATN(thisPixel);

                        FLOATN Weight = coeff[abs(xx)]*coeff[abs(yy)];

                        FLOATN Channel = CONV_FLOATN(color[(xOrg%REP_UNIT)+(yOrg%REP_UNIT)*REP_UNIT]);
                        Weight *= Channel; // Select Channel

                        normalizeCoeff += Weight;
                        sum += Weight * currentPixel;
                }
        }
        sum /= normalizeCoeff;
     }

     dstBuffer[centerIndex] = sum;
}


__kernel void ShrinkColKernel(__global ushort  *srcBuffer,
                              __global FLOATN  *dstBuffer,
                               __constant float* const coeff __attribute__((max_constant_size (32))),
                               __constant UCHARN* const color __attribute__((max_constant_size (1024))),
                               const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int widthShrink = width/SHRINK_SIZE;
    int heightShrink = height/SHRINK_SIZE;
    int centerIndex = y * widthShrink + x;
    FLOATN  sum = (FLOATN) 0.0f;

    {
         int yOrg = y;

         for (int xx=-(SHRINK_TAP-1); xx<=(SHRINK_TAP-1); xx++)
         {
                 int xOrg = x * SHRINK_SIZE + xx;
                 if(xOrg < 0 || xOrg >= width ) continue;

                 int thisIndex = yOrg * width + xOrg;
                 USHORTN thisPixel = srcBuffer[thisIndex];

                 FLOATN  currentPixel = CONV_FLOATN(thisPixel);

                 FLOATN Weight = coeff[abs(xx)];

                 FLOATN Channel = CONV_FLOATN(color[(xOrg%REP_UNIT)+(yOrg%REP_UNIT)*REP_UNIT]);
                 Weight *= Channel; // Select Channel

                 sum += Weight * currentPixel;
          }
     }

     dstBuffer[centerIndex] = sum;
}

__kernel void ShrinkRowKernel(__global FLOATN  *srcBuffer,
                              __global FLOATN  *dstBuffer,
                               __constant float*  const coeff    __attribute__((max_constant_size (32))),
                               __constant FLOATN* const coeffsum __attribute__((max_constant_size (256))),
                               const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int widthShrink = width/SHRINK_SIZE;
    int heightShrink = height/SHRINK_SIZE;
    int centerIndex = y * widthShrink + x;
    FLOATN  sum = (FLOATN) 0.0f;

    {
        for (int yy=-(SHRINK_TAP-1); yy<=(SHRINK_TAP-1); yy++)
        {
                int yOrg = y * SHRINK_SIZE + yy;
                if(yOrg < 0 || yOrg >= height ) continue;

                int thisIndex = yOrg * widthShrink + x;

                FLOATN currentPixel = srcBuffer[thisIndex];

                FLOATN Weight = coeff[abs(yy)];

                sum += Weight * currentPixel;
        }

        int    sel = (x!=0)+(y!=0)*2;
        FLOATN normalizeCoeff = coeffsum[sel];
        sum /= normalizeCoeff;
     }

     dstBuffer[centerIndex] = sum;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Nr Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////

__kernel void NRKernel(    __global FLOATN *srcBuffer,
                           __global FLOATN *dstBuffer,
			   const int NR_TAP,
                           const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;
    FLOATN  sum   = (FLOATN) 0.0f;
    int     coeff = 0;
    int     NR_TAPH = NR_TAP/2;

    {
	for (int yy=-NR_TAPH; yy<=NR_TAPH; yy++)
	{
	    	int yOrg = y + yy;
		if(yOrg < 0 || yOrg >= height) continue; 
	
		for (int xx=-NR_TAPH; xx<=NR_TAPH; xx++)
		{
			int xOrg = x + xx;

			if(xOrg < 0 || xOrg >= width) continue; 

			int thisIndex = yOrg * width + xOrg;
			FLOATN currentPixel = srcBuffer[thisIndex];
            
			sum += currentPixel;
			coeff++; 
		}
	}

	sum /= coeff;
	dstBuffer[centerIndex] = sum;
    }

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

  #if (VECM==16)
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
    #if (VECM==8)
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
	DotMatrixVector(srcBuffer[centerIndex], lambda[ 0]),
	DotMatrixVector(srcBuffer[centerIndex], lambda[ 1]),
	DotMatrixVector(srcBuffer[centerIndex], lambda[ 2]),
	DotMatrixVector(srcBuffer[centerIndex], lambda[ 3])
    );
    #endif
  #endif

}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  App Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////


__kernel void AppKernel  (    __global FLOATM *srcBuffer,
                   	      __global FLOATM *dstBuffer,
                              const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;

    dstBuffer[centerIndex] = srcBuffer[centerIndex];
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Zoom Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////

__kernel void ZoomKernel(__global FLOATM *srcBuffer,
                         __global float  *gainBuffer,
                         __global FLOATM *dstBuffer,
		         __constant float* const coeff __attribute__((max_constant_size (32))),
			 const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int widthShrink = width/SHRINK_SIZE;
    int heightShrink = height/SHRINK_SIZE;	

#ifdef USE_LM_FOR_ZOOM
    __local FLOATM cache[(MAX_LOCAL_MEMORY_SIZE/(4*VECM))];

    int lx = get_local_id(0);
    int ly = get_local_id(1);
    int lwidth = get_local_size(0)+1;

    cache[lx+ly*lwidth] = srcBuffer[x+y*widthShrink];

    if((lx==(get_local_size(0)-1)) && (ly==(get_local_size(1)-1)))
       if(((x+1)<widthShrink)&&((y+1)<heightShrink))
          cache[lx+1+(ly+1)*lwidth] = srcBuffer[(x+1)+(y+1)*widthShrink];
       else
          cache[lx+1+(ly+1)*lwidth] = srcBuffer[x+y*widthShrink];

    if(lx==(get_local_size(0)-1))
       if((x+1)<widthShrink)
          cache[lx+1+ly*lwidth] = srcBuffer[(x+1)+y*widthShrink];
       else
          cache[lx+1+ly*lwidth] = srcBuffer[x+y*widthShrink];

    if(ly==(get_local_size(1)-1))
       if((y+1)<heightShrink)
          cache[lx+(ly+1)*lwidth] = srcBuffer[x+(y+1)*widthShrink];
       else
          cache[lx+(ly+1)*lwidth] = srcBuffer[x+y*widthShrink];

    barrier(CLK_LOCAL_MEM_FENCE);

#endif

    for (int yy=0; yy<SHRINK_SIZE; yy++){
        for (int xx=0; xx<SHRINK_SIZE; xx++){
	    FLOATM sum = (FLOATM) 0.0f;
	    float coefx[2];
	    coefx[0] = coeff[xx];
	    coefx[1] = 1.0f - coefx[0];
	    float coefy[2];
	    coefy[0] = coeff[yy];
	    coefy[1] = 1.0f - coefy[0];

	    for (int yyy=0; yyy<2; yyy++){
	        for (int xxx=0; xxx<2; xxx++){

		    FLOATM Weight = coefx[xxx]*coefy[yyy];
#ifdef USE_LM_FOR_ZOOM
		    FLOATM Pixels = cache[(lx+xxx)+(ly+yyy)*lwidth];
#else
		    int xShr = x+xxx;
		    if(xShr > widthShrink-1) xShr = widthShrink-1;

		    int yShr = y+yyy;
		    if(yShr > heightShrink-1) yShr = heightShrink-1;
	            

	       	    FLOATM Pixels = srcBuffer[xShr+yShr*widthShrink];

#endif
		    sum += Weight * Pixels;
		}
            }

	    if(((x*SHRINK_SIZE+xx)<width)&&((y*SHRINK_SIZE+yy)<height)){
	        float gain = gainBuffer[(x*SHRINK_SIZE+xx)+(y*SHRINK_SIZE+yy)*width];
		dstBuffer[(x*SHRINK_SIZE+xx)+(y*SHRINK_SIZE+yy)*width] = sum * gain;
            }
	}
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Guide (Tentative)
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#define GUIDE_TAP (REP_UNIT+1)
#define GUIDE_TAP_HALF (GUIDE_TAP/2)

__kernel void GuideKernel (   __global ushort  *srcBuffer,
                              __global float   *dstBuffer,
			      __constant float* const coeff,
//			      __constant FLOATN* const coeff,
			      __constant uchar* const color,
			      const int width, const int height)
{
    int    x = get_global_id(0);
    int    y = get_global_id(1);
    int    centerIndex = y * width + x;
    float  sum = 0.0f;
    float  normalizeCoeff = 0.0f;
    int    tapnum = REP_UNIT+1;

#ifdef USE_LM_FOR_GUIDE
    __local ushort  cache[MAX_LOCAL_MEMORY_SIZE];

    int    lx     = get_local_id(0);
    int    ly     = get_local_id(1);
    int    lwidth = get_local_size(0)+GUIDE_TAP_HALF*2;

    cache[(lx+GUIDE_TAP_HALF)+(ly+GUIDE_TAP_HALF)*lwidth] = srcBuffer[x+y*width];

    if( lx < GUIDE_TAP_HALF ){
    	cache[(lx)+(ly+GUIDE_TAP_HALF)*lwidth] = srcBuffer[(x-GUIDE_TAP_HALF)+y*width];         
    }else if ( lx >= get_local_size(0)-GUIDE_TAP_HALF ){
    	cache[(lx+GUIDE_TAP_HALF*2)+(ly+GUIDE_TAP_HALF)*lwidth] = srcBuffer[(x+GUIDE_TAP_HALF)+y*width];         
    }

    if( ly < GUIDE_TAP_HALF ){
    	cache[(lx+GUIDE_TAP_HALF)+(ly)*lwidth] = srcBuffer[x+(y-GUIDE_TAP_HALF)*width];         
    }else if ( ly >= get_local_size(1)-GUIDE_TAP_HALF ){
    	cache[(lx+GUIDE_TAP_HALF)+(ly+GUIDE_TAP_HALF*2)*lwidth] = srcBuffer[x+(y+GUIDE_TAP_HALF)*width];         
    }

    barrier(CLK_LOCAL_MEM_FENCE);
#endif

/*
    union {
      FLOATN vec;
      float  no[VECN] __attribute__ ((packed));
    } Coeff;

    Coeff.vec = coeff[((x%REP_UNIT)+(y%REP_UNIT)*REP_UNIT)];
*/


    {
	for (int yy=0; yy<tapnum; yy++){
            for (int xx=0; xx<tapnum; xx++){

		int cx  = (x + xx - tapnum/2);
		int cy  = (y + yy - tapnum/2);

//		uchar ch       = color[(cx%REP_UNIT)+(cy%REP_UNIT)*REP_UNIT];
//		float Weight   = coeff[((x%REP_UNIT)+(y%REP_UNIT)*REP_UNIT)*VECN + ch];

		uchar ch       = color[(cx&(REP_UNIT-1))+(cy&(REP_UNIT-1))*REP_UNIT];
		float Weight   = coeff[((x&(REP_UNIT-1))+(y&(REP_UNIT-1))*REP_UNIT)*VECN + ch];

//		float Weight   = Coeff.no[ch];
//		float Weight   = 1.0;

		cx = (cx < 0) ? 0 : cx;
		cy = (cy < 0) ? 0 : cy;
		cx = (cx > width-1)  ? width -1 : cx;						
		cy = (cy > height-1) ? height-1 : cy;				
//		if(cx < 0) cx = 0;		
//		if(cy < 0) cy = 0;
//		if(cx > width -1) cx = width -1;
//		if(cy > height-1) cy = height -1;


#ifdef USE_LM_FOR_GUIDE
	        float currentPixel = (float) srcBuffer[(ly+yy-GUIDE_TAP_HALF) * lwidth + (lx+xx-GUIDE_TAP_HALF)];
#else
	        float currentPixel = (float) srcBuffer[cy * width + cx];
#endif		
		sum += Weight * currentPixel;    
		normalizeCoeff += Weight;
	    }
    	}

	sum /= normalizeCoeff;
     }

     dstBuffer[centerIndex] = sum;
}

__kernel void GuideColKernel (   __global ushort  *srcBuffer,
                                 __global float   *dstBuffer,
                                 const int width, const int height)
{
    int    x = get_global_id(0);
    int    y = get_global_id(1);
    int    centerIndex = y * width + x;
    float  sum = 0.0f;
    int    tapnum = REP_UNIT+1;

    {
        for (int xx=0; xx<tapnum; xx++){

                int cx  = (x + xx - tapnum/2);
                int cy  = y;

                float Weight   = 1.0;

                cx = (cx < 0) ? 0 : cx;
                cy = (cy < 0) ? 0 : cy;
                cx = (cx > width-1)  ? width -1 : cx;
                cy = (cy > height-1) ? height-1 : cy;

                float currentPixel = (float) srcBuffer[cy * width + cx];

                sum += Weight * currentPixel;
        }
     }

     dstBuffer[centerIndex] = sum;
}

__kernel void GuideRowKernel (   __global float   *srcBuffer,
                                 __global float   *dstBuffer,
                                 const int width, const int height)
{
    int    x = get_global_id(0);
    int    y = get_global_id(1);
    int    centerIndex = y * width + x;
    float  sum = 0.0f;
    int    tapnum = REP_UNIT+1;
    float  normalizeCoeff = tapnum*tapnum;

    {
        for (int yy=0; yy<tapnum; yy++){
                int cx  = x;
                int cy  = (y + yy - tapnum/2);

                float Weight   = 1.0;

                cx = (cx < 0) ? 0 : cx;
                cy = (cy < 0) ? 0 : cy;
                cx = (cx > width-1)  ? width -1 : cx;
                cy = (cy > height-1) ? height-1 : cy;

                float currentPixel = (float) srcBuffer[cy * width + cx];

                sum += Weight * currentPixel;
        }

        sum /= normalizeCoeff;
     }

     dstBuffer[centerIndex] = sum;
}




////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Combine (Tentative)
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef COMBINE_TAP
  #define COMBINE_TAP ((REP_UNIT/2) + (REP_UNIT/4))
#endif

__kernel void CombineKernel ( __global float  *srcBuffer,
                              __global float  *dstBuffer,
			      __constant float* const coeff,
			      const float combine_noise, // 0.1
			      const int width, const int height)
{
    int    x = get_global_id(0);
    int    y = get_global_id(1);
    int    centerIndex = y * width + x;
    float  sum = 0.0f;
    float  normalizeCoeff = 0.0f;
    float  hpf;

    {
	for(int yy = -(COMBINE_TAP-1); yy <= (COMBINE_TAP-1); yy++) {
   	    for(int xx = -(COMBINE_TAP-1); xx <= (COMBINE_TAP-1); xx++) {

	    	int cx  = x + xx;
		int cy  = y + yy;

		if(cx < 0) cx = 0;
		if(cy < 0) cy = 0;
		if(cx > width -1) cx = width -1;
		if(cy > height-1) cy = height -1;
                        
		float Weight = coeff[abs(yy)] * coeff[abs(xx)];

		float currentPixel = srcBuffer[cy * width + cx];
	        sum += Weight * currentPixel;    
		normalizeCoeff += Weight;
	    }
    	}

	sum /= normalizeCoeff;
	hpf = max(srcBuffer[y * width + x], combine_noise) / max(sum, combine_noise);
     }

     dstBuffer[centerIndex] = hpf;
}


 __kernel void CombineColKernel ( __global float  *srcBuffer,
                                  __global float  *dstBuffer,
 			          __constant float* const coeff __attribute__( (max_constant_size(28) ) ),
 			          const float combine_noise, // 0.1				  
 			          const int width, const int height)
 {
    int    x = get_global_id(0);    // column index
    int    y = get_global_id(1);    // row index

    int    cyW = y*width;
    int    centerIndex = cyW + x;
    float  sum = 0.0f;
    float  normalizeCoeff = 0.0f;

    for(int xx = -(COMBINE_TAP-1); xx <= (COMBINE_TAP-1); xx++) {
        int cx  = x + xx;
        if(cx < 0) cx = 0;
        if(cx > width -1) cx = width -1;
        int cxW = cyW + cx;
        // Calculate the weight
        float Weight = coeff[abs(xx)];

        // Filtering
        float currentPixel = srcBuffer[cxW];
        sum += Weight * currentPixel;
//        sum = mad(Weight, currentPixel, sum);
    }

    dstBuffer[centerIndex] = sum;
 }

 __kernel void CombineRowKernel ( __global float  *srcBuffer,
                                  __global float  *intBuffer,
                                  __global float  *dstBuffer,
 			          __constant float* const coeff __attribute__( (max_constant_size(28) ) ),
 			          const float combine_noise, // 0.1
 			          const int width, const int height)
 {
    int    x = get_global_id(0);    // column index
    int    y = get_global_id(1);    // row index

    int    centerIndex = y*width + x;
    float  sum = 0.0f;
    float  normalizeCoeff = 0.0f;
    float  hpf;

    for(int yy = -(COMBINE_TAP-1); yy <= (COMBINE_TAP-1); yy++) {
        int cy  = y + yy;
        if(cy < 0) cy = 0;
        if(cy > height -1) cy = height -1;
        int cyW = cy*width + x;
        // Calculate the weight
        float Weight = coeff[abs(yy)];

        // Filtering
        float currentPixel = intBuffer[cyW];
        sum += Weight * currentPixel;
    }

 	sum /= coeff[COMBINE_TAP];
 	hpf = max(srcBuffer[centerIndex], combine_noise) / max(sum, combine_noise);
    dstBuffer[centerIndex] = hpf;
 }


////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Pre Kernel
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#define SET_HASS_WDR

#ifdef RAW10
__kernel void PreKernel (     __global uchar   *srcBuffer,
                              __global ushort  *dstBuffer,
			      const int dx, const int dy,
			      const ushort blacklevel,			      
                              const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    int i0  = x*5;
    int i1  = x*5+1;
    int i2  = x*5+2;
    int i3  = x*5+3;
    int i4  = x*5+4;

    int o0  = x*4;
    int o1  = x*4+1;
    int o2  = x*4+2;
    int o3  = x*4+3;

    int xx0  = ((o0+dx)>=width)  ? width-1  : o0+dx;
    int xx1  = ((o1+dx)>=width)  ? width-1  : o1+dx;
    int xx2  = ((o2+dx)>=width)  ? width-1  : o2+dx;
    int xx3  = ((o3+dx)>=width)  ? width-1  : o3+dx;        
    int yy  = ((y +dy)>=height) ? height-1 : y+dy;

    int raw10_width = (width * 10)>>3; 
    uchar  lb = srcBuffer[yy * raw10_width + i4];
    
    ushort p0 = srcBuffer[y * raw10_width + i0];
    ushort p1 = srcBuffer[y * raw10_width + i1];
    ushort p2 = srcBuffer[y * raw10_width + i2];
    ushort p3 = srcBuffer[y * raw10_width + i3];

    p0 = (p0<<2)|((lb   )&0x3);
    p1 = (p1<<2)|((lb>>2)&0x3); 
    p2 = (p2<<2)|((lb>>4)&0x3);
    p3 = (p3<<2)|((lb>>6)&0x3);

#ifndef SET_HASS_WDR
    p0 = (p0 < blacklevel) ? 0 : p0-blacklevel;
    p1 = (p1 < blacklevel) ? 0 : p1-blacklevel;
    p2 = (p2 < blacklevel) ? 0 : p2-blacklevel;
    p3 = (p3 < blacklevel) ? 0 : p3-blacklevel;    
#endif

    dstBuffer[yy * width + xx0] = p0;
    dstBuffer[yy * width + xx1] = p1;
    dstBuffer[yy * width + xx2] = p2;
    dstBuffer[yy * width + xx3] = p3;
}

#else

#ifdef RAW_SENSOR
__kernel void PreKernel (     __global ushort *srcBuffer,
#else
__kernel void PreKernel (     __global uchar *srcBuffer,
#endif
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

#endif


////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Post Kernel 
//
////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef RAW_BIT
  #define  RAW_BIT 10
#endif

__kernel void PostKernel (    __global FLOATM *srcBuffer,
                              __global uchar4 *dstBuffer,
                              __constant uchar *gamma  __attribute__( (max_constant_size(1<<RAW_BIT) ) ),
                              const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;

    FLOATM tmp = srcBuffer[centerIndex];


//    int    sat   = ((tmp.s0 > (float)((1<<RAW_BIT)-1))&0x1)+
//    	   	   ((tmp.s1 > (float)((1<<RAW_BIT)-1))&0x1)+
//    	   	   ((tmp.s2 > (float)((1<<RAW_BIT)-1))&0x1);

    int sat = 0;

    ushort r_pix = (ushort)clamp(tmp.s0, 0.0f, (float)((1<<RAW_BIT)-1)); 
    ushort g_pix = (ushort)clamp(tmp.s1, 0.0f, (float)((1<<RAW_BIT)-1)); 
    ushort b_pix = (ushort)clamp(tmp.s2, 0.0f, (float)((1<<RAW_BIT)-1)); 

    dstBuffer[centerIndex].s0 = (sat>0) ? 0xff : gamma[r_pix];
    dstBuffer[centerIndex].s1 = (sat>0) ? 0xff : gamma[g_pix];
    dstBuffer[centerIndex].s2 = (sat>0) ? 0xff : gamma[b_pix];
    dstBuffer[centerIndex].s3 = 0xFF;

}


__kernel void NDVIKernel (    __global FLOATM *srcBuffer,
                              __global uchar4 *dstBuffer,
                              __constant uchar  *lut __attribute__( (max_constant_size(256) ) ),			      
                              const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;
    FLOATM pix = srcBuffer[centerIndex];

    float valR  = max(pix.s0, 0.0);
    float valIR = max(pix.s1, 0.0);    

    float ndvi = (valIR - valR) / max(valIR + valR, 0.0000001);

    uchar idx  = (uchar)(63.0 * ndvi);

    uchar  idx_r = idx;
    uchar  idx_g = idx + 64;
    uchar  idx_b = idx + 64*2;

    dstBuffer[centerIndex].s0 = lut[idx_r];
    dstBuffer[centerIndex].s1 = lut[idx_g];
    dstBuffer[centerIndex].s2 = lut[idx_b];
    dstBuffer[centerIndex].s3 = 0xFF;
}

__kernel void MultKernel (    __global FLOATM *srcBuffer,
                              __global uchar4 *dstBuffer,
                              __constant uchar *gamma  __attribute__( (max_constant_size(1<<RAW_BIT) ) ),
                              const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int centerIndex = y * width + x;

#if (VECM==16)
    int sw = 4;
    int sh = 4;
#else
#if (VECM==8)
    int sw = 3;
    int sh = 3;
#else
    int sw = 2;
    int sh = 2;
#endif
#endif

    int swidth  = width  / sw;
    int sheight = height / sh;

    int sx = (x%swidth ) * sw;
    int sy = (y%sheight) * sh;

    int sourceIndex = sy * width + sx;

    int    ch  = (x/swidth) + (y/sheight)*sw;
    FLOATM tmp =  srcBuffer[sourceIndex];
    float  wl;

    switch(ch){
#if (VECM==16)    
    case 15: wl = tmp.sf; break;    
    case 14: wl = tmp.se; break;
    case 13: wl = tmp.sd; break;
    case 12: wl = tmp.sc; break;
    case 11: wl = tmp.sb; break;
    case 10: wl = tmp.sa; break;
    case 9:  wl = tmp.s9; break;
    case 8:  wl = tmp.s8; break;
    case 7:  wl = tmp.s7; break;
    case 6:  wl = tmp.s6; break;
    case 5:  wl = tmp.s5; break;
    case 4:  wl = tmp.s4; break;
#else
#if (VECM==8)
    case 7:  wl = tmp.s7; break;
    case 6:  wl = tmp.s6; break;
    case 5:  wl = tmp.s5; break;
    case 4:  wl = tmp.s4; break;
#endif
#endif
    case 3:  wl = tmp.s3; break;
    case 2:  wl = tmp.s2; break;
    case 1:  wl = tmp.s1; break;
    case 0:  wl = tmp.s0; break;
    default: wl = 0.0;    break;    
    }

    ushort pix = (ushort)clamp(wl, 0.0f, (float)((1<<RAW_BIT)-1));

    dstBuffer[centerIndex].s0 = gamma[pix];
    dstBuffer[centerIndex].s1 = gamma[pix];
    dstBuffer[centerIndex].s2 = gamma[pix];
    dstBuffer[centerIndex].s3 = 0xFF;
}



////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Wdr Kernel 
//
////////////////////////////////////////////////////////////////////////////////////////////////////

__kernel void WdrKernel(__global ushort  *srcBuffer,
                 	__global ushort  *shortBuffer,
                 	__global ushort  *longBuffer,
                 	__global ushort  *wdrBuffer,
			const ushort      blacklevel,
			const int         soffset,
			const int         loffset,
			const float       gain,
			const int         blst,
			const int         bled,			
			const int         width,
			const int         iwidth,
			__global uint     *aveBuffer,
			const float       average_gain,
			__global int      *mode
)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    
    const int LEFT_OFFSET = 15;

    ushort s = srcBuffer[x+(2*y+soffset)*iwidth + LEFT_OFFSET];
    ushort l = srcBuffer[x+(2*y+loffset)*iwidth + LEFT_OFFSET];

    int shrt_val = (int)s - (int)blacklevel;
    int long_val = (int)l - (int)blacklevel;
    
    int shrt_val_clip = (shrt_val < 0) ? 0 : shrt_val;
    int long_val_clip = (long_val < 0) ? 0 : long_val;

    float wf;
    float sf = (float)shrt_val_clip;    
    float lf = (float)long_val;

    float a = (float) (long_val_clip - blst) / (float) (bled - blst);

    float nm_sf = gain * sf;
    
    ushort tmp_wdr;

	// motion detection
    //float noise_long = (1*1*(100+10*lf));
    //float noise_shrt = (16*16*(100+10*sf));
    //float diff_sl = nm_sf - lf;
    //float alpha_move = (noise_long+noise_shrt) / (noise_long+noise_shrt+diff_sl*diff_sl);
    //alpha_move = 1.0 - alpha_move;

    if( a >= 0 ){
        if(a > 1.0){
            a = 1.0f;
        }
        else {
            ;
            //for improve SN
            //float alpha_min = 1.0f / gain;
            //if(a < alpha_min) a = alpha_min;

            // motion detection
            //if(a > alpha_move) a = alpha_move;
        }
    }
    else{
        a = 0.0f;
    }
    
    //HDR_ON_OFF
    if(0) a = 0;

    wf = nm_sf * a + (1.0f-a)* lf;

    wf = wf + (float)blacklevel;

    if(wf > (float)((1<<16) - 1)) wf = (float)((1<<16) - 1);

    // motion detection
    //if(long_val_clip > (bled-8)) alpha_move = 0;
    //wdrBuffer[x+y*width]   = (ushort)(alpha_move*1023);

    shortBuffer[x+y*width] = (ushort)shrt_val_clip;
    longBuffer[x+y*width]  = (ushort)long_val_clip;

    tmp_wdr = (ushort)(wf + (float)0.5);

    if(mode[0] == 0) {
        wdrBuffer[x+y*width] = (ushort)tmp_wdr;
    }
    else if(mode[0] == 1){
        aveBuffer[x+y*width] = (int)tmp_wdr;
    }
    else {
        aveBuffer[x+y*width] += (int)tmp_wdr;
    }

    if(mode[0] == 3) {
        wdrBuffer[x+y*width] = (ushort)((float)aveBuffer[x+y*width] * average_gain + (float)0.5);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RawEnc Kernel 
//
////////////////////////////////////////////////////////////////////////////////////////////////////

__kernel void RawEncKernel(__global ushort  *srcBuffer,
                           __global ushort  *encTable,
                           const int         width
)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    srcBuffer[x+y*width] = srcBuffer[x+y*width] ^ encTable[x+y*width];
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  RawToArgb Kernel 
//
////////////////////////////////////////////////////////////////////////////////////////////////////

__kernel void RawToArgbKernel(__global ushort  *srcBuffer,
                              __global uint    *dispBuffer,
			const int         width,
			const int         display_lsb
)
{
    int x = get_global_id(0);
    int y = get_global_id(1);

    ushort us_src = srcBuffer[x+width*y];

    if(us_src < 60) us_src = 0;
    else            us_src = us_src - 60;

    us_src = (us_src >> display_lsb);

    if(us_src > 255)    us_src = 255;

    dispBuffer[x+y*width] = ((0xFF<<24)|(us_src<<16)|(us_src<<8)|us_src);
}

__kernel void HistKernel(__global ushort  *srcBuffer,
	      	         __global int     *histBuffer,
			 const int         nbin,
			 const int         bled,			
			 const int         width)

{
    int x = get_global_id(0);
    int y = get_global_id(1);    

    ushort s   = (srcBuffer[x+y*width]>>4);

    atomic_inc(&histBuffer[s]);
}

__kernel void SatMapKernel(__global ushort  *srcBuffer,
	      	           __global uchar   *mapBuffer,
			   const int        sat_th,
                           const int width, const int height)
{
    int x = get_global_id(0);
    int y = get_global_id(1);
    int widthShrink = width/SHRINK_SIZE;
    int heightShrink = height/SHRINK_SIZE;
    int centerIndex = y * widthShrink + x;
    uchar sat_flag = 0;

    {
        for (int yy=-(SHRINK_TAP-1); yy<=(SHRINK_TAP-1); yy++)
        {
                int yOrg = y * SHRINK_SIZE + yy;
                if(yOrg < 0 || yOrg >= height ) continue;

                for (int xx=-(SHRINK_TAP-1); xx<=(SHRINK_TAP-1); xx++)
                {
                        int xOrg = x * SHRINK_SIZE + xx;
                        if(xOrg < 0 || xOrg >= width ) continue;

                        int thisIndex = yOrg * width + xOrg;

                        ushort thisPixel = srcBuffer[thisIndex];

			if(thisPixel >= sat_th)
			{
			     sat_flag = 1;
			     break;
			}
                }
        }
     }

     mapBuffer[centerIndex] = sat_flag;
}

