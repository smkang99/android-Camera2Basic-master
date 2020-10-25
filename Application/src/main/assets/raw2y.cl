/*
__kernel void RAW2Y(
                     __global ushort const* const raw,
                     __global uchar4* yout
                    )
{
     int i = get_global_id(0);

     int i0 = 4*i;
     int i1 = i0+1;
     int i2 = i1+1;
     int i3 = i2+1;
     yout[i0] = raw[i];
     yout[i1] = raw[i];
     yout[i2] = raw[i];
     yout[i3] = 0xFF;
}

*/

__kernel void RAW2Y(
                     __global uchar const* const raw,
                     __global uchar* yout
                    )
 {
    int i = get_global_id(0);
    int i0  = i*5;
    int i1  = i*5+1;
    int i2  = i*5+2;
    int i3  = i*5+3;
    int i4  = i*5+4;

    int o0  = i*4;
    int o1  = i*4+1;
    int o2  = i*4+2;
    int o3  = i*4+3;

    uchar  lb = raw[i4];

    ushort p0 = raw[i0];
    ushort p1 = raw[i1];
    ushort p2 = raw[i2];
    ushort p3 = raw[i3];
/*
    char po0  = (p0<<2)|((lb   )&0x3);
    char po1  = (p1<<2)|((lb>>2)&0x3);
    char po2  = (p2<<2)|((lb>>4)&0x3);
    char po3  = (p3<<2)|((lb>>6)&0x3);
*/
    char po0  = p0;
    char po1  = p1;
    char po2  = p2;
    char po3  = p3;

    yout[4*o0]   = po0;
    yout[4*o0+1] = po0;
    yout[4*o0+2] = po0;
    yout[4*o0+3] = 0xFF;

    yout[4*o1]   = po1;
    yout[4*o1+1] = po1;
    yout[4*o1+2] = po1;
    yout[4*o1+3] = 0xFF;

    yout[4*o2]   = po2;
    yout[4*o2+1] = po2;
    yout[4*o2+2] = po2;
    yout[4*o2+3] = 0xFF;

    yout[4*o3]   = po3;
    yout[4*o3+1] = po3;
    yout[4*o3+2] = po3;
    yout[4*o3+3] = 0xFF;
 }