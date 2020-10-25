#ifndef HASSPROC_H
#define HASSPROC_H

#include "kernel.h"

//typedef signed char jbyte;

#define HASS_KERNEL_NUM 19
#define COEFF_FOR_DEMO_LENS

typedef enum {
    HASS_PRE_PROCESSING=0,
    HASS_GUIDE,
    HASS_GUIDE2,
    HASS_SHRINK,
    HASS_SHRINK2,
    HASS_NOISE_REDUCTION,
    HASS_RECONSTRUCTION,
    HASS_APP,
    HASS_COMBINE,
    HASS_COMBINE2,
    HASS_ZOOM,
    HASS_POST_PROCESSING,
    HASS_POST_NDVI,
    HASS_POST_MULT,
    HASS_READ_INPUT,
    HASS_WRITE_OUTPUT,
    HASS_WDR,
    HASS_RAWTOARGB,
    HASS_HIST,
    HASS_SATMAP,
    HASS_RAWENC,
} HassKernelType;

typedef enum {
    HASS_MODE_RGB=67,
    HASS_MODE_NDVI,
    HASS_MODE_MULT,
    HASS_MODE_RAW,
    HASS_MODE_NONE,
} HassModeType;

class HassProc
{
public:
    // Kernel names
    kernel_control _kernel_controls[HASS_KERNEL_NUM];

    // Kernel names
    char _kname_HassPre[256];
    char _kname_HassGuide[256];
    char _kname_HassGuide2[256];
    char _kname_HassShrink[256];
    char _kname_HassShrink2[256];
    char _kname_HassNr[256];
    char _kname_HassRecon[256];
    char _kname_HassApp[256];
    char _kname_HassCombine[256];
    char _kname_HassCombine2[256];
    char _kname_HassZoom[256];
    char _kname_HassPost[256];
    char _kname_HassPostNdvi[256];
    char _kname_HassPostMult[256];
    char _kname_HassWdr[256];
    char _kname_HassRawToArgb[256];
    char _kname_HassRawEnc[256];
    char _kname_HassHist[256];
    char _kname_HassSatMap[256];

    // Kernel filenames
    char _kfname_HassPre[256];
    char _kfname_HassGuide[256];
    char _kfname_HassShrink[256];
    char _kfname_HassNr[256];
    char _kfname_HassRecon[256];
    char _kfname_HassApp[256];
    char _kfname_HassCombine[256];
    char _kfname_HassZoom[256];
    char _kfname_HassPost[256];
    char _kfname_HassPostNdvi[256];
    char _kfname_HassPostMult[256];
    char _kfname_HassWdr[256];
    char _kfname_HassRawToArgb[256];
    char _kfname_HassRawEnc[256];
    char _kfname_HassHist[256];
    char _kfname_HassSatMap[256];

    // Compiler Options
    char _kcoption_Hass[1024];

    // filter
    /*
      D,O,C,C,B,M,A,A,
      D,K,C,I,B,G,A,E,
      M,G,G,K,F,E,E,I,
      M,O,G,N,F,M,E,L,
      B,N,A,B,D,P,C,D,
      B,G,A,E,D,K,C,I,
      F,F,E,J,M,H,G,L,
      F,M,E,L,M,O,G,N,
    */

    /*
       A,B,C,D,E,F,G,H,I,J, K, L, M, N, O, P
       0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15
    */

    // 8x8 16colors type

    int hass_filter_no[8][8] =
            {
                    { 3,14, 2, 2, 1,12, 0, 0},
                    { 3,10, 2, 8, 1, 6, 0, 4},
                    {12, 6, 6,10, 5, 4, 4, 8},
                    {12,14, 6,13, 5,12, 4,11},
                    { 1,13, 0, 1, 3,15, 2, 3},
                    { 1, 6, 0, 4, 3,10, 2, 8},
                    { 5, 5, 4, 9,12, 7, 6,11},
                    { 5,12, 4,11,12,14, 6,13}
            };

    // Hass Parameters
    int hass_raw_input_bit;
    int hass_raw_bit;
    int hass_raw_input_mode;
    int hass_channel;
    int hass_spectrum;
    int hass_rep_unit;
    int hass_shrink_size;
    int hass_shrink_tap;
    int hass_nr_tap;
    int hass_combine_tap;
    float hass_gamma;
    int hass_sat_th;
    float hass_sat_val;
    float hass_auto_exposure_speed_long;
    float hass_auto_exposure_speed_short;
    float hass_auto_exposure_target;
    float hass_auto_exposure_range;

    int hass_mode; /* RGB : 0, NDVI : 1 */

    // Buffers

    cl_mem d_input;
    cl_mem d_input1;
    cl_mem d_input2;
    cl_mem d_pre;
    cl_mem d_shrink;
    cl_mem d_shrinkint;
    cl_mem d_nr;
    cl_mem d_nri;
    cl_mem d_recon;
    cl_mem d_app;
    cl_mem d_zoom;
    cl_mem d_post;
    cl_mem d_post1;
    cl_mem d_post2;
    cl_mem d_guide;
    cl_mem d_guideint;
    cl_mem d_combine;
    cl_mem d_combint;
    cl_mem d_long;
    cl_mem d_short;
    cl_mem d_wdr;
    cl_mem d_ave;
    cl_mem d_hist;
    cl_mem d_disp;
    cl_mem d_mode;

    cl_mem d_shrink_coef;
    cl_mem d_shrink_coefsum;
    cl_mem d_color_ch;
    cl_mem d_filter_ch;
    cl_mem d_recon_coef;
    cl_mem d_guide_coef;
    cl_mem d_combine_coef;
    cl_mem d_filter_config;
    cl_mem d_gamma_tbl;
    cl_mem d_color_tbl;

    cl_mem d_enc_tbl;
    
    cl_mem d_sat_map;

    size_t d_input_sz;
    size_t d_pre_sz;
    size_t d_shrink_sz;
    size_t d_shrinkint_sz;
    size_t d_nr_sz;
    size_t d_recon_sz;
    size_t d_app_sz;
    size_t d_zoom_sz;
    size_t d_post_sz;
    size_t d_guide_sz;
    size_t d_combine_sz;
    size_t d_long_sz;
    size_t d_short_sz;
    size_t d_wdr_sz;
    size_t d_ave_sz;
    size_t d_hist_sz;
    size_t d_disp_sz;
    size_t d_mode_sz;

    size_t d_shrink_coef_sz;
    size_t d_shrink_coefsum_sz;
    size_t d_color_ch_sz;
    size_t d_filter_ch_sz;
    size_t d_recon_coef_sz;
    size_t d_guide_coef_sz;
    size_t d_combine_coef_sz;
    size_t d_enc_tbl_sz;

    size_t d_sat_map_sz;

    float *hass_shrink_coef;
    float *hass_shrink_coefsum;
    unsigned char *hass_color_ch;
    unsigned char *hass_filter_ch;
    float *hass_recon_coef;
    float *hass_guide_coef;
    float *hass_combine_coef;
    float hass_combine_noise;
    unsigned char *hass_gamma_tbl;
    unsigned short *hass_enc_tbl;

    int hass_iwidth;
    int hass_iheight;
    int hass_width;
    int hass_height;
    int hass_widthShrink;
    int hass_heightShrink;

    kernel_control* ctrl_pre;
    kernel_control* ctrl_post;

    jbyte *inp1;
    jbyte *outp1;
    jbyte *inp2;
    jbyte *outp2;

    cl_int hass_err;

    /* WDR & AE parameters */

    int SHS1;
    int SHS2;

    int RHS1;
    int VMAX;

    int shortExp;
    int longExp;
    int shs1Offset;
    int shs2Offset;
    int xOffset;
    int yOffset;
    unsigned short blackLevel;
    int display_lsb;
    float average_gain;

    float AEGain;

    int BLST;
    int BLED;
    float Gain;
    int nbin;
    int *hist;
    float *hist_f;

    int AEreq;

    HassProc(void);
    ~HassProc(void);

    void SetHassModeSPC(int s);
    void SetHassModeRGB(void);
    void SetHassModeNDVI(void);
    void SetHassModeMULT(void);    
    void SetHassNRTap(int);
    void SetHassGamma(float);
    void SetHassGammaTable(jbyte *);
    void SetHassAutoExposureSpeedShort(int);
    void SetHassAutoExposureSpeedLong(int);
    void SetHassAutoExposureTarget(int);
    void SetHassAutoExposureRange(int);
    void SetHassDisplayLsb(int);
    void SetHassAverageGain(float);
    int  GetHassNRTap(void);
    float GetHassGamma(void);
    float GetHassAutoExposureSpeedLong(void);
    float GetHassAutoExposureSpeedShort(void);
    float GetHassAutoExposureTarget(void);
    float GetHassAutoExposureRange(void);
    void SetHassImageOffset(int,int);
    void GetHassImageConfiguration(int*,int*,int*,int*,int*);
    void SetHassImageConfiguration(int,int,int,int,int);

    void GetHassFilterStructure(int **);
    void SetHassFilterStructure(int **);
    void GetHassFilterMatrix(float **);
    void SetHassFilterMatrix(float **);

    int GetWidth(void);
    int GetHeight(void);
    int GetHistNum(void);
    int GetSHS1(void);
    int GetSHS2(void);
    void CalcAE(void);
    bool DoneAE(void);
    void SetAEreq(int);

    void CalcHassGamma(unsigned char *);

    void SetImageSize(void);
    void SetKernelInformation(void);

    void CreateCoefficients(void);
    void SetCoefficients(void);
    void UpdateCoefficients(void);
    void ReleaseCoefficients(void);

    bool CreateHassBuffers(void);
    void ReleaseHassBuffers(void);

    bool CreateHassPreKernel(void);
    bool CreateHassShrinkKernel(void);
    bool CreateHassNoiseReductionKernel(void);
    bool CreateHassReconstructionKernel(void);
    bool CreateHassAppKernel(void);
    bool CreateHassZoomKernel(void);
    bool CreateHassPostProcessingKernel(void);
    bool CreateHassPostNdviKernel(void);
    bool CreateHassPostMultKernel(void);
    bool CreateHassGuideKernel(void);
    bool CreateHassCombineKernel(void);
    bool CreateHassWdrKernel(void);
    bool CreateHassRawToArgbKernel(void);
    bool CreateHassRawEncKernel(void);
    bool CreateHassHistKernel(void);
    bool CreateHassSatMapKernel(void);
    void CreateHassKernels(void);
    void RunHassPreKernel(void);
    void RunHassShrinkKernel(void);
    void RunHassNoiseReductionKernel(void);
    void RunHassReconstructionKernel(void);
    void RunHassAppKernel(void);
    void RunHassZoomKernel(void);
    void RunHassGuideKernel(void);
    void RunHassCombineKernel(void);
    void RunHassWdrKernel(void);
    void RunHassRawToArgbKernel(void);
    void RunHassRawEncKernel(void);
    void RunHassHistKernel(void);
    void RunHassSatMapKernel(void);
    void RunHassPostProcessingKernel(void);
    void RunHassKernelsDB(void* rawImage1, void* rawImage2, void* rgbOut1, void* rgbOut2);
    void RunHassKernelsSB(void* rawImage, void* rgbOut);
    void RunHassWdrRawKernelsSB(void *rawImage, void *rgbOut1, void *rgbOut2, int fnum, int displayoff);
    void RunHassKernelsTest(unsigned short* rawImage, float* rgbOut, int loop);
    void RunHassKernelsGoThrough(unsigned short* rawImage, float* rgbOut, unsigned char *out, int loop);
    void RunHassKernels(void);
    void ReleaseHassPreKernel(void) ;
    void ReleaseHassShrinkKernel(void) ;
    void ReleaseHassNoiseReductionKernel(void);
    void ReleaseHassReconstructionKernel(void);
    void ReleaseHassAppKernel(void);
    void ReleaseHassZoomKernel(void);
    void ReleaseHassGuideKernel(void);
    void ReleaseHassCombineKernel(void);
    void ReleaseHassWdrKernel(void);
    void ReleaseHassRawToArgbKernel(void);
    void ReleaseHassRawEncKernel(void);
    void ReleaseHassHistKernel(void);
    void ReleaseHassSatMapKernel(void);
    void ReleaseHassPostProcessingKernel(void);
    void ReleaseHassPostNdviKernel(void);
    void ReleaseHassPostMultKernel(void);
    void ReleaseHassKernels(void);

    bool UpdateHassPostProcessingKernel(void);
    bool UpdateHassPostNdviKernel(void);
    bool UpdateHassPostMultKernel(void);
    bool UpdateHassRawToArgbKernel(void);
    bool UpdateHassWdrKernel(void);

    bool compResult(float* ref, float* out, int width, int height, int ch, int edge, float aer);
    void TestHassGuideKernel(void);
    void TestHassCombineKernel(void);
    void TestHassShrinkKernel(void);
    void TestHassNoiseReductionKernel(void);
    void TestHassReconstructionKernel(void);
    void TestHassAppKernel(void);
    void TestHassZoomKernel(void);

    size_t GetHassIntermediateData(int sel, float *data);
    size_t GetHassRawData(void *data);    

    float LutHassLambda[67][16] = {
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

    // NDVI -> RGB Table

    unsigned char hass_color_tbl[64*3]
            = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 15, 31, 47, 63, 79, 95, 111, 127, 143, 159, 175, 191, 207, 223, 239, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
               0, 0, 0, 0, 0, 0, 0, 0, 15, 31, 47, 63, 79, 95, 111, 127, 143, 159, 175, 191, 207, 223, 239, 255, 255, 255, 255, 255, 255, 255, 255,255, 255, 255, 255, 255, 255, 255, 255, 255, 239, 223, 207, 191, 175, 159, 143, 127, 111, 95, 79, 63, 47, 31, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0,
               143, 159, 175, 191, 207, 223, 239, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 239, 223, 207, 191, 175, 159, 143, 127, 111, 95, 79, 63, 47, 31, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

};

typedef enum {
    WAVELENGTH_350,
    WAVELENGTH_360,
    WAVELENGTH_370,
    WAVELENGTH_380,
    WAVELENGTH_390,
    WAVELENGTH_400,
    WAVELENGTH_410,
    WAVELENGTH_420,
    WAVELENGTH_430,
    WAVELENGTH_440,
    WAVELENGTH_450,
    WAVELENGTH_460,
    WAVELENGTH_470,
    WAVELENGTH_480,
    WAVELENGTH_490,
    WAVELENGTH_500,
    WAVELENGTH_510,
    WAVELENGTH_520,
    WAVELENGTH_530,
    WAVELENGTH_540,
    WAVELENGTH_550,
    WAVELENGTH_560,
    WAVELENGTH_570,
    WAVELENGTH_580,
    WAVELENGTH_590,
    WAVELENGTH_600,
    WAVELENGTH_610,
    WAVELENGTH_620,
    WAVELENGTH_630,
    WAVELENGTH_640,
    WAVELENGTH_650,
    WAVELENGTH_660,
    WAVELENGTH_670,
    WAVELENGTH_680,
    WAVELENGTH_690,
    WAVELENGTH_700,
    WAVELENGTH_710,
    WAVELENGTH_720,
    WAVELENGTH_730,
    WAVELENGTH_740,
    WAVELENGTH_750,
    WAVELENGTH_760,
    WAVELENGTH_770,
    WAVELENGTH_780,
    WAVELENGTH_790,
    WAVELENGTH_800,
    WAVELENGTH_810,
    WAVELENGTH_820,
    WAVELENGTH_830,
    WAVELENGTH_840,
    WAVELENGTH_850,
    WAVELENGTH_860,
    WAVELENGTH_870,
    WAVELENGTH_880,
    WAVELENGTH_890,
    WAVELENGTH_900,
    WAVELENGTH_910,
    WAVELENGTH_920,
    WAVELENGTH_930,
    WAVELENGTH_940,
    WAVELENGTH_950,
    WAVELENGTH_960,
    WAVELENGTH_970,
    WAVELENGTH_980,
    WAVELENGTH_990,
    WAVELENGTH_1000,
    WAVELENGTH_NONE
} WAVELENGTH_index;


#endif

