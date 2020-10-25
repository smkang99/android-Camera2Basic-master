#include <android/log.h>
#include <jni.h>
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <math.h>
#include <time.h>

#include "hassproc.h"

#include <CL/cl_ext.h>
#include <string.h>

#define ACCELERATE_KERNEL_SHRINK
#define ACCELERATE_KERNEL_GUIDE
#define ACCELERATE_KERNEL_COMBINE
#define ACCELERATE_ZERO_COPY
//#define ACCELERATE_ZERO_COPY_NONBLOCKING          

#define SET_HASS_SAT_MAP
#define SET_HASS_WDR

#ifdef LOG_PRINTF
#define LOGI printf
  #define LOGE printf
#else
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "HassProc", __VA_ARGS__))
#define LOGE(...) ((void)__android_log_print(ANDROID_LOG_ERROR, "HassProc", __VA_ARGS__))
#endif

HassProc::HassProc(void){
     // Hass Parameters
     hass_raw_input_bit = 10;
#ifdef SET_HASS_WDR
     hass_raw_bit       = 12;
#else
     hass_raw_bit       = hass_raw_input_bit;
#endif
     hass_raw_input_mode= 0; // RAW10
     //hass_raw_input_mode= 1; // RAW_SENSOR
     hass_channel       = 16;

     hass_spectrum      = 4;
     //hass_spectrum      = 8;     
     //hass_spectrum      = 16;

     hass_rep_unit      = 8;
     hass_shrink_size   = 8;
     hass_shrink_tap    = 8;

     hass_nr_tap        = 1;
     hass_gamma         = 2.1f;
     hass_auto_exposure_target = 0.0f;
     hass_auto_exposure_range = 0.0f;
     hass_auto_exposure_speed_short = 1.0f;
     hass_auto_exposure_speed_long  = 1.0f;


     //hass_mode          = 0; // RGB
     hass_mode          = HASS_MODE_RGB; // RGB
     // hass_mode          = HASS_MODE_NDVI; // NDVI
     // hass_mode          = HASS_MODE_NONE;

     // WDR Parameters

     SHS1               = 2;
     // 16times
     SHS2               = 159;

#ifdef SET_HASS_WDR          
     // 16times
     RHS1               = 143;
     VMAX               = 1200;
#else
     RHS1               = 1125;
     VMAX               = 1125;
#endif     

     AEGain             = 1.0;
     shortExp           = RHS1 - SHS1 - 1;
     longExp            = 2*VMAX - SHS2 - 1;

     Gain               = (float) longExp / (float) shortExp;

     // otsuki j.f.
     shs1Offset         = RHS1+20;
     shs2Offset         = 20;
     //shs1Offset         = RHS1;
     //shs2Offset         = 0;

     blackLevel         = 60;
     display_lsb        = 2;
     average_gain       = 1;

     BLED               = ((1<<hass_raw_input_bit)-1)-blackLevel-1;
     BLST               = (BLED * 3) / 4;

     nbin               = 64;

     AEreq              = 1;

#ifdef SET_HASS_WDR
     xOffset            = 7;
     yOffset            = 1;
#else
     xOffset            = 3;
     yOffset            = 7;
#endif

     // Saturation Map Parameters

     hass_sat_th        = (1<<hass_raw_input_bit)-1-blackLevel-1;
     hass_sat_val       = (float) hass_sat_th;
}

HassProc::~HassProc(void){
}

void HassProc::SetImageSize(void) {
#ifdef SET_HASS_WDR
     hass_iwidth       = 1952;
     hass_iheight      = 2346;
#else
     hass_iwidth       = 1920;
     hass_iheight      = 1080;
#endif
     hass_width        = 1920;
     hass_height       = 1080;
     hass_widthShrink  = (int)(hass_width  / hass_shrink_size);
     hass_heightShrink = (int)(hass_height / hass_shrink_size);

//     LOGI("Set %d x %d \n", hass_width, hass_height);
}

void HassProc::SetHassModeSPC(int s)
{
     bool change_mode = (hass_mode >= HASS_MODE_RGB);

     //if(change_mode)
	 // ReleaseHassPostProcessingKernel();

     if((s >= 0) && (s < HASS_MODE_RGB))
          hass_mode = s;
     else
          hass_mode = HASS_MODE_RGB;

     //if(change_mode)
	 // sprintf(_kname_HassPost,"PostKernel");
     
     SetCoefficients();
     UpdateCoefficients();

     if(change_mode)
	  //CreateHassPostProcessingKernel();
     UpdateHassPostProcessingKernel();
}

void HassProc::SetHassModeRGB(void)
{
     bool change_mode = (hass_mode != HASS_MODE_RGB);

     if(change_mode){
	  //ReleaseHassPostProcessingKernel();

	  hass_mode = HASS_MODE_RGB;

	  //sprintf(_kname_HassPost,"PostKernel");
	  SetCoefficients();
	  UpdateCoefficients();
	  //CreateHassPostProcessingKernel();
          UpdateHassPostProcessingKernel();
     }
}

void HassProc::SetHassModeNDVI(void)
{
     bool change_mode = (hass_mode != HASS_MODE_NDVI);

     if(change_mode){
	  //ReleaseHassPostProcessingKernel();

	  hass_mode = HASS_MODE_NDVI;
	  
	  //sprintf(_kname_HassPost,"NDVIKernel");
	  SetCoefficients();
	  UpdateCoefficients();
	  //CreateHassPostProcessingKernel();
          UpdateHassPostNdviKernel();
     }
}

void HassProc::SetHassModeMULT(void)
{
     bool change_mode = (hass_mode != HASS_MODE_MULT);

     if(change_mode){          
	  //ReleaseHassPostProcessingKernel();

	  hass_mode = HASS_MODE_MULT;
	  
	  //sprintf(_kname_HassPost,"MultKernel");
	  SetCoefficients();
	  UpdateCoefficients();
	  //CreateHassPostProcessingKernel();
          UpdateHassPostMultKernel();
     }
}

void HassProc::SetHassDisplayLsb(int HassDisplayLsb) {
    display_lsb = HassDisplayLsb;
    UpdateHassRawToArgbKernel();
}

void HassProc::SetHassAverageGain(float tmp_ave_gain) {

    average_gain = tmp_ave_gain;
    UpdateHassWdrKernel();
}

void HassProc::SetHassNRTap(int tap)
{
     if(tap < 1)
          tap = 1;

     hass_nr_tap = tap;

     kernel_control* ctrl = &_kernel_controls[HASS_NOISE_REDUCTION];
     clSetKernelArg(ctrl->kernel,2,sizeof(hass_nr_tap),((void *)(&hass_nr_tap)));
}

void HassProc::CalcHassGamma(unsigned char *tbl) {
     int d_gamma_tbl_sz = (1<<hass_raw_bit);

     for(int i=0;i<d_gamma_tbl_sz;i++)
          tbl[i] = (unsigned char) (255 * powf ((float)i / (float)(d_gamma_tbl_sz-1), (1.0 / hass_gamma)));
}

void HassProc::SetHassAutoExposureSpeedShort(int HassAutoExposureSpeedShort) {
     // apply to AE;
    if(HassAutoExposureSpeedShort < 0)  hass_auto_exposure_speed_short = 0;
    else                                hass_auto_exposure_speed_short = (float)HassAutoExposureSpeedShort;
}

void HassProc::SetHassAutoExposureSpeedLong(int HassAutoExposureSpeedLong) {
     // apply to AE;
    if(HassAutoExposureSpeedLong < 0)  hass_auto_exposure_speed_long = 0;
    else                               hass_auto_exposure_speed_long = (float)HassAutoExposureSpeedLong;
}

void HassProc::SetHassAutoExposureTarget(int HassAutoExposureTarget) {
    hass_auto_exposure_target = (float)HassAutoExposureTarget;
}

void HassProc::SetHassAutoExposureRange(int HassAutoExposureRange) {
    hass_auto_exposure_range = (float)HassAutoExposureRange;
}

void HassProc::SetHassGamma(float gamma) {
     int d_gamma_tbl_sz = (1<<hass_raw_bit);
     hass_gamma = gamma;

     CalcHassGamma(hass_gamma_tbl);
     clEnqueueWriteBuffer(GetCommandQueue(), d_gamma_tbl, CL_TRUE,  0, d_gamma_tbl_sz, hass_gamma_tbl,    0, NULL, NULL);
}

void HassProc::SetHassGammaTable(jbyte *tbl) {
     int d_gamma_tbl_sz = (1<<hass_raw_bit);

     clEnqueueWriteBuffer(GetCommandQueue(), d_gamma_tbl, CL_TRUE,  0, d_gamma_tbl_sz,            tbl,    0, NULL, NULL);
}


float HassProc::GetHassGamma(void) {
     return hass_gamma;
}

float HassProc::GetHassAutoExposureSpeedLong(void) {
     return hass_auto_exposure_speed_long;
}

float HassProc::GetHassAutoExposureSpeedShort(void) {
     return hass_auto_exposure_speed_short;
}

float HassProc::GetHassAutoExposureRange(void) {
     return hass_auto_exposure_range;
}

float HassProc::GetHassAutoExposureTarget(void) {
     return hass_auto_exposure_target;
}

int HassProc::GetHassNRTap(void) {
     return hass_nr_tap;
}

void HassProc::SetHassImageOffset(int dx, int dy){
     xOffset = dx;
     yOffset = dy;

     kernel_control* ctrl = &_kernel_controls[HASS_PRE_PROCESSING];

     clSetKernelArg(ctrl->kernel,2,sizeof(xOffset),((void *)(&xOffset)));
     clSetKernelArg(ctrl->kernel,3,sizeof(yOffset),((void *)(&yOffset)));
}

void HassProc::GetHassImageConfiguration(int *rep, int *tap, int *size, int *ch, int *sp){

     *rep  = hass_rep_unit;
     *tap  = hass_shrink_tap;
     *size = hass_shrink_size;
     *ch   = hass_channel;
     *sp   = hass_spectrum;
}

void HassProc::SetHassImageConfiguration(int rep, int tap, int size, int ch, int sp){

     ReleaseHassKernels();
  
     hass_rep_unit    = rep;
     hass_shrink_tap  = tap;
     hass_shrink_size = size;
     hass_channel     = ch;
     hass_spectrum    = sp;

     CreateHassKernels();

     SetCoefficients();
     UpdateCoefficients();
}

void HassProc::GetHassFilterStructure(int **filter){
     for(int j=0;j<hass_rep_unit;j++) {
          for(int i=0;i<hass_rep_unit;i++) {
               filter[j][i] = hass_filter_no[j][i];
          }
     }
}

void HassProc::SetHassFilterStructure(int **filter) {

     for(int j=0;j<hass_rep_unit;j++) {
          for(int i=0;i<hass_rep_unit;i++) {
               hass_filter_no[j][i] = filter[j][i];
          }
     }

     SetCoefficients();
     UpdateCoefficients();
}

void HassProc::GetHassFilterMatrix(float **matrix) {
     for(int j=0;j<67;j++) {
          for(int i=0;i<hass_channel;i++) {
               matrix[j][i] = LutHassLambda[j][i];
          }
     }
}

void HassProc::SetHassFilterMatrix(float **matrix) {
     for(int j=0;j<67;j++) {
          for(int i=0;i<hass_channel;i++) {
               LutHassLambda[j][i] = matrix[j][i];
          }
     }

     SetCoefficients();
     UpdateCoefficients();
}

bool HassProc::DoneAE(void){
     return (AEreq == 0);
}

void HassProc::SetAEreq(int req){
     AEreq = req;
}


void HassProc::SetKernelInformation(void) {

     // Kernel Compile Option
     char use_lm_for_zoom_option[]="-DUSE_LM_FOR_ZOOM";

     if((hass_shrink_size < 4)||(hass_spectrum > 4))
          sprintf(use_lm_for_zoom_option, " "); // Over Local Memory Limits

     switch(hass_raw_input_mode){
          case 1:  // RAW_SENSOR
               sprintf(_kcoption_Hass, "-w -cl-std=CL2.0 -cl-fast-relaxed-math -DVECN=%d -DVECM=%d %s -DRAW_SENSOR -DRAW_BIT=%d -DREP_UNIT=%d -DSHRINK_SIZE=%d -DSHRINK_TAP=%d",
                       hass_channel, hass_spectrum, use_lm_for_zoom_option, hass_raw_bit, hass_rep_unit, hass_shrink_size, hass_shrink_tap);
             break;
          case 2:  // YUV
               sprintf(_kcoption_Hass, "-w -cl-std=CL2.0 -cl-fast-relaxed-math -DVECN=%d -DVECM=%d %s -DRAW_BIT=%d -DREP_UNIT=%d -DSHRINK_SIZE=%d -DSHRINK_TAP=%d",
                       hass_channel, hass_spectrum, use_lm_for_zoom_option, hass_raw_bit, hass_rep_unit, hass_shrink_size, hass_shrink_tap);
             break;
          default: // RAW10
               sprintf(_kcoption_Hass, "-w -cl-std=CL2.0 -cl-fast-relaxed-math -DVECN=%d -DVECM=%d %s -DRAW10 -DRAW_BIT=%d -DREP_UNIT=%d -DSHRINK_SIZE=%d -DSHRINK_TAP=%d",
                       hass_channel, hass_spectrum, use_lm_for_zoom_option, hass_raw_bit, hass_rep_unit, hass_shrink_size, hass_shrink_tap);
             break;
     }


     // Kernel's Name Setup

     sprintf(_kname_HassPre,     "PreKernel");
     sprintf(_kfname_HassPre,    "hass_kernels.cl");

#ifdef ACCELERATE_KERNEL_GUIDE
     sprintf(_kname_HassGuide,   "GuideColKernel");
     sprintf(_kname_HassGuide2,  "GuideRowKernel");
     sprintf(_kfname_HassGuide,  "hass_kernels.cl");
#else
     sprintf(_kname_HassGuide,   "GuideKernel");
     sprintf(_kfname_HassGuide,  "hass_kernels.cl");
#endif

#ifdef ACCELERATE_KERNEL_SHRINK
     sprintf(_kname_HassShrink,  "ShrinkColKernel");
     sprintf(_kname_HassShrink2, "ShrinkRowKernel");
     sprintf(_kfname_HassShrink, "hass_kernels.cl");
#else
     sprintf(_kname_HassShrink,  "ShrinkKernel");
     sprintf(_kfname_HassShrink, "hass_kernels.cl");
#endif

     sprintf(_kname_HassNr,      "NRKernel");
     sprintf(_kfname_HassNr,     "hass_kernels.cl");

     sprintf(_kname_HassRecon,   "ReconKernel");
     sprintf(_kfname_HassRecon,  "hass_kernels.cl");

     sprintf(_kname_HassApp,     "AppKernel");
     sprintf(_kfname_HassApp,    "hass_kernels.cl");

#ifdef ACCELERATE_KERNEL_COMBINE
     sprintf(_kname_HassCombine, "CombineColKernel");
     sprintf(_kname_HassCombine2,"CombineRowKernel");
#else
     sprintf(_kname_HassCombine, "CombineKernel");
#endif
     sprintf(_kfname_HassCombine,"hass_kernels.cl");


     sprintf(_kname_HassZoom,    "ZoomKernel");
     sprintf(_kfname_HassZoom,   "hass_kernels.cl");

//     switch(hass_mode){
//          case HASS_MODE_NDVI : sprintf(_kname_HassPost,    "NDVIKernel"); break; // NDVI
//          case HASS_MODE_MULT : sprintf(_kname_HassPost,    "MultKernel"); break; // Mult
//          default             : sprintf(_kname_HassPost,    "PostKernel"); break; // RGB
//     }
//     sprintf(_kfname_HassPost,   "hass_kernels.cl");

     sprintf(_kname_HassPost,    "PostKernel");
     sprintf(_kfname_HassPost,   "hass_kernels.cl");

     sprintf(_kname_HassPostNdvi,    "NDVIKernel");
     sprintf(_kfname_HassPostNdvi,   "hass_kernels.cl");

     sprintf(_kname_HassPostMult,    "MultKernel");
     sprintf(_kfname_HassPostMult,   "hass_kernels.cl");

     sprintf(_kname_HassWdr,     "WdrKernel");
     sprintf(_kfname_HassWdr,    "hass_kernels.cl");

     sprintf(_kname_HassRawToArgb,     "RawToArgbKernel");
     sprintf(_kfname_HassRawToArgb,    "hass_kernels.cl");

     sprintf(_kname_HassHist,    "HistKernel");
     sprintf(_kfname_HassHist,   "hass_kernels.cl");

     sprintf(_kname_HassSatMap,  "SatMapKernel");
     sprintf(_kfname_HassSatMap, "hass_kernels.cl");

     sprintf(_kname_HassRawEnc,     "RawEncKernel");
     sprintf(_kfname_HassRawEnc,    "hass_kernels.cl");

}

void HassProc::CreateCoefficients(void) {
     hass_shrink_coef = new float[hass_shrink_tap];
     hass_shrink_coefsum = new float[2 * 2 * hass_channel];
     hass_filter_ch = new unsigned char[hass_rep_unit*hass_rep_unit];
     hass_color_ch = new unsigned char[hass_rep_unit*hass_rep_unit*hass_channel];
     hass_recon_coef = new float[hass_channel*hass_spectrum];
     hass_guide_coef= new float[hass_rep_unit*hass_rep_unit*hass_channel];

     int combine_tap_center = hass_rep_unit/2;
     int combine_tap_side   = hass_rep_unit/4;
     hass_combine_tap = combine_tap_center + combine_tap_side;

     hass_combine_coef = new float[hass_combine_tap+1];
     hass_gamma_tbl = new unsigned char[(1<<hass_raw_bit)];
     // Histogram
     hist   = new int [nbin];
     hist_f = new float [nbin];
    
     //Encryption
     hass_enc_tbl = new unsigned short[hass_width*hass_height];
}

void HassProc::SetCoefficients(void) {

     // Coeff for Shrink/Zoom

     for(int i = 0; i < hass_shrink_tap; i++){
          float ds = (float)i/(float)hass_shrink_tap;
          hass_shrink_coef[i] = 1 - ds;                          //bi-liner
          //hass_shrink_coef[i] = 2.0*ds*ds*ds - 3.0*ds*ds + 1.0;    //simple bi-cubic
     }

     // Coeff Sum

     for(int y = 0;y < 2; y++)
          for(int x = 0;x < 2; x++)
               for(int c = 0;c < hass_channel; c++)
                    hass_shrink_coefsum[c+(x+2*y)*hass_channel] = 0.0f;

     for(int y = 0;y < 2; y++)
          for(int x = 0;x < 2; x++)
               for(int dyOrg = -(hass_shrink_tap-1); dyOrg <= (hass_shrink_tap-1); ++dyOrg){
                    int yOrg = y*hass_shrink_size+dyOrg;
                    if(yOrg < 0 || yOrg >= hass_height ) continue;
                    for(int dxOrg = -(hass_shrink_tap-1); dxOrg <= (hass_shrink_tap-1); ++dxOrg){
                         int xOrg = x*hass_shrink_size+dxOrg;
                         if(xOrg < 0 || xOrg >= hass_width ) continue;
                         int xPhaseOrg = xOrg % hass_rep_unit;
                         int yPhaseOrg = yOrg % hass_rep_unit;
                         int c = hass_filter_no[yPhaseOrg][xPhaseOrg];
                         float w = hass_shrink_coef[abs(dxOrg)]*hass_shrink_coef[abs(dyOrg)];
                         hass_shrink_coefsum[c+(x+2*y)*hass_channel] += w;
                    }
               }

     // Filter Channel

     for(int j=0;j < hass_rep_unit;j++){
          for(int i=0;i < hass_rep_unit;i++){
               hass_filter_ch[i+hass_rep_unit*j] = (unsigned char) hass_filter_no[j][i];
          }
     }

     // Color Channel

     for(int j=0;j < hass_rep_unit;j++){
          for(int i=0;i < hass_rep_unit;i++){
               for(int c=0;c < hass_channel;c++){
                    hass_color_ch[c+hass_channel*(i+hass_rep_unit*j)] = (hass_filter_no[j][i]==c) ? 1 : 0;
               }
          }
     }

     // Coeff for Reconstruction (Lambda)

     for(int c=0;c < hass_channel;c++){
          for(int s=0;s < hass_spectrum;s++){
               switch(hass_mode){
                    case HASS_MODE_MULT: // Multiple
                         switch(s){
                              case 0  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_350][c]; break;
                              case 1  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_400][c]; break;
                              case 2  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_450][c]; break;
                              case 3  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_500][c]; break;
                              case 4  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_550][c]; break;
                              case 5  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_600][c]; break;
                              case 6  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_650][c]; break;
                              case 7  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_700][c]; break;
                              case 8  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_720][c]; break;
                              case 9  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_750][c]; break;
                              case 10 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_780][c]; break;
                              case 11 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_800][c]; break;
                              case 12 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_850][c]; break;
                              case 13 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_900][c]; break;
                              case 14 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_950][c]; break;
                              case 15 : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_1000][c]; break;
                              default : hass_recon_coef[c+hass_channel*s] = 0 ; break;
                         }
                       break;
	            case HASS_MODE_NDVI: // NDVI
                         switch(s){
                              case 0  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_650][c]; break;
                              case 1  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_750][c]; break;
                              default : hass_recon_coef[c+hass_channel*s] = 0 ; break;
                         }
                       break;
                    case HASS_MODE_RGB:  // RGB
                         switch(s){
                              case 0  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_600][c]; break;
                              case 1  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_530][c]; break;
                              case 2  : hass_recon_coef[c+hass_channel*s] = LutHassLambda[WAVELENGTH_460][c]; break;
                              default : hass_recon_coef[c+hass_channel*s] = 0 ; break;
                         }
                       break;
                    case HASS_MODE_NONE: // None
                         hass_recon_coef[c+hass_channel*s] = (float)1.0/(float)hass_channel; break;
                    default:             // Spectrum
                         hass_recon_coef[c+hass_channel*s] = LutHassLambda[hass_mode][c]; break;
               }
          }
     }
     /*
     for(int s=0;s < hass_spectrum;s++)     
       LOGI("[%d] %f %f %f %f %f ...\n", s, hass_recon_coef[0+hass_channel*s], hass_recon_coef[1+hass_channel*s], hass_recon_coef[2+hass_channel*s], hass_recon_coef[3+hass_channel*s], hass_recon_coef[4+hass_channel*s]);
     */
     
     // Coeff for Guide

     float *power= new float[hass_channel];

     for(int c = 0;c < hass_channel; c++) power[c] = 0.0f;

     for(int j=0;j < hass_rep_unit;j++){
          for(int i=0;i < hass_rep_unit;i++){
               power[hass_filter_no[j][i]]+=1.0;
          }
     }

     for(int y = 0; y < hass_rep_unit*hass_rep_unit*hass_channel; ++y) hass_guide_coef[y] = 0;

     int tapnum = hass_rep_unit +1;

     for(int y = 0; y < hass_rep_unit; ++y) {
          for(int x = 0; x < hass_rep_unit; ++x) {
               for(int c = 0;c < hass_channel; c++){
                    for(int yy = 0; yy < tapnum; yy++) {
                         for(int xx = 0; xx < tapnum; xx++) {

                              int cx  = (x + xx - tapnum/2);
                              int cy  = (y + yy - tapnum/2);
                              int pcx = (cx + hass_rep_unit)%hass_rep_unit;
                              int pcy = (cy + hass_rep_unit)%hass_rep_unit;

                              if(c == hass_filter_no[pcy][pcx]){
                                   hass_guide_coef[(y*hass_rep_unit+x)*hass_channel + c] += 1.0;
                              }
                         }
                    }
               }

               //combine power and coef
               for(int c = 0; c < hass_channel; c++) {
                    hass_guide_coef[(y*hass_rep_unit + x)*hass_channel + c] = power[c] / hass_guide_coef[(y*hass_rep_unit + x)*hass_channel + c];
               }
          }
     }

     delete[] power;

     // Coefficient for Combine

     int combine_tap_center = hass_rep_unit/2;
     int combine_tap_side   = hass_rep_unit/4;
     hass_combine_tap = combine_tap_center + combine_tap_side;

     hass_combine_noise = 0.1;

     for(int i = 0; i < combine_tap_center; i++) hass_combine_coef[i] = 1.0;

     for(int i = combine_tap_center ; i < hass_combine_tap; i++) {
          float ds = (float)(i-combine_tap_center) / (float)combine_tap_center;
          hass_combine_coef[i] = 1.0 - ds;
     }

     float hass_combine_coef_sum = 0.0;

     for(int j = -(hass_combine_tap-1); j <= (hass_combine_tap-1);j++)
          for(int i = -(hass_combine_tap-1); i <= (hass_combine_tap-1);i++)
               hass_combine_coef_sum += hass_combine_coef[abs(i)]*hass_combine_coef[abs(j)];

     hass_combine_coef[hass_combine_tap] = hass_combine_coef_sum;

     // Gamma Table

     CalcHassGamma(hass_gamma_tbl);

    int aa = 0;
    for(int y = 0;y < hass_height; y++){
        for(int x = 0;x < hass_width; x++){
            aa = aa * 214013 + 2531011;
            hass_enc_tbl[hass_width * y + x] = (unsigned short)( ( aa >> 16 ) & 0x7fff );
        }
    }
}

void HassProc::UpdateCoefficients(void) {
     // Load Coefficients to OpenCL Memory

     size_t d_gamma_tbl_sz = (1<<hass_raw_bit);
     size_t d_color_tbl_sz = 64*3;

     clEnqueueWriteBuffer(GetCommandQueue(), d_shrink_coef    ,  CL_TRUE, 0,  d_shrink_coef_sz,    (void *) &hass_shrink_coef[0],    0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_shrink_coefsum ,  CL_TRUE, 0,  d_shrink_coefsum_sz, (void *) &hass_shrink_coefsum[0], 0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_color_ch       ,  CL_TRUE, 0,  d_color_ch_sz,       (void *) &hass_color_ch[0],       0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_recon_coef     ,  CL_TRUE, 0,  d_recon_coef_sz,     (void *) &hass_recon_coef[0],     0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_guide_coef     ,  CL_TRUE, 0,  d_guide_coef_sz,     (void *) &hass_guide_coef[0],     0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_filter_config  ,  CL_TRUE, 0, (64*sizeof(int)),     (void*)(&(hass_filter_no[0][0])), 0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_filter_ch      ,  CL_TRUE, 0,  d_filter_ch_sz,      (void *) &hass_filter_ch[0],      0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_combine_coef   ,  CL_TRUE, 0,  d_combine_coef_sz,   (void *) &hass_combine_coef[0],   0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_gamma_tbl      ,  CL_TRUE, 0,  d_gamma_tbl_sz,      (void *) &hass_gamma_tbl[0],      0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_color_tbl      ,  CL_TRUE, 0,  d_color_tbl_sz,      (void *) &hass_color_tbl[0],      0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_enc_tbl        ,  CL_TRUE, 0,  d_enc_tbl_sz,        (void *) &hass_enc_tbl[0],        0, NULL, NULL);
}

void HassProc::ReleaseCoefficients(void) {
     delete hass_shrink_coef;
     delete hass_shrink_coefsum;
     delete hass_filter_ch;
     delete hass_color_ch;
     delete hass_recon_coef;
     delete hass_guide_coef;
     delete hass_combine_coef;
     delete hass_gamma_tbl;
     delete hist;
     delete hist_f;
     delete hass_enc_tbl;
}

bool HassProc::CreateHassBuffers(void){

     ////////////////////////////////////////////////////////
     // For PreProcessing

     switch(hass_raw_input_mode){
          case 1: // RAW_SENSOR
               d_input_sz       = hass_iwidth*hass_iheight*sizeof(jbyte)*2;
             break;
          case 2: // YUV
               d_input_sz       = hass_iwidth*hass_iheight*sizeof(jbyte);
             break;
          default: // RAW10
               d_input_sz       = hass_iwidth*hass_iheight*sizeof(jbyte)*10/8;
             break;
     }

     d_pre_sz         = hass_iwidth*hass_iheight*sizeof(unsigned short);

     d_input    = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY, d_input_sz, NULL, &hass_err);

#ifdef ACCELERATE_ZERO_COPY
     d_input1   = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR , d_input_sz, NULL, &hass_err);
     d_input2   = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR , d_input_sz, NULL, &hass_err);
#endif

     if (hass_err < 0) return hass_err;

     d_pre      = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_pre_sz, NULL, &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For Shrink

     d_shrink_sz      = hass_widthShrink * hass_heightShrink * hass_channel * sizeof(float);
     d_shrinkint_sz   = hass_widthShrink * hass_height * hass_channel * sizeof(float);
     d_shrink_coef_sz = hass_shrink_tap * sizeof(float);
     d_color_ch_sz    = hass_rep_unit * hass_rep_unit * hass_channel * sizeof(unsigned char);
     d_shrink_coefsum_sz = 2 * 2 * hass_channel * sizeof(float);

     d_shrink_coef = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_shrink_coef_sz, (void *) &hass_shrink_coef[0], &hass_err);
     if (hass_err < 0) return hass_err;

     d_shrink_coefsum = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_shrink_coefsum_sz, (void *) &hass_shrink_coefsum[0], &hass_err);
     if (hass_err < 0) return hass_err;

     d_color_ch    = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_color_ch_sz, (void *) &hass_color_ch[0], &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For Saturation Map

     d_sat_map_sz     = hass_widthShrink * hass_heightShrink;

     ////////////////////////////////////////////////////////
     // For Noise Reduction

     d_nr_sz  = hass_widthShrink * hass_heightShrink * hass_channel * sizeof(float);

     d_nri    = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_nr_sz, NULL, &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For Reconstruction

     d_recon_sz       = hass_widthShrink * hass_heightShrink * hass_spectrum * sizeof(float);
     d_recon_coef_sz  = hass_channel * hass_spectrum * sizeof(float);

     d_recon_coef = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_recon_coef_sz, (void *) &hass_recon_coef[0], &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For App

     d_app_sz  = hass_widthShrink * hass_heightShrink * hass_spectrum * sizeof(float);

     ////////////////////////////////////////////////////////
     // For Zoom

     d_zoom_sz = hass_width * hass_height * hass_spectrum * sizeof(float);

     ////////////////////////////////////////////////////////
     // For Guide

     d_guide_sz       = hass_width * hass_height * sizeof(float);
     d_guide_coef_sz  = hass_rep_unit * hass_rep_unit * hass_channel * sizeof(float);
     d_filter_ch_sz   = hass_rep_unit * hass_rep_unit * sizeof(unsigned char);

     d_guide_coef     = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_guide_coef_sz, (void *) &hass_guide_coef[0], &hass_err);
     if (hass_err < 0) return hass_err;

     d_filter_config  = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR,(64*sizeof(int)), ((void*)(&(hass_filter_no[0][0]))),&hass_err);
     if (hass_err < 0) return hass_err;

     d_filter_ch    = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_filter_ch_sz, (void *) &hass_filter_ch[0], &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For Combine

     d_combine_sz       = hass_width * hass_height * sizeof(float);
     d_combine_coef_sz  = (hass_combine_tap+1) * sizeof(float);

     d_combine_coef = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_combine_coef_sz, (void *) &hass_combine_coef[0], &hass_err);
     if (hass_err < 0) return hass_err;

     ////////////////////////////////////////////////////////
     // For PostProcessing

     d_post_sz = 4 * hass_width * hass_height * sizeof(jbyte);
     size_t d_gamma_tbl_sz = (1<<hass_raw_bit);
     size_t d_color_tbl_sz = 64*3;

     
     //     d_post1   = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, d_post_sz, NULL, &hass_err);
     //     d_post2   = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR, d_post_sz, NULL, &hass_err);

     if (hass_err < 0) return hass_err;

     d_gamma_tbl  = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_gamma_tbl_sz, (void *) &hass_gamma_tbl[0], &hass_err);
     if (hass_err < 0) return hass_err;

     d_color_tbl  = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_color_tbl_sz, (void *) &hass_color_tbl[0], &hass_err);
     if (hass_err < 0) return hass_err;

#ifdef ACCELERATE_ZERO_COPY
     ctrl_pre  = &_kernel_controls[HASS_PRE_PROCESSING];
     ctrl_post = &_kernel_controls[HASS_POST_PROCESSING];

#ifdef ACCELERATE_ZERO_COPY_NONBLOCKING
     inp1  = (jbyte *)clEnqueueMapBuffer(GetReadWriteQueue(), d_input1, CL_TRUE, CL_MAP_WRITE, 0, d_input_sz, 0, NULL, NULL, &hass_err);
     outp1 = (jbyte *)clEnqueueMapBuffer(GetReadWriteQueue(), d_post1,  CL_TRUE, CL_MAP_READ,  0, d_post_sz,  0, NULL, NULL, &hass_err);
     inp2  = (jbyte *)clEnqueueMapBuffer(GetReadWriteQueue(), d_input2, CL_TRUE, CL_MAP_WRITE, 0, d_input_sz, 0, NULL, NULL, &hass_err);
     outp2 = (jbyte *)clEnqueueMapBuffer(GetReadWriteQueue(), d_post2,  CL_TRUE, CL_MAP_READ,  0, d_post_sz,  0, NULL, NULL, &hass_err);
#endif

#endif

     ////////////////////////////////////////////////////////
     // For WDR

     d_long_sz  = hass_width * hass_height * sizeof(unsigned short);
     d_short_sz = hass_width * hass_height * sizeof(unsigned short);
     d_wdr_sz   = hass_width * hass_height * sizeof(unsigned short);
     d_ave_sz   = hass_width * hass_height * sizeof(unsigned int);
     d_disp_sz  = hass_width * hass_height * sizeof(unsigned short) * 2;
     d_hist_sz  = nbin * sizeof(int);
     d_enc_tbl_sz  = hass_width * hass_height * sizeof(unsigned short);
     d_mode_sz  = 1* sizeof(int);

     d_long     = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_long_sz,  NULL, &hass_err);
     d_short    = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_short_sz, NULL, &hass_err);
     d_wdr      = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_wdr_sz,   NULL, &hass_err);
     d_ave      = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_ave_sz,   NULL, &hass_err);
     d_disp     = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_disp_sz,  NULL, &hass_err);
     d_hist     = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_hist_sz,  NULL, &hass_err);
     d_enc_tbl  = clCreateBuffer(GetContext(),CL_MEM_READ_ONLY|CL_MEM_COPY_HOST_PTR, d_enc_tbl_sz, (void *) &hass_enc_tbl[0], &hass_err);
     d_mode     = clCreateBuffer(GetContext(),CL_MEM_READ_WRITE, d_mode_sz,  NULL, &hass_err);

     return hass_err;
}

void HassProc::ReleaseHassBuffers(void){

     clReleaseMemObject(d_pre);
     clReleaseMemObject(d_input);
#ifdef ACCELERATE_ZERO_COPY
     clReleaseMemObject(d_input1);
     clReleaseMemObject(d_input2);
#endif
     clReleaseMemObject(d_shrink_coef);
     clReleaseMemObject(d_shrink_coefsum);
     clReleaseMemObject(d_color_ch);
     clReleaseMemObject(d_recon_coef);
     clReleaseMemObject(d_guide_coef);
     clReleaseMemObject(d_filter_ch);
     clReleaseMemObject(d_filter_config);
     clReleaseMemObject(d_combine);
     clReleaseMemObject(d_combint);
     clReleaseMemObject(d_combine_coef);
     clReleaseMemObject(d_gamma_tbl);
     clReleaseMemObject(d_color_tbl);
     clReleaseMemObject(d_enc_tbl);
     clReleaseMemObject(d_nri);

     clReleaseMemObject(d_long);
     clReleaseMemObject(d_short);
     clReleaseMemObject(d_wdr);
     clReleaseMemObject(d_ave);
     clReleaseMemObject(d_disp);
     clReleaseMemObject(d_hist);
     clReleaseMemObject(d_mode);
}


bool HassProc::CreateHassPreKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_PRE_PROCESSING];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name = _kname_HassPre;           // Kernel name
     ctrl->filename = _kfname_HassPre;      // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_input)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_pre)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(xOffset),((void *)(&xOffset)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(yOffset),((void *)(&yOffset)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(blackLevel) ,((void *)(&blackLevel)));
     if (hass_err < 0) return hass_err;     

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_iwidth),((void *)(&hass_iwidth)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,6,sizeof(hass_iheight),((void *)(&hass_iheight)));
     if (hass_err < 0) return hass_err;


     return succeed;
}


bool HassProc::CreateHassShrinkKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_SHRINK];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name = _kname_HassShrink;        // Kernel name
     ctrl->filename = _kfname_HassShrink;   // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

#ifdef ACCELERATE_KERNEL_SHRINK
     kernel_control* ctrl2 = &_kernel_controls[HASS_SHRINK2];

     hass_err = CL_SUCCESS;
     ctrl2->name = _kname_HassShrink2;        // Kernel name
     ctrl2->filename = _kfname_HassShrink;   // Kernel file name
     ctrl2->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl2);
#endif

     ////////////////////////////////////////////////////////
     // Setup Arguments
#ifdef SET_HASS_WDR
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_wdr)));
#else
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_pre)));
#endif
     if (hass_err < 0) return hass_err;

#ifdef ACCELERATE_KERNEL_SHRINK
     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_shrinkint)));
     if (hass_err < 0) return hass_err;
#else
     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_shrink)));
     if (hass_err < 0) return hass_err;
#endif
     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_shrink_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(cl_mem),((void *)(&d_color_ch)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

#ifdef ACCELERATE_KERNEL_SHRINK
     hass_err=clSetKernelArg(ctrl2->kernel,0,sizeof(cl_mem),((void *)(&d_shrinkint)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,1,sizeof(cl_mem),((void *)(&d_shrink)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,2,sizeof(cl_mem),((void *)(&d_shrink_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,3,sizeof(cl_mem),((void *)(&d_shrink_coefsum)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,4,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,5,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;
#endif

     return succeed;
}

bool HassProc::CreateHassNoiseReductionKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_NOISE_REDUCTION];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassNr;        // Kernel name
     ctrl->filename = _kfname_HassNr;       // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_shrink)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_nr)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(hass_nr_tap),((void *)(&hass_nr_tap)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_widthShrink),((void *)(&hass_widthShrink)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_heightShrink),((void *)(&hass_heightShrink)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassReconstructionKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RECONSTRUCTION];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassRecon;     // Kernel name
     ctrl->filename = _kfname_HassRecon;    // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_nr)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_recon)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_recon_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(cl_mem),((void *)(&d_sat_map)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_sat_val),((void *)(&hass_sat_val)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_widthShrink),((void *)(&hass_widthShrink)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,6,sizeof(hass_heightShrink),((void *)(&hass_heightShrink)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassAppKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_APP];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassApp;       // Kernel name
     ctrl->filename = _kfname_HassApp;      // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_recon)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_app)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(hass_widthShrink),((void *)(&hass_widthShrink)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_heightShrink),((void *)(&hass_heightShrink)));
     if (hass_err < 0) return hass_err;

     return succeed;
}


bool HassProc::CreateHassZoomKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_ZOOM];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassZoom;      // Kernel name
     ctrl->filename = _kfname_HassZoom;     // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_app)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_combine)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(cl_mem),((void *)(&d_shrink_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return succeed;
}



bool HassProc::CreateHassGuideKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_GUIDE];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name = _kname_HassGuide;         // Kernel name
     ctrl->filename = _kfname_HassGuide;    // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

#ifdef ACCELERATE_KERNEL_GUIDE
     kernel_control* ctrl2 = &_kernel_controls[HASS_GUIDE2];

     hass_err = CL_SUCCESS;
     ctrl2->name = _kname_HassGuide2;         // Kernel name
     ctrl2->filename = _kfname_HassGuide;    // Kernel file name
     ctrl2->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl2);
#endif

     ////////////////////////////////////////////////////////
     // Setup Arguments

#ifdef ACCELERATE_KERNEL_GUIDE

#ifdef SET_HASS_WDR
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_wdr)));
#else
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_pre)));
#endif
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_guideint)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,0,sizeof(cl_mem),((void *)(&d_guideint)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,1,sizeof(cl_mem),((void *)(&d_guide)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,2,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,3,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;
#else

     #ifdef SET_HASS_WDR
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_wdr)));
#else
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_pre)));
#endif
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_guide)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_guide_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(cl_mem),((void *)(&d_filter_ch)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;
#endif

     return succeed;
}

bool HassProc::CreateHassCombineKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_COMBINE];

     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassCombine;   // Kernel name
     ctrl->filename = _kfname_HassCombine;  // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

#ifdef ACCELERATE_KERNEL_COMBINE
     kernel_control* ctrl2 = &_kernel_controls[HASS_COMBINE2];
     ctrl2->name     = _kname_HassCombine2;   // Kernel name
     ctrl2->filename = _kfname_HassCombine;  // Kernel file name
     ctrl2->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl2);
#endif
     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_guide)));
     if (hass_err < 0) return hass_err;
#ifdef ACCELERATE_KERNEL_COMBINE
     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_combint)));
     if (hass_err < 0) return hass_err;
#else
     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_combine)));
     if (hass_err < 0) return hass_err;
#endif

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_combine_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_combine_noise),((void *)(&hass_combine_noise)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,5,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

#ifdef ACCELERATE_KERNEL_COMBINE
     hass_err=clSetKernelArg(ctrl2->kernel,0,sizeof(cl_mem),((void *)(&d_guide)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,1,sizeof(cl_mem),((void *)(&d_combint)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,2,sizeof(cl_mem),((void *)(&d_combine)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,3,sizeof(cl_mem),((void *)(&d_combine_coef)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,4,sizeof(hass_combine_noise),((void *)(&hass_combine_noise)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,5,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl2->kernel,6,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;
#endif

     return succeed;
}

bool HassProc::CreateHassPostProcessingKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_POST_PROCESSING];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassPost;      // Kernel name
     ctrl->filename = _kfname_HassPost;     // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

//     switch(hass_mode){
//          case HASS_MODE_NDVI: // NDVI
//               hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_color_tbl)));
//             if (hass_err < 0) return hass_err;
//             break;
//          default: // RGB
//               hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
//             if (hass_err < 0) return hass_err;
//             break;
//     }

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassPostNdviKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_POST_NDVI];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassPostNdvi;      // Kernel name
     ctrl->filename = _kfname_HassPostNdvi;     // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_color_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassPostMultKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_POST_MULT];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassPostMult;      // Kernel name
     ctrl->filename = _kfname_HassPostMult;     // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassWdrKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_WDR];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassWdr;       // Kernel name
     ctrl->filename = _kfname_HassWdr;      // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_pre)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_short)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(cl_mem)     ,((void *)(&d_long)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 3,sizeof(cl_mem)     ,((void *)(&d_wdr)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 4,sizeof(blackLevel) ,((void *)(&blackLevel)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 5,sizeof(shs1Offset) ,((void *)(&shs1Offset)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 6,sizeof(shs2Offset) ,((void *)(&shs2Offset)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 7,sizeof(Gain)       ,((void *)(&Gain)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 8,sizeof(BLST)       ,((void *)(&BLST)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 9,sizeof(BLED)       ,((void *)(&BLED)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,10,sizeof(hass_width), ((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,11,sizeof(hass_iwidth),((void *)(&hass_iwidth)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,12,sizeof(cl_mem)     ,((void *)(&d_ave)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,13,sizeof(average_gain)     ,((void *)(&average_gain)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,14,sizeof(cl_mem)     ,((void *)(&d_mode)));
     if (hass_err < 0) return hass_err;
     
     return succeed;
}

bool HassProc::CreateHassRawToArgbKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RAWTOARGB];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassRawToArgb;       // Kernel name
     ctrl->filename = _kfname_HassRawToArgb;      // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_wdr)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_disp)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(hass_width), ((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 3,sizeof(display_lsb), ((void *)(&display_lsb)));
     if (hass_err < 0) return hass_err;
     return succeed;
}

bool HassProc::CreateHassRawEncKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RAWENC];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassRawEnc;       // Kernel name
     ctrl->filename = _kfname_HassRawEnc;      // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_wdr)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_enc_tbl)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(hass_width), ((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassHistKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_HIST];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassHist;      // Kernel name
     ctrl->filename = _kfname_HassHist;     // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

#ifdef SET_HASS_WDR
     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_short)));
#else
     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_pre)));
#endif
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_hist)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(nbin)       ,((void *)(&nbin)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 3,sizeof(BLED)       ,((void *)(&BLED)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 4,sizeof(hass_width) ,((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     return succeed;
}

bool HassProc::CreateHassSatMapKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_SATMAP];
     bool succeed = false;
     hass_err = CL_SUCCESS;
     ctrl->name     = _kname_HassSatMap;    // Kernel name
     ctrl->filename = _kfname_HassSatMap;   // Kernel file name
     ctrl->coption  = _kcoption_Hass;       // Kernel compiler option
     succeed = CreateKernel(ctrl);

     ////////////////////////////////////////////////////////
     // Setup Arguments

#ifdef SET_HASS_WDR
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_wdr)));
#else
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_pre)));
#endif
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_sat_map)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(hass_sat_th),((void *)(&hass_sat_th)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return succeed;
}


void HassProc::CreateHassKernels(void){

     SetImageSize();
     SetKernelInformation();
     CreateCoefficients();
     SetCoefficients();

     CreateHassBuffers();

     CreateHassPreKernel();

    /*
     CreateHassGuideKernel();
     CreateHassCombineKernel();

     CreateHassShrinkKernel();
     CreateHassNoiseReductionKernel();
     CreateHassReconstructionKernel();
     CreateHassAppKernel();
     CreateHassZoomKernel();
     CreateHassPostProcessingKernel();
     CreateHassPostNdviKernel();
     CreateHassPostMultKernel();
*/
     CreateHassWdrKernel();
     CreateHassRawToArgbKernel();
     CreateHassHistKernel();
     CreateHassRawEncKernel();

     //CreateHassSatMapKernel();
}

void HassProc::RunHassPreKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_PRE_PROCESSING];
     size_t globalSize[2]  = {(size_t)((hass_raw_input_mode==0) /* RAW10 */ ? (hass_iwidth/4) : hass_iwidth), (size_t)hass_iheight};
     size_t *localSize     = NULL;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, 2, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassShrinkKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_SHRINK];

#ifdef ACCELERATE_KERNEL_SHRINK
     kernel_control* ctrl2 = &_kernel_controls[HASS_SHRINK2];
     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_height};
     size_t globalSize2[2] = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t localSize [2]  = {(size_t)128,              (size_t)8};
     cl_int dimSize        = 2;
#else
     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t *localSize     = NULL;
     cl_int dimSize        = 2;
#endif

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);

#ifdef ACCELERATE_KERNEL_SHRINK

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl2->kernel, dimSize, NULL, globalSize2, localSize, 0, NULL, NULL);
#endif
}

void HassProc::RunHassNoiseReductionKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_NOISE_REDUCTION];
     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t localSize [2]  = {(size_t)128,              (size_t)8};
     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassReconstructionKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RECONSTRUCTION];
     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t localSize [2]  = {(size_t)128,              (size_t)8};
     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassAppKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_APP];

     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t *localSize     = NULL;
     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassZoomKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_ZOOM];
     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t localSize[2]   = {(size_t)16, (size_t)16};
     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassGuideKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_GUIDE];
#ifdef ACCELERATE_KERNEL_GUIDE
     kernel_control* ctrl2 = &_kernel_controls[HASS_GUIDE2];
     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize[2]   = {(size_t)128, (size_t)8};
     cl_int dimSize        = 2;
#else
     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t *localSize     = NULL;
     cl_int dimSize        = 2;
#endif

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
#ifdef ACCELERATE_KERNEL_GUIDE
     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl2->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
#endif
}

void HassProc::RunHassCombineKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_COMBINE];
#ifdef ACCELERATE_KERNEL_COMBINE
     kernel_control* ctrl2= &_kernel_controls[HASS_COMBINE2];

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };
     cl_int dimSize        = 2;
#else
     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t *localSize     = NULL;
     cl_int dimSize        = 2;
#endif

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);

#ifdef ACCELERATE_KERNEL_COMBINE
     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl2->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
#endif
}

void HassProc::RunHassPostProcessingKernel(void){
     kernel_control* ctrl = NULL;

     switch(hass_mode){
          case HASS_MODE_NDVI: // NDVI
               ctrl = &_kernel_controls[HASS_POST_NDVI];
             break;
          default: // RGB or Single Wavelength
               ctrl = &_kernel_controls[HASS_POST_PROCESSING];
             break;
     }

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };

     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassWdrKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_WDR];

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };

     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassRawToArgbKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RAWTOARGB];

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };

     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassRawEncKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RAWENC];

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };

     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}

void HassProc::RunHassHistKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_HIST];

     size_t globalSize[2]  = {(size_t)hass_width, (size_t)hass_height};
     size_t localSize [2]  = {(size_t)128,        (size_t)8          };

     cl_int dimSize        = 2;

     int zero = 0;

     clEnqueueFillBuffer   (GetCommandQueue(), d_hist, &zero, sizeof(int),  0, d_hist_sz,    0, NULL, NULL);
     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
     clEnqueueReadBuffer   (GetCommandQueue(), d_hist,   CL_TRUE,  0, d_hist_sz,  hist,      0, NULL, NULL);

     CalcAE();
}

void HassProc::RunHassSatMapKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_SATMAP];

     size_t globalSize[2]  = {(size_t)hass_widthShrink, (size_t)hass_heightShrink};
     size_t *localSize     = NULL;
     cl_int dimSize        = 2;

     clEnqueueNDRangeKernel(GetCommandQueue(), ctrl->kernel, dimSize, NULL, globalSize, localSize, 0, NULL, NULL);
}


void HassProc::CalcAE(void){

    //*********************************: parameters ************************************
     const float exception_target_level = 0.05;                 //Exception processing parameter in case of too dark
     const float min_pgain = ((float)(nbin+1))/((float)nbin);   //minimum gain up with current histogram
     const float min_mgain = ((float)(nbin-1))/((float)nbin);   //minimum gain down with current histogram
     const int saturation_margin = 10;                          //fixed margin to protect saturation
     const int histeresis_margin = 28;                          //AE control histeresis for worst case(shs1=2)
     const float histogram_max_value = 0.9999;                  //target histogram max value without 1 to protect saturation
     const float histogram_min_value = 0.0001;                  //target histogram max value without 0 to protect error
     const float exposure_ratio = 16.0f;                        //exposure ratio
     const int range_margin = 16;                               //histogram position on margin for protect to saturate


     //*********************************: histogram pre processing ************************************
     // The cumulative histogram
     for(int i=1;i<nbin;i++){
          hist[i] = hist[i] + hist[i-1];
     }

     for(int i=0;i<nbin;i++){
          hist_f[i] = (float)hist[i] / (float)(hass_width*hass_height);
     }

    // bottom, middle and top number of the cumulative histogram bin
    int low_i=0; int mid_i=0; int hgh_i=0;
    while(hist_f[low_i] < 1.0) low_i++;
    while(hist_f[mid_i] < 0.5) mid_i++;
    while(hist_f[hgh_i] <= 0.0) hgh_i++;


    //*********************************: AE condition calculate  ************************************
    //histogram value to check saturation
    float saturation_hist   = hist_f[nbin - saturation_margin];
    //histogram value to control exposure time
    float reference_hist    = hist_f[nbin - saturation_margin - 1];
    //histogram value to protect flicking with histeresis
    float histeresis_hist   = hist_f[nbin - saturation_margin - 1 - histeresis_margin];
    
    //Reduce the target value to allow saturation in biased situations
    float target_histogram_value;
    {
        //Value to weak this function in biased situations.
        float bias_value;
        float tmp_reference_hist;

        if(reference_hist < histogram_min_value)    tmp_reference_hist = histogram_min_value;
        else                                        tmp_reference_hist = reference_hist;

        //Value to weak this reduction processing in the bright case.
        bias_value = (hist_f[range_margin] + ((float)hass_auto_exposure_range)/100.0) / tmp_reference_hist;

        if(bias_value > 1.0) bias_value = 1.0;

        target_histogram_value = histogram_max_value - (((float)hass_auto_exposure_target)/10000.0) * bias_value;
    }
    
    
    //*********************************: AE short exposure target gain calculation  ************************************
    float LastAEGain = AEGain;  // to check mountain

    if((low_i <= 0) || (hgh_i >= (nbin-2))){
        //Exception processing for too dark/bright situation
        AEGain = exception_target_level / (((float)(mid_i+1))/((float)nbin));
    }
    else{
        //to check whether gain up or gain down
        if(saturation_hist >= target_histogram_value){
            //to check whether ideal exposure time or not
            if(reference_hist <= target_histogram_value){
                AEGain = 1.0;
            }
            else{
                //to keep position in histeresis area at mountain
                if((LastAEGain  <= 1.0) && (histeresis_hist<= target_histogram_value)){
                    AEGain = 1.0;
                }
                else {
                    AEGain = min_pgain * (1.0 + (hass_auto_exposure_speed_long / 10.0));
                }
            }
        }
        else{
            if(LastAEGain  >= 1.0){
                //to enter histeresis with the slowest speed
                AEGain = min_mgain;
            }
            else{
                AEGain = min_mgain / (1.0 + (hass_auto_exposure_speed_short / 10.0));
            }
        }
    }

    
    //*********************************: short exposure time calculation  ************************************
    unsigned short tmp_shortExp = (unsigned short)(AEGain * (float)shortExp + 0.5f);

    if(AEGain > 1.0) {
        if(shortExp == tmp_shortExp)    shortExp = shortExp + 1;
        else                            shortExp = tmp_shortExp;
    }
    else if(AEGain < 1.0){
        if(shortExp == tmp_shortExp)    shortExp = shortExp - 1;
        else                            shortExp = tmp_shortExp;
    }


    //*********************************: Long exposure time calculation  ************************************
     shortExp = (shortExp > (RHS1-2)) ? RHS1-2 : shortExp;
     shortExp = (shortExp < 1) ? 1 : shortExp;
     longExp  = (unsigned short)(exposure_ratio * (float)shortExp + 0.5f);

     SHS1 = RHS1 - shortExp -1;
#ifdef SET_HASS_WDR
     SHS2 = 2*VMAX - longExp -1;
#endif
    /*
    //HDR_ON_OFF
    {
        {
            float real_shortExp = (shortExp > (RHS1-2)) ? RHS1-2 : shortExp;
            real_shortExp = (real_shortExp < 1) ? 1 : real_shortExp;

            SHS1 = RHS1 - real_shortExp -1;
        }

        shortExp = (shortExp > (2*VMAX-2)) ? 2*VMAX-2 : shortExp;
        shortExp = (shortExp < 1) ? 1 : shortExp;

        SHS2 = 2*VMAX - shortExp -2;
    }
    */
}


void HassProc::RunHassKernelsDB(void* rawImage1, void* rawImage2, void* rgbOut1, void* rgbOut2) {

#ifdef ACCELERATE_ZERO_COPY
     clSetKernelArg(ctrl_pre->kernel, 0,sizeof(cl_mem),((void *)(&d_input2)));
     clSetKernelArg(ctrl_post->kernel,1,sizeof(cl_mem),((void *)(&d_post2)));
#endif

     RunHassKernels();

#ifdef ACCELERATE_ZERO_COPY

#ifndef ACCELERATE_ZERO_COPY_NONBLOCKING
     inp1  = (jbyte*)clEnqueueMapBuffer(GetReadWriteQueue(), d_input1, CL_FALSE, CL_MAP_WRITE, 0, d_input_sz, 0, NULL, NULL, &hass_err);
     outp1 = (jbyte*)clEnqueueMapBuffer(GetReadWriteQueue(), d_post1,  CL_FALSE, CL_MAP_READ,  0, d_post_sz,  0, NULL, NULL, &hass_err);
     clFinish(GetReadWriteQueue());
#endif

     memcpy(inp1,  rawImage1, d_input_sz);
     memcpy(rgbOut1, outp1,   d_post_sz);

#ifndef ACCELERATE_ZERO_COPY_NONBLOCKING
     clEnqueueUnmapMemObject(GetReadWriteQueue(), d_input1, inp1,  0, NULL, NULL);
     clEnqueueUnmapMemObject(GetReadWriteQueue(), d_post1,  outp1, 0, NULL, NULL);
#endif


#else
     clEnqueueReadBuffer(GetReadWriteQueue(), d_post, CL_FALSE, 0, d_post_sz, rgbOut1, 0, NULL, NULL );
     clEnqueueWriteBuffer(GetReadWriteQueue(), d_input, CL_FALSE, 0, d_input_sz, rawImage1,    0, NULL, NULL ); 

     clFinish(GetReadWriteQueue());    // Make sure the read and write is already done
     clFinish(GetCommandQueue());    // Wait for kernel to finish
#endif

#ifdef ACCELERATE_ZERO_COPY
     clSetKernelArg(ctrl_pre->kernel, 0,sizeof(cl_mem),((void *)(&d_input1)));
     clSetKernelArg(ctrl_post->kernel,1,sizeof(cl_mem),((void *)(&d_post1)));
#endif

     RunHassKernels();

#ifdef ACCELERATE_ZERO_COPY

#ifndef ACCELERATE_ZERO_COPY_NONBLOCKING
     inp2  = (jbyte*)clEnqueueMapBuffer(GetReadWriteQueue(), d_input2, CL_FALSE, CL_MAP_WRITE, 0, d_input_sz, 0, NULL, NULL, &hass_err);
     outp2 = (jbyte*)clEnqueueMapBuffer(GetReadWriteQueue(), d_post2,  CL_FALSE, CL_MAP_READ,  0, d_post_sz,  0, NULL, NULL, &hass_err);
     clFinish(GetReadWriteQueue());
#endif

     memcpy(inp2,  rawImage2, d_input_sz);
     memcpy(rgbOut2, outp2,   d_post_sz);

#ifndef ACCELERATE_ZERO_COPY_NONBLOCKING
     clEnqueueUnmapMemObject(GetReadWriteQueue(), d_input2, inp2,  0, NULL, NULL);
     clEnqueueUnmapMemObject(GetReadWriteQueue(), d_post2,  outp2, 0, NULL, NULL);
#endif


#else
     clEnqueueReadBuffer(GetReadWriteQueue(), d_post, CL_FALSE, 0, d_post_sz, rgbOut2, 0, NULL, NULL );
     clEnqueueWriteBuffer(GetReadWriteQueue(), d_input, CL_FALSE, 0, d_input_sz, rawImage2,   0, NULL, NULL );


     clFinish(GetReadWriteQueue());    // Make sure the read and write is already done
     clFinish(GetCommandQueue());    // Wait for kernel to finish
#endif
}

void HassProc::RunHassKernelsSB(void* rawImage, void* rgbOut){

     clEnqueueWriteBuffer(GetCommandQueue(), d_input, CL_FALSE, 0, d_input_sz, rawImage, 0, NULL, NULL);

     RunHassKernels();

     clEnqueueReadBuffer(GetCommandQueue(), d_post, CL_TRUE, 0, d_post_sz, rgbOut, 0, NULL, NULL);
}

void HassProc::RunHassWdrRawKernelsSB(void* rawImage, void* rawOut, void *dispOut, int tmp_fnum, int displayoff){

     clEnqueueWriteBuffer(GetCommandQueue(), d_input, CL_FALSE, 0, d_input_sz, rawImage, 0, NULL, NULL);
     clEnqueueWriteBuffer(GetCommandQueue(), d_mode, CL_FALSE, 0, d_mode_sz, &tmp_fnum, 0, NULL, NULL);

     RunHassPreKernel();
#ifdef SET_HASS_WDR
     RunHassWdrKernel();
#endif
     if(AEreq){
          RunHassHistKernel();
          AEreq = 0;
     }

    if((tmp_fnum == 0) || (tmp_fnum == 3)) {
        if(displayoff == 0) RunHassRawToArgbKernel();

        RunHassRawEncKernel();
        clEnqueueReadBuffer(GetCommandQueue(), d_wdr, CL_TRUE,  0, d_wdr_sz, rawOut, 0, NULL, NULL);

        if(displayoff == 0) 
        clEnqueueReadBuffer(GetCommandQueue(), d_disp, CL_TRUE,  0, d_disp_sz, dispOut, 0, NULL, NULL);
    }
}

void HassProc::RunHassKernelsTest(unsigned short* rawImage, float* rgbOut, int loop){

     clEnqueueWriteBuffer(GetCommandQueue(), d_pre, CL_TRUE, 0, d_pre_sz, rawImage,     0, NULL, NULL);

     for(int i=0;i<loop;i++)
     {
          RunHassGuideKernel();
          RunHassCombineKernel();

          RunHassShrinkKernel();
          RunHassNoiseReductionKernel();
          RunHassReconstructionKernel();
          RunHassAppKernel();
          RunHassZoomKernel();
     }

     clEnqueueReadBuffer(GetCommandQueue(), d_zoom, CL_TRUE, 0, d_zoom_sz, rgbOut, 0, NULL, NULL);
}


void HassProc::RunHassKernelsGoThrough( unsigned short* rawImage, float* rgbOut, unsigned char *out, int loop){

     clEnqueueWriteBuffer(GetCommandQueue(), d_pre, CL_TRUE, 0, d_pre_sz, rawImage,     0, NULL, NULL);

     for(int i=0;i<loop;i++)
     {
          RunHassGuideKernel();
          RunHassCombineKernel();

          RunHassShrinkKernel();
          RunHassNoiseReductionKernel();
          RunHassReconstructionKernel();
          RunHassAppKernel();
          RunHassZoomKernel();
          RunHassPostProcessingKernel();
     }

     clEnqueueReadBuffer(GetCommandQueue(), d_zoom, CL_TRUE, 0, d_zoom_sz, rgbOut, 0, NULL, NULL);
     clEnqueueReadBuffer(GetCommandQueue(), d_post, CL_TRUE, 0, d_post_sz, out,    0, NULL, NULL);

}



void HassProc::RunHassKernels(void){


//    clEnqueueNDRangeKernel(GetCommandQueue(), NULL, 1, NULL, NULL, NULL, 0, NULL, NULL);

     RunHassPreKernel();
#ifdef SET_HASS_WDR
     RunHassWdrKernel();
#endif
     if(AEreq){
          RunHassHistKernel();
          AEreq = 0;
     }

#ifdef SET_HASS_SAT_MAP
     RunHassSatMapKernel();
#endif

     RunHassGuideKernel();
     RunHassCombineKernel();
     RunHassShrinkKernel();
     RunHassNoiseReductionKernel();
     RunHassReconstructionKernel();
     RunHassAppKernel();
     RunHassZoomKernel();

     RunHassPostProcessingKernel();
}

void HassProc::ReleaseHassPreKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_PRE_PROCESSING];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassShrinkKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_SHRINK];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassNoiseReductionKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_NOISE_REDUCTION];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassReconstructionKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_RECONSTRUCTION];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassAppKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_APP];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassZoomKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_ZOOM];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassPostProcessingKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_PROCESSING];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassPostNdviKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_NDVI];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassPostMultKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_MULT];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassGuideKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_GUIDE];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassCombineKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_COMBINE];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassWdrKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_WDR];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassRawToArgbKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_RAWTOARGB];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassRawEncKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_RAWENC];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassHistKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_HIST];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassSatMapKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_SATMAP];
     ReleaseKernel(ctrl);
}

void HassProc::ReleaseHassKernels(void){
     ReleaseCoefficients();
     ReleaseHassPreKernel();
    /*
     ReleaseHassShrinkKernel();
     ReleaseHassNoiseReductionKernel();
     ReleaseHassReconstructionKernel();
     ReleaseHassAppKernel();
     ReleaseHassZoomKernel();
     ReleaseHassGuideKernel();
     ReleaseHassCombineKernel();
     ReleaseHassPostProcessingKernel();
     ReleaseHassPostNdviKernel();
     ReleaseHassPostMultKernel();
    */
     ReleaseHassWdrKernel();
     ReleaseHassRawToArgbKernel();
     ReleaseHassHistKernel();
     //ReleaseHassSatMapKernel();

     ReleaseHassBuffers();
}

bool HassProc::UpdateHassPostProcessingKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_PROCESSING];
     hass_err = CL_SUCCESS;

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

//     switch(hass_mode){
//          case HASS_MODE_NDVI: // NDVI
//               hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_color_tbl)));
//             if (hass_err < 0) return hass_err;
//             break;
//          default: // RGB
//               hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
//             if (hass_err < 0) return hass_err;
//             break;
//     }

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return true;
}

bool HassProc::UpdateHassPostNdviKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_NDVI];
     hass_err = CL_SUCCESS;

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_color_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return true;
}

bool HassProc::UpdateHassPostMultKernel(void) {
     kernel_control* ctrl = &_kernel_controls[HASS_POST_MULT];
     hass_err = CL_SUCCESS;

     ////////////////////////////////////////////////////////
     // Setup Arguments
     hass_err=clSetKernelArg(ctrl->kernel,0,sizeof(cl_mem),((void *)(&d_zoom)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,1,sizeof(cl_mem),((void *)(&d_post)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,2,sizeof(cl_mem),((void *)(&d_gamma_tbl)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,3,sizeof(hass_width),((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;

     hass_err=clSetKernelArg(ctrl->kernel,4,sizeof(hass_height),((void *)(&hass_height)));
     if (hass_err < 0) return hass_err;

     return true;
}

bool HassProc::UpdateHassRawToArgbKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_RAWTOARGB];
     hass_err = CL_SUCCESS;

    ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_wdr)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_disp)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(hass_width), ((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 3,sizeof(display_lsb), ((void *)(&display_lsb)));
     if (hass_err < 0) return hass_err;

    return true;
}
bool HassProc::UpdateHassWdrKernel(void){
     kernel_control* ctrl = &_kernel_controls[HASS_WDR];
     hass_err = CL_SUCCESS;
     
     ////////////////////////////////////////////////////////
     // Setup Arguments

     hass_err=clSetKernelArg(ctrl->kernel, 0,sizeof(cl_mem)     ,((void *)(&d_pre)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 1,sizeof(cl_mem)     ,((void *)(&d_short)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 2,sizeof(cl_mem)     ,((void *)(&d_long)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 3,sizeof(cl_mem)     ,((void *)(&d_wdr)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 4,sizeof(blackLevel) ,((void *)(&blackLevel)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 5,sizeof(shs1Offset) ,((void *)(&shs1Offset)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 6,sizeof(shs2Offset) ,((void *)(&shs2Offset)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 7,sizeof(Gain)       ,((void *)(&Gain)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 8,sizeof(BLST)       ,((void *)(&BLST)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel, 9,sizeof(BLED)       ,((void *)(&BLED)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,10,sizeof(hass_width), ((void *)(&hass_width)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,11,sizeof(hass_iwidth),((void *)(&hass_iwidth)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,12,sizeof(cl_mem)     ,((void *)(&d_ave)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,13,sizeof(average_gain) ,((void *)(&average_gain)));
     if (hass_err < 0) return hass_err;
     hass_err=clSetKernelArg(ctrl->kernel,14,sizeof(cl_mem)     ,((void *)(&d_mode)));
     if (hass_err < 0) return hass_err;

     return true;
}
int HassProc::GetWidth(void){
     return hass_width;
}

int HassProc::GetHeight(void){
     return hass_height;
}

int HassProc::GetHistNum(void){
     return nbin;
}


int HassProc::GetSHS1(void){
     return SHS1;
}

int HassProc::GetSHS2(void){
     return SHS2;
}

bool HassProc::compResult(float* ref, float* out, int width, int height, int ch, int edge, float aer)
{
     bool flag = true;
     int  errnum = 0;

     for(int j=edge;j<(height-edge);j++){
          for(int i=edge;i<(width-edge);i++){
               for(int c=0;c<ch;c++){
                    int index = c+(i+j*width)*ch;
                    float cmp0  = ref[index];
                    float cmp1  = out[index];


                    if( fabs(cmp0-cmp1) > aer )
                    {
                         errnum++;
                         LOGI("Error! at (%d,%d)[%d] Ref:%f  Out:%f\n", i, j, c, ref[index], out[index]);
                         flag=false;

                         if(errnum > 100){
                              LOGI("more than 100 errors... stop for now.\n");
                              return false;
                         }
                    }
               }
          }
     }

     return flag;
}

void HassProc::TestHassGuideKernel(void)
{
     unsigned short*  planeHassRaw = new unsigned short [hass_width * hass_height];
     float*           planeGuide   = new float [hass_width * hass_height];
     float*           oclGuide     = new float [hass_width * hass_height];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_pre,   CL_TRUE,  0, d_pre_sz,  planeHassRaw,     0, NULL, NULL);
     RunHassGuideKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_guide, CL_TRUE,  0, d_guide_sz,    oclGuide,     0, NULL, NULL);

     // Reference C Code

     int period = hass_rep_unit;
     int tapnum = hass_rep_unit+1;
     int colors = hass_channel;

     for(int y = 0; y < hass_height; ++y) {
          for(int x = 0; x < hass_width; ++x) {

               float  retval   = 0;
               float  coef_sum = 0;

               for(int yy = 0; yy < tapnum; yy++) {
                    for(int xx = 0; xx < tapnum; xx++) {

                         int cx  = (x + xx - tapnum/2);
                         int cy  = (y + yy - tapnum/2);
                         int pcx = (cx + period)%period;
                         int pcy = (cy + period)%period;

                         float sel_coef = hass_guide_coef[((y%period)*period + x%period)*colors + hass_filter_no[pcy][pcx]];

                         if(cx < 0) cx = 0;
                         if(cy < 0) cy = 0;
                         if(cx > hass_width -1) cx = hass_width -1;
                         if(cy > hass_height-1) cy = hass_height -1;

                         retval += planeHassRaw[cy * hass_width + cx] * sel_coef;
                         coef_sum += sel_coef;
                    }
               }

               planeGuide[y * hass_width + x] = retval/coef_sum;
          }
     }

     // Compare

     if(compResult(planeGuide, oclGuide, hass_width, hass_height, 1, 0, 0.1f))
          LOGI("OK.\n");

     delete[] planeHassRaw;
     delete[] planeGuide;
     delete[] oclGuide;
}


void HassProc::TestHassCombineKernel(void)
{
     float*           planeGuide   = new float [hass_width * hass_height];
     float*           planeCombine = new float [hass_width * hass_height];
     float*           oclCombine   = new float [hass_width * hass_height];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_guide,   CL_TRUE,  0, d_guide_sz,   planeGuide,     0, NULL, NULL);
     RunHassCombineKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_combine, CL_TRUE,  0, d_combine_sz, oclCombine,     0, NULL, NULL);

     // Reference C Code

     float param_combine_noise = hass_combine_noise;

     int combine_tap_center = hass_rep_unit/2;
     int combine_tap_side   = hass_rep_unit/4;
     int combine_tap = combine_tap_center + combine_tap_side;

     for(int y = 0; y < hass_height; ++y)
     {
          for(int x = 0; x < hass_width; ++x)
          {
               // simple combine with guide(temtative)
               float combine_sum = 0;
               float combine_coef_sum = 0;

               for(int yy = -(combine_tap-1); yy <= (combine_tap-1); yy++) {
                    for(int xx = -(combine_tap-1); xx <= (combine_tap-1); xx++) {

                         int cx  = x + xx;
                         int cy  = y + yy;

                         if(cx < 0) cx = 0;
                         if(cy < 0) cy = 0;
                         if(cx > hass_width -1) cx = hass_width -1;
                         if(cy > hass_height-1) cy = hass_height -1;

                         float tmpcoef = hass_combine_coef[abs(yy)] * hass_combine_coef[abs(xx)];

                         combine_sum += planeGuide[cy * hass_width + cx] * tmpcoef;
                         combine_coef_sum += tmpcoef;
                    }
               }

               float hpf = fmax(planeGuide[y * hass_width + x], param_combine_noise) / fmax((combine_sum/combine_coef_sum), param_combine_noise);

               planeCombine[y * hass_width + x] = hpf;
          }
     }

     // Compare

     if(compResult(planeCombine, oclCombine, hass_width, hass_height, 1, 0, 0.1))
          LOGI("OK.\n");

     delete[] planeGuide;
     delete[] planeCombine;
     delete[] oclCombine;
}



void HassProc::TestHassShrinkKernel(void)
{
     unsigned short*  planeHassRaw = new unsigned short [hass_width * hass_height];
     float*           planeShrink   = new float [hass_widthShrink * hass_heightShrink * hass_channel];
     float*           oclShrink     = new float [hass_widthShrink * hass_heightShrink * hass_channel];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_pre,    CL_TRUE,  0, d_pre_sz,  planeHassRaw,     0, NULL, NULL);
     RunHassShrinkKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_shrink, CL_TRUE,  0, d_shrink_sz,  oclShrink,     0, NULL, NULL);

     // Reference C Code

     for(int y = 0; y < hass_heightShrink; ++y)
     {
          for(int x = 0; x < hass_widthShrink; ++x)
          {
               float*coef_sum = new float[hass_channel];
               float*retval   = new float[hass_channel];

               for(int c = 0; c < hass_channel; ++c) {
                    coef_sum[c] = 0.0;
                    retval[c]   = 0.0;
               }

               for(int dyOrg = -(hass_rep_unit-1); dyOrg <= (hass_rep_unit-1); ++dyOrg) //
               {
                    int yOrg = y * hass_shrink_size + dyOrg;
                    if(yOrg < 0 || yOrg >= hass_height ) continue;

                    int yPhaseOrg = yOrg % hass_shrink_size;

                    for(int dxOrg = -1*(hass_rep_unit-1); dxOrg <= (hass_rep_unit-1); ++dxOrg)
                    {
                         int xOrg = x * hass_shrink_size + dxOrg;
                         if(xOrg < 0 || xOrg >= hass_width ) continue;

                         int xPhaseOrg = xOrg % hass_shrink_size;

                         int c = hass_filter_no[yPhaseOrg][xPhaseOrg];

                         float w = hass_shrink_coef[abs(dyOrg)] * hass_shrink_coef[abs(dxOrg)];

                         retval[c]   += w * planeHassRaw[yOrg * hass_width + xOrg];
                         coef_sum[c] += w;
                    }
               }

               for(int c = 0; c < hass_channel; ++c) {
                    planeShrink[c + (y * hass_widthShrink + x) * hass_channel] = retval[c] / coef_sum[c];
               }
               delete[] coef_sum;
               delete[] retval;
          }
     }


     // Compare

     if(compResult(planeShrink, oclShrink, hass_widthShrink, hass_heightShrink, hass_channel, 0, 1.0f))
          LOGI("OK.\n");

     delete[] planeHassRaw;
     delete[] planeShrink;
     delete[] oclShrink;
}


void HassProc::TestHassNoiseReductionKernel(void)
{
     float*           planeShrink   = new float [hass_widthShrink * hass_heightShrink * hass_channel];
     float*           planeNr       = new float [hass_widthShrink * hass_heightShrink * hass_channel];
     float*           oclNr         = new float [hass_widthShrink * hass_heightShrink * hass_channel];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_shrink, CL_TRUE,  0, d_shrink_sz,  planeShrink,   0, NULL, NULL);
     RunHassNoiseReductionKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_nr,     CL_TRUE,  0, d_nr_sz,      oclNr,         0, NULL, NULL);

     // Reference C Code

     int halfTapSize = (int)(hass_nr_tap / 2);

     for(int c = 0; c < hass_channel; ++c)
     {
          for(int y = 0; y < hass_heightShrink; ++y)
          {
               for(int x = 0; x < hass_widthShrink; ++x)
               {
                    float  valNr = 0.;
                    int    coeffSum = 0;

                    for(int dy = -halfTapSize; dy <= halfTapSize; ++dy)
                    {
                         if(y + dy < 0 || y + dy >= hass_heightShrink) continue;

                         for(int dx = -halfTapSize; dx <= halfTapSize; ++dx)
                         {
                              if(x + dx < 0 || x + dx >= hass_widthShrink) continue;

                              valNr += planeShrink[c +((y+dy) * hass_widthShrink + (x+dx))*hass_channel];
                              coeffSum++;
                         }
                    }

                    if(coeffSum > 0) planeNr[c + (y * hass_widthShrink + x)*hass_channel] = valNr / coeffSum;
                    else             planeNr[c + (y * hass_widthShrink + x)*hass_channel] = 0.;
               }
          }
     }

     // Compare

     if(compResult(planeNr, oclNr, hass_widthShrink, hass_heightShrink, hass_channel, 0, 1.0f))
          LOGI("OK.\n");

     delete[] planeShrink;
     delete[] planeNr;
     delete[] oclNr;
}



void HassProc::TestHassReconstructionKernel(void)
{
     float*           planeNr       = new float [hass_widthShrink * hass_heightShrink * hass_channel];
     float*           planeRecon    = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];
     float*           oclRecon      = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_nr,     CL_TRUE,  0, d_nr_sz,      planeNr,       0, NULL, NULL);
     RunHassReconstructionKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_recon,  CL_TRUE,  0, d_recon_sz,   oclRecon,      0, NULL, NULL);

     // Reference C Code

     for(int y = 0; y < hass_heightShrink; ++y)
     {
          for(int x = 0; x < hass_widthShrink; ++x)
          {
               for(int s = 0; s < hass_spectrum; ++s)
               {
                    float val = 0.;

                    for(int c = 0; c < hass_channel; ++c)
                    {
                         val += hass_recon_coef[c+hass_channel*s] * planeNr[c + (y * hass_widthShrink + x)*hass_channel];
                    }

                    planeRecon[s + (y * hass_widthShrink + x)*hass_spectrum] = val;
               }
          }
     }


     // Compare

     if(compResult(planeRecon, oclRecon, hass_widthShrink, hass_heightShrink, hass_spectrum, 0, 1.0f))
          LOGI("OK.\n");

     delete[] planeNr;
     delete[] planeRecon;
     delete[] oclRecon;
}


void HassProc::TestHassAppKernel(void)
{
     float*           planeRecon    = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];
     float*           planeApp      = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];
     float*           oclApp        = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_recon, CL_TRUE,  0, d_recon_sz,   planeRecon,    0, NULL, NULL);
     RunHassAppKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_app,   CL_TRUE,  0, d_app_sz,     oclApp,        0, NULL, NULL);

     // Reference C Code

     for(int i = 0; i < hass_spectrum; ++i)
     {
          if(i >= hass_spectrum) break;
          for(int y = 0; y < hass_heightShrink; ++y)
          {
               for(int x = 0; x < hass_widthShrink; ++x)
               {
                    planeApp[i + (y * hass_widthShrink + x)*hass_spectrum] = planeRecon[i + (y * hass_widthShrink + x)*hass_spectrum];
               }
          }
     }

     // Compare

     if(compResult(planeApp, oclApp, hass_widthShrink, hass_heightShrink, hass_spectrum, 0, 1.0f))
          LOGI("OK.\n");

     delete[] planeRecon;
     delete[] planeApp;
     delete[] oclApp;
}

void HassProc::TestHassZoomKernel(void)
{
     float*           planeApp      = new float [hass_widthShrink * hass_heightShrink * hass_spectrum];
     float*           planeCombine  = new float [hass_width * hass_height];
     float*           planesProc     = new float [hass_width * hass_height * hass_spectrum];
     float*           oclZoom       = new float [hass_width * hass_height * hass_spectrum];

     // OpenCL Code
     clEnqueueReadBuffer(GetCommandQueue(), d_app,     CL_TRUE,  0, d_app_sz,     planeApp,        0, NULL, NULL);
     clEnqueueReadBuffer(GetCommandQueue(), d_combine, CL_TRUE,  0, d_combine_sz, planeCombine,    0, NULL, NULL);
     RunHassZoomKernel();
     clEnqueueReadBuffer(GetCommandQueue(), d_zoom,    CL_TRUE,  0, d_zoom_sz,    oclZoom,         0, NULL, NULL);

     // Reference C Code

     int zoom_phase = hass_shrink_size;

     for(int i = 0; i < hass_spectrum; ++i)
     {
          for(int y = 0; y < hass_height; ++y)
          {
               float coefy[2];
               coefy[0] = hass_shrink_coef[y%zoom_phase];
               coefy[1] = 1.0f - coefy[0];

               for(int x = 0; x < hass_width; ++x)
               {
                    float coefx[2];
                    coefx[0] = hass_shrink_coef[x%zoom_phase];
                    coefx[1] = 1.0f - coefx[0];

                    float retval = 0;

                    for(int yy = 0; yy < 2; ++yy)
                    {
                         int yShr = (int)(y / zoom_phase) + yy;

                         if(yShr > hass_heightShrink-1) yShr = hass_heightShrink-1;

                         for(int xx = 0; xx < 2; ++xx)
                         {
                              int xShr = (int)(x / zoom_phase) + xx;

                              if(xShr > hass_widthShrink-1) xShr = hass_widthShrink-1;

                              float tmpcoef = coefx[xx] * coefy[yy];

                              retval  += planeApp[i + (yShr * hass_widthShrink + xShr) * hass_spectrum] * tmpcoef;
                         }
                    }

                    planesProc[i+(y * hass_width + x) * hass_spectrum ] = retval * planeCombine[y * hass_width + x];
               }
          }
     }


     // Compare

     if(compResult(planesProc, oclZoom, hass_width, hass_height, hass_spectrum, 0, 1.0f))
          LOGI("OK.\n");

     delete[] planeApp;
     delete[] planesProc;
     delete[] oclZoom;
}


size_t HassProc::GetHassIntermediateData(int sel, float *data)
{
     size_t size;

     switch(sel){
          case 0:
               clEnqueueReadBuffer(GetCommandQueue(), d_guide,   CL_TRUE,  0, d_guide_sz,   data, 0, NULL, NULL);
             size = d_guide_sz;
             break;
          case 1:
               clEnqueueReadBuffer(GetCommandQueue(), d_combine, CL_TRUE,  0, d_combine_sz, data, 0, NULL, NULL);
             size = d_combine_sz;
             break;
          case 2:
               clEnqueueReadBuffer(GetCommandQueue(), d_shrink,  CL_TRUE,  0, d_shrink_sz,  data, 0, NULL, NULL);
             size = d_shrink_sz;
             break;
          case 3:
               clEnqueueReadBuffer(GetCommandQueue(), d_nr,      CL_TRUE,  0, d_nr_sz,      data, 0, NULL, NULL);
             size = d_nr_sz;
             break;
          case 4:
               clEnqueueReadBuffer(GetCommandQueue(), d_recon,   CL_TRUE,  0, d_recon_sz,   data, 0, NULL, NULL);
             size = d_recon_sz;
             break;
          case 5:
               clEnqueueReadBuffer(GetCommandQueue(), d_app,     CL_TRUE,  0, d_app_sz,     data, 0, NULL, NULL);
             size = d_app_sz;
             break;
          default:
             clEnqueueReadBuffer(GetCommandQueue(), d_zoom,    CL_TRUE,  0, d_zoom_sz,    data, 0, NULL, NULL);
             size = d_zoom_sz;
             break;
     }

     return size;
}

size_t HassProc::GetHassRawData(void *data)
{
#ifdef SET_HASS_WDR
     size_t size = d_wdr_sz;
     clEnqueueReadBuffer(GetCommandQueue(), d_wdr,   CL_TRUE,  0, size,   data, 0, NULL, NULL);
#else
     size_t size = d_pre_sz;     
     clEnqueueReadBuffer(GetCommandQueue(), d_pre,   CL_TRUE,  0, size,   data, 0, NULL, NULL);     
#endif

     return size;
}

