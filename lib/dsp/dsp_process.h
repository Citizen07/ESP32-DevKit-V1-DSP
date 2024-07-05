#pragma once
#define DAC_24_BIT
#define WIFI_ON

#include <string.h>
#include <math.h>
#include "driver/i2s_std.h"

#ifdef ESP8266
  #include <ESP8266WiFi.h>
  #include <ESP8266mDNS.h>
  #include <WiFiUdp.h>
#else // ESP32
  #include <WiFi.h>
  #include <ESPmDNS.h>
#endif
#include <ArduinoOTA.h>
#include "TelnetSpy.h"


//------------------------------------------------------------------------------------
// Constant definitions
//------------------------------------------------------------------------------------

#define I2S_STD_MCLK_IO1        GPIO_NUM_0        // I2S master clock io number 0
#define I2S_STD_BCLK_IO1        GPIO_NUM_2        // I2S bit clock io number 2
#define I2S_STD_WS_IO1          GPIO_NUM_15       // I2S word select io number 15
#define I2S_STD_DOUT_IO1        GPIO_NUM_4        // I2S data out io number 4
#define I2S_STD_DIN_IO1         GPIO_NUM_16       // I2S data in io number 16

#define CORE_MAIN               1                 // Core running main loop 
#define CORE_DSP                0                 // Core running DSP loop

#define TASK_DELAY              10

#define PRC_FLT                 0                 // Set filter precision to float
#define PRC_DBL                 1                 // Set filter precision to double        

#define DSP_ALL_CHANNELS        -1                // Specify all channels processed
#define DSP_NUM_CHANNELS        2                 // Number of channels
#define DSP_MAX_FILTERS         20                // Max number of biquad filters
#define DSP_SAMPLE_RATE         48000             // The sample rate
#define DSP_MAX_GAIN            24                // Maximum gain for the channel
#define DSP_MAX_SAMPLES         96                // Maximum number of samples per channel each loop
#define DSP_MIN_DELAY_MILLIS    ((DSP_MAX_SAMPLES*1000)/DSP_SAMPLE_RATE+1)
#define DSP_MAX_DELAY_MILLIS    250               // Maximum delay allowed in milliseconds
#define DSP_MAX_DELAY_SAMPLES   ((DSP_MAX_DELAY_MILLIS*DSP_SAMPLE_RATE)/1000+1)
#define DSP_ADC_ATTENUATE       0                 // Attenuation of input by 0.5 dBs

#define DSP_FILTER_LOW_PASS     0
#define DSP_FILTER_HIGH_PASS    1
#define DSP_FILTER_BAND_PASS    2
#define DSP_FILTER_NOTCH        3
#define DSP_FILTER_APF          4
#define DSP_FILTER_PEAK_EQ      5
#define DSP_FILTER_LOW_SHELF    6
#define DSP_FILTER_HIGH_SHELF   7

#ifdef DAC_24_BIT
typedef int32_t    sample_t;
#define SAMPLE_BITS             24
#else
typedef int16_t    sample_t;
#define SAMPLE_BITS             16
#endif

#define SAMPLE_NULL_BITS        (sizeof(sample_t)*8 - SAMPLE_BITS)

#define DSP_BITS_PER_SAMPLE     ((i2s_bits_per_sample_t) (sizeof(sample_t)*8))
#define DSP_DAC_WORD_LENGTH     (sizeof(sample_t)/2+2) // 16-bit = 011, 32-bit = 100
#define DSP_MAX_LEVEL           ((1 << (SAMPLE_BITS - 1)) - 1)

#define DITHER_ON               0
#define DITHER_RANGE_DB         96
#define DITHER_BITS             (SAMPLE_BITS - DITHER_RANGE_DB/6)

//------------------------------------------------------------------------------------
// Type definitions
//------------------------------------------------------------------------------------

typedef struct {
  int           channel;                          // Filter channel
  int           filter_type;                      // Type of filter to apply
  float         frequency;                        // Centre frequency of filter
  double        Q;                                // Q value
  float         gain;                             // Gain value in dB
#if DOUBLE_PRECISION
  int           precision;                        // Implement as float or double
#endif
} filter_def_t;

typedef struct {
  int           channel;                          // Associated channel
  double        coeffs[5];                        // Biquad coefficients
#ifdef DOUBLE_PRECISION  
  int           precision;                        // Implement as float or double
#endif
} biquad_def_t;

typedef struct {
  double        coeffs_d[5];                      // The biquad coefficients for each of the filters (double precision)
  float         coeffs_f[5];                      // The biquad coefficients for each of the filters (float)
  float         w[2];                             // Array of historic W values for each biquad filter
  int           precision;                        // Precision calculation
  filter_def_t* filter_def;                       // Associated frequency defined filter
} dsp_filter_t;

typedef struct {
  float         scaling_factor;                   // Factor used to scale values for specified gain
  int           delay_samples;                    // Number of calculated samples delayed in buffer
  int           delay_offset;                     // Offset within the delay buffer for storing next set of input values
  int           in_clip_count;                    // Number of times input audio clipped per channel
  int           out_clip_count;                   // Number of times output audio clipped per channel
  long int      in_max_level;                     // Max input level per last sample
  long int      out_max_level;                    // Max output level per last sample
  sample_t      delay_buff[DSP_MAX_DELAY_SAMPLES];// Sample delay buffer
  dsp_filter_t  filter[DSP_MAX_FILTERS];          // Filters for channel
  int           num_filters;                      // Total number of filters in the channel
} dsp_data_t;

typedef struct {
  const char*   name;                             // Name of the channel
  int           inputs[DSP_NUM_CHANNELS];         // Input channels from the source
  float         gain_dB;                          // The amount of gain added to the channel
  int           delay_millis;                     // The delay (in millseconds) introduced into the channel
  dsp_data_t*   data;                             // Data buffer for the channel
} dsp_channel_t;


//------------------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------------------

extern char           strIPAddress[16];
extern dsp_channel_t  DSP_Channels[DSP_NUM_CHANNELS];
extern TelnetSpy      SerialAndTelnet;

#ifdef WIFI_ON
  #undef SERIAL        
  #define SERIAL      SerialAndTelnet
#else
  #undef SERIAL        
  #define SERIAL      Serial
#endif

//------------------------------------------------------------------------------------
// Global functions (C++)
//------------------------------------------------------------------------------------

esp_err_t         dsp_init( TaskHandle_t* taskDSP );
void              dsp_task( void* pvParameters );
void              dsp_command( char command );
void              dsp_filter_info( dsp_channel_t* channels );
void              dsp_plot( dsp_channel_t* channels );
esp_err_t         dsp_filter_init( dsp_channel_t* channels, biquad_def_t* biquad_defs, int biquad_def_count, filter_def_t* filter_defs, int filter_def_count );
esp_err_t         dsp_update_filters( filter_def_t* filter_defs, int filter_def_count );
esp_err_t         dsp_filter( dsp_channel_t* channels, sample_t* input_buffer, sample_t* output_buffer, int buffer_len, bool filters_enabled, bool* clip_flag );
esp_err_t         dsp_get_biquad( filter_def_t* filter, double* coeffs );
biquad_def_t*     dsp_import_filters( int* import_filter_count );
int32_t           dsp_dither( int32_t sample );


//------------------------------------------------------------------------------------
// Global functions (C)
//------------------------------------------------------------------------------------

extern "C" {
  esp_err_t       dsps_biquad_f32_ae32( const float* input, float* output, int len, float* coef, float* w );
}

extern "C" {
  esp_err_t       dsps_biquad_f32_dbl( const float *input, float *output, int len, double *coef, float* w);
}
