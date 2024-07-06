#include "dsp_process.h"
#include "dsp_config.h"
#include "driver/gpio.h"

static  bool            dsp_filters_enabled   = true;
static  bool            dsp_output_enabled    = true; 
static  bool            dsp_ok_flag           = true;       

i2s_chan_handle_t tx_handle;
i2s_chan_handle_t rx_handle;
#define AUDIO_BUFF_SIZE       (DSP_MAX_SAMPLES * sizeof(sample_t))


//------------------------------------------------------------------------------------ 
// Flash on board LED
//------------------------------------------------------------------------------------
static void esp_led_flash( bool flash, unsigned long duration ) {

  static  unsigned long  esp_led_flash_start  = 0;

  if( flash && (esp_led_flash_start == 0) ) {
    esp_led_flash_start = esp_timer_get_time()/1000;
    gpio_set_level(LED_OUTPUT, 1);
  } else if( esp_timer_get_time()/1000 - esp_led_flash_start > duration ) {
    esp_led_flash_start = 0;
    gpio_set_level(LED_OUTPUT, 0);
  }
}


//------------------------------------------------------------------------------------ 
// User input command processing
//------------------------------------------------------------------------------------
void dsp_command( char command ) {

  if( !dsp_ok_flag ) {
    SERIAL.printf( "E-DSP: DSP initialization error. Reload.\r\n" );
    return;
  }

  switch( command ) {
    case 'i' :
      dsp_filter_info( DSP_Channels );
      break;

    case 'e' :
      dsp_filters_enabled = true;
      SERIAL.printf("I-DSP: DSP processing ENABLED\r\n");
      break;

    case 'd' :
      dsp_filters_enabled = false;
      SERIAL.printf("I-DSP: DSP processing DISABLED\r\n");
      break;

    case 's' :
      dsp_output_enabled = false;
      SERIAL.printf("I-DSP: DSP is now STOPPED\r\n");
      break;

    case 'r' :
      dsp_output_enabled = true;
      SERIAL.printf("I-DSP: DSP is now RUNNING\r\n");
      break;

    case 'p' :
      dsp_plot( DSP_Channels );
      break;

    case 'u' :
      static filter_def_t FREQ_Filters[] = {
             {0, DSP_FILTER_PEAK_EQ, 60, 2.0, 3.0},
             {0, DSP_FILTER_PEAK_EQ, 80, 5.0, 2.0},
             {0, DSP_FILTER_PEAK_EQ, 100, 5.0, 2.0},
             {0, DSP_FILTER_PEAK_EQ, 120, 5.0, 2.0},
             {0, DSP_FILTER_PEAK_EQ, 140, 5.0, 2.0},
             {1, DSP_FILTER_PEAK_EQ, 60, 2.0, 3.0},
             {1, DSP_FILTER_PEAK_EQ, 80, 5.0, 2.0},
             {1, DSP_FILTER_PEAK_EQ, 100, 5.0, 2.0},
             {1, DSP_FILTER_PEAK_EQ, 120, 5.0, 2.0},
             {1, DSP_FILTER_PEAK_EQ, 140, 5.0, 2.0}             
             };
             
      dsp_update_filters( FREQ_Filters, 10 );
      SERIAL.printf("I-DSP: DSP filters updated\r\n");       
      break;
  }
}


//------------------------------------------------------------------------------------ 
// DSP initialization
//------------------------------------------------------------------------------------
esp_err_t dsp_init( TaskHandle_t* taskDSP ) {
  
  esp_err_t res = ESP_OK;
  
  /* Allocate a pair of I2S channel */
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  /* Allocate for TX and RX channel at the same time, then they will work in full-duplex mode */
  SERIAL.printf("Init I2S Channel: %x\n", i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
  /* Set the configurations for BOTH TWO channels, since TX and RX channel have to be same in full-duplex mode */
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(DSP_SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {
          .mclk = I2S_STD_MCLK_IO1,
          .bclk = I2S_STD_BCLK_IO1,
          .ws = I2S_STD_WS_IO1,
          .dout = I2S_STD_DOUT_IO1,
          .din = I2S_STD_DIN_IO1,
          .invert_flags = {
              .mclk_inv = false,
              .bclk_inv = false,
              .ws_inv = false,
          },
      },
  };
  std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  SERIAL.printf("Init I2S TX: %x\n", i2s_channel_init_std_mode(tx_handle, &std_cfg));
  SERIAL.printf("Init I2S RX: %x\n", i2s_channel_init_std_mode(rx_handle, &std_cfg));

  SERIAL.printf("Enable I2S TX: %x\n", i2s_channel_enable(tx_handle));
  SERIAL.printf("Enable I2S RX: %x\n", i2s_channel_enable(rx_handle));

  // set clipping LED to output
  gpio_set_direction(LED_OUTPUT, GPIO_MODE_OUTPUT);

  /*******************/

  // Set up DSP to run on core 0
  xTaskCreatePinnedToCore(
                    dsp_task,           /* Function to implement the task */
                    "DSP Loop",         /* Name of the task */
                    10000,              /* Stack size in words */
                    NULL,               /* Task input parameter */
                    tskIDLE_PRIORITY,   /* Priority of the task */
                    taskDSP,            /* Task handle. */
                    CORE_DSP );         /* Core running the DSP process */    

  return( ESP_OK );
}


//------------------------------------------------------------------------------------ 
// DSP processing initialization
//------------------------------------------------------------------------------------
static esp_err_t dsp_processing_init( dsp_channel_t* channels, biquad_def_t* biquad_defs, int biquad_def_count, filter_def_t* filter_defs, int filter_def_count) {
  
  esp_err_t res = ESP_OK;

  SERIAL.printf("I-DSP: Setting up channels...\r\n");

  res = dsp_filter_init( channels, biquad_defs, biquad_def_count, filter_defs, filter_def_count );
  if( res != ESP_OK ) {
    dsp_ok_flag = false;
    return( res );
  }

  dsp_filter_info( DSP_Channels );

  return( res );
}


//------------------------------------------------------------------------------------ 
// DSP task 
//------------------------------------------------------------------------------------
void dsp_task( void * pvParameters ) {
  
  size_t        i2s_bytes_read;
  size_t        i2s_bytes_written;
  bool          clip_flag;
  esp_err_t     res;  

  // Setup the DSP channels
  res = dsp_processing_init( DSP_Channels, BIQUAD_Filters, sizeof( BIQUAD_Filters )/sizeof( biquad_def_t ), FREQ_Filters, sizeof( FREQ_Filters )/sizeof( filter_def_t ) );
  
  if( res == ESP_OK ) {
    sample_t* w_buf = (sample_t*)calloc(sizeof(sample_t), DSP_MAX_SAMPLES);
    assert(w_buf); // Check if w_buf allocation success
    size_t w_bytes = AUDIO_BUFF_SIZE;

    /* Enable the TX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    sample_t* r_buf = (sample_t*)calloc(sizeof(sample_t), DSP_MAX_SAMPLES);
    assert(r_buf); // Check if r_buf allocation success
    size_t r_bytes = 0;

    /* Enable the RX channel */
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    SERIAL.println("Ready for write.");
    
    while(true) {
      if(dsp_output_enabled ) {   
        // Read buffer
        auto err_r = i2s_channel_read(rx_handle, r_buf, AUDIO_BUFF_SIZE, &r_bytes, 1000);
        if (err_r != ESP_OK) {
          SERIAL.printf("DSP Task: i2s read failed with code: %d\n", err_r);
        }

        // Apply filters to buffer
        clip_flag = false;
        dsp_filter(DSP_Channels, r_buf, w_buf, r_bytes, dsp_filters_enabled, &clip_flag);

        /* Write i2s data */
        auto err_w = i2s_channel_write(tx_handle, w_buf, r_bytes, &w_bytes, 1000);
        if (err_w != ESP_OK) {
          SERIAL.printf("DSP Task: i2s write failed with code: %d\n", err_w);
        }
      }
      
      // Check clipping LED
      esp_led_flash(clip_flag, 100);
    }
    
    free(r_buf);
    free(w_buf);
  } else {     
    while(true) {
      vTaskDelay(TASK_DELAY);
    }
  }
}
