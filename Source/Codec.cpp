#include "device.h"
#include "Owl.h"
#include "Codec.h"
#include "errorhandlers.h"
#include "ApplicationSettings.h"
#include <cstring>
#include "ProgramManager.h"

#include "SerialBuffer.hpp"
SerialBuffer<CODEC_BUFFER_SIZE, int32_t> audio_rx_buffer DMA_RAM;
SerialBuffer<CODEC_BUFFER_SIZE, int32_t> audio_tx_buffer DMA_RAM;

extern "C" {
  uint16_t codec_blocksize = 0;
  int32_t* codec_rxbuf;
  int32_t* codec_txbuf;
}

#ifdef USE_CS4271
#define HSAI_RX hsai_BlockB1
#define HSAI_TX hsai_BlockA1
#define HDMA_RX hdma_sai1_b
#define HDMA_TX hdma_sai1_a
#else
#define HSAI_RX hsai_BlockA1
#define HSAI_TX hsai_BlockB1
#define HDMA_RX hdma_sai1_a
#define HDMA_TX hdma_sai1_b
#endif

#ifdef USE_USBD_AUDIO
#include "usbd_audio.h"

#if AUDIO_BITS_PER_SAMPLE == 32
typedef int32_t audio_t;
#elif AUDIO_BITS_PER_SAMPLE == 16
typedef int16_t audio_t;
#elif AUDIO_BITS_PER_SAMPLE == 8
typedef int8_t audio_t;
#else
#error Invalid AUDIO_BITS_PER_SAMPLE
#endif

static void update_rx_read_index(){
#if defined USE_CS4271 || defined USE_PCM3168A
  extern DMA_HandleTypeDef HDMA_RX;
  // NDTR: the number of remaining data units in the current DMA Stream transfer.
  size_t pos = audio_rx_buffer.getCapacity() - __HAL_DMA_GET_COUNTER(&HDMA_RX);
  audio_rx_buffer.setReadIndex(pos);
#endif
}

static void update_tx_write_index(){
#if defined USE_CS4271 || defined USE_PCM3168A
  extern DMA_HandleTypeDef HDMA_TX;
  // NDTR: the number of remaining data units in the current DMA Stream transfer.
  size_t pos = audio_tx_buffer.getCapacity() - __HAL_DMA_GET_COUNTER(&HDMA_TX);
  audio_tx_buffer.setWriteIndex(pos);
#endif
}

void usbd_audio_tx_start_callback(uint16_t rate, uint8_t channels){
#if defined USE_USBD_AUDIO_TX && USBD_AUDIO_TX_CHANNELS > 0
  // set read head at half a ringbuffer distance from write head
  update_tx_write_index();
  size_t pos = audio_tx_buffer.getWriteIndex();
  size_t len = audio_tx_buffer.getCapacity();
  pos = (pos + len/2) % len;
  pos = (pos/AUDIO_CHANNELS)*AUDIO_CHANNELS; // round down to nearest frame
  audio_tx_buffer.setReadIndex(pos);
#ifdef DEBUG
  printf("start tx %d %d %d\n", rate, channels, pos);
#endif
#endif
}

void usbd_audio_tx_stop_callback(){
#ifdef DEBUG
  printf("stop tx\n");
#endif
}

void usbd_audio_rx_start_callback(uint16_t rate, uint8_t channels){
#if defined USE_USBD_AUDIO_RX && USBD_AUDIO_RX_CHANNELS > 0
  audio_rx_buffer.setAll(0);
  update_rx_read_index();
  size_t pos = audio_rx_buffer.getWriteIndex();
  size_t len = audio_rx_buffer.getCapacity();
  pos = (pos + len/2) % len;
  pos = (pos/AUDIO_CHANNELS)*AUDIO_CHANNELS; // round down to nearest frame
  audio_rx_buffer.setWriteIndex(pos);
  program.exitProgram(true);
  owl.setOperationMode(STREAM_MODE);
#ifdef DEBUG
  printf("start rx %d %d %d\n", rate, channels, pos);
#endif
#endif
}

void usbd_audio_rx_stop_callback(){
#if defined USE_USBD_AUDIO_RX && USBD_AUDIO_RX_CHANNELS > 0
  audio_rx_buffer.setAll(0);
  program.loadProgram(program.getProgramIndex());
  program.startProgram(true);
  owl.setOperationMode(RUN_MODE);
#ifdef DEBUG
  printf("stop rx\n");
#endif
#endif  
}

static int32_t usbd_audio_rx_flow = 0;
size_t usbd_audio_rx_callback(uint8_t* data, size_t len){
#if defined USE_USBD_AUDIO_RX && USBD_AUDIO_RX_CHANNELS > 0
  // copy audio to codec_txbuf aka audio_rx_buffer
  update_rx_read_index();
  audio_t* src = (audio_t*)data;
  size_t blocksize = len / (USBD_AUDIO_RX_CHANNELS*AUDIO_BYTES_PER_SAMPLE);
  size_t available = audio_rx_buffer.getWriteCapacity()/AUDIO_CHANNELS;
  if(available < blocksize){
    usbd_audio_rx_flow += blocksize-available;
    // skip some frames start and end of this block
    // src += (blocksize - available)*USBD_AUDIO_RX_CHANNELS/2;
    blocksize = available;
    len = blocksize*USBD_AUDIO_RX_CHANNELS*AUDIO_BYTES_PER_SAMPLE;
  }
  while(blocksize--){
      int32_t* dst = audio_rx_buffer.getWriteHead();
      size_t ch = USBD_AUDIO_RX_CHANNELS;
      while(ch--)
  	*dst++ = AUDIO_SAMPLE_TO_INT32(*src++);
      // should we leave in place or zero out any remaining channels?
      memset(dst, 0, (AUDIO_CHANNELS-USBD_AUDIO_RX_CHANNELS)*sizeof(int32_t));
      audio_rx_buffer.incrementWriteHead(AUDIO_CHANNELS);
  }
  // available = audio_rx_buffer.getWriteCapacity()*AUDIO_BYTES_PER_SAMPLE*USBD_AUDIO_RX_CHANNELS/AUDIO_CHANNELS;
  // if(available < AUDIO_RX_PACKET_SIZE)
  //   return available;
#endif
  return len;
}

static int32_t usbd_audio_tx_flow = 0;
// expect a 1 in 10k sample underflow (-0.01% sample accuracy)
void usbd_audio_tx_callback(uint8_t* data, size_t len){
#if defined USE_USBD_AUDIO_TX && USBD_AUDIO_TX_CHANNELS > 0
  update_tx_write_index();
  size_t blocksize = len / (USBD_AUDIO_TX_CHANNELS*AUDIO_BYTES_PER_SAMPLE);
  size_t available = audio_tx_buffer.getReadCapacity()/AUDIO_CHANNELS;
  if(available < blocksize){
    usbd_audio_tx_flow += blocksize-available;
    blocksize = available;
    len = blocksize*USBD_AUDIO_TX_CHANNELS*AUDIO_BYTES_PER_SAMPLE;
  }
  audio_t* dst = (audio_t*)data;
  while(blocksize--){
    int32_t* src = audio_tx_buffer.getReadHead();
    size_t ch = USBD_AUDIO_TX_CHANNELS;
    while(ch--)
      *dst++ = AUDIO_INT32_TO_SAMPLE(*src++); // shift, round, dither, clip, truncate, bitswap
    audio_tx_buffer.incrementReadHead(AUDIO_CHANNELS);
  }
  usbd_audio_write(data, len);
#endif
}
#endif // USE_USBD_AUDIO

void usbd_audio_mute_callback(int16_t gain){
  // todo!
}

void usbd_audio_gain_callback(int16_t gain){
  // codec_set_gain_in(gain); todo!
}

uint32_t usbd_audio_get_rx_count(){
  return 0; // todo!
}

uint16_t Codec::getBlockSize(){
  return codec_blocksize;
}

#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif

void Codec::init(){
  audio_tx_buffer.reset();
  codec_rxbuf = audio_tx_buffer.getWriteHead();
  audio_rx_buffer.reset();
  codec_txbuf = audio_rx_buffer.getReadHead();
  codec_init();
}

void Codec::reset(){
  // todo: this is called when blocksize is changed
  stop();
  audio_tx_buffer.reset();
  audio_rx_buffer.reset();
  start();
}

void Codec::ramp(uint32_t max){
  uint32_t incr = max/CODEC_BUFFER_SIZE;
  for(int i=0; i<CODEC_BUFFER_SIZE; ++i)
    codec_txbuf[i] = i*incr;
}

void Codec::clear(){
  set(0);
}

int32_t Codec::getMin(){
  int32_t min = codec_txbuf[0];
  for(int i=1; i<CODEC_BUFFER_SIZE; ++i)
    if(codec_txbuf[i] < min)
      min = codec_txbuf[i];
  return min;
}

int32_t Codec::getMax(){
  int32_t max = codec_txbuf[0];
  for(int i=1; i<CODEC_BUFFER_SIZE; ++i)
    if(codec_txbuf[i] > max)
      max = codec_txbuf[i];
  return max;
}

float Codec::getAvg(){
  float avg = 0;
  for(int i=0; i<CODEC_BUFFER_SIZE; ++i)
    avg += codec_txbuf[i];
  return avg / CODEC_BUFFER_SIZE;
}

void Codec::set(uint32_t value){
  for(int i=0; i<CODEC_BUFFER_SIZE; ++i){
    codec_txbuf[i] = value;
    codec_rxbuf[i] = value;
  }
}

void Codec::bypass(bool doBypass){
  codec_bypass(doBypass);
}

void Codec::mute(bool doMute){
  codec_set_gain_out(0); // todo: fixme!
}

void Codec::setInputGain(int8_t value){
  codec_set_gain_in(value);
}

void Codec::setOutputGain(int8_t value){
  codec_set_gain_out(value);
}

void Codec::setHighPass(bool hpf){
#ifdef USE_PCM3168A
  if(hpf)
    codec_write(82, 0b00000000); // enable HPF for all ADC channels
  else
    codec_write(82, 0b00000111); // disable HPF for all ADC channels
#endif
}

#ifdef USE_IIS3DWB
extern "C" {
  void iis3dwb_read();
}

void Codec::start(){
  extern TIM_HandleTypeDef htim8;
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  iis3dwb_read();
}

void Codec::stop(){
  extern TIM_HandleTypeDef htim8;
  HAL_TIM_Base_Stop(&htim8);
}
#endif

#ifdef USE_ADS1294
#include "ads.h"

void Codec::start(){
  codec_blocksize = AUDIO_BLOCK_SIZE;
  ads_start_continuous();
  extern TIM_HandleTypeDef htim8;
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void Codec::stop(){
  ads_stop_continuous();
  extern TIM_HandleTypeDef htim8;
  HAL_TIM_Base_Stop(&htim8);
}

#endif // USE_ADS1294

#ifdef USE_WM8731

extern "C" {
  extern I2S_HandleTypeDef hi2s2;
}

void Codec::stop(){
  HAL_I2S_DMAStop(&hi2s2);
}

void Codec::start(){
  setInputGain(settings.audio_input_gain);
  setOutputGain(settings.audio_output_gain);
  // codec_blocksize = min(CODEC_BUFFER_SIZE/4, settings.audio_blocksize);
  codec_blocksize = CODEC_BUFFER_SIZE/4;
  HAL_StatusTypeDef ret;
  /* See STM32F405 Errata, I2S device limitations */
  /* The I2S peripheral must be enabled when the external master sets the WS line at: */
  while(HAL_GPIO_ReadPin(I2S_LRCK_GPIO_Port, I2S_LRCK_Pin)); // wait for low
  /* High level when the I2S protocol is selected. */
  while(!HAL_GPIO_ReadPin(I2S_LRCK_GPIO_Port, I2S_LRCK_Pin)); // wait for high

  // When a 16-bit data frame or a 16-bit data frame extended is selected during the I2S
  // configuration phase, the Size parameter means the number of 16-bit data length
  // in the transaction and when a 24-bit data frame or a 32-bit data frame is selected
  // the Size parameter means the number of 16-bit data length.
  ret = HAL_I2SEx_TransmitReceive_DMA(&hi2s2, (uint16_t*)codec_txbuf, (uint16_t*)codec_rxbuf, codec_blocksize*4);
  ASSERT(ret == HAL_OK, "Failed to start I2S DMA");
}

void Codec::pause(){
  HAL_I2S_DMAPause(&hi2s2);
}

void Codec::resume(){
  HAL_I2S_DMAResume(&hi2s2);
}

extern "C"{
  
  void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
    audioCallback(codec_rxbuf, codec_txbuf, codec_blocksize);
  }

  void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s){
    audioCallback(codec_rxbuf+codec_blocksize*2, codec_txbuf+codec_blocksize*2, codec_blocksize);
  }

  void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s){
    error(CONFIG_ERROR, "I2S Error");
  }
  
}
#endif /* USE_WM8731 */

#if defined USE_CS4271 || defined USE_PCM3168A

extern "C" {
  extern SAI_HandleTypeDef HSAI_RX;
  extern SAI_HandleTypeDef HSAI_TX;
  void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai){
    audioCallback(codec_rxbuf, codec_txbuf, codec_blocksize);
  }
  void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai){
    audioCallback(codec_rxbuf+codec_blocksize*AUDIO_CHANNELS, codec_txbuf+codec_blocksize*AUDIO_CHANNELS, codec_blocksize);
  }
  void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai){
    error(CONFIG_ERROR, "SAI DMA Error");
  }
}

void Codec::txrx(){
  HAL_SAI_DMAStop(&HSAI_RX);
  HAL_SAI_Transmit_DMA(&HSAI_RX, (uint8_t*)codec_rxbuf, codec_blocksize*AUDIO_CHANNELS*2);
}

void Codec::stop(){
  HAL_SAI_DMAStop(&HSAI_RX);
  HAL_SAI_DMAStop(&HSAI_TX);
}

void Codec::start(){
  setOutputGain(settings.audio_output_gain);
  // codec_blocksize = min(CODEC_BUFFER_SIZE/(AUDIO_CHANNELS*2), settings.audio_blocksize);
  codec_blocksize = CODEC_BUFFER_SIZE/(AUDIO_CHANNELS*2);
  HAL_StatusTypeDef ret;
#ifdef USE_CS4271
  ret = HAL_SAI_Receive_DMA(&HSAI_RX, (uint8_t*)codec_rxbuf, codec_blocksize*AUDIO_CHANNELS*2);
  if(ret == HAL_OK)
    ret = HAL_SAI_Transmit_DMA(&HSAI_TX, (uint8_t*)codec_txbuf, codec_blocksize*AUDIO_CHANNELS*2);
#else // PCM3168A
  // start slave first (Noctua)
  ret = HAL_SAI_Transmit_DMA(&HSAI_TX, (uint8_t*)codec_txbuf, codec_blocksize*AUDIO_CHANNELS*2);
  if(ret == HAL_OK)
    ret = HAL_SAI_Receive_DMA(&HSAI_RX, (uint8_t*)codec_rxbuf, codec_blocksize*AUDIO_CHANNELS*2);
#endif
  ASSERT(ret == HAL_OK, "Failed to start SAI DMA");
}

void Codec::pause(){
  HAL_SAI_DMAPause(&HSAI_RX);
  HAL_SAI_DMAPause(&HSAI_TX);
}

void Codec::resume(){
  HAL_SAI_DMAResume(&HSAI_RX);
  HAL_SAI_DMAResume(&HSAI_TX);
}

extern "C" {

// void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai){
// }

}

#endif /* USE_PCM3168A */

