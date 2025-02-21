#include <Arduino.h>
#include "driver/adc.h"   //analog input driver for voltage range

// //Adding audio files and library
#include "Audio_agate_thakun.h"         //D
#include "Audio_Bame_Badha.h"           //D
#include "Audio_Bame_Jan.h"           //problem
#include "Audio_Dane_Badha.h"           //D
#include "Audio_Dane_Jan.h"           //problem
#include "Audio_Halka_Dane_Jan.h"       //D
#include "Audio_Power_Off.h"            //D
#include "Audio_Power_On.h"             //D
#include "Audio_Thamun.h"             //problem
//#include "SoundData.h"
#include "XT_DAC_Audio.h"

// //initializing and assigning class to the audio array
XT_Wav_Class Agate_Thakun_Audio(Agate_Thakun);
XT_Wav_Class Bame_Badha_Audio(Bame_Badha);
XT_Wav_Class Bame_Jan_Audio(Bame_Jan);
XT_Wav_Class Dane_Badha_Audio(Dane_Badha);
XT_Wav_Class Dane_Jan_Audio(Dane_Jan);
 XT_Wav_Class Halka_Dane_Audio(Halka_Dane_Jan);
XT_Wav_Class Power_Off_Audio(Power_Off);
XT_Wav_Class Power_On_Audio(Power_On);
XT_Wav_Class Thamu_Audio(Thamun);
// XT_Wav_Class Sample_audio(sample);   //initializing the sample audio data
XT_DAC_Audio_Class DacAudio(25, 0); //connected to pin10 or GPIO25 or DAC1 (create the main player class object)

void Play_Agate_Thakun()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Agate_Thakun_Audio);
  while (Agate_Thakun_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Bame_Badha()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Bame_Badha_Audio);
  while (Bame_Badha_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Bame_Jan()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Bame_Jan_Audio);
  while (Bame_Jan_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Dane_Badha()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Dane_Badha_Audio);
  while (Dane_Badha_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Dane_Jan()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Dane_Jan_Audio);
  while (Dane_Jan_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Halka_Dane_Jan()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Halka_Dane_Audio);
  while (Halka_Dane_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Power_Off()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Power_Off_Audio);
  while (Power_Off_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Power_On()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Power_On_Audio);
  while (Power_On_Audio.Playing) DacAudio.FillBuffer();
}
void Play_Thamun()
{
  DacAudio.FillBuffer();
  DacAudio.Play(&Thamu_Audio);
  while (Thamu_Audio.Playing) DacAudio.FillBuffer();
}
// void Play_Sample()
// {
//   DacAudio.FillBuffer();
//   DacAudio.Play(&Sample_audio);
//   while (Sample_audio.Playing) DacAudio.FillBuffer();
// }


void setup() {
  Serial.begin(115200);
  Serial.println("Start");
}

void loop(){    
    //playing sound
    Serial.println("Playing audio");
    Play_Agate_Thakun();
    Play_Bame_Badha();
    Play_Bame_Jan();
    Play_Dane_Badha();
    Play_Dane_Jan();
    Play_Halka_Dane_Jan();
    Play_Power_Off();
    Play_Power_On();
    Play_Thamun();
  }