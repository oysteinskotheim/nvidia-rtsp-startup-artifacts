#pragma once

#include <array>
#include <string>

struct Nvv4l2EncoderConfig {
  unsigned int controlRate = 1;
  bool rateControl = true;
  std::string qpRange = "5,30:10,30:-1,-1";
  unsigned int vbvSize = 10;
  unsigned int presetLevel = 1;
  unsigned int profile = 0;
  bool twopassCBR = false;
  bool insertSpsPps = true;
  bool maxPerfEnable = true;
};

struct EncoderConfig {
  unsigned long bitrate = 6000000;
  unsigned long peakBitrate = 10000000;
  Nvv4l2EncoderConfig nvv4l2;
};

struct StreamConfig {
  bool enable = true;
  bool fakeClient = true;
  unsigned int width = 1280;
  unsigned int height = 720;
  unsigned int fps = 30;
  EncoderConfig encoder;
};

struct VideoCaptureConfig {
  EncoderConfig encoder;
};

struct CameraConfig {
  // Source settings
  unsigned int sourceWidth = 3840;
  unsigned int sourceHeight = 2160;
  unsigned int sourceFps = 30;
  std::array<unsigned long, 2> exposureTimeRange = {44000, 358733000};
  std::array<unsigned int, 2> gainRange = {1, 16};
  std::array<unsigned int, 2> ispDigitalGainRange = {1, 8};
  bool autoExposureLock = false;
  bool autoWhiteBalanceLock = false;
  unsigned int whiteBalanceMode = 1;
  unsigned long timeout = 0;
  float saturation = 1.0f;
  unsigned char sensorId = 0;
  float exposureCompensation = 0.0f;
  unsigned int autoExposureAntiBanding = 1;
  unsigned int temporalNoiseReductionMode = 1;
  float temporalNoiseReductionStrength = -1.0f;
  unsigned int edgeEnhancementMode = 1;
  float edgeEnhancementStrength = -1.0f;

  // Stream settings
  StreamConfig videoStream1;

  // RTSP settings
  size_t rtspLatency = 200;
};
