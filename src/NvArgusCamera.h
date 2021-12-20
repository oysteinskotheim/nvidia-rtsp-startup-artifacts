#pragma once

#include "ArgusCameraGStreamerElements.h"
#include "CameraConfig.h"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <string>
#include <vector>

class NvArgusCamera {

protected:
  static void need_data(GstElement *appsrc, guint unused, GstElement *appsink);
  static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media,
                              gpointer user_data);
  static void media_disconnected(gpointer user_data);
  static GstFlowReturn new_sample(GstElement *elem, gpointer user_data);
  static gboolean _onBusMessage(GstBus *bus, GstMessage *msg, gpointer data);

protected:
  CameraConfig m_config;

  GstRTSPServer *m_server;
  GstRTSPMediaFactory *m_rtspMediaFactory;

  GstElement *m_pipeline;

  std::string m_teeName;
  ArgusCameraGStreamerElements m_elems;

  void createCaptureBin();
  void setupRtspServer();
  StreamingBin createStreamingBin(const StreamConfig &config);
  GstElement *createVideoEncoderElement(const EncoderConfig &config);
  void createPipeline();
  void startPipeline();
  void stopPipeline();
  void teardownPipeline();
  void startStream();

public:
  NvArgusCamera();
  virtual ~NvArgusCamera();

  void startStreaming();
  void dumpPipeline();
};
