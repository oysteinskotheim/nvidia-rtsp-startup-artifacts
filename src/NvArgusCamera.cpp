#include "NvArgusCamera.h"
#include <gst/app/app.h>
#include <gst/video/video-format.h>
#include <gst/video/video.h>
#include <iostream>
#include <sstream>
#include <string>

#define ASSERT_THROW(statement, err_msg)                                       \
  if (!(statement)) {                                                          \
    throw std::runtime_error{err_msg};                                         \
  }

template <typename T> std::string range2str(const std::array<T, 2> &range) {
  std::stringstream ss;
  ss << range[0] << " " << range[1];
  return ss.str();
}

GstElement *safe_gst_element_factory_make(const gchar *factoryname,
                                          const gchar *name) {
  GstElement *elem = gst_element_factory_make(factoryname, name);
  if (!elem)
    throw std::runtime_error{
        ("Failed to create element " + std::string{factoryname}).c_str()};
  return elem;
}

void NvArgusCamera::need_data(GstElement *appsrc, guint unused,
                              GstElement *appsink) {
  GstSample *sample;
  GstFlowReturn ret;

  sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));

  if (sample) {
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstSegment *seg = gst_sample_get_segment(sample);
    GstClockTime pts, dts;

    /* Convert the PTS/DTS to running time so they start from 0 */
    pts = GST_BUFFER_PTS(buffer);
    if (GST_CLOCK_TIME_IS_VALID(pts))
      pts = gst_segment_to_running_time(seg, GST_FORMAT_TIME, pts);

    dts = GST_BUFFER_DTS(buffer);
    if (GST_CLOCK_TIME_IS_VALID(dts))
      dts = gst_segment_to_running_time(seg, GST_FORMAT_TIME, dts);

    if (buffer) {
      /* Make writable so we can adjust the timestamps */
      GstBuffer *adjustedBuffer = gst_buffer_copy(buffer);
      GST_BUFFER_PTS(adjustedBuffer) = pts;
      GST_BUFFER_DTS(adjustedBuffer) = dts;
      g_signal_emit_by_name(appsrc, "push-buffer", adjustedBuffer, &ret);
      gst_buffer_unref(adjustedBuffer);
    }

    /* we don't need the appsink sample anymore */
    gst_sample_unref(sample);
  }
}

GstFlowReturn NvArgusCamera::new_sample(GstElement *elem, gpointer user_data) {
  NvArgusCamera *self = reinterpret_cast<NvArgusCamera *>(user_data);
  GstSample *sample = nullptr;

  /* get the sample from appsink */
  sample = gst_app_sink_pull_sample(GST_APP_SINK(elem));

  if (sample) {
    GstMapInfo map;
    gint width, height, channels;
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    const gchar *formatString = gst_structure_get_string(structure, "format");
    GstVideoFormat format = gst_video_format_from_string(formatString);
    const GstVideoFormatInfo *formatInfo = gst_video_format_get_info(format);
    channels = formatInfo->n_components;
    // gst_caps_unref(caps);

    gst_buffer_map(buffer, &map, GST_MAP_READ);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
  }

  return GstFlowReturn::GST_FLOW_OK;
}

void NvArgusCamera::media_disconnected(gpointer user_data) {
  NvArgusCamera *self = reinterpret_cast<NvArgusCamera *>(user_data);
  self->stopPipeline();
}

// Called when a new media pipeline is constructed. We can query the
// pipeline and configure our appsrc
void NvArgusCamera::media_configure(GstRTSPMediaFactory *factory,
                                    GstRTSPMedia *media, gpointer user_data) {
  GstElement *element, *appsrc, *appsink;

  NvArgusCamera *self = reinterpret_cast<NvArgusCamera *>(
      g_object_get_data(G_OBJECT(factory), "user_data"));

  size_t streamNumber = reinterpret_cast<size_t>(
      g_object_get_data(G_OBJECT(factory), "stream_number"));

  printf("media_configure called for stream number %ld\n", streamNumber);

  // get the element (bin) used for providing the streams of the media
  element = gst_rtsp_media_get_element(media);

  // make sure the data is freed when the media is gone
  g_object_set_data_full(G_OBJECT(media), "rtsp-extra-data", self,
                         (GDestroyNotify)media_disconnected);

  std::string videoSourceName = "videosrc" + std::to_string(streamNumber);
  std::string appSinkName = "videoappsink" + std::to_string(streamNumber);

  appsrc =
      gst_bin_get_by_name_recurse_up(GST_BIN(element), videoSourceName.c_str());

  ASSERT_THROW(appsrc, "Could not get video source");
  appsink = gst_bin_get_by_name(GST_BIN(self->m_pipeline), appSinkName.c_str());
  ASSERT_THROW(appsink, "Could not get appsink");
  gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
  g_object_set(G_OBJECT(appsrc), "caps", self->m_elems.streamingBin.appSinkCaps,
               nullptr);

  // Install the callback that will be called when a buffer is needed
  g_signal_connect(appsrc, "need-data", (GCallback)need_data, appsink);
  gst_object_unref(element);

  // Start the pipeline
  self->startPipeline();

  GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(
      GST_BIN(self->m_pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "media_configure");
}

gboolean NvArgusCamera::_onBusMessage(GstBus *bus, GstMessage *msg,
                                      gpointer data) {
  switch (GST_MESSAGE_TYPE(msg)) {
  case GST_MESSAGE_ERROR: {
    GError *err = NULL;
    gchar *name, *debug = NULL;

    name = gst_object_get_path_string(msg->src);
    gst_message_parse_error(msg, &err, &debug);

    printf("ERROR on bus: by %s: %s\n", name, err->message);
    if (debug != NULL)
      printf("debug info:\n%s\n", debug);

    g_error_free(err);
    g_free(debug);
    g_free(name);
  } break;

  case GST_MESSAGE_STATE_CHANGED: {
    GstState old, new_state, pending;
    gst_message_parse_state_changed(msg, &old, &new_state, &pending);

    GST_DEBUG_OBJECT(
        GST_OBJECT(msg->src), "changed state from %s to %s, pending %s\n",
        gst_element_state_get_name(old), gst_element_state_get_name(new_state),
        gst_element_state_get_name(pending));
  } break;

  case GST_MESSAGE_EOS: {
    printf("EOS received\n");
  } break;

  case GST_MESSAGE_APPLICATION: {
    const GstStructure *s;
    s = gst_message_get_structure(msg);

    if (gst_structure_has_name(s, "NvGstAppInterrupt")) {
      printf("Received NvGstAppInterrupt\n");
    }
  } break;

  case GST_MESSAGE_ELEMENT:
    break;

  default:
    break;
  }
  return TRUE;
}

NvArgusCamera::NvArgusCamera()
    : m_server(nullptr), m_pipeline(nullptr), m_teeName("sourceTee") {

  m_server = gst_rtsp_server_new();

  // Default configuration
  m_config.videoStream1.enable = true;

  createPipeline();
}

NvArgusCamera::~NvArgusCamera() { teardownPipeline(); }

void NvArgusCamera::createCaptureBin() {
  // Create capture pipeline bin
  m_elems.captureBin = gst_bin_new("cap_bin");

  /* Create the capture source element */
  m_elems.captureSource =
      safe_gst_element_factory_make("nvarguscamerasrc", nullptr);

  // CSI camera properties tuning
  g_object_set(G_OBJECT(m_elems.captureSource), "wbmode",
               m_config.whiteBalanceMode, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "saturation",
               m_config.saturation, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "timeout", m_config.timeout,
               nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "sensor-id", m_config.sensorId,
               nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "aelock",
               m_config.autoExposureLock ? 1 : 0, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "awblock",
               m_config.autoWhiteBalanceLock ? 1 : 0, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "exposurecompensation",
               m_config.exposureCompensation, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "aeantibanding",
               m_config.autoExposureAntiBanding, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "tnr-mode",
               m_config.temporalNoiseReductionMode, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "ee-mode",
               m_config.edgeEnhancementMode, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "tnr-strength",
               m_config.temporalNoiseReductionStrength, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "ee-strength",
               m_config.edgeEnhancementStrength, nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "exposuretimerange",
               range2str(m_config.exposureTimeRange).c_str(), nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "gainrange",
               range2str(m_config.gainRange).c_str(), nullptr);
  g_object_set(G_OBJECT(m_elems.captureSource), "ispdigitalgainrange",
               range2str(m_config.ispDigitalGainRange).c_str(), nullptr);

  m_elems.captureCapsFilter =
      safe_gst_element_factory_make("capsfilter", nullptr);

  GstCaps *caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "NV12", "width", G_TYPE_INT,
      m_config.sourceWidth, "height", G_TYPE_INT, m_config.sourceHeight,
      "framerate", GST_TYPE_FRACTION, m_config.sourceFps, 1, nullptr);

  GstCapsFeatures *feature = gst_caps_features_new("memory:NVMM", nullptr);
  gst_caps_set_features(caps, 0, feature);

  // Set capture caps on capture filter
  g_object_set(m_elems.captureCapsFilter, "caps", caps, nullptr);
  gst_caps_unref(caps);

  gst_bin_add_many(GST_BIN(m_elems.captureBin), m_elems.captureSource,
                   m_elems.captureCapsFilter, nullptr);

  ASSERT_THROW(
      gst_element_link(m_elems.captureSource, m_elems.captureCapsFilter),
      "Could not link nvarguscamerasrc and capsfilter");

  GstPad *pad = gst_element_get_static_pad(m_elems.captureCapsFilter, "src");
  ASSERT_THROW(pad, "Could not get static src pad of capsfilter");

  gst_element_add_pad(m_elems.captureBin, gst_ghost_pad_new("src", pad));
  gst_object_unref(GST_OBJECT(pad));
}

StreamingBin NvArgusCamera::createStreamingBin(const StreamConfig &config) {
  StreamingBin bin;
  std::string binName = "streaming_bin_0";
  std::string appSinkName = "videoappsink0";
  printf("Creating streaming bin");

  bin.bin = gst_bin_new(binName.c_str());
  bin.queue = safe_gst_element_factory_make("queue", nullptr);
  bin.videoConvert = safe_gst_element_factory_make("nvvidconv", nullptr);
  bin.encoder = createVideoEncoderElement(config.encoder);
  bin.parser = safe_gst_element_factory_make("h264parse", nullptr);
  bin.capsFilter = safe_gst_element_factory_make("capsfilter", nullptr);
  bin.appSink = safe_gst_element_factory_make("appsink", appSinkName.c_str());

  GstCaps *caps = gst_caps_new_simple(
      "video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT,
      config.width, "height", G_TYPE_INT, config.height, "framerate",
      GST_TYPE_FRACTION, config.fps, 1, nullptr);

  GstCapsFeatures *feature = gst_caps_features_new("memory:NVMM", nullptr);
  gst_caps_set_features(caps, 0, feature);

  g_object_set(bin.capsFilter, "caps", caps, nullptr);
  gst_caps_unref(caps);

  gst_bin_add_many(GST_BIN(bin.bin), bin.queue, bin.videoConvert,
                   bin.capsFilter, bin.encoder, bin.parser, bin.appSink,
                   nullptr);

  g_object_set(G_OBJECT(bin.appSink), "async", FALSE, nullptr);
  g_object_set(G_OBJECT(bin.appSink), "sync", FALSE, nullptr);
  g_object_set(G_OBJECT(bin.appSink), "drop", TRUE, nullptr);
  gst_util_set_object_arg(G_OBJECT(bin.appSink), "format", "time");
  gst_util_set_object_arg(G_OBJECT(bin.appSink), "max-buffers", "1");

  bin.appSinkCaps = gst_caps_new_simple(
      "video/x-h264", "stream-format", G_TYPE_STRING, "byte-stream",
      "alignment", G_TYPE_STRING, "au", "width", G_TYPE_INT, config.width,
      "height", G_TYPE_INT, config.height, "framerate", GST_TYPE_FRACTION,
      config.fps, 1, nullptr);

  g_object_set(G_OBJECT(bin.appSink), "caps", bin.appSinkCaps, NULL);

  gst_element_link_many(bin.queue, bin.videoConvert, bin.capsFilter,
                        bin.encoder, bin.parser, bin.appSink, nullptr);

  GstPad *pad = gst_element_get_static_pad(bin.queue, "sink");
  ASSERT_THROW(pad, "Could not get sink pad of streaming queue");
  gst_element_add_pad(bin.bin, gst_ghost_pad_new("sink", pad));
  gst_object_unref(GST_OBJECT(pad));

  return bin;
}

GstElement *
NvArgusCamera::createVideoEncoderElement(const EncoderConfig &config) {
  GstElement *encoder = nullptr;

  auto set_object_arg_print = [](GObject *object, const gchar *name,
                                 const gchar *value) {
    printf("  Setting %s to %s\n", name, value);
    gst_util_set_object_arg(object, name, value);
  };

  printf("Creating nvv4l2h264 encoder.\n");
  encoder = safe_gst_element_factory_make("nvv4l2h264enc", nullptr);

  set_object_arg_print(G_OBJECT(encoder), "control-rate",
                       std::to_string(config.nvv4l2.controlRate).c_str());
  set_object_arg_print(G_OBJECT(encoder), "bitrate",
                       std::to_string(config.bitrate).c_str());
  set_object_arg_print(G_OBJECT(encoder), "peak-bitrate",
                       std::to_string(config.peakBitrate).c_str());

  set_object_arg_print(G_OBJECT(encoder), "preset-level",
                       std::to_string(config.nvv4l2.presetLevel).c_str());

  set_object_arg_print(G_OBJECT(encoder), "profile",
                       std::to_string(config.nvv4l2.profile).c_str());

  set_object_arg_print(G_OBJECT(encoder), "maxperf-enable",
                       config.nvv4l2.maxPerfEnable ? "true" : "false");

  set_object_arg_print(G_OBJECT(encoder), "insert-sps-pps",
                       config.nvv4l2.insertSpsPps ? "true" : "false");

  // Don't set qp-range and vbv-size unless they are explicitly set
  if (config.nvv4l2.qpRange.length() > 0) {
    set_object_arg_print(G_OBJECT(encoder), "qp-range",
                         config.nvv4l2.qpRange.c_str());
  }

  if (config.nvv4l2.vbvSize > 0) {
    set_object_arg_print(G_OBJECT(encoder), "vbv-size",
                         std::to_string(config.nvv4l2.vbvSize).c_str());
  }

  set_object_arg_print(G_OBJECT(encoder), "EnableTwopassCBR",
                       config.nvv4l2.twopassCBR ? "true" : "false");

  set_object_arg_print(G_OBJECT(encoder), "ratecontrol-enable",
                       config.nvv4l2.rateControl ? "true" : "false");

  return encoder;
}

void NvArgusCamera::createPipeline() {
  GstPad *srcPad, *sinkPad;

  // Create the camera pipeline
  m_pipeline = gst_pipeline_new("capture_native_pipeline");

  if (!m_pipeline)
    throw std::runtime_error{"Capture native pipeline creation failed"};

  GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(m_pipeline));
  gst_bus_add_watch(bus, _onBusMessage, this);
  gst_object_unref(bus);

  g_object_set(m_pipeline, "message-forward", TRUE, NULL);

  createCaptureBin();

  m_elems.nvtee = safe_gst_element_factory_make("nvtee", nullptr);
  g_object_set(G_OBJECT(m_elems.nvtee), "name", "cam_t", nullptr);

  m_elems.streamingTee = safe_gst_element_factory_make("tee", "streamingTee");
  m_elems.streamingTeeQueue = safe_gst_element_factory_make("queue", nullptr);

  g_object_set(G_OBJECT(m_elems.streamingTeeQueue), "max-size-buffers", 90,
               nullptr);

  gst_util_set_object_arg(G_OBJECT(m_elems.streamingTeeQueue), "leaky",
                          "downstream");

  // Set capture mode (1: image, 2: video)
  g_object_set(G_OBJECT(m_elems.nvtee), "mode", 2, nullptr);

  gst_bin_add_many(GST_BIN(m_pipeline), m_elems.nvtee, m_elems.captureBin,
                   m_elems.streamingTeeQueue, m_elems.streamingTee, nullptr);

  if (m_config.videoStream1.enable) {
    m_elems.streamingBin = createStreamingBin(m_config.videoStream1);
    gst_bin_add(GST_BIN(m_pipeline), m_elems.streamingBin.bin);
  }

  // Link capture bin with nvtee
  gst_element_link(m_elems.captureBin, m_elems.nvtee);

  // Link pre_src of nvtee to streaming tee
  srcPad = gst_element_get_static_pad(m_elems.nvtee, "pre_src");
  if (!srcPad)
    throw std::runtime_error{"Failed to get preview source from nvtee"};
  sinkPad = gst_element_get_static_pad(m_elems.streamingTeeQueue, "sink");
  if (gst_pad_link(srcPad, sinkPad) != GST_PAD_LINK_OK)
    throw std::runtime_error{"Failed to link pre_src from nvtee to sink pad of "
                             "streaming queue"};
  gst_object_unref(srcPad);
  gst_object_unref(sinkPad);

  gst_element_link(m_elems.streamingTeeQueue, m_elems.streamingTee);

  // Link streaming bin to preview source of nvtee
  if (m_config.videoStream1.enable) {
    srcPad = gst_element_get_request_pad(m_elems.streamingTee, "src_%u");
    if (!srcPad)
      throw std::runtime_error{
          "Failed to get tee request pad for streaming bin"};
    sinkPad = gst_element_get_static_pad(m_elems.streamingBin.bin, "sink");
    if (!sinkPad || !srcPad)
      throw std::runtime_error{
          "Failed to get pads from nvtee and streaming bin"};
    if (gst_pad_link(srcPad, sinkPad) != GST_PAD_LINK_OK)
      throw std::runtime_error{"Failed to link source from nvtee to sink "
                               "pad of streaming bin"};
    gst_object_unref(srcPad);
    gst_object_unref(sinkPad);
  }
}

void NvArgusCamera::teardownPipeline() {
  gst_element_set_state(m_pipeline, GST_STATE_NULL);
  gst_object_unref(m_pipeline);
}

void NvArgusCamera::dumpPipeline() {
  GST_DEBUG_BIN_TO_DOT_FILE_WITH_TS(GST_BIN(m_pipeline),
                                    GST_DEBUG_GRAPH_SHOW_ALL, "NvArgusCamera");
}

void NvArgusCamera::startPipeline() {
  if (gst_element_set_state(m_pipeline, GST_STATE_PLAYING) ==
      GST_STATE_CHANGE_FAILURE)
    throw std::runtime_error{"Could not set pipeline to playing state"};
}

void NvArgusCamera::stopPipeline() {
  printf("Stopping pipeline\n");
  gst_element_set_state(m_pipeline, GST_STATE_READY);
}

void NvArgusCamera::startStreaming() {
  if (m_config.videoStream1.enable)
    startStream();
}

void NvArgusCamera::startStream() {
  printf("Starting RTSP stream\n");
  printf("Setting RTSP latency to %d\n",
         static_cast<int>(m_config.rtspLatency));

  m_rtspMediaFactory = gst_rtsp_media_factory_new();

  // Provide a way for the callbacks to get the pointer to this instance
  // of the class
  g_object_set_data(G_OBJECT(m_rtspMediaFactory), "user_data", this);
  g_object_set_data(G_OBJECT(m_rtspMediaFactory), "stream_number", (gpointer)0);

  gst_rtsp_media_factory_set_latency(m_rtspMediaFactory, m_config.rtspLatency);
  gst_rtsp_media_factory_set_shared(m_rtspMediaFactory, TRUE);

  gst_rtsp_media_factory_set_launch(m_rtspMediaFactory,
                                    std::string{"appsrc name=videosrc0"
                                                " is-live=1 ! h264parse ! "
                                                "rtph264pay name=pay0 pt=96 "}
                                        .c_str());

  // Provide a way for the callback to get the pointer to this
  // instance of the class
  g_object_set_data(G_OBJECT(m_rtspMediaFactory), "user_data", this);

  // notify when our media is ready, This is called whenever
  // someone asks for the media and a new pipeline with our appsrc
  // is created
  g_signal_connect(m_rtspMediaFactory, "media-configure",
                   (GCallback)media_configure, NULL);

  auto mounts = gst_rtsp_server_get_mount_points(m_server);
  gst_rtsp_mount_points_add_factory(mounts, "/video0", m_rtspMediaFactory);
  g_object_unref(mounts);

  // attach the server to the default maincontext
  gst_rtsp_server_attach(m_server, nullptr);

  printf("RTSP server ready at rtsp://localhost:8554/video0\n");
}
