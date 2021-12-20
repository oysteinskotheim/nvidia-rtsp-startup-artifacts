#pragma once

#include <gst/gst.h>
#include <vector>

struct StreamingBin {
  GstElement *bin;
  GstElement *queue;
  GstElement *videoConvert;
  GstElement *capsFilter;
  GstElement *encoder;
  GstElement *parser;
  GstElement *appSink;
  GstCaps *appSinkCaps;
};

struct ArgusCameraGStreamerElements {
  // The nvtee element
  GstElement *nvtee;
  GstElement *streamingTee;
  GstElement *streamingTeeQueue;

  // Elements for capture bin
  GstElement *captureBin;
  GstElement *captureSource;
  GstElement *captureCapsFilter;

  // Elements for streaming bin
  StreamingBin streamingBin;
};
