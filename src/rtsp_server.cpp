#include "NvArgusCamera.h"
#include <glib-unix.h>
#include <glib.h>
#include <iostream>
#include <signal.h>

GMainLoop *mainloop;

static gboolean exit_signal_handler(gpointer mainloop) {
  g_main_loop_quit((GMainLoop *)mainloop);
  return true;
}

static void setup_signal_handlers() {
  GSource *source = NULL;
  GMainContext *ctx = g_main_loop_get_context(mainloop);

  source = g_unix_signal_source_new(SIGINT);
  g_source_set_callback(source, exit_signal_handler, mainloop, NULL);
  g_source_attach(source, ctx);

  source = g_unix_signal_source_new(SIGTERM);
  g_source_set_callback(source, exit_signal_handler, mainloop, NULL);
  g_source_attach(source, ctx);
}

void signal_handler(int signal) { g_main_loop_quit(mainloop); }

int main(int argc, char *argv[]) {
  try {
    gst_init(&argc, &argv);

    mainloop = g_main_loop_new(NULL, FALSE);

    setup_signal_handlers();

    NvArgusCamera camera;
    camera.startStreaming();

    g_main_loop_run(mainloop);
    return EXIT_SUCCESS;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }
}
