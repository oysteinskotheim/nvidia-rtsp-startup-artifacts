#include "NvArgusCamera.h"
#include <iostream>
#include <signal.h>

GMainLoop *mainloop;

void signal_handler(int signal) { g_main_loop_quit(mainloop); }

int main(int argc, char *argv[]) {
  try {
    gst_init(&argc, &argv);

    mainloop = g_main_loop_new(NULL, FALSE);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    NvArgusCamera camera;
    camera.startStreaming();

    g_main_loop_run(mainloop);
    return EXIT_SUCCESS;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
  }
}
