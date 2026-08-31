#include "AppModes.h"
#include <iostream>
#include <string>
int main(int argc, char** argv) {
    if (argc < 2) { std::cerr << "Usage: poselink <vision|viewer|proxy> [options]\n"; return 2; }
    const std::string mode = argv[1];
    if (mode == "vision") return RunVisionMode(argc - 1, argv + 1);
    if (mode == "viewer") {
#ifdef POSELINK_WITH_VIEWER
        return RunViewerMode(argc - 1, argv + 1);
#else
        std::cerr << "Viewer mode was disabled at CMake configure time.\n";
        return 2;
#endif
    }
    if (mode == "proxy") return RunProxyMode(argc - 1, argv + 1);
    std::cerr << "Unknown mode: " << mode << '\n'; return 2;
}
