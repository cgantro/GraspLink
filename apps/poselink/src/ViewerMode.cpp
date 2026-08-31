#include "AppModes.h"
#include "poselink/streaming/PoseReceiver.h"
#include "poselink/pose/Time.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <string>
int RunViewerMode(int argc, char** argv) {
    uint16_t port=5000; uint64_t bufferUs=30'000;
    for(int i=1;i+1<argc;++i){std::string a=argv[i];if(a=="--port")port=uint16_t(std::stoi(argv[++i]));else if(a=="--buffer-ms")bufferUs=std::stoull(argv[++i])*1000;}
    std::string error; poselink::PoseReceiver receiver(port); if(!receiver.Start(error)){std::cerr<<error<<'\n';return 1;} if(!glfwInit())return 1;
    GLFWwindow* window=glfwCreateWindow(1280,720,"PoseLink Viewer",nullptr,nullptr);if(!window){glfwTerminate();return 1;}glfwMakeContextCurrent(window);if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))return 1;
    while(!glfwWindowShouldClose(window)){receiver.Poll();poselink::PoseSample pose{};bool interpolated=false;receiver.Sample(poselink::SteadyNowUs()-bufferUs,pose,interpolated);glClearColor(pose.valid?.06f:.25f,.08f,.12f,1);glClear(GL_COLOR_BUFFER_BIT);glfwSwapBuffers(window);glfwPollEvents();}glfwDestroyWindow(window);glfwTerminate();return 0;
}
