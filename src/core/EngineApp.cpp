#include "core/EngineApp.h"

/**
 * @brief EngineApp
 * @author 홍윤표
 * 
 */


void EngineApp::Run(){
    Init();
    MainLoop();
    Shutdown();
}

void EngineApp::Init(){
    m_Window = std::make_shared<Window>(1280,720,"MiniBCG");
    m_Window->Init();
}

void EngineApp::MainLoop(){
    while(!m_Window->ShouldClose()){
        
    }
}