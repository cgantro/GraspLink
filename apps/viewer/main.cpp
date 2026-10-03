#include "ViewerApp.h"
#include "iostream"
int main(){
    try
    {
        ViewerApp app;
        return app.Run();
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[Fatal Error] "
            << e.what()
            << std::endl;

        return -1;
    }
}
