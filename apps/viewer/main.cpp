// 앱 구성과 오류 보고만 담당. 데모는 명시적 옵션으로 시작한다.
#include "ViewerApp.h"

#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    try
    {
        ViewerOptions options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            if (argument == "--physics-demo") options.physicsDemo = true;
            else if (argument == "--smoke-test") options.smokeTest = true;
            else if (argument == "--help")
            {
                std::cout << "Usage: grasplink_simulator [--physics-demo] [--smoke-test]\n";
                return 0;
            }
            else
            {
                std::cerr << "Unknown option: " << argument << '\n';
                return 2;
            }
        }
        ViewerApp app(options);
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
