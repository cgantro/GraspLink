#include "ViewerApp.h"

#include <iostream>
#include <string>

/**
 * @brief 실행 인자를 읽어 ViewerApp을 설정하고 Viewer를 실행한다.
 * @details `--physics-demo`는 로봇 관절 움직임과 중력에 떨어지는 물리 상자를 켠다. `--smoke-test`는 화면에 보이지 않는 창에서 프레임 간격을 1/60초로 고정하고 렌더가 끝난 프레임을 8회 처리한 뒤 종료한다.
 * `--help`는 사용법을 출력한다. 지원하지 않는 옵션은 오류를 표시하고 종료 코드 2를 반환한다.
 * @param argc 프로그램 이름을 포함한 명령행 인자 개수.
 * @param argv 실행 파일 이름과 사용자가 전달한 각 옵션 문자열.
 * @return Viewer 실행 결과를 반환한다. 도움말은 0, 알 수 없는 옵션은 2, 예외가 발생하면 -1이다.
 */
int main(int argc, char** argv)
{
    try
    {
        ViewerOptions options;
        // 인자를 차례로 확인해 지원하는 옵션만 ViewerOptions에 기록한다. 옵션에 없는 실행 동작은 암묵적으로 켜지지 않는다.
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
