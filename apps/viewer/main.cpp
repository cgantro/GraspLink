#include "ViewerApp.h"

#include <iostream>
#include <string>

/**
 * @brief CLI 옵션을 해석하고 ViewerApp을 실행한다.
 * @details `--physics-demo`는 로봇 동작과 physics debug box를 요청한다. `--smoke-test`는
 * 숨겨진 창에서 고정 1/60초 delta로 8개의 렌더 완료 프레임을 실행한다. `--help`는 사용법을
 * 출력하며 알 수 없는 옵션은 종료 코드 2를 반환한다.
 * @param argc 명령행 인자 개수
 * @param argv 실행 파일 이름과 전달된 옵션 문자열
 * @return ViewerApp의 결과, 도움말이면 0, 알 수 없는 옵션이면 2, 예외면 -1
 */
int main(int argc, char** argv)
{
    try
    {
        ViewerOptions options;
        // 지원하는 인자만 ViewerOptions에 전달해 실행 동작을 명시적으로 선택한다.
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
