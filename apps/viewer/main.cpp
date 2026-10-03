/**
 * @file main.cpp
 * @brief GraspLink simulator 실행 진입점.
 *
 * main은 엔진 기능을 직접 구현하지 않고 ViewerApp을 실행한다.
 * 최상위에서 std::exception을 잡아 초기화/런타임 오류가 발생했을 때
 * 원인을 stderr에 남기고 비정상 종료 코드를 반환한다.
 */
#include "ViewerApp.h"

#include <iostream>

int main()
{
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
