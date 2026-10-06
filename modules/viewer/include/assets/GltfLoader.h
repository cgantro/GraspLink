#pragma once

#include "assets/GraphicsTypes.h"

#include <filesystem>

/**
 * @brief GLB 3D 모델 파일에서 삼각형 표면·이미지·부품 정보를 읽어 프로그램 메모리에 담는다.
 * @details
 * GLB는 삼각형 꼭짓점, 부품 관계, 표면 재질, 이미지를 담는 3D 모델 파일 형식이다. TinyGLTF는 이 파일의 숫자와 이미지 데이터를 읽는 라이브러리다.
 * glTF의 Buffer는 파일에 담긴 큰 원본 byte 묶음이고 BufferView는 그중 시작점과 길이를 정한 구간이다.
 * Accessor는 BufferView 안에서 읽을 값의 자료형과 개수, 첫 값 위치를 설명하는 기록이다. byteOffset은 첫 값까지 건너뛸 byte 수이고 byteStride는 연속된 값 사이의 byte 간격이다.
 * 예를 들어 한 정점의 위치·법선·이미지 좌표가 교차 저장되면 다음 위치는 위치 숫자만큼이 아니라 정점의 나머지 속성까지 건너뛴 곳에 있다.
 * 정점은 삼각형 꼭짓점 자료이며 위치·법선은 실수 3개, 이미지 좌표는 실수 2개로 읽는다. tangent는 표면을 따라가는 방향으로 네 숫자 중 앞의 세 개만 받는다.
 * 정수로 압축한 속성, 일부 값만 덮어쓰는 Sparse 형식, 삼각형 이외 표면은 지원하지 않는다.
 * 읽은 픽셀과 재질·형상 번호는 CPU 자료구조로 돌려준다. 그래픽 카드 업로드는 AssetManager가 맡는다.
 * 재질의 기본색과 기본색 이미지, 금속성·거칠기 계수, 발광색을 읽는다. 선택 장면은 최상위 부품 하나에서 시작하는 계층이어야 한다.
 * 그릴 형상이 없는 부품도 위치 기준점과 부모·자식 관계 보존을 위해 결과에 남긴다.
 */
class GltfLoader final
{
public:
    /**
     * @brief GLB에서 형상·재질·이미지와 부품 연결을 읽어 CPU 자료구조로 만든다.
     * @param path 읽을 GLB 파일 경로.
     * @return 파일 경로를 포함한 자원 번호, CPU 배열, 선택 장면의 시작 부품 번호.
     * @throws std::runtime_error 파일을 읽지 못했거나 바이트 범위가 잘못됐거나 무한대·NaN 또는 지원하지 않는 파일 형식을 만났을 때 발생한다. 자세한 내용은 예외 메시지에 있다.
     * @throws std::out_of_range model.accessors 배열 범위 밖 번호를 조회할 때 발생한다. Accessor는 BufferView 안에서 읽을 값의 형식과 위치를 설명하는 기록이다.
     * @details
     * 각 값은 Accessor가 지정한 첫 byte 위치부터 byteStride만큼 건너뛰며 복사한다. 예를 들어 위치·법선·UV를 정점마다 섞어 저장하면 다음 위치는 위치 숫자만큼이 아니라 그 정점의 나머지 값까지 건너뛴 곳에 있다.
     * 마지막 값도 BufferView와 실제 Buffer 범위 안에 있는지 확인한다. 정렬되지 않은 float 주소는 직접 읽지 않고 byte 복사한다.
     * 서로 다른 표면 묶음은 형상 배열 뒤에 이어 붙이고, 묶음 안의 정점 번호에 앞서 추가한 정점 개수를 더한다.
     * 번호가 없으면 정점 0부터 순서대로 사용한다. 각 표면의 재질 번호와 그릴 범위는 별도 기록한다.
     * 파일의 부품 변환은 위치·회전·크기로 풀어 저장한다. 축 기울임이나 원근은 이 세 값으로 재현할 수 없어 거부한다.
     * 회전은 길이 1인 quaternion 네 숫자로 보관한다. Quaternion은 축 방향과 회전량을 함께 나타내며 yaw/pitch/roll 세 축 회전값과 표현 방식이 다르다.
     * 파일 순서 [x,y,z,w]를 GLM 생성자 인자 순서 (w,x,y,z)로 바꾼다.
     * 이 함수는 GPU 객체를 만들지 않는다.
     */
    static ModelResource LoadGLB(const std::filesystem::path& path);

private:
    GltfLoader() = delete;
};
