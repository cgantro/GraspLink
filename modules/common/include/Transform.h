#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// mat4_cast와 quaternion 정규화는 안정적인 gtc/quaternion API로 충분하다.
// gtx/quaternion을 포함하면 GLM의 실험적 확장 사용 정의가 필요해 빌드 오류가 발생한다.

namespace PoseLink
{

struct Transform
{   
    
    // world 공간 좌표
    glm::vec3 position{0.0f,0.0f,0.0f};

    // 물체 회전
    glm::quat rotation{1.0f,0.0f,0.0f,0.0f};

    // 크기
    glm::vec3 scale{1.0f, 1.0f,1.0f};

    /*
        position / rotation / scale 값을 하나의 Model Matrix로 생성

        Mesh의 Vertex는 처음에는 자기 자신을 기준으로 한 Local 좌표를 가진다
        
        model matrix를 곱하면, 이 로컬 좌표가 실제 World 공간상의 좌표로 변환된다

        3D 좌표계에서는 4x4 행렬을 통해 회전과 이동을 하나의 행렬 곱으로 처리하려고 함
    */ 
    glm::mat4 GetMatrix() const{
        /*
            Translation Matrix

            물제를 Pos만큼 이동시키는 행렬
        */
       const glm::mat4 translation = glm::translate(glm::mat4(1.0f),position);

       /*
            쿼터니언을 변환해야함
            4x4 rotation Matrix로 변환
       */
        const glm::mat4 rotationMatrix = glm::mat4_cast(
            glm::normalize(rotation) // 먼저 왜곡 방지 정규화
        );

        // Scale
        const glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f),scale);

        /*
            최종 Model Matrix
                Translation * Rotation * Scale
            
            행렬은 Vertex 기준으로 오른쪽부터 적용
            Local Vertex -> Scale -> Rotate -> Translate -> World Vertex
        */
        return translation
             * rotationMatrix
             * scaleMatrix;
    }
};


} // namespace PoseLink
