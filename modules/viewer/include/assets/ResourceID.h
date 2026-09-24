#pragma once

#include <cstdint>
#include <functional>
#include <string_view>

/*
    Asset을 문자열 경로 자체가 아니라 정수 ID로 식별하기 위한 타입

    "경로" -> FNV-1a Hash -> 64-bit 정수

    AssetManager에서는 아래와 같이 사용한다
        unordered_map<ResourceID, MeshData>
        unordered_map<ResourceID, MaterialData>
        unordered_map<ResourceID, Texture>
    
    이렇게 하면 ECS 컴포넌트가 무거운 shared_ptr이나 문자열을 가지지 않는다.
    가벼운 Handle (ResourceID meshID)만 저장

     ResourceID = 리소스를 찾기 위한 Key
     AssetManager = 실제 리소스의 Owner/Cache
*/

/*
    FNV-1a 64-bit
    
    문자열을 64비트 정수로 변환하는 단순하고 빠른 Hash

    계산 방식:
        hash = offset_basis 
            생성하려는 해시의 비트 수(32비트, 64비트, 128비트 등)에 따라 
            컴퓨터 과학적으로 이미 정해진 고유한 상수 값
        각 바이트마다 : XOR(^=) 연산 후 소수(Prime) 곱셈(*=) 연산
            hash ^= byte
            hash *= FNC_prime
*/

inline constexpr std::uint64_t kFnvOffsetBasis64 =
    14695981039346656037ULL;

inline constexpr std::uint64_t kFnvPrime64 =
    1099511628211ULL;

struct ResourceID{
    // 0 : Invalid
    std::uint64_t value = 0;

    constexpr ResourceID() = default;

    /*
        저장된 ID를 역직렬화하거나 복원할 때 사용
        explicit을 붙여서 
            ResourceID id = 123;
        과 같은 암묵적 변환을 막는다
    */
    constexpr explicit ResourceID(std::uint64_t rawValue)
        : value(rawValue){ }
    /*
        문자열 → ResourceID.

        constexpr이므로 문자열 literal이라면
        compile-time 평가도 가능하다.
    */
    constexpr explicit ResourceID(std::string_view text)
        : value(Hash(text)){}

    /*  
        if(id)와 같은 형태 지원
        explicit이기 때문에 ResourceID가 의도치 않게
        정수 연산에 참여하지 않는다.
    */
    constexpr explicit operator bool() const noexcept { return value != 0;}

    // 이 함수는 예외를 던지지 않는다
    const bool IsValid() const noexcept {return value != 0;}
    constexpr bool operator==(const ResourceID& other) const noexcept{
        return value == other.value;
    }
    constexpr bool operator!=(const ResourceID& other) const noexcept{
        return !(*this == other);
    }
    constexpr bool operator<(const ResourceID& other) const noexcept{
        return value < other.value;
    }
private:

    /*  
        const string&
            : 리터럴 문자 넘길 때, 리터럴 자체를 복사해 임시 객체 생성
            : 문자열이 길면 힙 메모리 할당 -> 해제
        string_view 아래의 두개 구조(16바이트)
            : const char* ptr 구조
            : size_t len
            -> 어떤 형태의 문자열이 들어와도 복사나 힙 할당 없음
            -> 주소와 길이만 복사해서 바로 읽음
    */
    static constexpr std::uint64_t Hash(std::string_view text) noexcept{
        if(text.empty()) return 0;

        std::uint64_t hash = kFnvOffsetBasis64;

        for(const char c : text){
            /*  char는 컴파일러에 따라 signed일 수 있음(부호확장 버그)
                    비트 연산시에 char를 연산 장치 크기인 int나 해시 변수 크기로 확장함
                    unsigned라면 크기 확장할 때 앞을 0으로 채움
                    signed는 1로 채움
                    -> XOR 연산하면 하위 8비트만 영향이 있어야하는데
                    -> 상위 비트가 1로 변하면서 해시가 이상해짐
                    -> 컴파일러 환경에 따라 해시 결과가 달라짐
                
                byte 값 자체 Hash위해 unsigned로 변환
            */
            hash ^= static_cast<unsigned char>(c);
            hash *= kFnvPrime64;
        }

        // 0은 Invalid임으로 보정한다
        return hash == 0 ? 1 : hash;
    }
};

// unordered_map 지원

/*
    맵 사용하려면 std::hash 필요함

    ResourceID 자체가 이미 Hash 결과
    -> Hash 재수행 없이 value 반환
*/

namespace std
{
template<>
struct hash<ResourceID>
{   
    // size_t는 32/64 비트 환경에 따라 컴파일러가 알아서 정수 크기 맞춰줌
    // 해시 버킷 index로 사용한다
    std::size_t operator()(const ResourceID& id)const noexcept{
        return static_cast<std::size_t>(id.value);
    }
};
} // namespace std
