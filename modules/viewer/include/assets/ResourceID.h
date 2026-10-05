#pragma once

#include <cstdint>
#include <functional>
#include <string_view>

inline constexpr std::uint64_t kFnvOffsetBasis64 = 14695981039346656037ULL;

inline constexpr std::uint64_t kFnvPrime64 = 1099511628211ULL;

/**
 * @brief AssetManager 캐시에서 Mesh·Material·Texture를 찾는 64-bit ID다.
 * @details
 * GltfLoader는 GLB 경로와 리소스 종류·번호를 문자열로 묶어 ID를 만든다. 예를 들어 같은 파일의
 * mesh 0과 material 0은 종류 문자열이 달라 서로 다른 키가 된다. 문자열은 FNV-1a 64-bit로 해시되며
 * 같은 입력 문자열은 항상 같은 ID가 된다. 암호학적 검증이나 충돌 검사는 하지 않으므로 서로 다른
 * 문자열이 우연히 같은 ID가 될 수 있다.
 * 기본 ID와 빈 문자열의 ID는 0이며 invalid로 취급한다.
 */
struct ResourceID
{
    /// 해시 값. 0은 리소스 조회에 사용할 수 없는 상태다.
    std::uint64_t value = 0;

    constexpr ResourceID() = default;

    /// 이미 계산한 값을 감싼다. rawValue가 0이면 invalid ID다.
    constexpr explicit ResourceID(std::uint64_t rawValue)
        : value(rawValue)
    {
    }

    /// 문자열을 FNV-1a 64-bit로 해시한다. 빈 문자열은 invalid ID가 된다.
    constexpr explicit ResourceID(std::string_view text)
        : value(Hash(text))
    {
    }

    /// ID가 유효한 캐시 키인지 확인한다.
    constexpr explicit operator bool() const noexcept { return value != 0; }

    /// ID가 유효한 캐시 키인지 확인한다.
    constexpr bool IsValid() const noexcept { return value != 0; }

    constexpr bool operator==(const ResourceID& other) const noexcept
    {
        return value == other.value;
    }

    constexpr bool operator!=(const ResourceID& other) const noexcept
    {
        return !(*this == other);
    }

    constexpr bool operator<(const ResourceID& other) const noexcept
    {
        return value < other.value;
    }

private:
    static constexpr std::uint64_t Hash(std::string_view text) noexcept
    {
        if (text.empty()) return 0;

        std::uint64_t hash = kFnvOffsetBasis64;

        for (const char c : text)
        {
            hash ^= static_cast<unsigned char>(c);
            hash *= kFnvPrime64;
        }

        // 0은 invalid ID이므로 해시 결과 0은 1로 바꾼다.
        return hash == 0 ? 1 : hash;
    }
};

namespace std
{
template<>
struct hash<ResourceID>
{
    std::size_t operator()(const ResourceID& id) const noexcept
    {
        return static_cast<std::size_t>(id.value);
    }
};
}
