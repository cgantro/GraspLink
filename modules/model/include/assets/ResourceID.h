#pragma once

#include <cstdint>
#include <functional>
#include <string_view>

inline constexpr std::uint64_t kFnvOffsetBasis64 = 14695981039346656037ULL;

inline constexpr std::uint64_t kFnvPrime64 = 1099511628211ULL;

/**
 * @brief 자원 문자열을 64-bit 값으로 바꿔 형상·재질·이미지 캐시에서 찾게 한다.
 * @details
 * GltfLoader는 파일 경로, 자원 종류, glTF 안 항목 번호를 이어 붙인 문자열을 만든다. 이미지 번호는 디코드된 이미지 번호가 아니라 glTF texture 항목 번호다.
 * 예를 들어 같은 파일의 첫 Mesh와 첫 재질은 종류 이름이 달라 다른 문자열이 된다. 해시는 문자열을 고정 길이 숫자로 요약하는 계산이며 ResourceID는 FNV-1a 64-bit 방식을 적용한다.
 * 같은 문자열은 같은 값을 내지만 다른 문자열이 같은 숫자로 겹치는 충돌 가능성을 확인하지 않으므로 암호학적 검증 용도로 쓸 수 없다.
 * 기본값과 빈 문자열의 해시 값은 0이며 캐시 키로 쓰지 않는다.
 */
struct ResourceID
{
    /// 문자열에서 계산한 값. 0은 캐시 조회에 쓸 수 없다.
    std::uint64_t value = 0;

    constexpr ResourceID() = default;

    /// 이미 계산한 값을 보관한다. rawValue가 0이면 쓸 수 없는 식별자다.
    constexpr explicit ResourceID(std::uint64_t rawValue)
        : value(rawValue)
    {
    }

    /// 문자열을 FNV-1a 64-bit로 해시한다. 빈 문자열은 0이 되어 캐시 키로 쓸 수 없다.
    constexpr explicit ResourceID(std::string_view text)
        : value(Hash(text))
    {
    }

    /// 값이 0이 아니면 캐시 조회에 사용할 수 있는 ID다. 실제 자원 존재 여부는 조회 결과로 확인한다.
    constexpr explicit operator bool() const noexcept { return value != 0; }

    /// 값이 0인지 확인해 캐시 키로 쓸 수 있는지 알려준다.
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

        // 0은 빈 문자열과 기본 ID에 예약되어 있으므로 해시 결과가 0이면 1로 바꾼다.
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
