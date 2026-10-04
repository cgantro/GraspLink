#pragma once

#include <cstdint>
#include <functional>
#include <string_view>

/** @brief FNV-1a 64-bit offset basis 상수. */
inline constexpr std::uint64_t kFnvOffsetBasis64 = 14695981039346656037ULL;

/** @brief FNV-1a 64-bit prime 상수. */
inline constexpr std::uint64_t kFnvPrime64 = 1099511628211ULL;

/**
 * @brief 문자열 기반 Asset 식별자를 64-bit 정수로 저장하는 lightweight handle.
 *
 * @details
 * AssetManager가 문자열 경로나 shared_ptr를 ECS Component에 직접 저장하지 않도록
 * ResourceID를 key로 사용한다. 문자열은 FNV-1a hash로 변환되며 value==0은 invalid를 의미한다.
 *
 * ResourceID = 리소스를 찾기 위한 Key
 * AssetManager = 실제 Mesh/Material/Texture의 Owner/Cache
 *
 * @note Hash collision 가능성은 이론적으로 존재한다. 현재 프로젝트 규모에서는 단순/빠른 FNV-1a를 사용한다.
 * @todo [FUTURE] 외부 asset database/serialization이 커지면 collision 검증 또는 stable asset GUID 도입을 검토한다.
 */
/*
 * [추가 용어 설명]
 * - Handle: 실제 객체를 직접 담지 않고 그 객체를 찾거나 가리키기 위한 작은 식별값.
 * - Hash: 문자열처럼 긴 입력을 일정 크기의 숫자로 바꾸는 함수 결과.
 * - FNV-1a: 구현이 단순하고 빠른 비암호학적 hash 알고리즘. 보안용 암호 hash가 아니다.
 * - Hash Collision: 서로 다른 문자열이 우연히 같은 hash 값을 만드는 경우.
 * - Sentinel: 특별한 상태를 표시하기 위해 예약한 값. 여기서는 value==0을 invalid로 예약한다.
 * - Cache Key: AssetManager의 unordered_map에서 리소스를 찾을 때 사용하는 key.
 * - GUID: 장기적으로 asset을 안정적으로 식별하기 위해 사용할 수 있는 별도 고유 식별자 방식.
 *
 * ResourceID는 GPU object ID가 아니다. AssetManager 내부에서 실제 Mesh/Material/Texture를 찾기 위한 프로젝트 측 ID다.
 */
struct ResourceID
{
    std::uint64_t value = 0;

    /** @brief invalid(value=0) ID를 생성한다. */
    constexpr ResourceID() = default;

    /**
     * @brief 이미 계산된 raw ID를 복원한다.
     * @param rawValue 저장/역직렬화된 64-bit 값.
     */
    constexpr explicit ResourceID(std::uint64_t rawValue)
        : value(rawValue)
    {
    }

    /**
     * @brief 문자열을 FNV-1a로 hash해 ID를 생성한다.
     * @param text resource path/name 문자열.
     */
    constexpr explicit ResourceID(std::string_view text)
        : value(Hash(text))
    {
    }

    /** @brief `if (id)` 형태로 validity를 검사할 수 있게 한다. */
    constexpr explicit operator bool() const noexcept { return value != 0; }

    /** @brief value가 invalid sentinel(0)이 아닌지 반환한다. */
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
    /**
     * @brief 문자열 byte를 FNV-1a 64-bit hash로 변환한다.
     *
     * @details
     * 매 byte마다 `hash ^= byte`, `hash *= prime` 순서로 계산한다.
     * char의 signedness가 플랫폼마다 다를 수 있으므로 unsigned char로 변환해
     * 상위 bit sign-extension에 의해 hash가 달라지는 문제를 피한다.
     */
    static constexpr std::uint64_t Hash(std::string_view text) noexcept
    {
        if (text.empty()) return 0;

        std::uint64_t hash = kFnvOffsetBasis64;

        for (const char c : text)
        {
            hash ^= static_cast<unsigned char>(c);
            hash *= kFnvPrime64;
        }

        // 0은 invalid sentinel로 예약했으므로 실제 hash가 0이면 1로 보정한다.
        return hash == 0 ? 1 : hash;
    }
};

namespace std
{
/**
 * @brief ResourceID를 unordered_map key로 사용할 수 있게 하는 std::hash specialization.
 *
 * ResourceID 자체가 이미 hash 결과이므로 추가 문자열 hash 없이 value를 size_t로 변환한다.
 */
template<>
struct hash<ResourceID>
{
    std::size_t operator()(const ResourceID& id) const noexcept
    {
        return static_cast<std::size_t>(id.value);
    }
};
} // namespace std
