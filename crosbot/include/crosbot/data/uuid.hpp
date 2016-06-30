/*
 * uuid.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_UUID_HPP_
#define CROSBOT_DATA_UUID_HPP_

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <string>

#include <crosbot/data/defines.hpp>

namespace crosbot {

/**
 * Universal Unique Identifier.
 * Primarily used to record unique identification markers in global maps.
 */
struct UUID {
public:
    uint64_t value1;
    uint64_t value2;

    UUID() : value1(0), value2(0) {}
    UUID(uint64_t value1, uint64_t value2) : value1(value1), value2(value2) {}
    UUID(std::string uuid) : value1(0), value2(0) {
        uint64_t v1, v2, v3, v4, v5;
#if __WORDSIZE == 64
        int n = sscanf(uuid.c_str(), "%lx-%lx-%lx-%lx-%lx",
#else
        int n = sscanf(uuid.c_str(), "%llx-%llx-%llx-%llx-%llx",
#endif
        &v1, &v2, &v3, &v4, &v5);

        if (n == 5) {
            value1 = ((v1 & 0xFFFFFFFFLU) << 32) | ((v2 & 0xFFFFLU) << 16) | (v3 & 0xFFFFLU);
            value2 = ((v4 & 0xFFFFLU) << 48) | (v5 & 0xFFFFFFFFFFFF);
        } else if (n >= 2) {
            value1 = v1;
            value2 = v2;
        } else if (n == 1) {
            value1 = v1;
        }
    }

    std::string toString() const {
        char uuidStr[256];

#if __WORDSIZE == 64
        sprintf(uuidStr, "%08lx-%04lx-%04lx-%04lx-%012lx",
#else
        sprintf(uuidStr, "%08llx-%04llx-%04llx-%04llx-%012llx",
#endif
                (value1 & 0xFFFFFFFF00000000) >> 32,
                (value1 & 0x00000000FFFF0000LU) >> 16, value1 & 0x000000000000FFFFLU,
                (value2 & 0xFFFF000000000000LU) >> 48, value2 & 0x0000FFFFFFFFFFFFLU);
        return std::string(uuidStr);
    }

    inline bool operator==(const UUID& other) const {
        return value1 == other.value1 && value2 == other.value2;
    }
};

} // namespace crosbot


#endif /* CROSBOT_DATA_UUID_HPP_ */
