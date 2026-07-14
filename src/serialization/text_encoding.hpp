#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace sailroute::serialization_detail {

struct DecodedCodePoint {
    std::uint32_t value;
    std::size_t length;
};

inline DecodedCodePoint decode_utf8(std::string_view text, std::size_t offset) noexcept {
    constexpr std::uint32_t replacement = 0xFFFD;
    const auto first = static_cast<unsigned char>(text[offset]);
    if (first < 0x80U) {
        return {first, 1};
    }

    std::size_t length = 0;
    std::uint32_t value = 0;
    std::uint32_t minimum = 0;
    if ((first & 0xE0U) == 0xC0U) {
        length = 2;
        value = first & 0x1FU;
        minimum = 0x80;
    } else if ((first & 0xF0U) == 0xE0U) {
        length = 3;
        value = first & 0x0FU;
        minimum = 0x800;
    } else if ((first & 0xF8U) == 0xF0U) {
        length = 4;
        value = first & 0x07U;
        minimum = 0x10000;
    } else {
        return {replacement, 1};
    }

    if (offset + length > text.size()) {
        return {replacement, 1};
    }
    for (std::size_t index = 1; index < length; ++index) {
        const auto byte = static_cast<unsigned char>(text[offset + index]);
        if ((byte & 0xC0U) != 0x80U) {
            return {replacement, 1};
        }
        value = (value << 6U) | (byte & 0x3FU);
    }

    if (value < minimum || value > 0x10FFFFU ||
        (value >= 0xD800U && value <= 0xDFFFU)) {
        return {replacement, 1};
    }
    return {value, length};
}

inline void append_utf8(std::string& output, std::uint32_t value) {
    if (value <= 0x7FU) {
        output.push_back(static_cast<char>(value));
    } else if (value <= 0x7FFU) {
        output.push_back(static_cast<char>(0xC0U | (value >> 6U)));
        output.push_back(static_cast<char>(0x80U | (value & 0x3FU)));
    } else if (value <= 0xFFFFU) {
        output.push_back(static_cast<char>(0xE0U | (value >> 12U)));
        output.push_back(static_cast<char>(0x80U | ((value >> 6U) & 0x3FU)));
        output.push_back(static_cast<char>(0x80U | (value & 0x3FU)));
    } else {
        output.push_back(static_cast<char>(0xF0U | (value >> 18U)));
        output.push_back(static_cast<char>(0x80U | ((value >> 12U) & 0x3FU)));
        output.push_back(static_cast<char>(0x80U | ((value >> 6U) & 0x3FU)));
        output.push_back(static_cast<char>(0x80U | (value & 0x3FU)));
    }
}

inline void append_json_hex_escape(std::string& output, std::uint32_t value) {
    constexpr char digits[] = "0123456789ABCDEF";
    output.append("\\u");
    output.push_back(digits[(value >> 12U) & 0xFU]);
    output.push_back(digits[(value >> 8U) & 0xFU]);
    output.push_back(digits[(value >> 4U) & 0xFU]);
    output.push_back(digits[value & 0xFU]);
}

inline void append_json_string(std::string& output, std::string_view text) {
    output.push_back('"');
    for (std::size_t offset = 0; offset < text.size();) {
        const DecodedCodePoint decoded = decode_utf8(text, offset);
        offset += decoded.length;
        switch (decoded.value) {
            case '"': output.append("\\\""); break;
            case '\\': output.append("\\\\"); break;
            case '\b': output.append("\\b"); break;
            case '\f': output.append("\\f"); break;
            case '\n': output.append("\\n"); break;
            case '\r': output.append("\\r"); break;
            case '\t': output.append("\\t"); break;
            default:
                if (decoded.value < 0x20U) {
                    append_json_hex_escape(output, decoded.value);
                } else {
                    append_utf8(output, decoded.value);
                }
                break;
        }
    }
    output.push_back('"');
}

inline bool is_xml_code_point(std::uint32_t value) noexcept {
    return value == 0x09U || value == 0x0AU || value == 0x0DU ||
           (value >= 0x20U && value <= 0xD7FFU) ||
           (value >= 0xE000U && value <= 0xFFFD) ||
           (value >= 0x10000U && value <= 0x10FFFFU);
}

inline void append_xml_text(std::string& output, std::string_view text) {
    constexpr std::uint32_t replacement = 0xFFFD;
    for (std::size_t offset = 0; offset < text.size();) {
        DecodedCodePoint decoded = decode_utf8(text, offset);
        offset += decoded.length;
        if (!is_xml_code_point(decoded.value)) {
            decoded.value = replacement;
        }
        switch (decoded.value) {
            case '&': output.append("&amp;"); break;
            case '<': output.append("&lt;"); break;
            case '>': output.append("&gt;"); break;
            case '"': output.append("&quot;"); break;
            case '\'': output.append("&apos;"); break;
            default: append_utf8(output, decoded.value); break;
        }
    }
}

}  // namespace sailroute::serialization_detail
