/**
 * @file tiff_depth_camera.cpp
 * @brief 最小 TIFF 解码器实现：支持无压缩单通道 float32/uint16/uint8，逐行 strip。
 *
 * 现场深度图为 little-endian、未压缩、每行一条 strip 的 float32 TIFF（1280x720）。
 * 不依赖 libtiff；压缩（LZW/deflate 等）与不支持的采样格式明确报错。
 */

#include "trajectory_plan/camera/tiff_depth_camera.h"

#include <algorithm>
#include <cstring>

namespace openmind::trajectory_plan
{
namespace
{

constexpr uint16_t kTagImageWidth = 256;
constexpr uint16_t kTagImageLength = 257;
constexpr uint16_t kTagBitsPerSample = 258;
constexpr uint16_t kTagCompression = 259;
constexpr uint16_t kTagPhotometric = 262;
constexpr uint16_t kTagStripOffsets = 273;
constexpr uint16_t kTagSamplesPerPixel = 277;
constexpr uint16_t kTagRowsPerStrip = 278;
constexpr uint16_t kTagStripByteCounts = 279;
constexpr uint16_t kTagSampleFormat = 339;

constexpr uint16_t kTypeByte = 1;
constexpr uint16_t kTypeShort = 3;
constexpr uint16_t kTypeLong = 4;

constexpr uint16_t kSampleFormatUint = 1;
constexpr uint16_t kSampleFormatInt = 2;
constexpr uint16_t kSampleFormatFloat = 3;

/**
 * @brief 按字节序读取一个 16 位无符号整数。
 */
uint16_t ReadUint16(const uint8_t* data, bool little_endian)
{
    return little_endian ? static_cast<uint16_t>(data[0] | (data[1] << 8))
                         : static_cast<uint16_t>((data[0] << 8) | data[1]);
}

/**
 * @brief 按字节序读取一个 32 位无符号整数。
 */
uint32_t ReadUint32(const uint8_t* data, bool little_endian)
{
    if (little_endian)
    {
        return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
               (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
    }
    return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

/**
 * @brief 读取 IFD 字段的值或偏移（按 type 与 count 判断内联或外置）。
 */
uint32_t ReadTagValue(const uint8_t* bytes, size_t size, const uint8_t* entry, bool little_endian, bool& inline_value)
{
    const uint16_t type = ReadUint16(entry + 2, little_endian);
    const uint32_t count = ReadUint32(entry + 4, little_endian);
    const uint8_t* value_field = entry + 8;
    size_t type_size = 0;
    switch (type)
    {
        case kTypeByte:
            type_size = 1;
            break;
        case kTypeShort:
            type_size = 2;
            break;
        case kTypeLong:
            type_size = 4;
            break;
        default:
            type_size = 4;
            break;
    }
    const uint64_t total_bytes = static_cast<uint64_t>(type_size) * count;
    if (total_bytes <= 4)
    {
        inline_value = true;
        return ReadUint32(value_field, little_endian);
    }
    inline_value = false;
    const uint32_t offset = ReadUint32(value_field, little_endian);
    if (offset + total_bytes > size)
    {
        return 0;
    }
    return offset;
}

} // namespace

const std::string TiffDepthDecoder::kName = "tiff";

const std::string& TiffDepthDecoder::Name() const
{
    return kName;
}

bool TiffDepthDecoder::Decode(const std::vector<uint8_t>& bytes, DepthMap& map, std::string& error) const
{
    if (bytes.size() < 8)
    {
        error = "TIFF 文件过短";
        return false;
    }
    bool little_endian = false;
    if (bytes[0] == 'I' && bytes[1] == 'I')
    {
        little_endian = true;
    }
    else if (bytes[0] == 'M' && bytes[1] == 'M')
    {
        little_endian = false;
    }
    else
    {
        error = "不是 TIFF 文件（缺少 II/MM 字节序标记）";
        return false;
    }
    if (ReadUint16(bytes.data() + 2, little_endian) != 42)
    {
        error = "TIFF magic 不是 42";
        return false;
    }
    const uint32_t ifd_offset = ReadUint32(bytes.data() + 4, little_endian);
    if (ifd_offset + 2 > bytes.size())
    {
        error = "TIFF IFD 偏移越界";
        return false;
    }
    const uint16_t entry_count = ReadUint16(bytes.data() + ifd_offset, little_endian);

    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t bits_per_sample = 0;
    uint32_t compression = 1;
    uint32_t samples_per_pixel = 1;
    uint32_t sample_format = kSampleFormatUint;
    uint32_t strip_offsets_offset = 0;
    uint32_t strip_offsets_count = 0;
    size_t strip_offsets_entry_bytes = 4; ///< SHORT 型偏移表为 2 字节，其余为 4
    uint32_t strip_byte_counts_offset = 0;
    uint32_t strip_byte_counts_count = 0;
    size_t strip_byte_counts_entry_bytes = 4;
    uint32_t rows_per_strip = 0;

    for (uint16_t index = 0; index < entry_count; ++index)
    {
        const uint8_t* entry = bytes.data() + ifd_offset + 2 + static_cast<size_t>(index) * 12;
        if (ifd_offset + 2 + static_cast<size_t>(index + 1) * 12 > bytes.size())
        {
            error = "TIFF IFD 条目越界";
            return false;
        }
        const uint16_t tag = ReadUint16(entry, little_endian);
        const uint16_t type = ReadUint16(entry + 2, little_endian);
        const uint32_t count = ReadUint32(entry + 4, little_endian);
        bool inline_value = false;
        const uint32_t value = ReadTagValue(bytes.data(), bytes.size(), entry, little_endian, inline_value);
        switch (tag)
        {
            case kTagImageWidth:
                width = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            case kTagImageLength:
                height = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            case kTagBitsPerSample:
                bits_per_sample = inline_value ? (value & 0xFFFF)
                                               : ReadUint16(bytes.data() + value, little_endian);
                break;
            case kTagCompression:
                compression = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            case kTagStripOffsets:
                strip_offsets_offset = value;
                strip_offsets_count = count;
                strip_offsets_entry_bytes = (type == kTypeShort) ? 2u : 4u;
                break;
            case kTagSamplesPerPixel:
                samples_per_pixel = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            case kTagRowsPerStrip:
                rows_per_strip = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            case kTagStripByteCounts:
                strip_byte_counts_offset = value;
                strip_byte_counts_count = count;
                strip_byte_counts_entry_bytes = (type == kTypeShort) ? 2u : 4u;
                break;
            case kTagSampleFormat:
                sample_format = inline_value ? value : ReadUint32(bytes.data() + value, little_endian);
                break;
            default:
                break;
        }
    }

    if (width == 0 || height == 0)
    {
        error = "TIFF 缺少图像尺寸";
        return false;
    }
    if (compression != 1)
    {
        error = "TIFF 压缩格式不支持（仅支持无压缩）: compression=" + std::to_string(compression);
        return false;
    }
    if (samples_per_pixel != 1)
    {
        error = "TIFF 仅支持单通道深度图: samples_per_pixel=" + std::to_string(samples_per_pixel);
        return false;
    }
    if (bits_per_sample != 32 && bits_per_sample != 16 && bits_per_sample != 8)
    {
        error = "TIFF 采样位数不支持（支持 32/16/8）: " + std::to_string(bits_per_sample);
        return false;
    }
    if (strip_offsets_count == 0 || strip_offsets_count != strip_byte_counts_count)
    {
        error = "TIFF strip 偏移与字节数表不一致";
        return false;
    }
    if (rows_per_strip == 0)
    {
        rows_per_strip = height;
    }

    const size_t bytes_per_sample = bits_per_sample / 8;
    const size_t row_bytes = static_cast<size_t>(width) * bytes_per_sample;
    const size_t expected_rows = (height + rows_per_strip - 1) / rows_per_strip;
    if (expected_rows != strip_offsets_count)
    {
        error = "TIFF strip 行数与 rows_per_strip 不一致: rows=" + std::to_string(expected_rows) +
                " strips=" + std::to_string(strip_offsets_count);
        return false;
    }

    map.width = static_cast<int32_t>(width);
    map.height = static_cast<int32_t>(height);
    map.depth_mm.assign(static_cast<size_t>(width) * height, 0.0);

    const size_t offsets_entry_bytes = strip_offsets_entry_bytes;
    for (uint32_t strip = 0; strip < strip_offsets_count; ++strip)
    {
        const uint32_t data_offset =
            ReadUint32(bytes.data() + strip_offsets_offset + static_cast<size_t>(strip) * offsets_entry_bytes,
                       little_endian);
        const uint32_t data_bytes =
            (strip_byte_counts_entry_bytes == 2)
                ? ReadUint16(bytes.data() + strip_byte_counts_offset +
                                 static_cast<size_t>(strip) * strip_byte_counts_entry_bytes,
                             little_endian)
                : ReadUint32(bytes.data() + strip_byte_counts_offset +
                                 static_cast<size_t>(strip) * strip_byte_counts_entry_bytes,
                             little_endian);
        const uint32_t rows_in_strip = std::min(rows_per_strip, height - strip * rows_per_strip);
        const size_t expected_bytes = row_bytes * rows_in_strip;
        if (data_bytes != expected_bytes)
        {
            error = "TIFF strip 字节数与期望不符: " + std::to_string(data_bytes) +
                    " != " + std::to_string(expected_bytes);
            return false;
        }
        if (data_offset + data_bytes > bytes.size())
        {
            error = "TIFF strip 数据越界";
            return false;
        }
        const uint8_t* data = bytes.data() + data_offset;
        for (uint32_t row = 0; row < rows_in_strip; ++row)
        {
            const uint32_t global_row = strip * rows_per_strip + row;
            const uint8_t* row_data = data + static_cast<size_t>(row) * row_bytes;
            double* destination = map.depth_mm.data() + static_cast<size_t>(global_row) * width;
            for (uint32_t column = 0; column < width; ++column)
            {
                const uint8_t* sample = row_data + static_cast<size_t>(column) * bytes_per_sample;
                if (bits_per_sample == 32)
                {
                    uint32_t raw = ReadUint32(sample, little_endian);
                    float value = 0.0F;
                    std::memcpy(&value, &raw, sizeof(value));
                    if (sample_format == kSampleFormatFloat)
                    {
                        // float32 像素数值直接表示 mm。
                        destination[column] = static_cast<double>(value);
                    }
                    else
                    {
                        destination[column] = static_cast<double>(raw);
                    }
                }
                else if (bits_per_sample == 16)
                {
                    destination[column] = static_cast<double>(ReadUint16(sample, little_endian));
                }
                else
                {
                    destination[column] = static_cast<double>(sample[0]);
                }
            }
        }
    }
    return true;
}

} // namespace openmind::trajectory_plan
