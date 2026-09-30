/**
 * @file grasp_label_load_algorithm.cpp
 * @brief 读取离线 NPZ：ZIP + NPY，开口过滤后按分数降序。
 */

#include "trajectory_plan/algorithm/grasp_label_load_algorithm.h"

#include <zlib.h>

#include <algorithm>
#include <cstdint>
#include <map>
#include <cstring>
#include <fstream>
#include <sstream>

namespace openmind::trajectory_plan
{
namespace
{

constexpr uint32_t kZipLocalSignature = 0x04034b50u;
constexpr uint16_t kZipStored = 0;
constexpr uint16_t kZipDeflated = 8;

bool ReadFileBytes(const std::string& path, std::vector<uint8_t>& bytes, std::string& error)
{
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if (!stream.is_open())
    {
        error = "无法打开 NPZ: " + path;
        return false;
    }
    stream.seekg(0, std::ios::end);
    const std::streamoff size = stream.tellg();
    if (size < 0)
    {
        error = "NPZ 长度无效: " + path;
        return false;
    }
    stream.seekg(0, std::ios::beg);
    bytes.resize(static_cast<size_t>(size));
    if (size > 0)
    {
        stream.read(reinterpret_cast<char*>(bytes.data()), size);
        if (!stream)
        {
            error = "读取 NPZ 失败: " + path;
            return false;
        }
    }
    return true;
}

uint16_t ReadU16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0] | (data[1] << 8));
}

uint32_t ReadU32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
}

bool InflateRaw(const uint8_t* source, size_t source_size, std::vector<uint8_t>& output, std::string& error)
{
    z_stream stream {};
    stream.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(source));
    stream.avail_in = static_cast<uInt>(source_size);
    if (inflateInit2(&stream, -MAX_WBITS) != Z_OK)
    {
        error = "zlib inflateInit 失败";
        return false;
    }
    output.resize(std::max<size_t>(source_size * 4, 4096));
    int status = Z_OK;
    while (status != Z_STREAM_END)
    {
        if (stream.total_out >= output.size())
        {
            output.resize(output.size() * 2);
        }
        stream.next_out = reinterpret_cast<Bytef*>(output.data() + stream.total_out);
        stream.avail_out = static_cast<uInt>(output.size() - stream.total_out);
        status = inflate(&stream, Z_NO_FLUSH);
        if (status != Z_OK && status != Z_STREAM_END)
        {
            inflateEnd(&stream);
            error = "zlib inflate 失败";
            return false;
        }
    }
    output.resize(stream.total_out);
    inflateEnd(&stream);
    return true;
}

bool ParseNpyDoubleArray(const std::vector<uint8_t>& npy, std::vector<double>& values, std::vector<int64_t>& shape,
                         std::string& error)
{
    if (npy.size() < 10 || std::memcmp(npy.data(), "\x93NUMPY", 6) != 0)
    {
        error = "不是 NPY";
        return false;
    }
    const uint8_t major = npy[6];
    size_t header_len = 0;
    size_t header_offset = 0;
    if (major == 1)
    {
        header_len = static_cast<size_t>(npy[8] | (npy[9] << 8));
        header_offset = 10;
    }
    else if (major == 2)
    {
        header_len = static_cast<size_t>(ReadU32(npy.data() + 8));
        header_offset = 12;
    }
    else
    {
        error = "不支持的 NPY 版本";
        return false;
    }
    if (header_offset + header_len > npy.size())
    {
        error = "NPY 头损坏";
        return false;
    }
    const std::string header(reinterpret_cast<const char*>(npy.data() + header_offset), header_len);
    if (header.find("'descr': '<f8'") == std::string::npos && header.find("\"descr\": \"<f8\"") == std::string::npos &&
        header.find("'descr': '<f8'") == std::string::npos)
    {
        if (header.find("<f8") == std::string::npos)
        {
            error = "NPY 必须是 little-endian float64";
            return false;
        }
    }
    const size_t shape_pos = header.find("'shape':");
    if (shape_pos == std::string::npos)
    {
        error = "NPY 缺少 shape";
        return false;
    }
    const size_t open = header.find('(', shape_pos);
    const size_t close = header.find(')', open);
    if (open == std::string::npos || close == std::string::npos)
    {
        error = "NPY shape 无效";
        return false;
    }
    std::string shape_text = header.substr(open + 1, close - open - 1);
    std::replace(shape_text.begin(), shape_text.end(), ',', ' ');
    std::istringstream shape_stream(shape_text);
    shape.clear();
    int64_t axis = 0;
    while (shape_stream >> axis)
    {
        shape.push_back(axis);
    }
    int64_t count = 1;
    for (int64_t dim : shape)
    {
        count *= dim;
    }
    const size_t data_offset = header_offset + header_len;
    if (data_offset + static_cast<size_t>(count) * sizeof(double) > npy.size())
    {
        error = "NPY 数据长度不足";
        return false;
    }
    values.resize(static_cast<size_t>(count));
    std::memcpy(values.data(), npy.data() + data_offset, static_cast<size_t>(count) * sizeof(double));
    return true;
}

bool ExtractZipMembers(const std::vector<uint8_t>& zip_bytes,
                       std::map<std::string, std::vector<uint8_t>>& members,
                       std::string& error)
{
    size_t offset = 0;
    while (offset + 30 <= zip_bytes.size())
    {
        if (ReadU32(zip_bytes.data() + offset) != kZipLocalSignature)
        {
            break;
        }
        const uint16_t method = ReadU16(zip_bytes.data() + offset + 8);
        const uint32_t compressed_size = ReadU32(zip_bytes.data() + offset + 18);
        const uint32_t uncompressed_size = ReadU32(zip_bytes.data() + offset + 22);
        const uint16_t name_length = ReadU16(zip_bytes.data() + offset + 26);
        const uint16_t extra_length = ReadU16(zip_bytes.data() + offset + 28);
        const size_t name_offset = offset + 30;
        const size_t data_offset = name_offset + name_length + extra_length;
        if (data_offset + compressed_size > zip_bytes.size())
        {
            error = "ZIP 局部文件损坏";
            return false;
        }
        const std::string name(reinterpret_cast<const char*>(zip_bytes.data() + name_offset), name_length);
        std::vector<uint8_t> payload;
        if (method == kZipStored)
        {
            payload.assign(zip_bytes.begin() + static_cast<std::ptrdiff_t>(data_offset),
                           zip_bytes.begin() + static_cast<std::ptrdiff_t>(data_offset + compressed_size));
        }
        else if (method == kZipDeflated)
        {
            if (!InflateRaw(zip_bytes.data() + data_offset, compressed_size, payload, error))
            {
                return false;
            }
            if (uncompressed_size > 0 && payload.size() != uncompressed_size)
            {
                payload.resize(uncompressed_size);
            }
        }
        else
        {
            error = "NPZ 压缩方法不支持: " + name;
            return false;
        }
        members[name] = std::move(payload);
        offset = data_offset + compressed_size;
    }
    if (members.empty())
    {
        error = "NPZ 中没有数组成员";
        return false;
    }
    return true;
}

} // namespace

bool GraspLabelLoadAlgorithm::LoadLabels(const std::string& npz_path, double max_width_m, std::string& error)
{
    labels_.clear();
    raw_label_count_ = 0;
    width_filtered_count_ = 0;
    npz_path_ = npz_path;
    std::vector<uint8_t> zip_bytes;
    if (!ReadFileBytes(npz_path, zip_bytes, error))
    {
        return false;
    }
    std::map<std::string, std::vector<uint8_t>> members;
    if (!ExtractZipMembers(zip_bytes, members, error))
    {
        return false;
    }
    auto find_member = [&members](const std::string& key) -> const std::vector<uint8_t>* {
        for (const auto& item : members)
        {
            if (item.first == key || item.first == key + ".npy" ||
                item.first.size() >= key.size() + 4 &&
                    item.first.compare(item.first.size() - (key.size() + 4), key.size() + 4, key + ".npy") == 0)
            {
                return &item.second;
            }
        }
        return nullptr;
    };
    const std::vector<uint8_t>* translations_npy = find_member("translations");
    const std::vector<uint8_t>* rotations_npy = find_member("rotations");
    const std::vector<uint8_t>* widths_npy = find_member("widths");
    const std::vector<uint8_t>* scores_npy = find_member("scores");
    if (translations_npy == nullptr || rotations_npy == nullptr || widths_npy == nullptr || scores_npy == nullptr)
    {
        error = "NPZ 缺少 translations/rotations/widths/scores: " + npz_path;
        return false;
    }
    std::vector<double> translations;
    std::vector<double> rotations;
    std::vector<double> widths;
    std::vector<double> scores;
    std::vector<int64_t> t_shape;
    std::vector<int64_t> r_shape;
    std::vector<int64_t> w_shape;
    std::vector<int64_t> s_shape;
    if (!ParseNpyDoubleArray(*translations_npy, translations, t_shape, error) ||
        !ParseNpyDoubleArray(*rotations_npy, rotations, r_shape, error) ||
        !ParseNpyDoubleArray(*widths_npy, widths, w_shape, error) ||
        !ParseNpyDoubleArray(*scores_npy, scores, s_shape, error))
    {
        return false;
    }
    int64_t count = 0;
    if (!t_shape.empty())
    {
        count = t_shape[0];
    }
    if (count <= 0 || static_cast<int64_t>(scores.size()) < count || static_cast<int64_t>(widths.size()) < count)
    {
        error = "NPZ 数组长度不一致: " + npz_path;
        return false;
    }
    const bool rotation_flat = (rotations.size() == static_cast<size_t>(count) * 9);
    if (static_cast<int64_t>(translations.size()) < count * 3 || !rotation_flat)
    {
        error = "NPZ translations/rotations 形状无效: " + npz_path;
        return false;
    }
    raw_label_count_ = count;
    for (int64_t index = 0; index < count; ++index)
    {
        const double width = widths[static_cast<size_t>(index)];
        // 过滤 width <= 0 或 width > gripper_width_max_m 的标签。
        if (width <= 0.0 || (max_width_m > 0.0 && width > max_width_m))
        {
            ++width_filtered_count_;
            continue;
        }
        GraspLabel label;
        label.translation = Eigen::Vector3d(translations[static_cast<size_t>(index * 3 + 0)],
                                            translations[static_cast<size_t>(index * 3 + 1)],
                                            translations[static_cast<size_t>(index * 3 + 2)]);
        for (int32_t row = 0; row < 3; ++row)
        {
            for (int32_t column = 0; column < 3; ++column)
            {
                label.rotation(row, column) = rotations[static_cast<size_t>(index * 9 + row * 3 + column)];
            }
        }
        label.width = width;
        label.score = scores[static_cast<size_t>(index)];
        labels_.push_back(label);
    }
    std::stable_sort(labels_.begin(), labels_.end(),
                     [](const GraspLabel& left, const GraspLabel& right) { return left.score > right.score; });
    return true;
}

std::vector<GraspLabel> GraspLabelLoadAlgorithm::TopLabels(int64_t candidate_count) const
{
    if (candidate_count <= 0)
    {
        return {};
    }
    const size_t take = std::min(labels_.size(), static_cast<size_t>(candidate_count));
    return std::vector<GraspLabel>(labels_.begin(), labels_.begin() + static_cast<std::ptrdiff_t>(take));
}

int64_t GraspLabelLoadAlgorithm::EligibleCount() const
{
    return static_cast<int64_t>(labels_.size());
}

int64_t GraspLabelLoadAlgorithm::RawLabelCount() const
{
    return raw_label_count_;
}

int64_t GraspLabelLoadAlgorithm::WidthFilteredCount() const
{
    return width_filtered_count_;
}

const std::string& GraspLabelLoadAlgorithm::NpzPath() const
{
    return npz_path_;
}

} // namespace openmind::trajectory_plan
