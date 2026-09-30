/**
 * @file trajectory_output_format_algorithm.cpp
 * @brief 关节轨迹 JSON、manifest、PLY、SHA-256。
 */

#include "trajectory_plan/algorithm/trajectory_output_format_algorithm.h"

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "trajectory_plan/algorithm/pose.h"

namespace openmind::trajectory_plan
{
namespace
{

uint32_t Rotr(uint32_t value, uint32_t bits)
{
    return (value >> bits) | (value << (32 - bits));
}

} // namespace

std::string TrajectoryOutputFormatAlgorithm::Sha256Hex(const std::string& content)
{
    static const uint32_t k[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
        0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
        0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
        0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
        0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
        0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
        0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t state[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                         0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::vector<uint8_t> data(content.begin(), content.end());
    const uint64_t bit_len = static_cast<uint64_t>(data.size()) * 8ull;
    data.push_back(0x80);
    while ((data.size() % 64) != 56)
    {
        data.push_back(0);
    }
    for (int32_t shift = 56; shift >= 0; shift -= 8)
    {
        data.push_back(static_cast<uint8_t>((bit_len >> shift) & 0xff));
    }
    for (size_t offset = 0; offset < data.size(); offset += 64)
    {
        uint32_t w[64] {};
        for (int32_t i = 0; i < 16; ++i)
        {
            w[i] = (static_cast<uint32_t>(data[offset + static_cast<size_t>(i) * 4]) << 24) |
                   (static_cast<uint32_t>(data[offset + static_cast<size_t>(i) * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(data[offset + static_cast<size_t>(i) * 4 + 2]) << 8) |
                   static_cast<uint32_t>(data[offset + static_cast<size_t>(i) * 4 + 3]);
        }
        for (int32_t i = 16; i < 64; ++i)
        {
            const uint32_t s0 = Rotr(w[i - 15], 7) ^ Rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = Rotr(w[i - 2], 17) ^ Rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
        uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
        for (int32_t i = 0; i < 64; ++i)
        {
            const uint32_t s1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
            const uint32_t ch = (e & f) ^ ((~e) & g);
            const uint32_t t1 = h + s1 + ch + k[i] + w[i];
            const uint32_t s0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t t2 = s0 + maj;
            h = g;
            g = f;
            f = e;
            e = d + t1;
            d = c;
            c = b;
            b = a;
            a = t1 + t2;
        }
        state[0] += a;
        state[1] += b;
        state[2] += c;
        state[3] += d;
        state[4] += e;
        state[5] += f;
        state[6] += g;
        state[7] += h;
    }
    std::ostringstream stream;
    stream << std::hex << std::setfill('0');
    for (uint32_t word : state)
    {
        stream << std::setw(8) << word;
    }
    return stream.str();
}

nlohmann::json TrajectoryOutputFormatAlgorithm::BuildJointTrajectoryDocument(const JointTrajectory& trajectory,
                                                                             const std::string& route,
                                                                             const std::string& request_id,
                                                                             const std::string& generation_id,
                                                                             const std::string& workpiece_type,
                                                                             double time_step_s)
{
    nlohmann::json document;
    document["schema"] = "aubo_joint_path/v1";
    // format / units / 速度归属是与现场行为树和第三流程共用的执行契约字段，
    // 消费端按它判断这是一条"只给位置、速度由行为树控制"的正式路径。
    document["format"] = "aubo_joint_path/v1";
    document["units"] = {{"positions", "rad"}, {"time", "s"}};
    document["speed_profile_included"] = false;
    document["speed_control_owner"] = "behavior_tree";
    document["route"] = route;
    document["request_id"] = request_id;
    document["generation_id"] = generation_id;
    document["workpiece_type"] = workpiece_type;
    document["dt_s"] = time_step_s;
    document["point_count"] = trajectory.PointCount();
    // 每点只有 6 个 rad 关节角，保留实际圈数，不按 360° 取模，也不混入角度制。
    nlohmann::json points = nlohmann::json::array();
    for (const JointRadians& q : trajectory.points)
    {
        nlohmann::json positions = nlohmann::json::array();
        for (size_t joint = 0; joint < 6; ++joint)
        {
            positions.push_back(q[joint]);
        }
        points.push_back({{"positions", positions}});
    }
    document["points"] = points;
    return document;
}

nlohmann::json TrajectoryOutputFormatAlgorithm::BuildManifestDocument(const ManifestInput& input,
                                                                      const CycleQuality& cycle)
{
    auto file_entry = [](const FileEntry& entry) {
        nlohmann::json node;
        node["path"] = entry.path;
        node["sha256"] = entry.sha256;
        node["point_count"] = entry.item_count;
        return node;
    };

    nlohmann::json document;
    document["schema"] = "trajectory_manifest/v2";
    // 执行契约：两条正式轨迹都是只给位置的 aubo_joint_path/v1，速度由行为树控制。
    document["execution_contract"] = "aubo_joint_path/v1";
    document["speed_profile_included"] = false;
    document["speed_control_owner"] = "behavior_tree";
    document["status"] = input.status;
    document["ready"] = input.ready;
    document["request_id"] = input.request_id;
    document["generation_id"] = input.generation_id;
    document["workpiece_type"] = input.workpiece_type;
    if (!input.reason.empty())
    {
        document["reason"] = input.reason;
    }
    // 所选抓取 rank 与放置 rank 一并绑定，便于回放定位到具体候选。
    document["selected_candidate_index"] = input.selected_candidate_index;
    document["selected_grasp_rank"] = input.selected_grasp_rank;
    document["selected_place_rank"] = input.selected_place_rank;
    document["grade"] = TrajectoryQualityGradeAlgorithm::GradeName(cycle.grade);
    document["photo_to_grasp_grade"] =
        TrajectoryQualityGradeAlgorithm::GradeName(cycle.photo_to_grasp.grade);
    document["grasp_to_place_grade"] =
        TrajectoryQualityGradeAlgorithm::GradeName(cycle.grasp_to_place.grade);
    document["total_score"] = cycle.total_score;
    document["photo_to_grasp"] = file_entry(input.photo_to_grasp);
    document["grasp_to_place"] = file_entry(input.grasp_to_place);
    // 点云也带哈希与点数，使消费端能验证它是否属于本轮。
    // 改动六：轨迹优先应答时点云尚未写出，此时 status="pending"，
    // sha256 与点数为空；后台写完后补发同代次 manifest，status="ready"。
    nlohmann::json point_cloud = file_entry(input.point_cloud);
    point_cloud["status"] = input.point_cloud_status;
    document["point_cloud"] = point_cloud;
    if (!input.collision_world.is_null())
    {
        document["collision_world"] = input.collision_world;
    }
    // 固定模型关联：与启动时加载的模型对应，只用机型名称不足以判断一致。
    document["collision_model"] = {
        {"model_id", input.collision_model_id},
        {"sha256", input.collision_model_sha256},
    };
    document["joint_link_check"] = {
        {"pass", input.joint_link_pass},
        {"max_joint_diff_deg", input.joint_link_max_diff_deg},
    };
    if (!input.grasp_preview.is_null())
    {
        document["grasp_preview"] = input.grasp_preview;
    }
    if (!input.statistics.is_null())
    {
        document["candidate_statistics"] = input.statistics;
    }
    return document;
}

std::string TrajectoryOutputFormatAlgorithm::BuildPlyBinary(
    const std::vector<Eigen::Vector3d>& points,
    const std::vector<std::array<uint8_t, 3>>& colors,
    const std::vector<std::string>& comments)
{
    static_assert(sizeof(float) == 4, "PLY 顶点按 float32 写出，本平台 float 必须是 4 字节");

    std::string header = "ply\nformat binary_little_endian 1.0\n";
    for (const std::string& comment : comments)
    {
        header += "comment " + comment + "\n";
    }
    header += "element vertex " + std::to_string(points.size()) + "\n";
    header += "property float x\nproperty float y\nproperty float z\n";
    header += "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n";

    // 每个顶点 3×float32 + 3×uchar = 15 字节，无填充。
    constexpr size_t kBytesPerVertex = 3 * sizeof(float) + 3;
    std::string body;
    body.resize(points.size() * kBytesPerVertex);
    char* cursor = body.data();
    for (size_t index = 0; index < points.size(); ++index)
    {
        const auto color =
            (index < colors.size()) ? colors[index] : std::array<uint8_t, 3>{180, 180, 180};
        const float xyz[3] = {static_cast<float>(points[index].x()),
                              static_cast<float>(points[index].y()),
                              static_cast<float>(points[index].z())};
        // 本项目只在小端 x86-64 上运行；大端平台需在此处反转字节序。
        std::memcpy(cursor, xyz, sizeof(xyz));
        cursor += sizeof(xyz);
        std::memcpy(cursor, color.data(), color.size());
        cursor += color.size();
    }
    return header + body;
}

} // namespace openmind::trajectory_plan
