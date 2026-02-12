// AutoLabelWriter.cpp
#include "AutoLabelWriter.hpp"

#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <cstring>
#include <cerrno>

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <limits.h>

#include "../config.hpp"


// ===== 보조 함수 =====
static inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}
static inline double wrapDeg(double deg) {
    // [-180, 180)
    while (deg >= 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
}
static inline double hypot2(double x, double y) {
    return std::sqrt(x*x + y*y);
}

static std::string getCwdString() {
    char buf[PATH_MAX] = {0};
    if (getcwd(buf, sizeof(buf)) == nullptr) {
        return ".";
    }
    return std::string(buf);
}

static bool ensureDirExists(const std::string& path) {
    if (path.empty() || path == ".") {
        return true;
    }
    struct stat st {};
    if (stat(path.c_str(), &st) == 0) {
        return S_ISDIR(st.st_mode);
    }

    // Create parent first
    const size_t slash = path.find_last_of('/');
    if (slash != std::string::npos) {
        const std::string parent = path.substr(0, slash);
        if (!parent.empty() && !ensureDirExists(parent)) {
            return false;
        }
    }

    if (mkdir(path.c_str(), 0755) == 0) {
        return true;
    }
    if (errno == EEXIST) {
        return true;
    }
    return false;
}

// path polyline에 대해 obstacle의 (최소거리, along_s) 계산
// - along_s: path 시작점부터 projection 지점까지 누적 거리 (path_xy 단위 그대로)
// - 반환 false: path가 너무 짧음
static bool computeDistAndAlongS(
    double px, double py,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    double& out_dist,
    double& out_along_s,
    double& out_path_heading_deg // projection된 segment의 heading(선택)
) {
    if (path_x.size() < 2 || path_y.size() < 2 || path_x.size() != path_y.size()) {
        return false;
    }

    double best_d2 = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double best_heading = 0.0;

    // prefix length
    double s_prefix = 0.0;
    for (size_t i = 0; i + 1 < path_x.size(); ++i) {
        const double x0 = path_x[i],     y0 = path_y[i];
        const double x1 = path_x[i + 1], y1 = path_y[i + 1];

        const double vx = x1 - x0;
        const double vy = y1 - y0;
        const double seg_len2 = vx*vx + vy*vy;
        const double seg_len = std::sqrt(seg_len2);

        if (seg_len2 < 1e-12) {
            continue;
        }

        // projection t in [0,1]
        const double wx = px - x0;
        const double wy = py - y0;
        double t = (wx*vx + wy*vy) / seg_len2;
        t = clamp(t, 0.0, 1.0);

        const double proj_x = x0 + t*vx;
        const double proj_y = y0 + t*vy;
        const double dx = px - proj_x;
        const double dy = py - proj_y;
        const double d2 = dx*dx + dy*dy;

        if (d2 < best_d2) {
            best_d2 = d2;
            best_s = s_prefix + t*seg_len;
            best_heading = std::atan2(vy, vx) * 180.0 / M_PI; // (x,y) map축 기준 heading
        }

        s_prefix += seg_len;
    }

    out_dist = std::sqrt(best_d2);
    out_along_s = best_s;
    out_path_heading_deg = best_heading;
    return true;
}

// hazard_class는 0(없음), 1~10(시나리오 번호)
static inline int labelIndexFromHazardClass(uint8_t hazard_class) {
    switch (hazard_class) {
        case 1:  return 0; // S1
        case 2:  return 1; // S2
        case 3:  return 2; // S3
        case 4:  return 3; // S4
        case 5:  return 4; // S5
        case 6:  return 5; // S6
        // 7,8 제외
        case 9:  return 6; // S9
        case 10: return 7; // S10
        default: return -1;
    }
}


AutoLabelWriter::AutoLabelWriter(const Config& config, const std::string& csv_path)
    : enabled_(false)
{
    (void)csv_path;
    const std::string cwd = getCwdString();
    adcm::Log::Info() << "AutoLabelWriter CWD: " << cwd;

    if (!config.labelWrite) {
        return;
    }

    const bool has_config_path = !config.labelOutputPath.empty();
    std::string output_path = has_config_path ? config.labelOutputPath : "";

    if (!has_config_path) {
        // Generate timestamped filename under CWD/log when no path is provided.
        const auto now = std::chrono::system_clock::now();
        const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#if defined(_WIN32)
        localtime_s(&tm_buf, &now_time);
#else
        localtime_r(&now_time, &tm_buf);
#endif
        std::ostringstream ts;
        ts << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        output_path = cwd + "/log/auto_labels_" + ts.str() + ".csv";
    } else if (!output_path.empty() && output_path[0] != '/') {
        output_path = cwd + "/" + output_path;
    }

    if (output_path.empty()) {
        adcm::Log::Error() << "Label output path is empty. AutoLabelWriter disabled.";
        return;
    }

    std::string parent_dir = ".";
    const size_t slash = output_path.find_last_of('/');
    if (slash != std::string::npos) {
        parent_dir = output_path.substr(0, slash);
        if (parent_dir.empty()) parent_dir = "/";
    }
    if (!ensureDirExists(parent_dir)) {
        adcm::Log::Error() << "Failed to create directory: " << parent_dir
                           << " (" << std::strerror(errno) << "). AutoLabelWriter disabled.";
        return;
    }

    ofs_.open(output_path, std::ios::out);
    if (!ofs_) {
        adcm::Log::Error() << "Failed to open csv: " << output_path
                           << ". AutoLabelWriter disabled.";
        return;
    }

    enabled_ = true;
    writeHeader();
}

void AutoLabelWriter::writeFrame(
    uint64_t frame_id,
    const obstacleListVector& obstacle_list,
    const adcm::vehicleListStruct& ego,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    const adcm::risk_assessment_Objects& risk
) {
    if (!enabled_) return;
    // 1) riskAssessmentList로부터 obstacle_id별 라벨(멀티) 생성
    struct Lbl {
        std::array<int, 8> y{};             // 0/1
        std::array<double, 8> conf{};       // confidence (없으면 0)
        Lbl() { y.fill(0); conf.fill(0.0); }
    };
    std::unordered_map<uint16_t, Lbl> lbl_map;

    for (const auto& r : risk.riskAssessmentList) {
        const int li = labelIndexFromHazardClass(r.hazard_class);
        if (li < 0) continue;
        auto& entry = lbl_map[r.obstacle_id];
        entry.y[li] = 1;
        // 같은 라벨이 여러 번 들어오면 confidence max로 저장
        entry.conf[li] = std::max(entry.conf[li], static_cast<double>(r.confidence));
    }

    // 2) obstacle_list를 순회하며 (피처 + 라벨) 한 행씩 기록
    const double ego_x = ego.position_x;
    const double ego_y = ego.position_y;
    const double ego_vx = ego.velocity_x;
    const double ego_vy = ego.velocity_y;
    const double ego_speed = hypot2(ego_vx, ego_vy);
    const double ego_heading = ego.heading_angle; // deg

    for (const auto& obs : obstacle_list) {
        const uint16_t oid = obs.obstacle_id;

        // ===== 피처 =====
        const double ox = obs.fused_position_x;
        const double oy = obs.fused_position_y;

        const double ovx = obs.fused_velocity_x;
        const double ovy = obs.fused_velocity_y;
        const double ospeed = hypot2(ovx, ovy);

        const double rel_x = ox - ego_x;
        const double rel_y = oy - ego_y;
        const double dist_ego = hypot2(rel_x, rel_y);

        const double rel_vx = ovx - ego_vx;
        const double rel_vy = ovy - ego_vy;

        const double heading_diff = wrapDeg(obs.fused_heading_angle - ego_heading);

        double dist_path = std::numeric_limits<double>::quiet_NaN();
        double along_s   = std::numeric_limits<double>::quiet_NaN();
        double path_heading = std::numeric_limits<double>::quiet_NaN();
        if (!computeDistAndAlongS(ox, oy, path_x, path_y, dist_path, along_s, path_heading)) {
            // path가 없으면 NaN 유지
        }
        const double heading_vs_path =
            std::isnan(path_heading) ? std::numeric_limits<double>::quiet_NaN()
                                     : wrapDeg(obs.fused_heading_angle - path_heading);

        // ===== 라벨 =====
        auto it = lbl_map.find(oid);
        std::array<int, 8> y{}; y.fill(0);
        std::array<double, 8> c{}; c.fill(0.0);
        if (it != lbl_map.end()) {
            y = it->second.y;
            c = it->second.conf;
        }

        // ===== CSV 출력 =====
        ofs_ << frame_id << ','
             << oid << ','
             << static_cast<int>(obs.obstacle_class) << ','
             << static_cast<int>(obs.stop_count) << ','
             << obs.fused_cuboid_x << ','
             << obs.fused_cuboid_y << ','
             << obs.fused_cuboid_z << ','
             << obs.fused_heading_angle << ','
             << ox << ',' << oy << ','
             << ovx << ',' << ovy << ','
             << ospeed << ','
             << rel_x << ',' << rel_y << ','
             << dist_ego << ','
             << rel_vx << ',' << rel_vy << ','
             << ego_speed << ','
             << ego.heading_angle << ','
             << ego.velocity_ang << ','
             << dist_path << ','
             << along_s << ','
             << heading_vs_path << ',';

        // labels 8개
        for (int i = 0; i < 8; ++i) {
            ofs_ << y[i] << (i == 7 ? ',' : ',');
        }
        // confidences 8개(옵션: 학습에 쓰기 싫으면 제거 가능)
        for (int i = 0; i < 8; ++i) {
            ofs_ << c[i];
            if (i != 7) ofs_ << ',';
        }
        ofs_ << '\n';
    }
}

void AutoLabelWriter::writeHeader() {
    ofs_ << "frame_id,obstacle_id,obstacle_class,stop_count,"
         << "cuboid_x,cuboid_y,cuboid_z,obs_heading,"
         << "obs_x,obs_y,obs_vx,obs_vy,obs_speed,"
         << "rel_x,rel_y,dist_to_ego,rel_vx,rel_vy,"
         << "ego_speed,ego_heading,ego_yaw_rate,"
         << "dist_to_path,along_path_s,heading_vs_path,"
         << "y_s1,y_s2,y_s3,y_s4,y_s5,y_s6,y_s9,y_s10,"
         << "c_s1,c_s2,c_s3,c_s4,c_s5,c_s6,c_s9,c_s10\n";
    ofs_ << std::fixed << std::setprecision(6);
}
