#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "../main_riskassessment.hpp"

class Config;

class AutoLabelWriter {
public:
    // 라벨 CSV 출력기를 초기화한다. (설정 기반으로 활성/비활성 결정)
    AutoLabelWriter(const Config& config, const std::string& csv_path);

    // 프레임 단위로 obstacle 피처 + 라벨을 CSV 한 행씩 기록
    void writeFrame(
        uint64_t frame_id,
        const obstacleListVector& obstacle_list,
        const adcm::vehicleListStruct& ego,
        const std::vector<double>& path_x,
        const std::vector<double>& path_y,
        const adcm::risk_assessment_Objects& risk,
        std::uint8_t edge_state
    );

private:
    // CSV 헤더 작성 및 출력 포맷 설정
    void writeHeader();

    std::ofstream ofs_;
    bool enabled_;
};
