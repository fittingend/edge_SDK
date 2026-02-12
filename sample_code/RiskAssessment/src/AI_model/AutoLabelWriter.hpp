#pragma once

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "../main_riskassessment.hpp"

class Config;

class AutoLabelWriter {
public:
    AutoLabelWriter(const Config& config, const std::string& csv_path);

    void writeFrame(
        uint64_t frame_id,
        const obstacleListVector& obstacle_list,
        const adcm::vehicleListStruct& ego,
        const std::vector<double>& path_x,
        const std::vector<double>& path_y,
        const adcm::risk_assessment_Objects& risk
    );

private:
    void writeHeader();

    std::ofstream ofs_;
    bool enabled_;
};
