#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <onnxruntime_cxx_api.h>

#include "../main_riskassessment.hpp"

struct AIRawRow {
    uint64_t frame_id = 0;
    uint64_t obs_timestamp_ms = 0;
    uint64_t obstacle_id = 0;
    int obstacle_class = 0;
    int stop_count = 0;

    double cuboid_x = 0.0;
    double cuboid_y = 0.0;
    double cuboid_z = 0.0;

    double obs_heading = 0.0;
    double obs_x = 0.0;
    double obs_y = 0.0;
    double obs_vx = 0.0;
    double obs_vy = 0.0;

    double ego_x = 0.0;
    double ego_y = 0.0;
    double ego_vx = 0.0;
    double ego_vy = 0.0;
    double ego_heading = 0.0;
    double ego_yaw_rate = 0.0;

    double dist_to_path = 0.0;
    double along_path_s = 0.0;
    double heading_vs_path = 0.0;
};

struct AIInferenceRow {
    std::uint64_t obstacle_id = 0;
    std::vector<float> probs;
};

class AIInputBuilder {
public:
    bool buildRawRows(
        uint64_t frame_id,
        const obstacleListVector& obstacle_list,
        const adcm::vehicleListStruct& ego,
        const std::vector<double>& path_x,
        const std::vector<double>& path_y,
        std::uint8_t edge_state,
        std::vector<AIRawRow>& out_rows,
        std::string* err
    ) const;
};

class OnnxRiskInfer {
public:
    bool init(const std::string& model_path,
              const std::string& meta_path,
              std::string* err);

    bool inferFromRawRows(const std::vector<AIRawRow>& rows,
                          std::vector<AIInferenceRow>& out_rows,
                          std::string* err);

    const std::vector<std::string>& labelCols() const { return label_cols_; }
    std::size_t featureDim() const { return feature_dim_; }

private:
    struct Meta {
        std::vector<std::string> label_cols;
        std::vector<int> valid_classes;
        std::vector<std::string> class_feature_cols;
        std::vector<std::string> nan_feature_cols;
        std::vector<std::string> continuous_feature_cols;
        std::vector<float> scaler_mean;
        std::vector<float> scaler_scale;
    } meta_;

    bool ready_ = false;
    std::size_t feature_dim_ = 0;
    std::vector<std::string> label_cols_;

    std::unique_ptr<Ort::Env> env_;
    std::unique_ptr<Ort::Session> session_;
    std::string input_name_;
    std::string output_name_;

    bool loadMeta(const std::string& meta_path, std::string* err);
    bool loadSession(const std::string& model_path, std::string* err);
};

class AIResultMapper {
public:
    AIResultMapper() = default;

    bool mapToRiskAssessment(const std::vector<AIInferenceRow>& rows,
                             const std::vector<std::string>& label_cols,
                             adcm::risk_assessment_Objects& out,
                             std::string* err) const;

private:
    int labelToHazardClass(const std::string& label) const;
};
