#include "AIInferencePipeline.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace {
static inline double clamp(double v, double lo, double hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

static double normalizeDeg(double deg) {
    double v = std::fmod(deg, 360.0);
    if (v > 180.0) v -= 360.0;
    if (v < -180.0) v += 360.0;
    return v;
}

static bool computeDistAndAlongS(
    double px, double py,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    double& out_dist,
    double& out_along_s,
    double& out_path_heading_deg
) {
    if (path_x.size() < 2 || path_y.size() < 2 || path_x.size() != path_y.size()) {
        return false;
    }

    double best_d2 = std::numeric_limits<double>::max();
    double best_s = 0.0;
    double best_heading = 0.0;

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
            best_heading = std::atan2(vy, vx) * 180.0 / M_PI;
        }

        s_prefix += seg_len;
    }

    out_dist = std::sqrt(best_d2);
    out_along_s = best_s;
    out_path_heading_deg = best_heading;
    return true;
}

static float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}
}

bool AIInputBuilder::buildRawRows(
    uint64_t frame_id,
    const obstacleListVector& obstacle_list,
    const adcm::vehicleListStruct& ego,
    const std::vector<double>& path_x,
    const std::vector<double>& path_y,
    std::uint8_t edge_state,
    std::vector<AIRawRow>& out_rows,
    std::string* err
) const {
    (void)edge_state;
    out_rows.clear();
    out_rows.reserve(obstacle_list.size());

    for (const auto& obs : obstacle_list) {
        AIRawRow row{};
        row.frame_id = frame_id;
        row.obs_timestamp_ms = obs.timestamp;
        row.obstacle_id = obs.obstacle_id;
        row.obstacle_class = static_cast<int>(obs.obstacle_class);
        row.stop_count = static_cast<int>(obs.stop_count);

        row.cuboid_x = obs.fused_cuboid_x;
        row.cuboid_y = obs.fused_cuboid_y;
        row.cuboid_z = obs.fused_cuboid_z;

        row.obs_heading = obs.fused_heading_angle;
        row.obs_x = obs.fused_position_x;
        row.obs_y = obs.fused_position_y;
        row.obs_vx = obs.fused_velocity_x;
        row.obs_vy = obs.fused_velocity_y;

        row.ego_x = ego.position_x;
        row.ego_y = ego.position_y;
        row.ego_vx = ego.velocity_x;
        row.ego_vy = ego.velocity_y;
        row.ego_heading = ego.heading_angle;
        row.ego_yaw_rate = ego.velocity_ang;

        row.dist_to_path = std::numeric_limits<double>::quiet_NaN();
        row.along_path_s = std::numeric_limits<double>::quiet_NaN();
        double path_heading = std::numeric_limits<double>::quiet_NaN();
        if (computeDistAndAlongS(row.obs_x, row.obs_y, path_x, path_y,
                                 row.dist_to_path, row.along_path_s, path_heading)) {
            // ok
        }

        row.heading_vs_path = std::numeric_limits<double>::quiet_NaN();
        if (std::isfinite(row.obs_heading) && std::isfinite(path_heading)) {
            row.heading_vs_path = normalizeDeg(row.obs_heading - path_heading);
        }

        out_rows.push_back(row);
    }

    if (out_rows.empty()) {
        if (err) *err = "no obstacles to build features";
        return false;
    }

    return true;
}

bool OnnxRiskInfer::loadMeta(const std::string& meta_path, std::string* err) {
    std::ifstream ifs(meta_path);
    if (!ifs.is_open()) {
        if (err) *err = "Failed to open meta json: " + meta_path;
        return false;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    auto extract_array = [&](const std::string& key) -> std::string {
        const std::string needle = "\"" + key + "\"";
        size_t pos = content.find(needle);
        if (pos == std::string::npos) return "";
        pos = content.find('[', pos);
        if (pos == std::string::npos) return "";
        size_t end = content.find(']', pos);
        if (end == std::string::npos) return "";
        return content.substr(pos + 1, end - pos - 1);
    };

    auto parse_string_array = [&](const std::string& key, std::vector<std::string>& out) -> bool {
        std::string arr = extract_array(key);
        if (arr.empty()) return false;
        out.clear();
        size_t i = 0;
        while (i < arr.size()) {
            while (i < arr.size() && arr[i] != '"') i++;
            if (i >= arr.size()) break;
            size_t j = arr.find('"', i + 1);
            if (j == std::string::npos) break;
            out.push_back(arr.substr(i + 1, j - i - 1));
            i = j + 1;
        }
        return !out.empty();
    };

    auto parse_int_array = [&](const std::string& key, std::vector<int>& out) -> bool {
        std::string arr = extract_array(key);
        if (arr.empty()) return false;
        out.clear();
        std::stringstream ss(arr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                out.push_back(std::stoi(token));
            } catch (...) {
                // skip
            }
        }
        return !out.empty();
    };

    auto parse_float_array = [&](const std::string& key, std::vector<float>& out) -> bool {
        std::string arr = extract_array(key);
        if (arr.empty()) return false;
        out.clear();
        std::stringstream ss(arr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            try {
                out.push_back(std::stof(token));
            } catch (...) {
                // skip
            }
        }
        return !out.empty();
    };

    if (!parse_string_array("label_cols", meta_.label_cols) ||
        !parse_int_array("valid_classes", meta_.valid_classes) ||
        !parse_string_array("class_feature_cols", meta_.class_feature_cols) ||
        !parse_string_array("nan_feature_cols", meta_.nan_feature_cols) ||
        !parse_string_array("continuous_feature_cols", meta_.continuous_feature_cols) ||
        !parse_float_array("scaler_mean", meta_.scaler_mean) ||
        !parse_float_array("scaler_scale", meta_.scaler_scale)) {
        if (err) *err = "Meta json missing/invalid fields";
        return false;
    }

    if (meta_.continuous_feature_cols.size() != meta_.scaler_mean.size() ||
        meta_.continuous_feature_cols.size() != meta_.scaler_scale.size()) {
        if (err) *err = "Meta mismatch: continuous_feature_cols / scaler sizes";
        return false;
    }

    feature_dim_ = meta_.class_feature_cols.size() + meta_.nan_feature_cols.size() + meta_.continuous_feature_cols.size();
    label_cols_ = meta_.label_cols;

    return true;
}

bool OnnxRiskInfer::loadSession(const std::string& model_path, std::string* err) {
    try {
        if (!env_) {
            env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "risk_infer");
        }
        Ort::SessionOptions opts;
        opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        opts.SetIntraOpNumThreads(1);

        session_.reset();
        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), opts);

        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name_holder = session_->GetInputNameAllocated(0, allocator);
        auto output_name_holder = session_->GetOutputNameAllocated(0, allocator);
        input_name_ = input_name_holder.get();
        output_name_ = output_name_holder.get();
    } catch (const Ort::Exception& e) {
        if (err) *err = std::string("ONNX init failed: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        if (err) *err = std::string("ONNX init error: ") + e.what();
        return false;
    }
    return true;
}

bool OnnxRiskInfer::init(const std::string& model_path,
                         const std::string& meta_path,
                         std::string* err) {
    if (!loadMeta(meta_path, err)) {
        return false;
    }
    if (!loadSession(model_path, err)) {
        return false;
    }
    ready_ = true;
    return true;
}

bool OnnxRiskInfer::inferFromRawRows(const std::vector<AIRawRow>& rows,
                                     std::vector<AIInferenceRow>& out_rows,
                                     std::string* err) {
    if (!ready_) {
        if (err) *err = "OnnxRiskInfer not initialized";
        return false;
    }
    if (rows.empty()) {
        if (err) *err = "raw rows empty";
        return false;
    }

    const size_t n = rows.size();
    const size_t n_nan = meta_.nan_feature_cols.size();
    const size_t n_cont = meta_.continuous_feature_cols.size();

    auto get_continuous = [](const AIRawRow& r, const std::string& col) -> float {
        if (col == "stop_count") return (r.stop_count >= 1) ? 1.0f : 0.0f;
        if (col == "cuboid_x") return static_cast<float>(r.cuboid_x);
        if (col == "cuboid_y") return static_cast<float>(r.cuboid_y);
        if (col == "cuboid_z") return static_cast<float>(r.cuboid_z);
        if (col == "obs_x") return static_cast<float>(r.obs_x);
        if (col == "obs_y") return static_cast<float>(r.obs_y);
        if (col == "obs_vx") return static_cast<float>(r.obs_vx);
        if (col == "obs_vy") return static_cast<float>(r.obs_vy);
        if (col == "obs_speed") {
            return static_cast<float>(std::hypot(r.obs_vx, r.obs_vy));
        }
        if (col == "rel_x") return static_cast<float>(r.obs_x - r.ego_x);
        if (col == "rel_y") return static_cast<float>(r.obs_y - r.ego_y);
        if (col == "dist_to_ego") return static_cast<float>(std::hypot(r.obs_x - r.ego_x, r.obs_y - r.ego_y));
        if (col == "rel_vx") return static_cast<float>(r.obs_vx - r.ego_vx);
        if (col == "rel_vy") return static_cast<float>(r.obs_vy - r.ego_vy);
        if (col == "ego_speed") return static_cast<float>(std::hypot(r.ego_vx, r.ego_vy));
        if (col == "ego_yaw_rate") return static_cast<float>(r.ego_yaw_rate);
        if (col == "dist_to_path") return static_cast<float>(r.dist_to_path);
        if (col == "along_path_s") return static_cast<float>(r.along_path_s);
        if (col == "obs_heading_sin") return static_cast<float>(std::sin(r.obs_heading * M_PI / 180.0));
        if (col == "obs_heading_cos") return static_cast<float>(std::cos(r.obs_heading * M_PI / 180.0));
        if (col == "ego_heading_sin") return static_cast<float>(std::sin(r.ego_heading * M_PI / 180.0));
        if (col == "ego_heading_cos") return static_cast<float>(std::cos(r.ego_heading * M_PI / 180.0));
        if (col == "heading_vs_path_sin") return static_cast<float>(std::sin(r.heading_vs_path * M_PI / 180.0));
        if (col == "heading_vs_path_cos") return static_cast<float>(std::cos(r.heading_vs_path * M_PI / 180.0));
        return std::numeric_limits<float>::quiet_NaN();
    };

    std::vector<float> batch;
    batch.reserve(n * feature_dim_);

    for (const auto& r : rows) {
        int cls = r.obstacle_class;
        for (int v : meta_.valid_classes) {
            batch.push_back((cls == v) ? 1.0f : 0.0f);
        }

        std::vector<float> cont;
        cont.reserve(n_cont);
        std::vector<float> nan_mask;
        nan_mask.reserve(n_cont);

        for (const auto& col : meta_.continuous_feature_cols) {
            float val = get_continuous(r, col);
            if (std::isfinite(val)) {
                nan_mask.push_back(0.0f);
                cont.push_back(val);
            } else {
                nan_mask.push_back(1.0f);
                cont.push_back(0.0f);
            }
        }

        if (nan_mask.size() != n_nan) {
            if (err) *err = "nan_feature_cols size mismatch";
            return false;
        }

        for (float m : nan_mask) batch.push_back(m);

        for (size_t i = 0; i < cont.size(); ++i) {
            float denom = meta_.scaler_scale[i];
            if (denom == 0.0f) denom = 1.0f;
            float scaled = (cont[i] - meta_.scaler_mean[i]) / denom;
            batch.push_back(scaled);
        }
    }

    if (batch.size() != n * feature_dim_) {
        if (err) *err = "feature vector size mismatch";
        return false;
    }

    try {
        std::array<int64_t, 2> input_shape{static_cast<int64_t>(n), static_cast<int64_t>(feature_dim_)};
        Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            mem_info, batch.data(), batch.size(), input_shape.data(), input_shape.size());

        const char* input_names[] = {input_name_.c_str()};
        const char* output_names[] = {output_name_.c_str()};

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names, &input_tensor, 1,
            output_names, 1);

        float* logits = outputs[0].GetTensorMutableData<float>();
        const auto out_shape = outputs[0].GetTensorTypeAndShapeInfo().GetShape();
        if (out_shape.size() != 2 || static_cast<size_t>(out_shape[0]) != n || static_cast<size_t>(out_shape[1]) != meta_.label_cols.size()) {
            if (err) *err = "output shape mismatch";
            return false;
        }

        out_rows.clear();
        out_rows.reserve(n);
        for (size_t i = 0; i < n; ++i) {
            AIInferenceRow row_out;
            row_out.obstacle_id = rows[i].obstacle_id;
            row_out.probs.resize(meta_.label_cols.size());
            for (size_t j = 0; j < meta_.label_cols.size(); ++j) {
                float logit = logits[i * meta_.label_cols.size() + j];
                row_out.probs[j] = sigmoid(logit);
            }
            out_rows.push_back(std::move(row_out));
        }

    } catch (const Ort::Exception& e) {
        if (err) *err = std::string("ONNX inference failed: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        if (err) *err = std::string("Inference error: ") + e.what();
        return false;
    }

    return true;
}

int AIResultMapper::labelToHazardClass(const std::string& label) const {
    try {
        if (label.rfind("y_s", 0) == 0 || label.rfind("c_s", 0) == 0) {
            return std::stoi(label.substr(3));
        }
    } catch (...) {
        return -1;
    }
    return -1;
}

bool AIResultMapper::mapToRiskAssessment(const std::vector<AIInferenceRow>& rows,
                                         const std::vector<std::string>& label_cols,
                                         adcm::risk_assessment_Objects& out,
                                         std::string* err) const {
    out.riskAssessmentList.clear();

    if (label_cols.empty()) {
        if (err) *err = "label_cols is empty";
        return false;
    }

    for (const auto& row : rows) {
        if (row.probs.size() != label_cols.size()) {
            if (err) *err = "prob size mismatch";
            return false;
        }
        for (size_t i = 0; i < label_cols.size(); ++i) {
            const float prob = row.probs[i];
            const int hazard_class = labelToHazardClass(label_cols[i]);
            if (hazard_class <= 0) continue;

            adcm::riskAssessmentStruct r{};
            r.obstacle_id = row.obstacle_id;
            r.hazard_class = static_cast<std::uint8_t>(hazard_class);
            r.confidence = prob;
            r.hazard_path = true;
            out.riskAssessmentList.push_back(r);
        }
    }

    return true;
}
