# RiskAssessment AI 코드 구조 정리

이 문서는 AI 관련 코드가 **런타임 / 라벨링 / 오프라인 / 학습**으로 어떻게 분리되어 있는지 정리합니다.

**런타임 추론 (RiskAssessment 실행 시 사용)**
- `sample_code/RiskAssessment/src/AI_model/AIInferencePipeline.cpp`
- `sample_code/RiskAssessment/src/AI_model/AIInferencePipeline.hpp`
- `sample_code/RiskAssessment/src/main_riskassessment.cpp`
  - `ThreadRASSAI()`에서 ONNX 로딩 및 추론 수행

**런타임 라벨링 (운영 중 자동 라벨 CSV 생성)**
- `sample_code/RiskAssessment/src/AI_model/AutoLabelWriter.cpp`
- `sample_code/RiskAssessment/src/AI_model/AutoLabelWriter.hpp`
- `sample_code/RiskAssessment/src/main_riskassessment.cpp`
  - `label.LabelWrite = true`일 때만 활성화

**오프라인 전처리 (학습용 CSV 파생 피처 생성)**
- `sample_code/RiskAssessment/tools/ai_offline/preprocess_autolabels.py`
  - 런타임에서 호출되지 않음
  - 학습 데이터 준비 단계에서만 사용

**오프라인 모델 테스트 (Python 권장)**
- `sample_code/RiskAssessment/tools/ai_offline/onnx_infer.py`
  - 현재 `risk_meta.json` 포맷(`label_cols`, `valid_classes`, `continuous_feature_cols`, `scaler_*`)과 일치
  - 입력 CSV에 없는 컬럼은 `NaN`으로 처리됨

**학습 코드**
- `sample_code/RiskAssessment/tools/ai_training/train_risk.py`
  - PyTorch 기반 학습 및 ONNX 내보내기

## 권장 사용 흐름
- 런타임 추론: `ThreadRASSAI()` 경로 사용
- 라벨링 데이터 생성: `label.LabelWrite=true`
- 오프라인 전처리: `preprocess_autolabels.py`
- 오프라인 검증: `onnx_infer.py`
- 학습: `train_risk.py`
