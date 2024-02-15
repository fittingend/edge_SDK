# edge_sdk_r22-11

## SDK_240112
- EDGE 모듈간 인터페이스 정의 v1.6 기반 SDk release
    - map_data 데이터 타입 수정(map_2d vector type 변경)
    - risk_assessment 데이터 타입 수정(wgs84_xy_start, wgs84_xy_end vector type 변경)


## SDK_240110
- EDGE 모듈간 인터페이스 정의 v1.5 기반 SDk release
    - map_data 데이터 타입 수정(map_2d_location vector type 변경)
- 시뮬레이터 툴 추가
    - json packet 변경(json array -> json object)
    - DataFusion Test용 기능 추가(파일로 저장된 데이터셋 전송)

## SDK_231221
- EDGE 모듈간 인터페이스 정의 v1.4.2 기반 SDk release
    - build_path, map_data, risk_assessment, hub_data 데이터 타입 수정
    - build_path_test 서비스 추가

## SDK_231019
- EDGE 모듈간 인터페이스 정의 v1.3 기반 SDk release
    - work_order 서비스 제거
    - build_path 서비스 Method 추가
    - 변경된 data type 적용

## SDK_23911
- EDGE 모듈간 인터페이스 정의 v1.2 기반 SDk release
    - map_data 서비스 인터페이스의 road_z 데이터 타입 수정(float64 -> float64 array)

## SDK_23823
- EDGE 모듈간 인터페이스 정의 v1.1 기반 SDk release
    - build_path, hub_data 데이터 타입 수정

## SDK_230707
- EDGE 모듈간 인터페이스 정의 v0.9 기반 SDk release
    - 관제(ControlHub) / 데이터 융합(DataFusion) / 위험판단(RiseAssessment) / 위험회피알고리즘(RiskAvoidance) / 작업관리및경로생성(TaskManagerPathBuilder) 모듈 인터페이스 적용.
