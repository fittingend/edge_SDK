<template>
  <SiteInfoLayer :data="workInfo" :vehicle-map-coordinates="vehicleMapCoordinates"></SiteInfoLayer>
  <div id="map" ref="map">
    <div v-if="isDebugging" class="debug-panel">
      <div class="debug-timestamp">{{ currentTimestamp }}</div>
      <div class="debug-timestamp">{{ currentDateTime }}</div>
      <div class="debug-status">
        <div class="status-label">EDGE Status:</div>
        <div class="status-value" :class="getStatusClass(edgeStatus)">{{ getStatusText(edgeStatus) }}</div>
      </div>
    </div>
    <v-btn class="debug_btn" :class="{ 'debug_on': isDebugging, 'debug_off': !isDebugging }" @click="toggleDebugging">디버깅</v-btn>
    <v-btn class="roadZ_btn" :class="{ 'roadZ_on': isroadZOn, 'roadZ_off': !isroadZOn }" @click="toggleRoadZBtn">road_z</v-btn>
    <v-btn v-if="workState === WorkState.WAIT" class="standby_btn" variant="outlined" :class="{ 'standby-active': missionStandby }" :disabled="missionStandby" @click="standbyMission">
      {{ missionStandby ? '임무 대기중' : '임무대기' }}
    </v-btn>
    <v-btn v-if="workState === WorkState.RUNNING || workState === WorkState.PAUSE" class="reset_btn" variant="outlined" @click="resetMission">초기화</v-btn>
    <v-btn v-if="workState === WorkState.RUNNING" class="stop_btn" variant="outlined" @click="stopMission">중단</v-btn>
    <v-btn v-if="workState === WorkState.PAUSE" class="resume_btn" variant="outlined" @click="resumeMission">재개</v-btn>
  </div>
  <!-- 차량 선택 모달 -->
  <ModalFrame ref="$vehicleSelectModal" :persistent="true">
    <template #dialog>
      <v-card class="pa-5" width="500">
        <v-card-title>차량을 선택해주세요</v-card-title>
        <v-card-text>
          <div v-if="availableVehicles.main.length > 0" class="mb-4">
            <div class="text-subtitle-2 mb-2">메인 차량</div>
            <template v-for="vehicleId in availableVehicles.main" :key="vehicleId">
              <v-checkbox v-model="selectedVehicles" :value="vehicleId" :label="vehicleId" density="compact" />
            </template>
          </div>
          <div v-if="availableVehicles.sub.length > 0">
            <div class="text-subtitle-2 mb-2">보조 차량</div>
            <template v-for="vehicleId in availableVehicles.sub" :key="vehicleId">
              <v-checkbox v-model="selectedVehicles" :value="vehicleId" :label="vehicleId" density="compact" />
            </template>
          </div>
        </v-card-text>
        <v-card-text v-if="selectedVehicles.length === 0" class="error--text">
          최소 1대 이상의 차량을 선택해주세요.
        </v-card-text>
        <v-card-actions class="justify-space-around mt-4">
          <v-btn variant="outlined" @click="closeVehicleSelectModal">취소</v-btn>
          <v-btn variant="outlined" :disabled="selectedVehicles.length === 0" @click="confirmVehicleSelection">
            확인 ({{selectedVehicles.length}}대 선택됨)
          </v-btn>
        </v-card-actions>
      </v-card>
    </template>
  </ModalFrame>

  <!-- 임무 시작 모달 -->
  <ModalFrame ref="$modalRef" :persistent="true">
    <template #dialog>
      <v-card class="align-self-center pa-5" width="auto">
        <v-card-title class="align-self-center">임무를 시작하시겠습니까?</v-card-title>
        <v-card-actions class="justify-space-around">
          <v-btn id="cancel_btn" variant="outlined" @click="closeModal">취소</v-btn>
          <v-btn id="action_btn" variant="outlined" @click="subscribe">시작</v-btn>
        </v-card-actions>
      </v-card>
    </template>
  </ModalFrame>
</template>

<script setup lang="ts">
import * as vMap from '@/composeables/vMapController';
import { computed, onMounted, onUnmounted, provide, Ref, ref, watchEffect } from 'vue';
import { Fill, Icon, Stroke, Style, Text } from 'ol/style';
import { fromLonLat, toLonLat } from 'ol/proj';
import { getMissionInfo, getWorkInfo } from '@/api';
import { getScaleForZoom, lineStyle } from '@/composeables/vMapController';
import { HubDataObject, IconSource, MapObstacle, Path, RoutePoint, SensorObstacle, VehicleMetaInfo, WorkInformation } from '@/constant/map/type';
import { Feature as OlFeature, Map as OlMap } from 'ol';
import { LineString as OlLineString, Point as OlPoint, Point, Polygon } from 'ol/geom';
import { useMapStore, useNatsStore } from "@/stores";
import { useWorkInfoStore, VehicleInfo, WorkInfo, WorkState } from '@/stores/workStore';
import { calculateCenter } from '@/composeables/calculateCenter';
import CircleStyle from 'ol/style/Circle';
import { Coordinate } from 'ol/coordinate';
import { EdgeStatus } from '@/constant/type';
import env from '@/env';
import { getRoadZColor } from '@/constant/colors';
import { isEmpty } from 'lodash';
import { messageBox } from '@/composeables/toastMessage';
// import { missionDataCollector } from '@/composeables/missionDataCollector'; // 데이터 수집 관련 모듈 주석 처리 (리포트)
import { MissionTableData } from '@/composeables/useTable';
import ModalFrame from '@/components/modal/ModalFrame.vue';
import munkres from 'munkres-js'
import OLVector from 'ol/layer/Vector.js';
import { Vector as OSVector } from 'ol/source';
import proj4 from 'proj4';
import router from '@/routes';
import { SiteInfoLayer } from '@/components/map';
import { storeToRefs } from 'pinia';
import { Subscription } from 'nats.ws';
import { useRoute } from 'vue-router';

// const { proxy }: any = getCurrentInstance();
const natsStore = useNatsStore();
const { natsConnection, stringCodec } = storeToRefs(natsStore);
const { workState } = storeToRefs(useWorkInfoStore());
const route = useRoute();
const map = ref("map");
let olMap: OlMap; // Map 객체 담을 변수 선언
let olLayer: OLVector<OSVector>; // layer위에 vector들을 그림
let osVector: OSVector; // 선언된 feature를 vector안에 넣음
let center: number[]; // 지도의 중앙값을 담을 변수 선언

// 작업 정보 관련
const { workInfo } = storeToRefs(useWorkInfoStore())

// 미션 정보 관련
const missionInfo = ref<MissionTableData>({
  missionId: "",
  name: "",
  siteId: {
    siteId: "", // 초기값 설정
    name: "",
    address: "",
    zone: "",
    coordinates: [], // 빈 배열로 초기화
    entranceCoord: [], // 빈 배열로 초기화
    exitCoord: [], // 빈 배열로 초기화
    img: "", // 빈 문자열로 초기화
  },
  latitude: 0,
  longitude: 0,
  registerDate: "",
});

// 모달 관련
const modalTriggered = ref(false);
const missionStandby = ref(false);  // 임무대기 상태를 관리하는 ref 추가
const $modalRef = ref();
const $vehicleSelectModal = ref();

// road_z 관련
const isroadZOn = ref(false);

// 디버깅 관련
const isDebugging = ref(false);
const currentTimestamp = ref('');
const currentDateTime = ref('');
const edgeStatus = ref<number | null>(null);
let timestampInterval: number | null = null;

// 타임스탬프 업데이트 함수
const updateTimestamp = (): void => {
  const now = new Date();
  const year = now.getFullYear();
  const month = String(now.getMonth() + 1).padStart(2, '0');
  const day = String(now.getDate()).padStart(2, '0');
  const hours = String(now.getHours()).padStart(2, '0');
  const minutes = String(now.getMinutes()).padStart(2, '0');
  const seconds = String(now.getSeconds()).padStart(2, '0');
  const milliseconds = String(now.getMilliseconds()).padStart(3, '0');
  
  currentTimestamp.value = Date.now().toString();
  currentDateTime.value = `${year}/${month}/${day} ${hours}:${minutes}:${seconds}.${milliseconds}`;
};

// EDGE status 관련 함수들
const getStatusText = (status: number | null): string => {
  if (status === null) return 'Not Connected';
  
  switch (status) {
    case EdgeStatus.IDLE:
      return 'Idle';
    case EdgeStatus.SEARCH:
      return 'Search';
    case EdgeStatus.RETURN:
      return 'Return';
    case EdgeStatus.MOVE:
      return 'Move';
    case EdgeStatus.WORK:
      return 'Work';
    default:
      return 'Unknown';
  }
};

const getStatusClass = (status: number | null): string => {
  if (status === null) return 'status-disconnected';
  
  switch (status) {
    case EdgeStatus.IDLE:
      return 'status-idle';
    case EdgeStatus.SEARCH:
      return 'status-search';
    case EdgeStatus.RETURN:
      return 'status-return';
    case EdgeStatus.MOVE:
      return 'status-move';
    case EdgeStatus.WORK:
      return 'status-work';
    default:
      return 'status-unknown';
  }
};

// EDGE status 구독
const subscribeControlStatus = async (): Promise<void> => {
  const statusSub = natsStore.subscribe(env.natsSubscribeInfo.controlStatus);
  if (statusSub) {
    (async () => {
      for await (const msg of statusSub) {
        try {
          const data = JSON.parse(natsStore.stringCodec.decode(msg.data));
          edgeStatus.value = data;
        } catch (error) {
          console.error('EDGE status 파싱 오류:', error);
        }
      }
    })();
  }
};

provide('missionTriggered', modalTriggered);
provide('missionStandby', missionStandby);  // 임무대기 상태 provide

const openModal = (): void => {
  $modalRef.value.onState();
}

const closeModal = (): void => {
  $modalRef.value.offState();
  modalTriggered.value = false;
}

// 개별 차량 선택 관련 상태
const selectedVehicles = ref<string[]>([]);

// 차량 크기 상수
const VEHICLE_DIMENSIONS = {
  MAIN: { length: 9250, width: 4985 },
  SUB: { length: 500, width: 750 }
};

// 사용 가능한 차량 목록 (workInfo에서 추출)
const availableVehicles = computed(() => {
  const mainVehicles: string[] = [];
  const subVehicles: string[] = [];
  
  Object.keys(workInfo.value.vehicleInfo).forEach(vehicleId => {
    if (vehicleId.includes('F')) {
      mainVehicles.push(vehicleId);
    } else {
      subVehicles.push(vehicleId);
    }
  });
  
  return { main: mainVehicles, sub: subVehicles };
});

// 선택된 총 차량 수 (더 이상 사용하지 않지만 호환성을 위해 유지)
const totalSelectedVehicles = computed(() => {
  return selectedVehicles.value.length;
});

interface ObstacleData {
  timestamp: number;
  obstacle_class: number;
  fused_position_x: number;
  fused_position_y: number;
  fused_position_z?: number;
  fused_cuboid_x?: number;
  fused_cuboid_y?: number;
  fused_cuboid_z?: number;
  fused_heading_angle?: number;
  fused_velocity_x?: number;
  fused_velocity_y?: number;
  fused_velocity_z?: number;
  obstacle_id?: number; // 부여된 ID
  map_2d_location: Point2D[];
  stop_count?: number;
  isMapCoordinate?: boolean; // 맵 좌표로 변환된 상태인지 표시
}

// const hubMap = new Map<string | number, ObstacleData[]>(); // vehicle_class별 장애물 리스트 (더 이상 사용 안함)
let previousObstacleList: ObstacleData[] = []; // 이전 주기의 장애물

// IDManager class to manage obstacle IDs
class IDManager {
  private idCounter: number;

  private idNotUsed: number[];

  private readonly OBSTACLE_MAX = 256;

  constructor() {
    // idCounter는 0부터 시작 (사용 가능한 ID 개수)
    this.idCounter = 0;
    // idNotUsed 배열을 [1, 2, 3, ..., 256]으로 초기화
    this.idNotUsed = Array.from({ length: this.OBSTACLE_MAX }, (_, i) => i + 1);
  }

  allocID(): number {
    // ID를 다 쓰면 자동으로 초기화 (순환 사용)
    if (this.idCounter >= this.OBSTACLE_MAX) {
      this.reset();
    }
    
    const allocatedId = this.idNotUsed[this.idCounter];
    this.idCounter += 1;
    return allocatedId;
  }

  retID(id: number): void {
    if (this.idCounter > 0 && id > 0 && id <= this.OBSTACLE_MAX) {
      this.idCounter -= 1;
      this.idNotUsed[this.idCounter] = id;
    }
  }

  // ID 매니저 초기화 (순환 사용을 위한 리셋)
  reset(): void {
    this.idCounter = 0;
    this.idNotUsed = Array.from({ length: this.OBSTACLE_MAX }, (_, i) => i + 1);
  }

  getNum(): number {
    return this.idCounter;
  }

  // 사용 중인 ID 목록 반환 (디버깅용)
  getUsedIds(): number[] {
    return this.idNotUsed.slice(0, this.idCounter);
  }
}

const idManager = new IDManager();

// 융합 데이터 구조
interface FusionData {
  vehicle: VehicleData;
  obstacle_list: ObstacleData[];
}

interface VehicleData {
  vehicle_class: string;
  timestamp: number;
  position_lat: number;
  position_long: number;
  position_height: number;
  position_x: number;
  position_y: number;
  position_z: number;
  yaw: number;
  roll: number;
  pitch: number;
  velocity_long: number;
  velocity_lat: number;
  velocity_x: number;
  velocity_y: number;
  velocity_ang: number;
  map_2d_location: Point2D[];
}

interface Point2D {
  x: number;
  y: number;
}

// 노면 데이터 관련 인터페이스
interface RoadZData {
  x: number; // WGS84 경도
  y: number; // WGS84 위도
  roadZ: number;
  timestamp: number;
  meterX?: number; // 미터 단위 X 좌표 (1m x 1m 집계용)
  meterY?: number; // 미터 단위 Y 좌표 (1m x 1m 집계용)
}

// 노면 데이터 저장소 (10cm x 10cm 셀)
const roadZDataStore: RoadZData[] = [];

// 0.1m x 0.1m 집계된 노면 데이터 저장소 (1/10 압축)
interface MeterGridData {
  x: number; // 0.1m x 0.1m 셀의 중심 X 좌표 (미터 단위)
  y: number; // 0.1m x 0.1m 셀의 중심 Y 좌표 (미터 단위)
  roadZ: number; // 집계된 road_z 값 (0-22)
  count: number; // 해당 0.1m x 0.1m 셀에 포함된 10cm x 10cm 셀 개수
  validCount: number; // 유효한 데이터가 있는 10cm x 10cm 셀 개수
  lastUpdated: number; // 마지막 업데이트 시간
}

const meterGridDataStore: MeterGridData[] = [];

// 메모리 최적화를 위한 상수
const MAX_ROAD_Z_DATA = 5000; // 최대 10cm 데이터 개수 (5천개로 대폭 감소)
const MAX_METER_GRID_DATA = 2000; // 최대 0.1m x 0.1m 데이터 개수 (2천개로 대폭 감소)
const DATA_CLEANUP_INTERVAL = 5000; // 5초마다 정리 (더 자주)

// 데이터 샘플링을 위한 상수
const SAMPLE_RATE = 3; // 3개당 1개 선택 (33% 샘플링)

// 노면 데이터 초기화 함수 (필요시 호출)
const clearRoadZData = (): void => {
  roadZDataStore.length = 0;
  meterGridDataStore.length = 0;
};

// 성능 데이터 인터페이스 (융합된 전체 시스템 데이터)
interface PerformanceData {
  pathGenerationTime?: number; // 경로생성 처리시간 (ms)
  mapGenerationTime?: number;  // 맵생성 처리시간 (ms)
  timestamp: number;
  source?: string; // 데이터 소스 (예: "F0", "1", "2" 등)
}

// 성능 데이터 저장소 (전체 시스템 융합 데이터)
const performanceDataStore: PerformanceData = {};

// 성능 데이터 수집 함수들 (전체 융합 데이터)
const collectPathPerformanceData = (data: any): void => {
  if (data.path_generation_time !== undefined) {
    performanceDataStore.pathGenerationTime = data.path_generation_time;
    performanceDataStore.timestamp = Date.now();
    performanceDataStore.source = data.vehicle_id || 'unknown';
    console.log(`📊 [collectPathPerformanceData] 경로생성 처리시간: ${data.path_generation_time}ms (소스: ${performanceDataStore.source})`);
  }
};

const collectMapPerformanceData = (data: any): void => {
  if (data.map_generation_time !== undefined) {
    performanceDataStore.mapGenerationTime = data.map_generation_time;
    performanceDataStore.timestamp = Date.now();
    performanceDataStore.source = data.vehicle_id || 'unknown';
    console.log(`📊 [collectMapPerformanceData] 맵생성 처리시간: ${data.map_generation_time}ms (소스: ${performanceDataStore.source})`);
  }
};

// 차량 맵 좌표 저장소
const vehicleMapCoordinates: { [key: string]: { x: number; y: number } } = {};

// 새로운 노면 데이터 정의에 맞춘 상수
const ROAD_Z_DEADZONE = 0.3; // 차량 주변 0.3M 데드존 (1/10 크기)
const ROAD_Z_GRID_SIZE = 1; // 10cm x 10cm 그리드
const ROAD_Z_CELL_SIZE = 1; // 각 셀의 크기 (10cm x 10cm)
const ROAD_Z_DEFAULT_VALUE = 255; // 기본값 (0xFF)

// 1m x 1m 셀 집계를 위한 상수 (1/10 압축 적용)
const METER_GRID_SIZE = 0.1; // 0.1m x 0.1m 그리드 (1/10 압축)
const CELLS_PER_METER = 10; // 1m당 10cm 셀 개수 (10개)

// 동적 차량 데이터 관리 (Map 사용)
interface VehicleDataStore {
  fusionData: FusionData;
  vehicle: VehicleData;
  obstacleList: ObstacleData[];
}

const vehicleDataMap = new Map<string, VehicleDataStore>();

// 실제 데이터가 들어온 차량 ID 추적
const activeVehicleIds = new Set<string>();

// 차량 데이터 가져오기 또는 생성
const getOrCreateVehicleData = (vehicleId: string): VehicleDataStore => {
  if (!vehicleDataMap.has(vehicleId)) {
    vehicleDataMap.set(vehicleId, {
      fusionData: { vehicle: {} as VehicleData, obstacle_list: [] },
      vehicle: {} as VehicleData,
      obstacleList: [],
    });
  }
  const store = vehicleDataMap.get(vehicleId);
  if (!store) {
    throw new Error(`차량 데이터를 가져올 수 없습니다: ${vehicleId}`);
  }
  return store;
};

// 맵 관련 변수
let originX = 278835; // 동적으로 계산됨 (기본값)
let originY = 3980050; // 동적으로 계산됨 (기본값)
const M_TO_10CM_PRECISION = 10; // 미터를 10cm 단위로 변환

// GPS to UTM 변환 함수
function GPStoUTM(lon: number, lat: number): { utmX: number, utmY: number } {
  // 상수 정의
  const WGS84_A = 6378137.0;
  const WGS84_E = 0.0818191908;
  const k0 = 0.9996;
  const eSq = WGS84_E * WGS84_E;
  const ePrimeSq = eSq / (1 - eSq);
  const DEG_TO_RAD = Math.PI / 180.0;

  // UTM Zone 설정 (Zone 52 고정)
  const zone = 52;
  const lonOrigin = (zone - 1) * 6 - 180 + 3; // 중앙 자오선
  const lonOriginRad = lonOrigin * DEG_TO_RAD;

  // 위도/경도 라디안 변환
  const latRad = lat * DEG_TO_RAD;
  const lonRad = lon * DEG_TO_RAD;

  // 삼각 함수 계산
  const sinLat = Math.sin(latRad);
  const cosLat = Math.cos(latRad);
  const tanLat = Math.tan(latRad);

  // 보조 항 계산
  const N = WGS84_A / Math.sqrt(1 - eSq * sinLat ** 2);
  const T = tanLat ** 2;
  const C = ePrimeSq * cosLat ** 2;
  const A = cosLat * (lonRad - lonOriginRad);

  // 자오선 거리 (Meridional Arc Length)
  const M = WGS84_A * (
    (1 - eSq / 4 - 3 * eSq ** 2 / 64 - 5 * eSq ** 3 / 256) * latRad -
    (3 * eSq / 8 + 3 * eSq ** 2 / 32 + 45 * eSq ** 3 / 1024) * Math.sin(2 * latRad) +
    (15 * eSq ** 2 / 256 + 45 * eSq ** 3 / 1024) * Math.sin(4 * latRad) -
    (35 * eSq ** 3 / 3072) * Math.sin(6 * latRad)
  );

  // UTM X 계산
  const utmX = k0 * N * (
    A + 
    (1 - T + C) * A ** 3 / 6 + 
    (5 - 18 * T + T ** 2 + 72 * C - 58 * ePrimeSq) * A ** 5 / 120
  ) + 500000.0;

  // UTM Y 계산
  let utmY = k0 * (
    M + 
    N * tanLat * (
      A ** 2 / 2 + 
      (5 - T + 9 * C + 4 * C ** 2) * A ** 4 / 24 + 
      (61 - 58 * T + T ** 2 + 600 * C - 330 * ePrimeSq) * A ** 6 / 720
    )
  );

  // 남반구 보정
  if (lat < 0) {
    utmY += 10000000.0;
  }

  return { utmX, utmY };
}

// 장애물 감지 최대 거리
const DETECT_DIST_LIMIT = 85;
// 장애물 매칭 거리 임계값 (10cm 단위)
const STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD = 50; // 정적 장애물: 0.5m = 50cm
const DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD = 100; // 동적 장애물: 1m = 100cm

// UTM Zone 52N에 대한 정의
const utmProjection = '+proj=utm +zone=52 +datum=WGS84 +units=m +no_defs';
const wgs84Projection = '+proj=longlat +datum=WGS84 +no_defs';

/**
 * 장애물의 상대 좌표를 WGS84 좌표로 변환
 * @param relativeX 장애물의 상대 X 좌표
 * @param relativeY 장애물의 상대 Y 좌표
 * @returns {lat: number, lon: number} WGS84 좌표
 */
// 장애물용 좌표 변환 (10cm 단위)
const convertObstaclePositionToWgs84 = (relativeX: number, relativeY: number): { lat: number, lon: number } => {
  // 10cm 단위를 미터 단위로 변환
  const relativeXinMeters = relativeX / 10;  // 10cm -> m 변환
  const relativeYinMeters = relativeY / 10;  // 10cm -> m 변환
  const utmX = relativeXinMeters + originX;
  const utmY = relativeYinMeters + originY;

  // UTM 좌표를 WGS84로 변환
  const [lon, lat] = proj4(utmProjection, wgs84Projection, [utmX, utmY]);
  return { lat, lon };
};

// 노면데이터용 좌표 변환 (미터 단위)
const convertRoadPositionToWgs84 = (relativeX: number, relativeY: number): { lat: number, lon: number } => {
  // 이미 미터 단위로 들어옴
  const utmX = relativeX + originX;
  const utmY = relativeY + originY;

  // UTM 좌표를 WGS84로 변환
  const [lon, lat] = proj4(utmProjection, wgs84Projection, [utmX, utmY]);
  return { lat, lon };
};


// 사각형 폴리곤 생성 함수 (간격 없애기 위해 약간 크게 생성)
const createRectanglePolygon = (centerX: number, centerY: number, size: number): number[][] => {
  const halfSize = size / 2 + 0.000001; // 약간 크게 만들어서 간격 없애기
  return [
    [centerX - halfSize, centerY - halfSize],
    [centerX + halfSize, centerY - halfSize],
    [centerX + halfSize, centerY + halfSize],
    [centerX - halfSize, centerY + halfSize],
    [centerX - halfSize, centerY - halfSize] // 닫기
  ];
};

// 0.1m x 0.1m 셀로 집계하는 함수 (누적 방식, 1/10 압축)
const aggregateToMeterGrid = (): void => {  
  // 기존 0.1m x 0.1m 데이터 초기화 (매번 전체 재집계)
  meterGridDataStore.length = 0;
  
  // 0.1m x 0.1m 그리드 맵 생성 (집계용)
  const meterGridMap = new Map<string, {
    roadZValues: number[];
    positions: { x: number; y: number }[];
    count: number;
    validCount: number;
  }>();
  
  // 10cm x 10cm 데이터를 0.1m x 0.1m로 집계 (중복 제거 및 최신 데이터 우선)
  const processedCells = new Set<string>(); // 이미 처리된 셀 추적
  
  roadZDataStore.forEach((data) => {
    // 미터 단위 좌표가 있는지 확인
    if (data.meterX === undefined || data.meterY === undefined) {
      return;
    }
    
    // 0.1m x 0.1m 셀의 중심 좌표 계산 (미터 단위)
    const meterX = Math.floor(data.meterX / METER_GRID_SIZE) * METER_GRID_SIZE + METER_GRID_SIZE / 2;
    const meterY = Math.floor(data.meterY / METER_GRID_SIZE) * METER_GRID_SIZE + METER_GRID_SIZE / 2;
    
    const gridKey = `${meterX}_${meterY}`;
    const cellKey = `${meterX}_${meterY}_${data.meterX}_${data.meterY}`; // 정확한 위치까지 포함
    
    // 이미 처리된 셀인지 확인 (같은 0.1m x 0.1m 셀 내에서 중복 방지)
    if (processedCells.has(cellKey)) {
      return;
    }
    processedCells.add(cellKey);
    
    if (!meterGridMap.has(gridKey)) {
      meterGridMap.set(gridKey, {
        roadZValues: [],
        positions: [],
        count: 0,
        validCount: 0
      });
    }
    
    const gridData = meterGridMap.get(gridKey);
    if (gridData) {
      gridData.roadZValues.push(data.roadZ);
      if (data.meterX !== undefined && data.meterY !== undefined) {
        gridData.positions.push({ x: data.meterX, y: data.meterY });
      }
      gridData.count += 1;
      
      // 유효한 데이터만 카운트 (255가 아닌 값)
      if (data.roadZ !== ROAD_Z_DEFAULT_VALUE) {
        gridData.validCount += 1;
      }
    }
  });
  
  // 집계된 데이터를 0.1m x 0.1m 저장소에 저장
  meterGridMap.forEach((gridData, gridKey) => {
    if (gridData.validCount === 0) return; // 유효한 데이터가 없으면 스킵
    
    // road_z 값들의 평균 계산 (가중평균 또는 최빈값 사용)
    const validRoadZValues = gridData.roadZValues.filter(z => z !== ROAD_Z_DEFAULT_VALUE);
    const averageRoadZ = validRoadZValues.length > 0 
      ? Math.round(validRoadZValues.reduce((sum, val) => sum + val, 0) / validRoadZValues.length)
      : ROAD_Z_DEFAULT_VALUE;
    
    // 0.1m x 0.1m 셀의 중심 좌표 (미터 단위)
    const [meterX, meterY] = gridKey.split('_').map(Number);
    
    meterGridDataStore.push({
      x: meterX,
      y: meterY,
      roadZ: averageRoadZ,
      count: gridData.count,
      validCount: gridData.validCount,
      lastUpdated: Date.now()
    });
  });
};

// 차량 위치 기반으로 road_z 데이터를 맵 좌표로 변환 (새로운 정의에 맞춤)
const processRoadZData = (vehicleData: HubDataObject): void => {
  if (!vehicleData.road_z || vehicleData.road_z.length === 0) {
    console.log('⚠️ [processRoadZData] road_z 데이터가 없거나 비어있음');
    return;
  }

  // 누적 방식: 기존 데이터 유지하고 새 데이터 추가
  // roadZDataStore.length = 0; // 주석 처리하여 누적 방식으로 변경

  // 차량의 GPS 좌표를 맵 좌표로 변환 (미터 단위)
  const { utmX: vehicleUtmX, utmY: vehicleUtmY } = GPStoUTM(vehicleData.position_long, vehicleData.position_lat);
  const vehicleMapX = vehicleUtmX - originX; // 미터 단위
  const vehicleMapY = vehicleUtmY - originY; // 미터 단위

  const vehiclePosition = {
    x: vehicleMapX,
    y: vehicleMapY
  };

  // 새로운 정의: 차량 전방 0.4M x 1M 영역 (데드존 0.3M 제외) - 1/10 크기
  // 4000개 데이터를 모두 사용하되, 표시 영역만 1/10로 압축
  const gridWidth = 40; // 4M / 0.1M = 40개 셀 (원본과 동일)
  const gridHeight = 100; // 10M / 0.1M = 100개 셀 (원본과 동일)
  const displayScale = 0.1; // 표시 크기를 1/10로 압축
  
  // 차량의 방향 (yaw)을 고려하여 그리드 방향 결정
  const vehicleYaw = vehicleData.yaw || 0;
  const yawRad = vehicleYaw * Math.PI / 180.0;
  
  let validDataCount = 0;
  let processedDataCount = 0;

  // 각 그리드 셀의 상대 위치 계산 (데이터 샘플링 적용)
  for (let row = 0; row < gridHeight; row += 1) {
    for (let col = 0; col < gridWidth; col += 1) {
      const index = row * gridWidth + col;
      if (index >= vehicleData.road_z.length) {
        // eslint-disable-next-line no-continue
        continue;
      }
      
      // 데이터 샘플링: 3개당 1개만 선택 (33% 샘플링)
      if (index % SAMPLE_RATE !== 0) {
        // eslint-disable-next-line no-continue
        continue;
      }
      
      const roadZ = vehicleData.road_z[index];
      processedDataCount += 1;
      
      // 기본값(255)인 경우 스킵
      if (roadZ === ROAD_Z_DEFAULT_VALUE) {
        // eslint-disable-next-line no-continue
        continue;
      }
      
      // 유효한 범주 값인지 확인 (0-22 범위)
      if (roadZ < 0 || roadZ > 22) {
        console.warn(`⚠️ [processRoadZData] 유효하지 않은 road_z 값: ${roadZ} (유효 범위: 0-22)`);
        // eslint-disable-next-line no-continue
        continue;
      }
      
      validDataCount += 1;
      
      // 그리드 셀의 상대 좌표 (차량 중심 기준, 10cm 단위)
      // col: 좌우 방향 (-2M ~ +2M), row: 전방 방향 (0M ~ 10M)
      // 표시할 때는 1/10 크기로 압축
      const relativeX = (col - gridWidth / 2) * ROAD_Z_GRID_SIZE * displayScale; // 좌우 방향 (1/10 압축)
      const relativeY = row * ROAD_Z_GRID_SIZE * displayScale; // 전방 방향 (1/10 압축)
      
      // 데드존 체크 (전방 0.3M 이내 제외, 1/10 압축 적용)
      if (relativeY < ROAD_Z_DEADZONE * displayScale) {
        // eslint-disable-next-line no-continue
        continue;
      }
      
      // 차량 방향을 고려한 좌표 변환
      const rotatedX = relativeX * Math.cos(yawRad) - relativeY * Math.sin(yawRad);
      const rotatedY = relativeX * Math.sin(yawRad) + relativeY * Math.cos(yawRad);
      
      // 상대 좌표를 절대 좌표로 변환 (미터 단위)
      const absoluteX = vehiclePosition.x + rotatedX;
      const absoluteY = vehiclePosition.y + rotatedY;
      
      // WGS84 좌표로 변환 (노면데이터용)
      const wgs84Coords = convertRoadPositionToWgs84(absoluteX, absoluteY);
      
      // 데이터 저장 (WGS84 좌표와 미터 좌표 모두 저장)
      roadZDataStore.push({
        x: wgs84Coords.lon,
        y: wgs84Coords.lat,
        roadZ,
        timestamp: Date.now(),
        // 1m x 1m 집계를 위한 미터 단위 좌표 추가
        meterX: absoluteX, // 이미 미터 단위
        meterY: absoluteY
      });
    }
  }
};

// 텔레컨스 사이트에만 간단한 그리드 효과 그리기 (고정된 선 개수)
const drawTeleconsGrid = (): void => {
  // 텔레컨스 사이트가 아닌 경우 그리드 그리지 않음
  if (workInfo.value.name !== '텔레컨스') {
    return;
  }

  // 이미 그리드가 그려져 있는지 확인
  const existingGrid = osVector.getFeatureById('grid_telecons');
  if (existingGrid) {
    return; // 이미 그려져 있으면 중복 그리기 방지
  }

  // 작업영역 경계 좌표 가져오기
  const boundaryCoords = workInfo.value.coordinates;
  if (!boundaryCoords || boundaryCoords.length < 3) {
    return;
  }

  // 경계 영역의 최소/최대 좌표 계산
  const lats = boundaryCoords.map(coord => coord[1]);
  const lons = boundaryCoords.map(coord => coord[0]);
  const minLat = Math.min(...lats);
  const maxLat = Math.max(...lats);
  const minLon = Math.min(...lons);
  const maxLon = Math.max(...lons);

  // 작업영역 경계선은 그리지 않음 (그리드만 표시)

  // 2. 작업영역의 방향(회전) 계산
  // 첫 번째와 두 번째 점을 연결하는 선의 각도 계산
  const dx = boundaryCoords[1][0] - boundaryCoords[0][0];
  const dy = boundaryCoords[1][1] - boundaryCoords[0][1];
  const rotationAngle = Math.atan2(dy, dx);
  
  // 3. 작업영역의 중심점 계산
  const centerX = boundaryCoords.reduce((sum, coord) => sum + coord[0], 0) / boundaryCoords.length;
  const centerY = boundaryCoords.reduce((sum, coord) => sum + coord[1], 0) / boundaryCoords.length;

  // 4. 작업영역의 크기 계산 (회전된 좌표계 기준)
  // 경계의 최소/최대 좌표를 회전된 좌표계로 변환
  const rotatedCoords = boundaryCoords.map(coord => {
    const x = coord[0] - centerX;
    const y = coord[1] - centerY;
    const rotatedX = x * Math.cos(-rotationAngle) - y * Math.sin(-rotationAngle);
    const rotatedY = x * Math.sin(-rotationAngle) + y * Math.cos(-rotationAngle);
    return [rotatedX, rotatedY];
  });
  
  const rotatedMinX = Math.min(...rotatedCoords.map(coord => coord[0]));
  const rotatedMaxX = Math.max(...rotatedCoords.map(coord => coord[0]));
  const rotatedMinY = Math.min(...rotatedCoords.map(coord => coord[1]));
  const rotatedMaxY = Math.max(...rotatedCoords.map(coord => coord[1]));

  // 5. 그리드 셀을 사각형으로 생성 (장애물 위치에 따른 색상 변경 가능)
  const verticalSpacing = (rotatedMaxX - rotatedMinX) / 20;
  const horizontalSpacing = (rotatedMaxY - rotatedMinY) / 10;
  
  // 각 그리드 셀을 사각형으로 생성
  for (let row = 0; row < 10; row += 1) {
    for (let col = 0; col < 20; col += 1) {
      const rotatedX1 = rotatedMinX + (verticalSpacing * col);
      const rotatedX2 = rotatedMinX + (verticalSpacing * (col + 1));
      const rotatedY1 = rotatedMinY + (horizontalSpacing * row);
      const rotatedY2 = rotatedMinY + (horizontalSpacing * (row + 1));
      
      // 회전된 좌표를 실제 좌표로 변환 (4개 꼭지점)
      const corners = [
        [rotatedX1, rotatedY1], [rotatedX2, rotatedY1],
        [rotatedX2, rotatedY2], [rotatedX1, rotatedY2]
      ].map(([rx, ry]) => [
        centerX + rx * Math.cos(rotationAngle) - ry * Math.sin(rotationAngle),
        centerY + rx * Math.sin(rotationAngle) + ry * Math.cos(rotationAngle)
      ]);
      
             const gridCell = new OlFeature({
         geometry: new Polygon([corners]),
       });
      
      const gridId = `grid_cell_${row}_${col}`;
      gridCell.setId(gridId);
      
      // 기본 그리드 셀 스타일
      const gridStyle = new Style({
        fill: new Fill({
          color: 'rgba(240, 208, 146, 0.1)', // 매우 연한 베이지색
        }),
        stroke: new Stroke({
          color: '#F0D092', // 베이지색 테두리
          width: 1,
        }),
      });
      
             gridCell.setStyle(gridStyle);
       osVector.addFeature(gridCell);
     }
   }
 };

// roadz 데이터 표시 제거 함수
const removeRoadZFeatures = (): void => {
  const allFeatures = osVector.getFeatures();
  const roadZFeaturesToRemove: OlFeature[] = [];
  
  allFeatures.forEach((feature) => {
    const featureId = feature.getId() as string;
    if (featureId && featureId.startsWith('metergrid_')) {
      roadZFeaturesToRemove.push(feature);
    }
  });
  
  roadZFeaturesToRemove.forEach((feature) => {
    osVector.removeFeature(feature);
  });
};

// 0.1m x 0.1m 셀 노면 heatmap 그리기 (1/10 압축)
const drawMeterGridHeatmap = (): void => {
  if (meterGridDataStore.length === 0) {
    console.log('⚠️ [drawMeterGridHeatmap] 0.1m x 0.1m 데이터가 없음');
    return;
  }
  
  // 기존 0.1m x 0.1m 노면 데이터 제거
  removeRoadZFeatures();
  
  // 새로운 0.1m x 0.1m 노면 데이터 그리기
  meterGridDataStore.forEach((data, index) => {
    // 0.1m x 0.1m 셀을 WGS84 좌표로 변환 (노면데이터용)
    const wgs84Coords = convertRoadPositionToWgs84(data.x, data.y);
    
    // 사각형 폴리곤 생성 (0.1m x 0.1m 셀)
    // 0.1m를 도 단위로 변환 (대략 0.1/111000도)
    const polygonCoords = createRectanglePolygon(wgs84Coords.lon, wgs84Coords.lat, METER_GRID_SIZE / 111000);
    const meterGridPolygon = new OlFeature({
      geometry: new Polygon([polygonCoords.map(coord => fromLonLat(coord))]),
    });
    
    const featureId = `metergrid_${Date.now()}_${index}`;
    meterGridPolygon.setId(featureId);
    
    // 색상 계산 및 스타일 적용 (road_z는 0~22 범위의 범주 값)
    const color = getRoadZColor(data.roadZ);
    
    const meterGridStyle = new Style({
      fill: new Fill({ 
        color 
      }),
      // stroke 제거하여 블럭같은 느낌 없애기
    });
    
    meterGridPolygon.setStyle(meterGridStyle);
    osVector.addFeature(meterGridPolygon);
  });
};

// 장애물 데이터 필터링 (차량 데이터 제거)
function filterVehicleData(obstacles: ObstacleData[]): void {
  for (let i = obstacles.length - 1; i >= 0; i -= 1) {
    if (obstacles[i].obstacle_class === 51) {
      obstacles.splice(i, 1); // 직접 제거 (C++의 erase와 동일)
    } else if (obstacles[i].obstacle_class === 50) {
      obstacles.splice(i, 1); // 직접 제거 (C++의 erase와 동일)
    }
  }
}

// 차량 좌표계 변환
function gpsToMapCoordinate(inputVehicle: VehicleData): void {
  const vehicle = inputVehicle; // 참조 복사
  const positionX = vehicle.position_long;
  const positionY = vehicle.position_lat;
  const { utmX: vehUtmX, utmY: vehUtmY } = GPStoUTM(positionX, positionY);
        
  const beforeScalingX = vehUtmX - originX;
  const beforeScalingY = vehUtmY - originY;
    
  vehicle.position_x = beforeScalingX * M_TO_10CM_PRECISION;
  vehicle.position_y = beforeScalingY * M_TO_10CM_PRECISION;
    
  vehicle.velocity_x = vehicle.velocity_long * M_TO_10CM_PRECISION;
  vehicle.velocity_y = vehicle.velocity_lat * M_TO_10CM_PRECISION;
}

// 장애물 상대좌표를 맵좌표로 변환
function relativeToMapCoordinate(inputObstacleList: ObstacleData[], vehicle: VehicleData): void {
  const obstacleList = inputObstacleList; // 참조 복사
  const theta = vehicle.yaw * Math.PI / 180.0;
  
  // const velocityAng = vehicle.velocity_ang;  
  obstacleList.forEach((inputObstacle) => {
    const obstacle = inputObstacle; // 참조 복사
    
    // 이미 맵 좌표로 변환된 장애물은 스킵
    if (obstacle.isMapCoordinate) {
      return;
    }
    
    const obstaclePositionX = obstacle.fused_position_x;
    const obstaclePositionY = obstacle.fused_position_y;
    const obstacleVelocityX = obstacle.fused_velocity_x || 0;
    const obstacleVelocityY = obstacle.fused_velocity_y || 0;
  
    const rotatedObstacleX =  -obstaclePositionX * Math.sin(theta) - obstaclePositionY * Math.cos(theta);
    const rotatedObstacleY = obstaclePositionX * Math.cos(theta) - obstaclePositionY * Math.sin(theta);

    const finalX = vehicle.position_x + rotatedObstacleX * M_TO_10CM_PRECISION;
    const finalY = vehicle.position_y + rotatedObstacleY * M_TO_10CM_PRECISION;

    obstacle.fused_position_x = finalX;
    obstacle.fused_position_y = finalY;
    obstacle.fused_velocity_x = obstacleVelocityX + vehicle.velocity_x;
    obstacle.fused_velocity_y = obstacleVelocityY + vehicle.velocity_y;
    obstacle.fused_heading_angle = vehicle.yaw + (obstacle.fused_heading_angle || 0);
    
    // *** 중요: 맵 좌표로 변환 완료 표시 ***
    obstacle.isMapCoordinate = true;
  });
}

// 차량 데이터 처리
function processVehicleData(vehicleData: FusionData, inputVehicle: VehicleData, inputObstacleList: ObstacleData[], vehicleId: string): void {
  const vehicle = inputVehicle; // 참조 복사
  const obstacleList = inputObstacleList; // 참조 복사
  
  Object.assign(vehicle, vehicleData.vehicle);

  obstacleList.length = 0;
  vehicleData.obstacle_list.forEach(obs => {
    obstacleList.push(obs);
  });
  
  filterVehicleData(obstacleList);
  gpsToMapCoordinate(vehicle);
  relativeToMapCoordinate(obstacleList, vehicle);
  
  // 차량의 맵 좌표 저장
  vehicleMapCoordinates[vehicleId] = {
    x: vehicle.position_x,
    y: vehicle.position_y // y 좌표는 - 처리를 해야 실제 맵좌표(?)
  };
}

// 유클리디안 거리 계산
function euclideanDistance(a: ObstacleData, b: ObstacleData): number {
  const dx = a.fused_position_x - b.fused_position_x;
  const dy = a.fused_position_y - b.fused_position_y;
  return Math.sqrt(dx * dx + dy * dy);
}

// 정적 장애물인지 확인 (클래스 30, 40)
function isStaticObstacle(obstacleClass: number): boolean {
  return obstacleClass === 30 || obstacleClass === 40;
}

// 정적 장애물 위치 히스토리 인터페이스
interface StaticObstacleHistory {
  obstacle: ObstacleData;
  positionHistory: Array<{ x: number; y: number }>; // 최대 10프레임 (x, y만)
}

// 동적 장애물 트래커: 5프레임 매칭 안되면 제거
class DynamicObstacleTracker {
  private trackedObstacles: Map<number, { obstacle: ObstacleData; unmatchedFrames: number }> = new Map();

  private readonly DISTANCE_THRESHOLD = 100; // 1m = 100cm

  private readonly MAX_UNMATCHED_FRAMES = 5; // 5프레임

  track(newList: ObstacleData[]): ObstacleData[] {
    const trackedList: ObstacleData[] = [];
    const matchedIds = new Set<number>();

    // 현재 프레임의 동적 장애물과 기존 트래킹 중인 장애물 매칭
    for (const currentObstacle of newList) {
      if (isStaticObstacle(currentObstacle.obstacle_class)) {
        // eslint-disable-next-line no-continue
        continue; // 정적 장애물은 제외
      }

      let matched = false;
      let bestMatchId: number | null = null;
      let bestDistance = Infinity;

      // 기존 트래킹 중인 장애물과 거리 비교
      for (const [id, tracked] of this.trackedObstacles) {
        if (matchedIds.has(id)) {
          // eslint-disable-next-line no-continue
          continue;
        }

        const distance = euclideanDistance(currentObstacle, tracked.obstacle);

        if (distance < this.DISTANCE_THRESHOLD && distance < bestDistance) {
          bestMatchId = id;
          bestDistance = distance;
          matched = true;
        }
      }

      if (matched && bestMatchId !== null) {
        // 매칭된 경우: 현재 좌표로 교체, 이전 ID 유지
        const tracked = this.trackedObstacles.get(bestMatchId);
        if (!tracked) {
          // eslint-disable-next-line no-continue
          continue;
        }
        const trackedObstacle: ObstacleData = {
          ...currentObstacle,
          obstacle_id: tracked.obstacle.obstacle_id
        };
        trackedList.push(trackedObstacle);
        // 매칭 성공: unmatchedFrames 리셋
        this.trackedObstacles.set(bestMatchId, {
          obstacle: trackedObstacle,
          unmatchedFrames: 0
        });
        matchedIds.add(bestMatchId);
      } else {
        // 매칭되지 않은 경우: 새로 추가
        const newId = currentObstacle.obstacle_id || idManager.allocID();
        const trackedObstacle: ObstacleData = {
          ...currentObstacle,
          obstacle_id: newId
        };
        trackedList.push(trackedObstacle);
        this.trackedObstacles.set(newId, {
          obstacle: trackedObstacle,
          unmatchedFrames: 0
        });
        matchedIds.add(newId);
      }
    }

    // 매칭되지 않은 기존 장애물들의 unmatchedFrames 증가
    for (const [id, tracked] of this.trackedObstacles) {
      if (!matchedIds.has(id)) {
        tracked.unmatchedFrames += 1;
        // 5프레임 이상 매칭 안되면 제거
        if (tracked.unmatchedFrames >= this.MAX_UNMATCHED_FRAMES) {
          this.trackedObstacles.delete(id);
        }
      }
    }

    return trackedList;
  }

  reset(): void {
    this.trackedObstacles.clear();
  }
}

// 정적 장애물 트래커: 10프레임 평균 위치 사용
class StaticObstacleTracker {
  private trackedObstacles: Map<number, StaticObstacleHistory> = new Map();

  private readonly DISTANCE_THRESHOLD = 100; // 1m = 100cm

  private readonly POSITION_HISTORY_SIZE = 10; // 10프레임

  track(newList: ObstacleData[]): ObstacleData[] {
    const trackedList: ObstacleData[] = [];
    const matchedIds = new Set<number>();

    // 현재 프레임의 정적 장애물과 기존 트래킹 중인 장애물 매칭
    for (const currentObstacle of newList) {
      if (!isStaticObstacle(currentObstacle.obstacle_class)) {
        // eslint-disable-next-line no-continue
        continue; // 동적 장애물은 제외
      }

      let matched = false;
      let bestMatchId: number | null = null;
      let bestDistance = Infinity;

      // 기존 트래킹 중인 장애물과 거리 비교
      for (const [id, history] of this.trackedObstacles) {
        if (matchedIds.has(id)) {
          // eslint-disable-next-line no-continue
          continue;
        }

        const distance = euclideanDistance(currentObstacle, history.obstacle);

        if (distance < this.DISTANCE_THRESHOLD && distance < bestDistance) {
          bestMatchId = id;
          bestDistance = distance;
          matched = true;
        }
      }

      if (matched && bestMatchId !== null) {
        // 매칭된 경우: 위치 히스토리에 추가하고 평균 위치 계산
        const history = this.trackedObstacles.get(bestMatchId);
        if (!history) {
          // eslint-disable-next-line no-continue
          continue;
        }
        
        // 현재 위치를 히스토리에 추가 (x, y만)
        history.positionHistory.push({
          x: currentObstacle.fused_position_x,
          y: currentObstacle.fused_position_y
        });

        // 10프레임 초과하면 오래된 것 제거
        if (history.positionHistory.length > this.POSITION_HISTORY_SIZE) {
          history.positionHistory.shift();
        }

        // 평균 위치 계산 (x, y만)
        const avgX = history.positionHistory.reduce((sum, pos) => sum + pos.x, 0) / history.positionHistory.length;
        const avgY = history.positionHistory.reduce((sum, pos) => sum + pos.y, 0) / history.positionHistory.length;

        // 평균 위치로 업데이트된 장애물 생성 (z는 현재 값 유지)
        const trackedObstacle: ObstacleData = {
          ...currentObstacle,
          obstacle_id: history.obstacle.obstacle_id,
          fused_position_x: avgX,
          fused_position_y: avgY
        };
        trackedList.push(trackedObstacle);
        
        // 히스토리 업데이트
        history.obstacle = trackedObstacle;
        matchedIds.add(bestMatchId);
      } else {
        // 매칭되지 않은 경우: 새로 추가
        const newId = currentObstacle.obstacle_id || idManager.allocID();
        const trackedObstacle: ObstacleData = {
          ...currentObstacle,
          obstacle_id: newId
        };
        trackedList.push(trackedObstacle);
        
        // 히스토리 초기화 (x, y만)
        this.trackedObstacles.set(newId, {
          obstacle: trackedObstacle,
          positionHistory: [{
            x: currentObstacle.fused_position_x,
            y: currentObstacle.fused_position_y
          }]
        });
        matchedIds.add(newId);
      }
    }

    // 매칭되지 않은 기존 정적 장애물도 유지 (계속 남겨줌)
    for (const [id, history] of this.trackedObstacles) {
      if (!matchedIds.has(id)) {
        // 평균 위치로 유지 (x, y만)
        if (history.positionHistory.length > 0) {
          const avgX = history.positionHistory.reduce((sum, pos) => sum + pos.x, 0) / history.positionHistory.length;
          const avgY = history.positionHistory.reduce((sum, pos) => sum + pos.y, 0) / history.positionHistory.length;

          const trackedObstacle: ObstacleData = {
            ...history.obstacle,
            fused_position_x: avgX,
            fused_position_y: avgY
          };
          trackedList.push(trackedObstacle);
        } else {
          trackedList.push(history.obstacle);
        }
      }
    }

    return trackedList;
  }

  reset(): void {
    this.trackedObstacles.clear();
  }
}

// 통합 트래커: 하나의 리스트에서 정적/동적 장애물을 분기 처리
class ObstacleTracker {
  private dynamicObstacles: Map<number, { obstacle: ObstacleData; unmatchedFrames: number }> = new Map();

  private staticObstacles: Map<number, StaticObstacleHistory> = new Map();

  private readonly DISTANCE_THRESHOLD = 100; // 동적 장애물: 1m = 100cm

  private readonly STATIC_DISTANCE_THRESHOLD = 50; // 정적 장애물: 0.5m = 50cm

  private readonly MAX_UNMATCHED_FRAMES = 5; // 동적 장애물: 5프레임

  private readonly POSITION_HISTORY_SIZE = 10; // 정적 장애물: 10프레임 버퍼

  track(newList: ObstacleData[]): ObstacleData[] {
    const trackedList: ObstacleData[] = [];
    const matchedDynamicIds = new Set<number>();
    const matchedStaticIds = new Set<number>();

    // 하나의 리스트에서 정적/동적 장애물 분기 처리
    for (const currentObstacle of newList) {
      const isStatic = isStaticObstacle(currentObstacle.obstacle_class);

      if (isStatic) {
        // 정적 장애물 처리
        let matched = false;
        let bestMatchId: number | null = null;
        let bestDistance = Infinity;

        // 기존 트래킹 중인 정적 장애물과 거리 비교
        for (const [id, history] of this.staticObstacles) {
          if (matchedStaticIds.has(id)) {
            // eslint-disable-next-line no-continue
            continue;
          }

          const distance = euclideanDistance(currentObstacle, history.obstacle);

          if (distance < this.STATIC_DISTANCE_THRESHOLD && distance < bestDistance) {
            bestMatchId = id;
            bestDistance = distance;
            matched = true;
          }
        }

        if (matched && bestMatchId !== null) {
          // 매칭된 경우: 위치 히스토리에 추가하고 평균 위치 계산
          const history = this.staticObstacles.get(bestMatchId);
          if (!history) {
            // eslint-disable-next-line no-continue
            continue;
          }

          // 현재 위치를 히스토리에 추가 (x, y만)
          history.positionHistory.push({
            x: currentObstacle.fused_position_x,
            y: currentObstacle.fused_position_y
          });

          // 10프레임 초과하면 오래된 것 제거
          if (history.positionHistory.length > this.POSITION_HISTORY_SIZE) {
            history.positionHistory.shift();
          }

          // 평균 위치 계산 (x, y만)
          const avgX = history.positionHistory.reduce((sum, pos) => sum + pos.x, 0) / history.positionHistory.length;
          const avgY = history.positionHistory.reduce((sum, pos) => sum + pos.y, 0) / history.positionHistory.length;

          // 평균 위치로 업데이트된 장애물 생성 (z는 현재 값 유지)
          const trackedObstacle: ObstacleData = {
            ...currentObstacle,
            obstacle_id: history.obstacle.obstacle_id,
            fused_position_x: avgX,
            fused_position_y: avgY
          };
          trackedList.push(trackedObstacle);

          // 히스토리 업데이트
          history.obstacle = trackedObstacle;
          matchedStaticIds.add(bestMatchId);
        } else {
          // 매칭되지 않은 경우: 새로 추가
          const newId = currentObstacle.obstacle_id || idManager.allocID();
          const trackedObstacle: ObstacleData = {
            ...currentObstacle,
            obstacle_id: newId
          };
          trackedList.push(trackedObstacle);

          // 히스토리 초기화 (x, y만)
          this.staticObstacles.set(newId, {
            obstacle: trackedObstacle,
            positionHistory: [{
              x: currentObstacle.fused_position_x,
              y: currentObstacle.fused_position_y
            }]
          });
          matchedStaticIds.add(newId);
        }
      } else {
        // 동적 장애물 처리
        let matched = false;
        let bestMatchId: number | null = null;
        let bestDistance = Infinity;

        // 기존 트래킹 중인 동적 장애물과 거리 비교
        for (const [id, tracked] of this.dynamicObstacles) {
          if (matchedDynamicIds.has(id)) {
            // eslint-disable-next-line no-continue
            continue;
          }

          const distance = euclideanDistance(currentObstacle, tracked.obstacle);

          if (distance < this.DISTANCE_THRESHOLD && distance < bestDistance) {
            bestMatchId = id;
            bestDistance = distance;
            matched = true;
          }
        }

        if (matched && bestMatchId !== null) {
          // 매칭된 경우: 현재 좌표로 교체, 이전 ID 유지
          const tracked = this.dynamicObstacles.get(bestMatchId);
          if (!tracked) {
            // eslint-disable-next-line no-continue
            continue;
          }
          const trackedObstacle: ObstacleData = {
            ...currentObstacle,
            obstacle_id: tracked.obstacle.obstacle_id
          };
          trackedList.push(trackedObstacle);
          // 매칭 성공: unmatchedFrames 리셋
          this.dynamicObstacles.set(bestMatchId, {
            obstacle: trackedObstacle,
            unmatchedFrames: 0
          });
          matchedDynamicIds.add(bestMatchId);
        } else {
          // 매칭되지 않은 경우: 새로 추가
          const newId = currentObstacle.obstacle_id || idManager.allocID();
          const trackedObstacle: ObstacleData = {
            ...currentObstacle,
            obstacle_id: newId
          };
          trackedList.push(trackedObstacle);
          this.dynamicObstacles.set(newId, {
            obstacle: trackedObstacle,
            unmatchedFrames: 0
          });
          matchedDynamicIds.add(newId);
        }
      }
    }

    // 매칭되지 않은 기존 동적 장애물 처리: 5프레임 이상 매칭 안되면 제거
    for (const [id, tracked] of this.dynamicObstacles) {
      if (!matchedDynamicIds.has(id)) {
        tracked.unmatchedFrames += 1;
        // 5프레임 이상 매칭 안되면 제거
        if (tracked.unmatchedFrames >= this.MAX_UNMATCHED_FRAMES) {
          this.dynamicObstacles.delete(id);
        } else {
          // 아직 제거되지 않은 경우 trackedList에 추가 (lifecycle 관리)
          trackedList.push(tracked.obstacle);
        }
      }
    }

    // 매칭되지 않은 기존 정적 장애물 처리: 매칭 안되어도 계속 유지
    for (const [id, history] of this.staticObstacles) {
      if (!matchedStaticIds.has(id)) {
        // 평균 위치로 유지 (x, y만)
        if (history.positionHistory.length > 0) {
          const avgX = history.positionHistory.reduce((sum, pos) => sum + pos.x, 0) / history.positionHistory.length;
          const avgY = history.positionHistory.reduce((sum, pos) => sum + pos.y, 0) / history.positionHistory.length;

          const trackedObstacle: ObstacleData = {
            ...history.obstacle,
            fused_position_x: avgX,
            fused_position_y: avgY
          };
          trackedList.push(trackedObstacle);
        } else {
          trackedList.push(history.obstacle);
        }
      }
    }

    return trackedList;
  }

  reset(): void {
    this.dynamicObstacles.clear();
    this.staticObstacles.clear();
  }
}

const obstacleTracker = new ObstacleTracker();

// 간단한 헝가리안 알고리즘 (Munkres) 구현
function solveAssignment(costMatrix: number[][]): number[] {
  // 1. 행렬 복사 (C++와 동일)
  const matrix: number[][] = [];
  for (let i = 0; i < costMatrix.length; i += 1) {
    matrix[i] = [];
    for (let j = 0; j < costMatrix[i].length; j += 1) {
      matrix[i][j] = costMatrix[i][j];
    }
  }
  
  // 2. Munkres 알고리즘 실행 (C++의 munkres.solve(matrix)와 동일)
  const assignments = munkres(matrix);
  
  // 3. C++ 방식으로 결과 추출 - 0인 위치 찾기
  const assignment: number[] = Array(costMatrix.length).fill(-1);
  
  // munkres-js는 매칭 쌍을 반환하므로, 이를 C++ 방식으로 변환
  for (const [i, j] of assignments) {
    assignment[i] = j;
  }
  
  return assignment;
}

// 차량 간 융합 처리 (listA의 ID 우선 유지)
/* eslint-disable no-param-reassign */
function processFusionForVehiclePair(presList: ObstacleData[], prevList: ObstacleData[], assignment: number[]): void {
  const newList: ObstacleData[] = [];
  
  // 1. 매칭된 장애물 처리
  for (let i = 0; i < assignment.length; i += 1) {
    const j = assignment[i];
    if (j >= 0) {
      // presList(listA)의 ID가 있으면 우선 사용, 없으면 prevList(listB)의 ID 사용
      if (presList[j].obstacle_id === undefined && prevList[i].obstacle_id !== undefined) {
        presList[j].obstacle_id = prevList[i].obstacle_id;
      }
      // presList의 ID가 이미 있으면 그대로 유지
      newList.push(presList[j]);
    }
  }
  
  // 2. prevList에서 매칭되지 않은 장애물 처리
  for (let i = 0; i < prevList.length; i += 1) {
    const j = assignment[i];
    if (j < 0) {
      // prevList에서 매칭되지 않은 항목을 newList에 추가
      newList.push(prevList[i]);
    }
  }
  
  // 3. presList에서 매칭되지 않은 장애물 처리
  for (let i = 0; i < presList.length; i += 1) {
    // assignment에 i가 포함되어 있는지 확인
    if (assignment.findIndex((val) => val === i) === -1) {
      // presList에서 매칭되지 않은 항목에 ID가 없으면 새로 할당
      if (presList[i].obstacle_id === undefined) {
        presList[i].obstacle_id = idManager.allocID();
      }
      newList.push(presList[i]);
    }
  }
  
  // 새로운 리스트로 갱신
  presList.length = 0;
  presList.push(...newList);
}

// 프레임 간 융합 처리 (C++ 코드와 동일한 로직)
/* eslint-disable no-param-reassign */
function processFusion(presList: ObstacleData[], prevList: ObstacleData[], assignment: number[]): void {
  const startTime = performance.now();
  const newList: ObstacleData[] = [];
  
  // assignment[i]는 prevList[i]가 매칭된 presList의 인덱스
  // assignment.length = prevList.length
  
  // presList와 prevList 출력
  console.log('📋 [processFusion] 리스트 정보:', {
    presList: presList.map(obs => ({
      id: obs.obstacle_id,
      x: obs.fused_position_x,
      y: obs.fused_position_y,
      z: obs.fused_position_z,
      class: obs.obstacle_class
    })),
    prevList: prevList.map(obs => ({
      id: obs.obstacle_id,
      x: obs.fused_position_x,
      y: obs.fused_position_y,
      z: obs.fused_position_z,
      class: obs.obstacle_class
    })),
    assignment,
    presListLength: presList.length,
    prevListLength: prevList.length,
    assignmentLength: assignment.length
  });
  
  // 1. 매칭된 장애물 처리
  for (let i = 0; i < assignment.length; i += 1) {
    const j = assignment[i]; // j는 presList의 인덱스
    if (j >= 0) {
      // // 배열 범위 체크
      // if (i >= prevList.length || j >= presList.length) {
      //   console.warn('⚠️ [processFusion] 배열 인덱스 범위 초과:', {
      //     i,
      //     j,
      //     presListLength: presList.length,
      //     prevListLength: prevList.length,
      //     assignmentLength: assignment.length
      //   });
      //   // eslint-disable-next-line no-continue
      //   continue;
      // }
      
      const prevObstacle = prevList[i]; // prevList[i]가 매칭됨
      const currentObstacle = presList[j]; // presList[j]와 매칭됨
      // console.log('prevObstacle', prevObstacle);
      // // undefined 체크
      // if (!prevObstacle || !currentObstacle) {
      //   console.warn('⚠️ [processFusion] 장애물 데이터가 undefined:', {
      //     i,
      //     j,
      //     prevObstacle: !!prevObstacle,
      //     currentObstacle: !!currentObstacle
      //   });
      //   // eslint-disable-next-line no-continue
      //   continue;
      // }
      
      // obstacle_class가 다르면 매칭 취소 (현재 장애물만 추가, 이전 장애물은 제거하지 않음)
      if (prevObstacle.obstacle_class !== currentObstacle.obstacle_class) {
        currentObstacle.obstacle_id = idManager.allocID();
        newList.push(currentObstacle);
        // 이전 시점 장애물은 제거 (누적 방지)
        // eslint-disable-next-line no-continue
        continue;
      }
      
      // 위치가 임계값 이상 벌어지면 매칭 취소 (정적/동적 장애물별 다른 임계값)
      const distance = euclideanDistance(prevObstacle, currentObstacle);
      const isStatic = isStaticObstacle(currentObstacle.obstacle_class);
      const threshold = isStatic ? STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD : DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD;
      
      if (distance >= threshold) {
        currentObstacle.obstacle_id = idManager.allocID();
        newList.push(currentObstacle);
        // 이전 시점 장애물은 제거 (누적 방지)
        // eslint-disable-next-line no-continue
        continue;
      }
      
      // 매칭된 장애물 유지
      currentObstacle.obstacle_id = prevObstacle.obstacle_id;
      newList.push(currentObstacle);
    }
  }
  
  // 2. prevList에서 매칭되지 않은 장애물은 제거 (누적 방지)
  // 이전 프레임의 장애물이 매칭 안되면 사라진 것으로 간주하고 제거
  
  // 3. presList에서 매칭되지 않은 장애물 처리
  // presList[j]가 매칭되었는지 확인하려면 assignment 배열에서 j를 값으로 가진 인덱스가 있는지 확인
  const matchedPresIndices = new Set<number>();
  for (let i = 0; i < assignment.length; i += 1) {
    if (assignment[i] >= 0) {
      matchedPresIndices.add(assignment[i]);
    }
  }
  
  for (let i = 0; i < presList.length; i += 1) {
    if (!matchedPresIndices.has(i)) {
      const currentObstacle = presList[i];
      currentObstacle.obstacle_id = idManager.allocID();
      newList.push(currentObstacle);
    }
  }
  
  // newList 출력
  console.log('📋 [processFusion] newList 정보:', {
    newList: newList.map(obs => ({
      id: obs.obstacle_id,
      x: obs.fused_position_x,
      y: obs.fused_position_y,
      z: obs.fused_position_z,
      class: obs.obstacle_class
    })),
    newListLength: newList.length
  });
  
  // 트래커를 사용하여 newList 정리
  const trackerStartTime = performance.now();
  const trackedList = obstacleTracker.track(newList);
  const trackerTime = performance.now() - trackerStartTime;
  
  // trackedList 출력
  console.log('📋 [processFusion] trackedList 정보:', {
    trackedList: trackedList.map(obs => ({
      id: obs.obstacle_id,
      x: obs.fused_position_x,
      y: obs.fused_position_y,
      z: obs.fused_position_z,
      class: obs.obstacle_class
    })),
    trackedListLength: trackedList.length
  });
  
  // 속도 측정 출력
  const totalTime = performance.now() - startTime;
  console.log('⚡ [processFusion] 성능 측정:', {
    totalTime: `${totalTime.toFixed(2)}ms`,
    trackerTime: `${trackerTime.toFixed(2)}ms`,
    newListLength: newList.length,
    trackedListLength: trackedList.length,
    presListLength: presList.length,
    prevListLength: prevList.length
  });
  
  // 새로운 리스트로 갱신
  presList.length = 0;
  presList.push(...trackedList);
}
/* eslint-enable no-param-reassign */

function createDistanceMatrix(
  listA: ObstacleData[],
  listB: ObstacleData[],
  maxDistance = Infinity
): number[][] {
  const matrix: number[][] = []
  for (let i = 0; i < listA.length; i+=1) {
    matrix[i] = []
    for (let j = 0; j < listB.length; j+=1) {
      const distance = euclideanDistance(listA[i], listB[j])
      // 거리가 임계값을 넘으면 매우 큰 값으로 설정 (매칭 방지)
      matrix[i][j] = distance > maxDistance ? 999999 : distance
    }
  }
  return matrix; // C++와 동일하게 2차원 배열만 반환
}

// 동적 차량 리스트를 받는 융합 함수
/* eslint-disable no-param-reassign */
function mergeAndCompareListsDynamic(
  previousFusionList: ObstacleData[],
  obstacleLists: ObstacleData[][],
  vehicles: VehicleData[]
): ObstacleData[] {
  const nonEmptyVehicles: VehicleData[] = [];
  const nonEmptyLists: ObstacleData[][] = [];
  let mergedList: ObstacleData[] = [];

  // 장애물 리스트가 비어있지 않으면 timestamp 여부와 관계없이 수집
  for (let i = 0; i < vehicles.length; i += 1) {
    if (obstacleLists[i] && obstacleLists[i].length > 0) {
      nonEmptyVehicles.push(vehicles[i]);
      nonEmptyLists.push(obstacleLists[i]);
    }
  }

  // 디버깅 로그
  if (nonEmptyLists.length === 0) {
    console.log('🔍 [mergeAndCompareListsDynamic] 장애물 리스트 없음:', {
      totalVehicles: vehicles.length,
      obstacleListsLengths: obstacleLists.map(list => list?.length || 0),
      hasTimestamp: vehicles.map(v => !!v.timestamp)
    });
  }

  // 융합할 리스트 필터링
  if (nonEmptyLists.length === 0) {
    // 모든 리스트가 비어있음 - 빈 배열 반환
    mergedList = [];
  } else if (nonEmptyLists.length === 1) {
    // 유일한 리스트 하나가 있을 경우 그대로 사용
    // 참조가 아닌 복사본을 만들어야 ID 할당이 반영됨
    mergedList = [...nonEmptyLists[0]];
  } else if (nonEmptyLists.length > 1) {
    // 둘 이상 리스트가 있을 때 융합 수행
    const handleFusionForPair = (listA: ObstacleData[], listB: ObstacleData[]): ObstacleData[] => {
      const fusionList = [...listA];
      if (listA.length > 0 && listB.length > 0) {
        // 차량 간 융합: 거리 제한 없이 매칭 (같은 시점의 다른 차량 데이터)
        const distMatrix = createDistanceMatrix(listB, listA);
        const assignment = solveAssignment(distMatrix);
        // 차량 간 융합: listA의 ID를 우선 유지
        processFusionForVehiclePair(fusionList, listB, assignment);
      }
      return fusionList;
    };

    // 처음 두 개 리스트 융합
    mergedList = handleFusionForPair(nonEmptyLists[0], nonEmptyLists[1]);

    // 나머지 리스트들과 순차적으로 융합
    for (let i = 2; i < nonEmptyLists.length; i += 1) {
      mergedList = handleFusionForPair(mergedList, nonEmptyLists[i]);
    }
  }

  // Worker 중복 제거 로직 제거 - 모든 Worker가 표시되도록 함

  if (mergedList.length === 0) {
    if (previousFusionList.length === 0) {
      console.log('📭 [mergeAndCompareLists] 장애물 리스트 비어있음');
    } else {
      console.log('📦 [mergeAndCompareLists] 현재 TimeStamp 장애물 X, 이전 TimeStamp 장애물리스트 그대로 사용');
    }
    return previousFusionList;
  }
  
  if (previousFusionList.length === 0) {
    // 첫 프레임인 경우 모든 장애물에 새 ID 할당
    mergedList.forEach((obstacle) => {
      const newId = idManager.allocID();
      obstacle.obstacle_id = newId;
    });
    return mergedList;
  }
  
  // 이전 데이터와 융합하여 ID 부여
  // 정적/동적 장애물별로 다른 거리 임계값 적용
  // 정적 장애물과 동적 장애물을 분리하여 처리
  const staticPrevList: ObstacleData[] = [];
  const dynamicPrevList: ObstacleData[] = [];
  const staticMergedList: ObstacleData[] = [];
  const dynamicMergedList: ObstacleData[] = [];
  
  // 이전 프레임 장애물 분리
  previousFusionList.forEach(obs => {
    if (isStaticObstacle(obs.obstacle_class)) {
      staticPrevList.push(obs);
    } else {
      dynamicPrevList.push(obs);
    }
  });
  
  // 현재 프레임 장애물 분리
  mergedList.forEach(obs => {
    if (isStaticObstacle(obs.obstacle_class)) {
      staticMergedList.push(obs);
    } else {
      dynamicMergedList.push(obs);
    }
  });
  
  // 정적 장애물 매칭
  if (staticPrevList.length > 0 && staticMergedList.length > 0) {
    const staticDistMatrix = createDistanceMatrix(staticPrevList, staticMergedList, STATIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
    const staticAssignment = solveAssignment(staticDistMatrix);
    processFusion(staticMergedList, staticPrevList, staticAssignment);
  } else if (staticMergedList.length > 0) {
    // 이전 프레임에 정적 장애물이 없으면 새 ID 할당
    staticMergedList.forEach(obs => {
      if (!obs.obstacle_id) {
        obs.obstacle_id = idManager.allocID();
      }
    });
  }
  
  // 동적 장애물 매칭
  if (dynamicPrevList.length > 0 && dynamicMergedList.length > 0) {
    const dynamicDistMatrix = createDistanceMatrix(dynamicPrevList, dynamicMergedList, DYNAMIC_OBSTACLE_MATCH_DISTANCE_THRESHOLD);
    const dynamicAssignment = solveAssignment(dynamicDistMatrix);
    processFusion(dynamicMergedList, dynamicPrevList, dynamicAssignment);
  } else if (dynamicMergedList.length > 0) {
    // 이전 프레임에 동적 장애물이 없으면 새 ID 할당
    dynamicMergedList.forEach(obs => {
      if (!obs.obstacle_id) {
        obs.obstacle_id = idManager.allocID();
      }
    });
  }
  
  // 정적/동적 장애물 합치기
  mergedList.length = 0;
  mergedList.push(...staticMergedList, ...dynamicMergedList);
  
  return mergedList;
}
/* eslint-enable no-param-reassign */

// 데이터 채우기 함수들
/* eslint-disable no-param-reassign */
function fillVehicleData(vehicle: VehicleData, data: HubDataObject): void {
  vehicle.vehicle_class = data.vehicle_id;
  vehicle.timestamp = data.timestamp;
  vehicle.position_lat = data.position_lat;
  vehicle.position_long = data.position_long;
  vehicle.position_height = data.position_height;
  vehicle.yaw = data.yaw;
  vehicle.roll = data.roll;
  vehicle.pitch = data.pitch;
  vehicle.velocity_long = data.velocity_long;
  vehicle.velocity_lat = data.velocity_lat;
  vehicle.velocity_ang = data.velocity_ang;
}
/* eslint-enable no-param-reassign */

/* eslint-disable no-param-reassign */
function fillObstacleList(obstacle_list: ObstacleData[], data: HubDataObject): void {
  obstacle_list.length = 0; // clear()와 동일
  
  if (data.obstacle) {
    data.obstacle.forEach((obstacle) => {
      // 새 객체 생성 후 데이터 복사
      const obstacleToPush: ObstacleData = {
        obstacle_class: obstacle.obstacle_class,
        timestamp: data.timestamp,
        fused_cuboid_x: obstacle.cuboid_x,
        fused_cuboid_y: obstacle.cuboid_y,
        fused_cuboid_z: obstacle.cuboid_z,
        fused_heading_angle: obstacle.heading_angle,
        fused_position_x: obstacle.position_x,
        fused_position_y: obstacle.position_y,
        fused_position_z: obstacle.position_z,
        fused_velocity_x: obstacle.velocity_x,
        fused_velocity_y: obstacle.velocity_y,
        fused_velocity_z: obstacle.velocity_z,
        map_2d_location: [],
        isMapCoordinate: false // 새로운 센서 데이터는 아직 변환되지 않음
      };
      
      obstacle_list.push(obstacleToPush);
    });
  }
}

// 실제 데이터가 처음 들어왔는지 추적
let isFirstDataReceived = false;

// 메인 센싱 데이터 처리 함수 (통합)
const processSensingData = (data: HubDataObject): void => {
  try {
    // vehicle_state가 2(fail)인지 확인
    if (data.vehicle_state === 2) {
      messageBox('error', '현장 확인이 필요합니다');
      // eslint-disable-next-line @typescript-eslint/no-use-before-define
      resetMission();
      return;
    }
    
    // road_z 데이터 처리 추가 (isroadZOn이 true일 때만)
    if (isroadZOn.value) {
      processRoadZData(data);
      // 0.1m x 0.1m 셀로 집계 (1/10 압축)
      aggregateToMeterGrid();
    }
    
    // vehicle_id가 없으면 처리하지 않음
    if (!data.vehicle_id) {
      // vehicle_id가 없어도 obstacle이 있으면 토픽에서 추출 시도
      console.warn('⚠️ [processSensingData] vehicle_id가 없습니다. 데이터:', {
        hasObstacle: !!data.obstacle,
        obstacleCount: data.obstacle?.length || 0
      });
      return;
    }
  } catch (error) {
    console.error('❌ [processSensingData] 처리 중 오류 발생:', error, data);
    return;
  }

  try {
    // 실제 데이터가 들어온 차량으로 표시
    activeVehicleIds.add(data.vehicle_id);

  // 차량별 데이터 분류 및 처리 (동적)
  const vehicleStore = getOrCreateVehicleData(data.vehicle_id);
  const fusionData: FusionData = { vehicle: {} as VehicleData, obstacle_list: [] };
  fillVehicleData(fusionData.vehicle, data);
  fillObstacleList(fusionData.obstacle_list, data);
  
  // 차량 데이터 저장
  vehicleStore.fusionData = fusionData;
  
  // 현재 들어온 차량 데이터 처리
  processVehicleData(vehicleStore.fusionData, vehicleStore.vehicle, vehicleStore.obstacleList, data.vehicle_id);
  
  // 모든 차량의 장애물 리스트 수집 (메인차량 우선, 보조차량 순서)
  const allVehicleStores = Array.from(vehicleDataMap.entries())
    .sort(([idA], [idB]) => {
      // F로 시작하는 차량(메인차량)을 먼저, 그 다음 보조차량
      const isMainA = idA.startsWith('F');
      const isMainB = idB.startsWith('F');
      if (isMainA && !isMainB) return -1;
      if (!isMainA && isMainB) return 1;
      return idA.localeCompare(idB);
    })
    .map(([, store]) => store);
  
  // 장애물 데이터 융합 및 지도 표시
  const obstacleList = mergeAndCompareListsDynamic(
    previousObstacleList,
    allVehicleStores.map(store => store.obstacleList),
    allVehicleStores.map(store => store.vehicle)
  );
  
  // previousObstacleList 업데이트 (빈 배열이 아닐 때만 업데이트)
  if (obstacleList.length > 0) {
    previousObstacleList = obstacleList;
  }
  
  // 융합된 장애물 리스트를 지도에 표시
  // eslint-disable-next-line @typescript-eslint/no-use-before-define
  drawObstacle(obstacleList);
  
  // 텔레컨스 사이트 그리드 효과 그리기
  drawTeleconsGrid();
  
  // 노면 heatmap 그리기 (0.1m x 0.1m 셀, 1/10 압축) - isroadZOn이 true일 때만
  if (isroadZOn.value) {
    drawMeterGridHeatmap();
  } else {
    // isroadZOn이 false일 때 기존 roadz 데이터 제거
    removeRoadZFeatures();
  }
} catch (error) {
  console.error('❌ [processSensingData] 처리 중 오류 발생:', error, data);
}
};

watchEffect(() => {
  if (modalTriggered.value) {
    openModal();
  }
})

/**
 * 장애물 클래스에 따른 아이콘 소스를 반환하는 함수
 * @param obstacleClass 장애물 클래스 번호
 * @returns {iconSource: string} 아이콘 소스
 */
const getObstacleProperties = (obstacleClass: number): { iconSource: string } | undefined => {
  switch (obstacleClass) {
    // 건설 기계류 (1~9)
    case 1:
      return { iconSource: IconSource.EXCVATOR };
    case 2:
      return { iconSource: IconSource.FORK_LIFT };
    case 3:
      return { iconSource: IconSource.CONCRETE_MIXER_TRUCK };
    case 4:
      return { iconSource: IconSource.CRANE };
    case 6:
      return { iconSource: IconSource.DUMP_TRUCK };

    // 차량류 (10~19)
    case 10:
      return { iconSource: IconSource.CAR };
    case 11:
      return { iconSource: IconSource.SUV };
    case 12:
      return { iconSource: IconSource.TRUCK };
    case 14:
      return { iconSource: IconSource.MOTORCYCLE };
    case 15:
      return { iconSource: IconSource.BYCICLE };

    // 사람 (20~29)
    case 20:
      return { iconSource: IconSource.WORKER };
    case 21:
      return { iconSource: IconSource.ROBOT };

    // 건축자재 (30~39)
    case 30:
      return { iconSource: IconSource.SCAFFOLDING_PIPE };
    case 31:
      return { iconSource: IconSource.CEMENT };
    case 32:
      return { iconSource: IconSource.BRICK };

    // 안전용품 (40~49)
    case 40:
      return { iconSource: IconSource.DRUM_CAN };
    case 41:
      return { iconSource: IconSource.FENCE };
    case 42:
      return { iconSource: IconSource.RUBBER_CONE };
    case 43:
      return { iconSource: IconSource.SIGNBOARD };

    default:
      return undefined;
  }
};

const allTopic = env.natsSubscribeInfo;
const globalTopics = [allTopic.globalRoute, allTopic.subTopic, allTopic.globalVehicleState, allTopic.mapData];

// TODO: 추후, 차량 좌표가 WGS84 좌표계로 변경될 경우 삭제
// 경로 생성 테스트에서 사용되는 영역 절대 좌표
// 이 좌표를 기준으로 하는 차량의 상대 좌표
const SITE_ABSOLUTE_COORDINATE = [14087278.085107934, 4292292.6313868845];

// 차량 상태를 workInfo의 vehicleInfo에 업데이트
const setVehicleState = (vehicleData: any): void => {
  // vehicle_id 추출
  const vehicleId = vehicleData.vehicle_id;
  if (!vehicleId) return;

  // 실제 데이터가 들어온 차량으로 표시
  activeVehicleIds.add(vehicleId);

  // workInfo.vehicleInfo가 없으면 초기화
  if (!workInfo.value.vehicleInfo) {
    workInfo.value.vehicleInfo = {};
  }

  // 기존 차량 데이터
  const existingVehicleData = workInfo.value.vehicleInfo[vehicleId] || {};

  // 필드명 매핑 (battery_info, signal_state 모두 대응)
  const mappedData: VehicleInfo = {
    ...vehicleData,
    battery_info: vehicleData.battery_info ?? vehicleData.battery,
    signal_state: vehicleData.signal_state ?? vehicleData.state,
  };

  // 차량 타입 판단 (F로 시작하면 메인차량, 아니면 보조차량)
  const isMainVehicle = vehicleId.startsWith('F');
  
  // 기존 데이터와 새 데이터 병합 (새 데이터가 undefined/null이면 기존 값 유지)
  const vehicleType = isMainVehicle ? 'main' : 'sub';
  
  // 서버에서 가져온 기본 차량 정보가 있으면 유지하고, 실제 데이터로 업데이트
  const mergedData: VehicleInfo = {
    ...existingVehicleData, // 서버에서 가져온 기본 정보 또는 이전에 업데이트된 정보
    ...mappedData, // 실제 데이터로 업데이트
    // 차량 타입 정보 (기존에 없으면 새로 설정, 있으면 유지)
    type: existingVehicleData.type || vehicleType,
    vehicle_type: (existingVehicleData as any).vehicle_type || vehicleType,
    // vehicle_id는 항상 최신 값으로 업데이트
    vehicle_id: vehicleId,
  };

  // 반응성 보장
  workInfo.value = {
    ...workInfo.value,
    vehicleInfo: {
      ...workInfo.value.vehicleInfo,
      [vehicleId]: mergedData,
    },
  };
};

// 장애물 감지 여부(보조차량 기준으로 특정 거리 안에 있는지 감지)
const isNearBySub = (obstacleCoord: number[]): boolean => {
  const distance = Math.sqrt(obstacleCoord[0] ** 2 + obstacleCoord[1] ** 2);
  return distance < DETECT_DIST_LIMIT;
}

// 점이 폴리곤 내부에 있는지 확인하는 함수 (Ray Casting 알고리즘)
const isPointInsidePolygon = (point: [number, number], polygon: number[][]): boolean => {
  if (!polygon || polygon.length < 3) {
    return true; // 바운더리가 없으면 모든 장애물 표시
  }
  
  const [lon, lat] = point;
  let inside = false;
  
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i, i += 1) {
    const [xi, yi] = polygon[i];
    const [xj, yj] = polygon[j];
    
    const intersect = ((yi > lat) !== (yj > lat)) &&
      (lon < (xj - xi) * (lat - yi) / (yj - yi) + xi);
    
    if (intersect) {
      inside = !inside;
    }
  }
  
  return inside;
}

function drawObstacle(fusionList: ObstacleData[]): void {
  // 융합 리스트가 비어있으면 기존 상태 유지 (장애물 제거하지 않음)
  if (!fusionList || fusionList.length === 0) {
    console.log('📭 [drawObstacle] 융합 리스트가 비어있음 - 기존 장애물 상태 유지 (제거하지 않음)');
    // 빈 리스트일 때는 아무것도 하지 않음 (기존 장애물 유지)
    return;
  }
  
  // 디버깅 로그 (필요시만 활성화)
  // console.log('🎯 [drawObstacle] 시작 - fusionList 길이:', fusionList.length);
  
  // 기존 장애물 맵 생성 (ID로 빠른 검색) - 제거하지 않고 유지
  const allFeatures = osVector.getFeatures();
  const existingObstacles = new Map<string, OlFeature>();
  
  allFeatures.forEach((feature) => {
    const featureId = feature.getId() as string;
    if (featureId && featureId.startsWith('obstacle_')) {
      existingObstacles.set(featureId, feature);
    }
  });
  
  // 현재 fusionList에 있는 장애물 ID 추적
  const currentObstacleIds = new Set<string>();
  
  let processedCount = 0;
  let skippedCount = 0;
  let newCount = 0;
  let updatedCount = 0;
  
  // 바운더리 좌표 가져오기 (투영 좌표계 -> WGS84 변환)
  const boundaryCoords = workInfo.value?.coordinates || [];
  const boundaryCoordsWgs84: number[][] = [];
  
  // 바운더리 좌표를 WGS84로 변환 (투영 좌표계에서 WGS84로)
  if (boundaryCoords.length >= 3) {
    try {
      boundaryCoords.forEach((coord) => {
        if (Array.isArray(coord) && coord.length >= 2) {
          const [lon, lat] = toLonLat(coord);
          if (typeof lon === 'number' && typeof lat === 'number' && !Number.isNaN(lon) && !Number.isNaN(lat)) {
            boundaryCoordsWgs84.push([lon, lat]);
          }
        }
      });
    } catch (error) {
      console.error('바운더리 좌표 변환 중 오류:', error);
      // 변환 실패 시 바운더리 체크를 건너뛰고 모든 장애물 표시
    }
  }
  
  fusionList.forEach((obs) => {
    processedCount += 1;
    const obstacleProperties = getObstacleProperties(obs.obstacle_class);
    if (!obstacleProperties) {
      skippedCount += 1;
      // console.log('⚠️ [drawObstacle] obstacleProperties 없음:', obs.obstacle_class);
      return;
    }
    
    const { iconSource } = obstacleProperties;
    
    // 맵 좌표를 OpenLayers 좌표계로 변환
    const { lat: rotatedLat, lon: rotatedLon } = convertObstaclePositionToWgs84(
      obs.fused_position_x,
      obs.fused_position_y,
    );
    
    // 바운더리 내부에 있는지 확인 (변환이 성공적으로 완료된 경우에만)
    if (boundaryCoordsWgs84.length >= 3) {
      try {
        const isInside = isPointInsidePolygon([rotatedLon, rotatedLat], boundaryCoordsWgs84);
        if (!isInside) {
          // 바운더리 밖에 있으면 표시하지 않음
          skippedCount += 1;
          // console.log('⚠️ [drawObstacle] 바운더리 밖:', obs.obstacle_id, obs.obstacle_class);
          return;
        }
      } catch (error) {
        console.error('바운더리 체크 중 오류:', error);
        // 체크 실패 시 장애물을 표시 (안전 조치)
      }
    }
    
    const obstaclePosition = fromLonLat([rotatedLon, rotatedLat]);
    const obsId = `obstacle_${obs.obstacle_id || 'unknown'}`;
    currentObstacleIds.add(obsId);
    
    // 같은 ID의 장애물이 이미 있는지 확인
    const existingObstacle = existingObstacles.get(obsId);
    
    if (existingObstacle) {
      // 기존 장애물이 있으면 좌표와 데이터만 업데이트 (유지)
      updatedCount += 1;
      const geometry = existingObstacle.getGeometry() as OlPoint;
      if (geometry) {
        geometry.setCoordinates(obstaclePosition);
      }
      existingObstacle.set('obstacleData', obs);
      
      // 스타일 업데이트 (디버깅 모드 변경 대응)
      const scale = getScaleForZoom(olMap.getView().getZoom() || 8);
      let iconStyle;
      if (isDebugging.value) {
        iconStyle = new Style({
          image: new Icon({
            src: iconSource,
            scale,
          }),
          text: new Text({
            text: `ID: ${obs.obstacle_id || 'N/A'}\n(${obs.fused_position_x.toFixed(2)}, ${obs.fused_position_y.toFixed(2)})`,
            font: '12px Arial',
            fill: new Fill({ color: '#fff' }),
            stroke: new Stroke({ color: '#000', width: 2 }),
            offsetY: -30,
            textAlign: 'center',
          }),
        });
      } else {
        iconStyle = new Style({
          image: new Icon({
            src: iconSource,
            scale,
          }),
        });
      }
      existingObstacle.setStyle(iconStyle);
    } else {
      // 새로운 장애물 생성 (추가)
      newCount += 1;
      const newObstacle = new OlFeature({
        geometry: new OlPoint(obstaclePosition),
      });
      newObstacle.setId(obsId);
      newObstacle.set('obstacleData', obs);
      osVector.addFeature(newObstacle);
      
      const scale = getScaleForZoom(olMap.getView().getZoom() || 8);
      let iconStyle;
      if (isDebugging.value) {
        iconStyle = new Style({
          image: new Icon({
            src: iconSource,
            scale,
          }),
          text: new Text({
            text: `ID: ${obs.obstacle_id || 'N/A'}\n(${obs.fused_position_x.toFixed(2)}, ${obs.fused_position_y.toFixed(2)})`,
            font: '12px Arial',
            fill: new Fill({ color: '#fff' }),
            stroke: new Stroke({ color: '#000', width: 2 }),
            offsetY: -30,
            textAlign: 'center',
          }),
        });
      } else {
        iconStyle = new Style({
          image: new Icon({
            src: iconSource,
            scale,
          }),
        });
      }
      newObstacle.setStyle(iconStyle);
    }
  });
  
  // 디버깅 로그 (필요시만 활성화)
  // console.log('🎯 [drawObstacle] 완료 - 처리:', processedCount, '새로 추가:', newCount, '업데이트:', updatedCount, '스킵:', skippedCount);
  
  // 더 이상 존재하지 않는 장애물만 제거 (ID가 다른 경우만)
  existingObstacles.forEach((feature, featureId) => {
    if (!currentObstacleIds.has(featureId)) {
      osVector.removeFeature(feature);
    }
  });
}

// 디버깅 모드 변경 시 장애물 스타일 업데이트
const updateObstacleStyles = (): void => {
  const allFeatures = osVector.getFeatures();
  
  allFeatures.forEach((feature) => {
    const featureId = feature.getId() as string;
    if (featureId && featureId.startsWith('obstacle')) {
      const obstacleData = feature.get('obstacleData'); // 장애물 데이터 저장 필요
      if (obstacleData) {
        const obstacleProperties = getObstacleProperties(obstacleData.obstacle_class);
        if (!obstacleProperties) return;
        
        const { iconSource } = obstacleProperties;
        const scale = getScaleForZoom(olMap.getView().getZoom() || 8);
        
        let iconStyle;
        if (isDebugging.value) {
          iconStyle = new Style({
            image: new Icon({
              src: iconSource,
              scale,
            }),
            text: new Text({
              text: `ID: ${obstacleData.obstacle_id || 'N/A'}\n(${obstacleData.fused_position_x.toFixed(2)}, ${obstacleData.fused_position_y.toFixed(2)})`,
              font: '12px Arial',
              fill: new Fill({ color: '#fff' }),
              stroke: new Stroke({ color: '#000', width: 2 }),
              offsetY: -30,
              textAlign: 'center',
            }),
          });
        } else {
          iconStyle = new Style({
            image: new Icon({
              src: iconSource,
              scale,
            }),
          });
        }
        
        feature.setStyle(iconStyle);
      }
    }
  });
};

// 차량 디버깅 스타일 업데이트 함수
const updateVehicleDebugStyle = (marker: OlFeature<Point>, vehicleId: string): void => {
  const scale = getScaleForZoom(olMap.getView().getZoom() || 8);
  
  // 기존 스타일 가져오기
  const existingStyle = marker.getStyle();
  if (!existingStyle) return;
  
  // 아이콘 스타일 복사
  const iconStyle = existingStyle.getImage();
  if (!iconStyle) return;
  
  // vehicleMapCoordinates에서 맵 좌표 가져오기
  const mapCoords = vehicleMapCoordinates[vehicleId];
  if (!mapCoords) return;
  
  // 디버깅 모드일 때 텍스트 추가
  const debugStyle = new Style({
    image: iconStyle,
    text: new Text({
      text: `(${mapCoords.x.toFixed(2)}, ${mapCoords.y.toFixed(2)})`,
      font: '12px Arial',
      fill: new Fill({ color: '#fff' }),
      stroke: new Stroke({ color: '#000', width: 2 }),
      offsetY: -40, // 차량 아이콘 위쪽에 텍스트 표시
      textAlign: 'center',
    }),
  });
  
  marker.setStyle(debugStyle);
};

// 모든 차량의 디버깅 스타일 업데이트
const updateVehicleStyles = (): void => {
  const allFeatures = osVector.getFeatures();
  
  allFeatures.forEach((feature) => {
    const featureId = feature.getId() as string;
    if (featureId && featureId.startsWith('vehicle')) {
      // vehicleId 추출 (vehicle1, vehicle2 등)
      const vehicleId = featureId.replace('vehicle', '');
      
      if (isDebugging.value) {
        updateVehicleDebugStyle(feature, vehicleId);
      } else {
        // 디버깅 모드가 아닐 때는 원래 아이콘만 표시
        const existingStyle = feature.getStyle();
        if (existingStyle) {
          const iconStyle = existingStyle.getImage();
          if (iconStyle) {
            feature.setStyle(new Style({ image: iconStyle }));
          }
        }
      }
    }
  });
};

// origin 좌표를 동적으로 계산하는 함수 (C++ 코드와 동일)
const calculateOriginFromBoundary = (): void => {
  if (!workInfo.value?.coordinates || workInfo.value.coordinates.length === 0) {
    console.warn('[calculateOriginFromBoundary] workInfo.coordinates가 없습니다.');
    return;
  }

  // boundary 좌표를 WGS84로 변환
  const boundaryCoords = workInfo.value.coordinates.map(coord => {
    const [lon, lat] = toLonLat(coord);
    return {
      x: parseFloat(lon.toFixed(12)),
      y: parseFloat(lat.toFixed(13))
    };
  });

  if (boundaryCoords.length === 0) {
    console.warn('[calculateOriginFromBoundary] 변환된 좌표가 없습니다.');
    return;
  }

  // min/max 계산
  let minLon = boundaryCoords[0].x;
  let minLat = boundaryCoords[0].y;
  let maxLon = boundaryCoords[0].x;
  let maxLat = boundaryCoords[0].y;

  for (let i = 1; i < boundaryCoords.length; i += 1) {
    minLon = boundaryCoords[i].x < minLon ? boundaryCoords[i].x : minLon;
    minLat = boundaryCoords[i].y < minLat ? boundaryCoords[i].y : minLat;
    maxLon = boundaryCoords[i].x > maxLon ? boundaryCoords[i].x : maxLon;
    maxLat = boundaryCoords[i].y > maxLat ? boundaryCoords[i].y : maxLat;
  }

  // GPS를 UTM으로 변환하여 origin 설정
  const { utmX: minUtmX, utmY: minUtmY } = GPStoUTM(minLon, minLat);
  const { utmX: maxUtmX, utmY: maxUtmY } = GPStoUTM(maxLon, maxLat);

  // origin은 min_utm 좌표로 설정 (C++ 코드와 동일)
  originX = minUtmX;
  originY = minUtmY;

  const mapX = (maxUtmX - minUtmX) * 10;
  const mapY = (maxUtmY - minUtmY) * 10;
};

// 동적으로 work_info 생성하는 함수 (선택된 개별 차량만 포함)
const generateWorkInfo = (): WorkInformation => {
  const mainVehicles: VehicleMetaInfo[] = [];
  const subVehicles: VehicleMetaInfo[] = [];

  // 선택된 개별 차량만 처리
  selectedVehicles.value.forEach((vehicleId: string) => {
    if (!workInfo.value.vehicleInfo[vehicleId]) {
      return;
    }

    // 메인 차량인지 보조 차량인지 판단
    const isMainVehicle = vehicleId.includes('F');
    
    if (isMainVehicle) {
      mainVehicles.push({
        length: VEHICLE_DIMENSIONS.MAIN.length,
        width: VEHICLE_DIMENSIONS.MAIN.width,
        id: vehicleId
      });
    } else {
      subVehicles.push({
        length: VEHICLE_DIMENSIONS.SUB.length,
        width: VEHICLE_DIMENSIONS.SUB.width,
        id: vehicleId
      });
    }
  });

  // 좌표를 왼쪽 상단부터 반시계 방향으로 정렬하는 함수
  const sortCoordinatesCounterClockwise = (coords: {x: number, y: number}[]): {x: number, y: number}[] => {
    if (coords.length < 3) return coords;

    // 1. 중심점 계산
    const centerPoint = {
      x: coords.reduce((sum, c) => sum + c.x, 0) / coords.length,
      y: coords.reduce((sum, c) => sum + c.y, 0) / coords.length
    };

    // 2. 각 점을 중심점 기준으로 각도 계산하여 정렬 (반시계 방향)
    const sortedCoords = coords.slice().sort((a, b) => {
      const angleA = Math.atan2(a.y - centerPoint.y, a.x - centerPoint.x);
      const angleB = Math.atan2(b.y - centerPoint.y, b.x - centerPoint.x);
      return angleA - angleB; // 반시계 방향
    });

    // 3. 왼쪽 상단 점 찾기 (가장 작은 x, 그 중에서 가장 큰 y)
    let topLeftIndex = 0;
    for (let i = 1; i < sortedCoords.length; i += 1) {
      const current = sortedCoords[i];
      const topLeft = sortedCoords[topLeftIndex];
      if (current.x < topLeft.x || (current.x === topLeft.x && current.y > topLeft.y)) {
        topLeftIndex = i;
      }
    }

    // 4. 왼쪽 상단 점이 첫 번째가 되도록 배열 회전
    return [...sortedCoords.slice(topLeftIndex), ...sortedCoords.slice(0, topLeftIndex)];
  };

  // working_area_boundary 생성 및 origin 좌표 계산
  const boundaryCoords = (() => {
    const coords = workInfo.value.coordinates.map(coord => ({
      x: parseFloat(toLonLat(coord)[0].toFixed(12)),
      y: parseFloat(toLonLat(coord)[1].toFixed(13))
    }));
    
    // 마지막 좌표가 첫 번째와 동일한 경우 제거
    if (coords.length > 1 && 
        coords[0].x === coords[coords.length - 1].x && 
        coords[0].y === coords[coords.length - 1].y) {
      coords.pop();
    }
    
    // 왼쪽 상단부터 반시계 방향으로 정렬
    return sortCoordinatesCounterClockwise(coords);
  })();

  // origin 좌표 계산 (C++ 코드와 동일)
  calculateOriginFromBoundary();

  return {
    main_vehicle: mainVehicles,
    sub_vehicle: subVehicles,
    working_area_boundary: boundaryCoords,
    timestamp: Date.now(),
    type: 1
  };
};

// 실제 임무대기 실행 (차량 선택 후)
const executeStandbyMission = (): void => {
  if (natsConnection.value) {
    // 선택된 차량으로 work_info 생성
    const dynamicWorkInfo = generateWorkInfo();
    
    // 임무 정보 생성
    const missionData = {
      mission_name: 'work',
      dest_lat: missionInfo.value.latitude,
      dest_lon: missionInfo.value.longitude,
      work_info: dynamicWorkInfo
    };
    
    const generateGlobalPath = JSON.stringify([missionData]);
    
    // command.ready 토픽으로 경로 정보 전송
    const topic = 'command.ready';
    
    try {
      natsConnection.value.publish(
        topic,
        stringCodec.value?.encode(generateGlobalPath)
      );
      console.log(`✅ NATS 임무대기 메시지 발행 성공 - 토픽: ${topic}`);
      console.log(`📤 발행된 메시지: ${generateGlobalPath}`);
      
      // 임무대기 상태로 설정
      missionStandby.value = true;
      messageBox('success', `임무대기 상태가 되었습니다. (총 ${selectedVehicles.value.length}대 차량)`);
    } catch (error) {
      messageBox('error', 'NATS 메시지 전송에 실패했습니다.');
    }
  } else {
    messageBox('error', 'NATS 연결이 되어있지 않습니다.');
  }
}

// 차량 선택 모달 열기
const openVehicleSelectModal = (): void => {
  // 기존 선택 초기화
  selectedVehicles.value = [];
  $vehicleSelectModal.value.onState();
};

// 차량 선택 모달 닫기
const closeVehicleSelectModal = (): void => {
  $vehicleSelectModal.value.offState();
};

// 차량 선택 확인
const confirmVehicleSelection = (): void => {
  closeVehicleSelectModal();
  // 실제 임무대기 실행
  executeStandbyMission();
};

// 임무대기 버튼 클릭 - 차량 선택 모달 열기
const standbyMission = (): void => {
  openVehicleSelectModal();
};

const hubData: Ref<HubDataObject> = ref({} as HubDataObject);

// 1세부 테스트 차량 표시
const setBasicVehicle = (lat: number, long: number): void => {
  const vehicleId = `vehicle`;
  let marker: OlFeature = osVector.getFeatureById(vehicleId) as OlFeature;
  
  // 텔레컨스 사이트인지 확인하여 적절한 아이콘 선택
  const isTeleconsSite = workInfo.value.name === '텔레컨스';
  const vehicleIcon = isTeleconsSite ? IconSource.T_SUB_VEHICLE_1 : IconSource.SUB_VEHICLE;
  
  if (marker != null) {
    marker.setGeometry(new OlPoint([long, lat]));
    vMap.addIcon(marker, vehicleIcon);
  }
  marker = new OlFeature({
    geometry: new OlPoint([long, lat]),
  });
  marker.setId(vehicleId);
  vMap.addIcon(marker, vehicleIcon);
  osVector.addFeature(marker);
}

// 1세부 테스트 경로 표시
const setBasicPath = (lat: number, long: number): void => {
  const lineId = `vehicle_l`;
  let line = osVector.getFeatureById(lineId);
  if (line != null) {
    const geo = line.getGeometry() as OlLineString;
    const coord = geo.getCoordinates();
    coord.push([long, lat]);
    line.setGeometry(new OlLineString(coord));
    return;
  }
  line = new OlFeature({
    geometry: new OlLineString([[long, lat]]),
  });
  line.setId(lineId);
  line.setStyle(lineStyle.MOVE);
  osVector.addFeature(line);
}

// 1세부 테스트 시 차량 및 경로 표시
const setBasicData = (ele: HubDataObject): void => {
  const [epsgLong, epsgLat] = fromLonLat([ele.position_long, ele.position_lat]);
  setBasicVehicle(epsgLat, epsgLong);
  setBasicPath(epsgLat, epsgLong);
}

const consumeMessage = async (sub: Subscription): Promise<void> => {
  let msgData;
  for await (const msg of sub) {
    const {subject} = msg; // 예: carInfo.1 또는 carState.F0
    const match = subject.match(/car(?:Info|State)\.(\w+)/);
    const vehicleIdFromTopic = match ? match[1] : undefined;

    msgData = stringCodec.value?.decode(msg?.data);

    try {
      if (!msgData) return;

      const data = JSON.parse(msgData);
      
      // vehicle_id가 없으면 토픽에서 추출
      if (!data.vehicle_id && vehicleIdFromTopic) {
        data.vehicle_id = vehicleIdFromTopic;
      }
      
      // vehicle_id가 여전히 없으면 경고하고 건너뛰기
      if (!data.vehicle_id && Object.keys(data).includes('obstacle')) {
        console.warn('⚠️ [consumeMessage] vehicle_id가 없고 토픽에서도 추출할 수 없습니다. 건너뜁니다.', {
          subject,
          hasObstacle: !!data.obstacle,
          obstacleCount: data.obstacle?.length || 0
        });
        return;
      }
      
      // yaw와 heading_angle 필드 통합 처리
      if (data.heading_angle !== undefined && data.yaw === undefined) {
        data.yaw = data.heading_angle;
      } else if (data.yaw === undefined && data.heading_angle === undefined) {
        // 둘 다 없는 경우 기본값 설정 (선택사항)
        // data.yaw = 0;
      }
      // sensor 데이터
      /* 장애물 데이터(sensor) 처리 로직 */
      if (Object.keys(data).includes('obstacle')) {
        // 실제 데이터가 처음 들어오는 시점에 workInfo.vehicleInfo 리셋
        if (!isFirstDataReceived && data.vehicle_id) {
          isFirstDataReceived = true;
          workInfo.value.vehicleInfo = {};
          console.log('🔄 [consumeMessage] 첫 데이터 수신 - workInfo.vehicleInfo 리셋');
        }
        
        // 디버깅 로그 (필요시만 활성화)
        // console.log('📦 센싱 데이터 수신:', data);
        hubData.value = data;
        
        // 임무 데이터 수집 (센싱 데이터가 있을 때만) // 데이터 수집 관련 모듈 주석 처리 (리포트)
        // if (missionDataCollector.isActive()) {
        //   missionDataCollector.addDataPoint(data as HubDataObject);
        // }
        
        // vehicle_state가 2인 경우 resetMission 후 setHubData 호출하지 않음
        if (data.vehicle_state === 2) {
          processSensingData(hubData.value); // resetMission 호출됨
          console.log('🚨 [consumeMessage] vehicle_state: 2 감지 - setHubData 호출 건너뜀');
        } else {
          processSensingData(hubData.value); // 새로운 융합 로직 적용
          // eslint-disable-next-line @typescript-eslint/no-use-before-define
          setHubData(hubData.value); // 기존 지도 표시 로직 유지
        }
        
        // 센싱 데이터가 들어올 때도 차량 정보 업데이트
        if (data.vehicle_id) {
          // eslint-disable-next-line @typescript-eslint/no-use-before-define
          setVehicleState(data);
        }
      }
      // 차량 위치 정보가 있으면 아이콘 표시 (obstacle이 없어도)
      else if (data.vehicle_id && data.position_long && data.position_lat) {
        // eslint-disable-next-line @typescript-eslint/no-use-before-define
        setHubData(data);
        console.log('🚘 차량 위치 정보 수신:', data);
      }
      // 차량 정보
      if (Object.keys(data).includes('vehicle_id')) {
        setVehicleState(data);
      }
      // 경로 (텔레컨스 사이트에서는 건너뜀 - 전용 토픽 사용)
      if (Object.keys(data).includes('route') && workInfo.value.name !== '텔레컨스') {
        console.log("Global Path created")
        // console.log(`Global Path: ${msgData}`)
        // eslint-disable-next-line @typescript-eslint/no-use-before-define
        setGlobalPath(data);
        // 경로 성능 데이터 수집
        collectPathPerformanceData(data);
      }
      // 차량 상태(배터리, 신호)
      if (Object.keys(data).includes('battery_info')) {
        console.log('🚘 carInfo 데이터 수신:', data);
        setVehicleState(data);
      }
    } catch (e: unknown) {
      // console.log(`skipped Data: ${msgData}, Error: ${e.stack}`);
    }
    // printFeatures();
  }
}

// 단일 JSON을 받는 경우
const subscribeHubData = async (): Promise<void> => {
  for (const topic of globalTopics) {
    const sub = natsStore.subscribe(topic);
    if (!sub) return;
    consumeMessage(sub);
  }
}

// Zone 그리드 업데이트 (기존 그리드 셀 색상 변경 방식)
const updateZoneGrid = (coordinates: Array<{position_long: number, position_lat: number}>, zoneType: 'red' | 'yellow' | 'green'): void => {
  // 텔레컨스 사이트가 아닌 경우 처리하지 않음
  if (workInfo.value.name !== '텔레컨스') {
    return;
  }

  // Zone 좌표를 투영 좌표계로 변환
  const zoneCoords = coordinates.map(coord => fromLonLat([coord.position_long, coord.position_lat]));

  // 모든 그리드 셀을 찾기
  const allFeatures = osVector.getFeatures();
  const gridCells = allFeatures.filter(feature => {
    const featureId = feature.getId() as string;
    return featureId && featureId.startsWith('grid_cell_');
  });

  // Zone이 있는 셀과 주변 셀들을 추적
  const cellsToHighlight = new Set<string>();

  gridCells.forEach(gridCell => {
    const geometry = gridCell.getGeometry();
    if (geometry && geometry.getType() === 'Polygon') {
      const polygon = geometry as Polygon;
      
      // Zone 좌표가 이 셀과 겹치는지 확인
      let hasZone = false;
      zoneCoords.forEach(zoneCoord => {
        if (polygon.intersectsCoordinate(zoneCoord)) {
          hasZone = true;
        }
      });
      
      // Zone이 있는 셀과 주변 셀들을 마킹
      if (hasZone) {
        const featureId = gridCell.getId() as string;
        const match = featureId.match(/grid_cell_(\d+)_(\d+)/);
        if (match) {
          const row = parseInt(match[1], 10);
          const col = parseInt(match[2], 10);
          cellsToHighlight.add(`grid_cell_${row}_${col}`);
        }
      }
    }
  });

  // 색상 설정
  let zoneStyle: Style;
  // eslint-disable-next-line default-case
  switch (zoneType) {
    case 'red':
      zoneStyle = new Style({
        fill: new Fill({
          color: 'rgba(255, 0, 0, 0.6)', // 반투명한 빨간색
        }),
        stroke: new Stroke({
          color: 'rgba(255, 0, 0, 0.6)', // 빨간색 테두리
          width: 1,
        }),
      });
      break;
    case 'yellow':
      zoneStyle = new Style({
        fill: new Fill({
          color: 'rgba(255, 255, 0, 0.6)', // 반투명한 노란색
        }),
        stroke: new Stroke({
          color: 'rgba(255, 255, 0, 0.6)', // 노란색 테두리
          width: 1,
        }),
      });
      break;
    case 'green':
      zoneStyle = new Style({
        fill: new Fill({
          color: 'rgba(0, 255, 0, 0.6)', // 반투명한 초록색
        }),
        stroke: new Stroke({
          color: 'rgba(0, 255, 0, 0.6)', // 초록색 테두리
          width: 1,
        }),
      });
      break;
  }

  // 마킹된 모든 셀을 해당 색상으로 변경
  gridCells.forEach(gridCell => {
    const featureId = gridCell.getId() as string;
    if (cellsToHighlight.has(featureId)) {
      gridCell.setStyle(zoneStyle);
    }
  });
}

// 모든 그리드 셀 색상을 원래대로 복원
const resetAllGridCellColors = (): void => {
  const allFeatures = osVector.getFeatures();
  const gridCells = allFeatures.filter(feature => {
    const featureId = feature.getId() as string;
    return featureId && featureId.startsWith('grid_cell_');
  });

  // 원래 그리드 스타일로 복원
  const originalStyle = new Style({
    fill: new Fill({
      color: 'rgba(240, 208, 146, 0.1)', // 원래 그리드 색상
    }),
    stroke: new Stroke({
      color: '#F0D092', // 원래 그리드 테두리 색상
      width: 1,
    }),
  });

  gridCells.forEach(gridCell => {
    gridCell.setStyle(originalStyle);
  });
}

// 텔레컨스 Zone(red, yellow, green) 구독
const subscribeTeleconsZone = async (): Promise<void> => {
  // RedZone 구독
  const redSub = natsStore.subscribe('telecons.redZone');
  if (redSub) {
    (async () => {
      for await (const msg of redSub) {
        try {
          const msgData = natsStore.stringCodec.decode(msg.data);
          const zoneData = JSON.parse(msgData);
          
          if (Array.isArray(zoneData)) {
            console.log('🔴 RedZone 데이터 수신:', zoneData);
            updateZoneGrid(zoneData as Array<{position_long: number, position_lat: number}>, 'red');
          }
        } catch (error) {
          console.error('RedZone 데이터 처리 중 에러:', error);
        }
      }
    })();
  }

  // YellowZone 구독
  const yellowSub = natsStore.subscribe('telecons.yellowZone');
  if (yellowSub) {
    (async () => {
      for await (const msg of yellowSub) {
        try {
          const msgData = natsStore.stringCodec.decode(msg.data);
          const zoneData = JSON.parse(msgData);
          
          if (Array.isArray(zoneData)) {
            console.log('🟡 YellowZone 데이터 수신:', zoneData);
            updateZoneGrid(zoneData as Array<{position_long: number, position_lat: number}>, 'yellow');
          }
        } catch (error) {
          console.error('YellowZone 데이터 처리 중 에러:', error);
        }
      }
    })();
  }

  // GreenZone 구독
  const greenSub = natsStore.subscribe('telecons.greenZone');
  if (greenSub) {
    (async () => {
      for await (const msg of greenSub) {
        try {
          const msgData = natsStore.stringCodec.decode(msg.data);
          const zoneData = JSON.parse(msgData);
          
          if (Array.isArray(zoneData)) {
            console.log('🟢 GreenZone 데이터 수신:', zoneData);
            updateZoneGrid(zoneData as Array<{position_long: number, position_lat: number}>, 'green');
          }
        } catch (error) {
          console.error('GreenZone 데이터 처리 중 에러:', error);
        }
      }
    })();
  }
}

// 텔레컨스 목적지 표시 함수
const updateTeleconsDestination = (vehicleId: string | number, positionLong: number, positionLat: number): void => {
  if (workInfo.value.name !== '텔레컨스') {
    return;
  }

  // vehicle_id를 문자열로 변환
  const vehicleIdStr = String(vehicleId);
  
  // 기존 목적지 제거 (같은 vehicle_id가 있으면)
  const existingDest = osVector.getFeatureById(`telecons_dest_${vehicleIdStr}`);
  if (existingDest) {
    osVector.removeFeature(existingDest);
  }

  // 새로운 목적지 추가
  const coord = fromLonLat([positionLong, positionLat]);
  
  const destFeature = new OlFeature({
    geometry: new OlPoint(coord),
  });
  destFeature.setId(`telecons_dest_${vehicleIdStr}`);

  // vehicle_id에 따른 아이콘 선택 (문자열 "1" 또는 숫자 1 모두 처리)
  const destIcon = (vehicleIdStr === '1' || vehicleId === 1) ? IconSource.T_SUB1_DEST : IconSource.T_SUB2_DEST;
  vMap.addIcon(destFeature, destIcon);
  
  osVector.addFeature(destFeature);
  
  console.log(`🎯 텔레컨스 목적지 업데이트: vehicle_id=${vehicleId}, 좌표=[${positionLong}, ${positionLat}]`);
};

// 텔레컨스 목적지 구독
const subscribeTeleconsDestination = async (): Promise<void> => {
  const destSub = natsStore.subscribe('telecons.regenPlanResult');
  if (destSub) {
    (async () => {
      for await (const msg of destSub) {
        try {
          const msgData = natsStore.stringCodec.decode(msg.data);
          const planData = JSON.parse(msgData);
          
          if (planData && (typeof planData.vehicle_id === 'number' || typeof planData.vehicle_id === 'string') && 
              typeof planData.position_long === 'number' && 
              typeof planData.position_lat === 'number') {
            console.log('🎯 RegenPlanResult 데이터 수신:', planData);
            updateTeleconsDestination(planData.vehicle_id, planData.position_long, planData.position_lat);
          }
        } catch (error) {
          console.error('RegenPlanResult 데이터 처리 중 에러:', error);
        }
      }
    })();
  }
};

// 전역 경로 데이터를 지도 좌표에 맞게 변환. WGS84 -> EPSG:3857
const generateRoute = (path: RoutePoint[]): Coordinate[] => {
  console.log("전역 경로 데이터를 지도 좌표");
  const coord: Coordinate[] = [];
  for (const point of path) {
    const x = point.x ?? point.longitude;
    const y = point.y ?? point.latitude;
    coord.push(fromLonLat([x, y]));
  }
  return coord;
}

// 차량별 색상 관리
const availableColors = ['#FF69B4', '#c869ff', '#69eeff', '#69ff96', '#ff8269'];
const vehicleColors = new Map<string, string>();

// 차량에 색상 할당 (이미 사용된 색 제외)
const assignVehicleColor = (vehicleId: string): string => {  
  // 이미 할당된 색이 있으면 그대로 사용
  if (vehicleColors.has(vehicleId)) {
    const existingColor = vehicleColors.get(vehicleId) as string;
    return existingColor;
  }
  
  // 사용 가능한 색상 중에서 랜덤 선택
  const usedColors = Array.from(vehicleColors.values());
  const availableColorsForSelection = availableColors.filter(color => !usedColors.includes(color));
  
  // 모든 색이 사용 중이면 처음부터 다시 시작
  const colorsToChooseFrom = availableColorsForSelection.length > 0 ? availableColorsForSelection : availableColors;
  
  const randomIndex = Math.floor(Math.random() * colorsToChooseFrom.length);
  const selectedColor = colorsToChooseFrom[randomIndex];
  
  vehicleColors.set(vehicleId, selectedColor);
  return selectedColor;
};

// 전역 경로 표시
const setGlobalPath = (path: Path): void => {
  const globalPath = generateRoute(path.route);
  
  // 차량별 색상 할당
  const vehicleId = String(path.vehicle_id);
  const vehicleColor = assignVehicleColor(vehicleId);
  
  // 기존 전역 경로 요소들 제거 (포인트와 라인 모두)
  const allFeatures = osVector.getFeatures();
  const pathFeaturesToRemove: OlFeature[] = [];
  
  allFeatures.forEach((feature) => {
    const featureId = feature.getId() as string;
    if (featureId && (
      featureId.startsWith(`global_path_point_${path.vehicle_id}_`) ||
      featureId.startsWith(`global_path_line_${path.vehicle_id}`)
    )) {
      pathFeaturesToRemove.push(feature);
    }
  });
  
  // 기존 경로 요소들 제거
  pathFeaturesToRemove.forEach((feature) => {
    osVector.removeFeature(feature);
  });
  
  // 경로 라인 생성 (가느다란 선)
  if (globalPath.length > 1) {
    const pathLine = new OlFeature({
      geometry: new OlLineString(globalPath),
    });
    pathLine.setId(`global_path_line_${path.vehicle_id}`);
    
    // 가느다란 선 스타일
    const pathLineStyle = new Style({
      stroke: new Stroke({
        color: vehicleColor,
        width: 2,
        lineDash: [3, 3] // 점선 효과
      }),
    });
    pathLine.setStyle(pathLineStyle);
    osVector.addFeature(pathLine);
  }
  
  // 각 경로 포인트를 개별 점으로 표시 (차량 ID 텍스트 포함)
  globalPath.forEach((coord, index) => {
    const pointId = `global_path_point_${path.vehicle_id}_${index}`;
    
    // 경로 점 생성
    const pathPoint = new OlFeature({
      geometry: new OlPoint(coord),
    });
    pathPoint.setId(pointId);
    
    // 점 스타일 적용 (텍스트 포함)
    const zoom = olMap.getView().getZoom() || 8;
    const pointStyle = new Style({
      image: new CircleStyle({
        radius: getScaleForZoom(zoom) * 22,
        fill: new Fill({ color: vehicleColor }),
        stroke: new Stroke({ color: '#FFFFFF', width: 1 }),
      }),
      text: new Text({
        text: String(path.vehicle_id),
        font: '8px Arial, sans-serif',
        fill: new Fill({ color: '#FFFFFF' }),
        textAlign: 'center',
        textBaseline: 'middle',
      }),
    });
    pathPoint.setStyle(pointStyle);
    
    osVector.addFeature(pathPoint);
  });
}

// 텔레컨스 경로 구독
const subscribeTeleconsRoute = async (): Promise<void> => {
  const routeSub = natsStore.subscribe('telecons.regenRouteResult');
  if (routeSub) {
    (async () => {
      for await (const msg of routeSub) {
        try {
          const msgData = natsStore.stringCodec.decode(msg.data);
          const routeData = JSON.parse(msgData);
          
          if (routeData && (typeof routeData.vehicle_id === 'number' || typeof routeData.vehicle_id === 'string') && Array.isArray(routeData.route)) {
            console.log('🛣️ RegenRouteResult 데이터 수신:', routeData);
            
            // 기존 경로 제거
            const allFeatures = osVector.getFeatures();
            for (const feature of allFeatures) {
              const featureId = feature.getId() as string;
              if (featureId && (
                featureId.startsWith(`global_path_point_${routeData.vehicle_id}_`) ||
                featureId.startsWith(`global_path_line_${routeData.vehicle_id}`)
              )) {
                osVector.removeFeature(feature);
              }
            }
            
            // 새로운 경로 표시 (기존 setGlobalPath 로직 사용)
            const path = {
              vehicle_id: routeData.vehicle_id,
              route: routeData.route.map((point: any) => ({
                latitude: point.position_lat,
                longitude: point.position_long
              }))
            };
            setGlobalPath(path);
          }
        } catch (error) {
          console.error('RegenRouteResult 데이터 처리 중 에러:', error);
        }
      }
    })();
  }
};

// 1세부
const subscribeFirst = async (): Promise<void> => {
  const sub = natsStore.subscribe(env.natsSubscribeInfo.subTopic);
  if (!sub) return;
  let msgData;
  for await (const msg of sub) {
    msgData = stringCodec.value?.decode(msg?.data);
    try {
      const json = JSON.parse(msgData as string);
      // eslint-disable-next-line no-continue
      if (!Object.keys(json).includes('lat')) continue;
      hubData.value = json;
      setBasicData(hubData.value);
    } catch (e: any) {
      console.log(`skipped Data: ${msgData}, Error: ${e.stack}`);
    }
  }
}

const subscribe = async (): Promise<void> => {
  // command.start 토픽으로 JSON 형태의 메시지 전송
  const topic = 'command.start';
  const message = JSON.stringify("start");
  try {
    natsConnection.value?.publish(
      topic,
      stringCodec.value?.encode(message)
    );
    console.log(`✅ NATS 임무시작 메시지 발행 성공 - 토픽: ${topic}`);
    console.log(`📤 발행된 메시지: ${message}`);
    
    // 임무 데이터 수집 시작 // 데이터 수집 관련 모듈 주석 처리 (리포트)
    const siteId = route.params.id as string;
    const missionId = missionInfo.value?.missionId;
    // const sessionId = missionDataCollector.startCollection(siteId, missionId);
    // console.log(`📊 임무 데이터 수집 시작 - Session ID: ${sessionId}`);
    
    workState.value = WorkState.RUNNING;
    $modalRef.value.offState();
    modalTriggered.value = false;
    if (route.params.id === 'one_div_test') {
      await subscribeFirst();
    } else {
      await subscribeHubData();
    }
  } catch (error) {
    messageBox('error', '임무시작 메시지 전송에 실패했습니다.');
  }
}

const stopMission = (): void => {
  workState.value = WorkState.PAUSE;
  // Unsubscribe from all topics
  for (const topic of globalTopics) {
    natsStore.unsubscribe(topic);
  }
}

const resumeMission = async (): Promise<void> => {
  workState.value = WorkState.RUNNING;
  // Resubscribe based on the condition
  if (route.params.id === 'one_div_test') {
    await subscribeFirst();
  } else {
    await subscribeHubData();
  }
}

const resetMission = async (): Promise<void> => {
  workState.value = WorkState.WAIT;
  // 임무대기 상태도 초기화
  missionStandby.value = false;
  // 장애물 ID 목록 초기화
  previousObstacleList.length = 0;
  // 노면 데이터 초기화
  clearRoadZData();
  
  // 차량 데이터 초기화
  vehicleDataMap.clear();
  activeVehicleIds.clear(); // 실제 데이터가 들어온 차량 추적도 초기화
  isFirstDataReceived = false; // 첫 데이터 수신 플래그 리셋
  console.log('🔄 [resetMission] 차량 데이터 초기화 완료');
  
  // 임무 데이터 수집 중지 // 데이터 수집 관련 모듈 주석 처리 (리포트)
  // const session = missionDataCollector.stopCollection();
  // if (session) {
  //   console.log(`📊 [resetMission] 데이터 수집 종료 - 총 ${session.dataPoints.length}개 데이터 포인트 수집됨`);
  // }
  
  // IDManager 초기화
  idManager.reset();
  console.log('🔄 [resetMission] IDManager 초기화 완료');
  // NATS 연결 해제
  for (const topic of globalTopics) {
    natsStore.unsubscribe(topic);
  }
  // 초기 표시 요소 제외 후 모든 요소 제거
  const initialFeatures = ['sitePolygon', 'entrance', 'exit', 'destination', 'basePoint'];
  const featuresToRemove = osVector.getFeatures().filter((feature) => {
    return !initialFeatures.includes(feature.getId() as string);
  });

  featuresToRemove.forEach((feature) => {
    osVector.removeFeature(feature);
    console.log('🔄 [resetMission] 요소 제거:', feature.getId());
  });
}

onMounted(async () => {
  // 타임스탬프 업데이트 시작
  updateTimestamp();
  timestampInterval = setInterval(updateTimestamp, 1);
  
  // NATS 연결
  await natsStore.setConnection();
  if (!natsStore.isConnected) {
    messageBox('error', '차량 연결에 실패했습니다.');
    router.push('/home');
  }
  // NATS Publish: 경로 생성 모듈 테스트시 차량 및 영역 정보 publish
  if (route.params.id !== 'one_div_test') {
    await getWorkInfo(route.params.id as string)
      .then((res: unknown | WorkInfo) => { 
        workInfo.value = res as WorkInfo;
        // workInfo의 vehicleInfo는 서버에서 가져온 매핑된 차량 정보를 그대로 사용
        // vehicleInfo가 없으면 빈 객체로 초기화
        if (!workInfo.value.vehicleInfo) {
          workInfo.value.vehicleInfo = {};
        }
        console.log('📋 [onMounted] 서버에서 가져온 차량 정보:', {
          vehicleInfo: workInfo.value.vehicleInfo,
          vehicleIds: Object.keys(workInfo.value.vehicleInfo),
        });
        // workInfo 로드 후 origin 좌표 계산 (센싱 데이터 처리 전에 설정)
        calculateOriginFromBoundary();
      })
    await getMissionInfo(route.params.id as string)
      .then((res: MissionTableData | unknown) => {
      if (res) {
        missionInfo.value = res as MissionTableData;
      } else {
        console.log('미션 정보가 없습니다.');
      }
    })
  }

  // 지도 초기화 및 구독
  if (route.params.id === 'one_div_test') {
    center = [14149396.222052526, 4494566.502095682];
    olMap = vMap.initMap(map, center);
    vMap.changeMapType(olMap);
    olMap.on('click', (evt) => {
      console.log('coordinate 투영좌표[] - ', evt.coordinate);
      console.log('coordinate => lnglat [경도,위도]', toLonLat(evt.coordinate));
    });
    olMap.getView().setZoom(16);
    osVector = new OSVector({
      features: [],
    });
    olLayer = new OLVector({
      source: osVector,
      zIndex: 3,
    });
    olMap.addLayer(olLayer);
    subscribeFirst();
    subscribeControlStatus();
  } else {
    // 경로 생성 모듈 테스트시 기본 좌표 사용
    center = workInfo && !isEmpty(workInfo.value.coordinates)
      ? calculateCenter(workInfo.value.coordinates)
      : fromLonLat(vMap.DEFAULT_DEMO_POSITION);

    olMap = vMap.initMap(map, center);
    olMap.getView().fit(vMap.setThumbnailCoords(workInfo.value.coordinates));
    vMap.changeMapType(olMap);
    olMap.on('click', (evt) => {
      console.log('coordinate 투영좌표[] - ', evt.coordinate);
      console.log('coordinate => lnglat [경도,위도]', toLonLat(evt.coordinate));
    });
    olMap.getView().setZoom(vMap.DEFAULT_SITE_ZOOM_LEVEL);
    osVector = new OSVector({
      // TODO: NATS 알림 params 적용되면 연산자 삭제
      features: vMap.getSiteFeatures(workInfo.value),
    });
    // 기준점 표시
    const basePoint = new OlFeature({
      geometry: new OlPoint(SITE_ABSOLUTE_COORDINATE),
    });
    basePoint.setId('basePoint');
    vMap.addIcon(basePoint, IconSource.BASE);
    osVector.addFeature(basePoint);
    // 목적지 표시 (missionInfo의 위도와 경도로 destCoord 설정)
    const destCoord = fromLonLat([missionInfo.value.longitude, missionInfo.value.latitude]);
    const dest = new OlFeature({
      geometry: new OlPoint(destCoord),
    });
    dest.setId('destination');
    vMap.addIcon(dest, IconSource.DESTINATION)
    osVector.addFeature(dest);
    olLayer = new OLVector({
      updateWhileAnimating: true,
      updateWhileInteracting: true,
      source: osVector,
      zIndex: 3,
    });
    olMap.addLayer(olLayer);
    
    // 텔레컨스 사이트인 경우 초기 그리드 그리기 및 Zone 구독
    if (workInfo.value.name === '텔레컨스') {
      drawTeleconsGrid();
      subscribeTeleconsZone();
      subscribeTeleconsDestination();
      subscribeTeleconsRoute();
    }
    
    subscribeHubData();
    subscribeControlStatus();
  }
  useMapStore().map = olMap;
});

onUnmounted(async () => {
  // 타임스탬프 인터벌 정리
  if (timestampInterval) {
    clearInterval(timestampInterval);
  }
  
  // NATS 구독 해제
  for (const topic of globalTopics) {
    natsStore.unsubscribe(topic);
  }
  
  // 텔레컨스 Zone 구독 해제
  if (workInfo.value.name === '텔레컨스') {
    natsStore.unsubscribe('telecons.redZone');
    natsStore.unsubscribe('telecons.yellowZone');
    natsStore.unsubscribe('telecons.greenZone');
    natsStore.unsubscribe('telecons.regenPlanResult');
    natsStore.unsubscribe('telecons.regenRouteResult');
  }
  
  await natsStore.closeConnection();
});

/* MapData 장애물 관련 코드 */
// 장애물 데이터를 저장할 배열
const storedObstacles: MapObstacle[] = [];

// 장애물의 좌표를 계산해 저장 (WGS84 좌표계)
const setMapObstacles = (obstacles: MapObstacle[]): void => {  
  if (!obstacles || obstacles.length === 0) {
    return;
  }
  console.log('setMapObstacles');
  const filteredObstacles = obstacles.filter((o) => o.obstacle_class >= 1);

  if (filteredObstacles.length > 0) {    
    // 기존 장애물 데이터 초기화 (옵션)
    storedObstacles.length = 0;
    // 새 장애물 데이터 저장
    storedObstacles.push(...filteredObstacles);
  }
}

// 차량의 좌표를 계산해 표시 (WGS84 좌표계)
// eslint-disable-next-line @typescript-eslint/no-shadow
const addVehicle = (hubData: HubDataObject, vehicle_long: number, vehicle_lat: number): void => {  
  const vehicleId = `vehicle${hubData.vehicle_id}`;
  const yaw = -hubData.yaw;
  
  let marker: OlFeature<Point> = osVector.getFeatureById(vehicleId) as OlFeature<Point>;
  const coord = fromLonLat([vehicle_long, vehicle_lat]);
  
  // 텔레컨스 사이트인지 확인
  const isTeleconsSite = workInfo.value.name === '텔레컨스';
  
  // 차량 타입에 따른 아이콘 선택
  const getVehicleIcon = (id: number): IconSource => {
    if (isTeleconsSite) {
      // 텔레컨스 사이트: 특장차량용 이미지 사용
      if (id <= 4) {
        // 보조차량 1, 2번에 따라 다른 이미지 사용
        return id === 1 ? IconSource.T_SUB_VEHICLE_1 : IconSource.T_SUB_VEHICLE_2;
      }
      return IconSource.T_MAIN_VEHICLE;
    }
    // 일반 사이트: 기본 이미지 사용
    return id <= 4 ? IconSource.SUB_VEHICLE : IconSource.MAIN_VEHICLE;
  };
  
  if (marker != null) {
    marker.getGeometry()?.setCoordinates(coord);
    // 차량 데이터 업데이트
    marker.set('vehicleData', { long: vehicle_long, lat: vehicle_lat });
    vMap.updateIcon(marker, getVehicleIcon(Number(hubData.vehicle_id)), yaw);
    
    // 디버깅 모드일 때 차량 좌표 표시
    if (isDebugging.value) {
      updateVehicleDebugStyle(marker, hubData.vehicle_id.toString());
    }
    return;
  }
  
  marker = new OlFeature({
    geometry: new OlPoint(coord),
  });
  marker.setId(vehicleId);
  // 차량 데이터 저장 (디버깅 모드에서 사용)
  marker.set('vehicleData', { long: vehicle_long, lat: vehicle_lat });
  vMap.addIcon(marker, getVehicleIcon(Number(hubData.vehicle_id)), yaw);
  osVector.addFeature(marker);
  
  // 디버깅 모드일 때 차량 좌표 표시
  if (isDebugging.value) {
    updateVehicleDebugStyle(marker, hubData.vehicle_id.toString());
  }
}

// 경로 표시 (WGS84 좌표계)
// eslint-disable-next-line @typescript-eslint/no-shadow
const addPath = (hubData: HubDataObject, vehicle_long: number, vehicle_lat: number): void => {
  const lineId = `movePath${hubData.vehicle_id}`;
  let line: OlFeature<OlLineString> = osVector.getFeatureById(lineId) as OlFeature<OlLineString>;
  const pointCoord = fromLonLat([vehicle_long, vehicle_lat]); // OlMap의 좌표계는 ESPG:3857이므로 WGS84 좌표계를 해당 좌표계로 변환 필요
  // 좌표가 (0, 0) 또는 특정 좌표일 경우 포함하지 않음
  if (
    (vehicle_long === 0 && vehicle_lat === 0) ||
    (vehicle_long === 126.54211179911135 && vehicle_lat === 35.94343439990743)
  ) {
    return;
  }
  if (line != null) {
    line.getGeometry()?.appendCoordinate(pointCoord);
    return;
  }
  line = new OlFeature({
    geometry: new OlLineString([pointCoord]),
  });
  line.setId(lineId);
  line.setStyle(lineStyle.MOVE);
  osVector.addFeature(line);
}

// 차량, 경로 데이터를 지도에 표시
const setHubData = (ele: HubDataObject): void => {
  // eslint-disable-next-line @typescript-eslint/naming-convention
  const { position_lat, position_long } = ele;
  addVehicle(ele, position_long, position_lat);
  addPath(ele, position_long, position_lat);
  // 장애물 표시는 processSensingData에서 통합 처리하므로 제거
  // if (obstacleDataSource.value === 'sensor') addSensorObstacles(ele);
}

const toggleRoadZBtn = (): void => {
  isroadZOn.value = !isroadZOn.value;
  
  // off 상태일 때 roadz 데이터 제거 및 초기화
  if (!isroadZOn.value) {
    removeRoadZFeatures();
    clearRoadZData();
  }
};

// 디버깅 모드 토글 함수
const toggleDebugging = (): void => {
  isDebugging.value = !isDebugging.value;
  // 디버깅 모드 변경 시 장애물 스타일 업데이트
  updateObstacleStyles();
  // 디버깅 모드 변경 시 차량 스타일 업데이트
  updateVehicleStyles();
};
</script>

<style lang="scss" scoped>
@import '@/styles/variables.scss';
#map {
  position: relative;
  height: 100vh;
}

.roadZ_btn {
  position: absolute;
  top: 23px;
  left: 85px;
  z-index: 10;
  border-radius: 20px;
}

.roadZ_on {
  background-color: #de9f00;
  color: white;
  &:hover {
    background-color: white;
    color: #de9f00;
  }
}

.roadZ_off {
  background-color: white;
  color: #de9f00;
  &:hover {
    background-color: #de9f00;
    color: white;
  }
}

.debug_btn {
  position: absolute;
  top: 23px;
  left: 190px;
  z-index: 10;
  border-radius: 20px;
}

.debug_on {
  background-color: #be2b2b;
  color: white;
  &:hover {
    background-color: white;
    color: #be2b2b;
  }
}

.debug_off {
  background-color: white;
  color: #be2b2b;
  &:hover {
    background-color: #be2b2b;
    color: white;
  }
}

.debug-panel {
  position: absolute;
  top: 70px;
  left: 20px;
  background-color: rgba(0, 0, 0, 0.496);
  color: red;
  padding: 10px;
  border-radius: 5px;
  font-size: 20px;
  font-weight: 800;
  z-index: 10;
}

.debug-timestamp {
  white-space: nowrap;
}

.debug-status {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-top: 5px;
}

.status-label {
  font-weight: bold;
  color: #fff;
}

.status-value {
  padding: 4px 8px;
  border-radius: 4px;
  font-weight: bold;
  min-width: 80px;
  text-align: center;
}

.status-disconnected {
  background-color: #666;
  color: #fff;
}

.status-idle {
  background-color: #4CAF50;
  color: #fff;
}

.status-search {
  background-color: #2196F3;
  color: #fff;
}

.status-return {
  background-color: #FF9800;
  color: #fff;
}

.status-move {
  background-color: #9C27B0;
  color: #fff;
}

.status-work {
  background-color: #F44336;
  color: #fff;
}

.status-unknown {
  background-color: #607D8B;
  color: #fff;
}

.standby_btn {
  position: absolute;
  top: 23px;
  right: 180px;
  background-color: white;
  color: #3D5BC1;
  border-radius: 20px;
  &:hover {
    background-color: #3D5BC1;
    color: white;
  }
  z-index: 10;
  
  &:disabled {
    opacity: 1 !important;  /* 투명도 유지 */
  }
}

.standby-active {
  background-color: #3D5BC1 !important;
  color: white !important;
  
  &:disabled {
    background-color: #3D5BC1 !important;
    color: white !important;
    opacity: 1 !important;
  }
}

.reset_btn {
  position: absolute;
  top: 23px;
  right: 290px;
  background-color: white;
  color: #087212;
  border-radius: 20px;
  &:hover {
    background-color: #087212;
    color: white;
  }
  z-index: 10;
}

.stop_btn {
  position: absolute;
  top: 23px;
  right: 220px;
  background-color: white;
  color: #be2b2b;
  border-radius: 20px;
  &:hover {
    background-color: #be2b2b;
    color: white;
  }
  z-index: 10;
}

.resume_btn {
  position: absolute;
  top: 23px;
  right: 220px;
  background-color: white;
  color: #3957c8;
  border-radius: 20px;
  &:hover {
    background-color: #3957c8;
    color: white;
  }
  z-index: 10;
}
</style>