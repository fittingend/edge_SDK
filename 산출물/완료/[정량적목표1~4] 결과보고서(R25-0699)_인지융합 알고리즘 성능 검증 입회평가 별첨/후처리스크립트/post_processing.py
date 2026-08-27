import argparse
import json
import math
import re
from collections import defaultdict, deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple, List, Set, Any

# v3: use exact polygon↔cell intersection rasterizer (SAT) from separate module
try:
    from rasterize_polygon_cells import rasterize_polygon_to_cells as rasterize_polygon_to_cells_sat
except Exception as e:
    rasterize_polygon_to_cells_sat = None  # will raise in scenario3 if used



@dataclass
class Config:
    scenario: int
    log_path: str
    fusion_count: Optional[int] = None
    only_module: str = "DAFU"   # "DAFU"면 DAFU 로그만 필터링

    # Scenario 3 옵션(없으면 기본값 사용)
    # - cell_size: 좌표 단위가 dm(=0.1m) 기준이면 1.0을 사용 (즉, 1 cell = 1 dm)
    #   좌표가 meter라면 0.1을 넣는 식으로 조정 가능.
    cell_size: float = 1.0

    # 출력에서 너무 길어지는 걸 방지
    max_list_cells: int = 30

    # Scenario 3: 특정 장애물만 보고 싶을 때 (없으면 전체)
    target_obstacle_ids: Optional[List[int]] = None

    # Scenario 4 옵션: 지정 장애물 위치 오차(유클리디안)
    # - target_obstacle_id: [FINAL_OBS]에서 찾을 장애물 ID
    # - gt_x/gt_y 또는 target_gt_xy: GT 좌표 (로그와 같은 단위; 예: dm)
    # - pos_error_thresh_cm: PASS 기준 (cm). 기본 80cm (=8dm)
    # - scenario4_not_found_policy: NOT_FOUND 프레임 처리 정책
    #     * "fail"(기본): NOT_FOUND가 1개라도 있으면 FAIL
    #     * "exclude": NOT_FOUND 프레임은 통계/판정에서 제외
    target_obstacle_id: Optional[int] = None
    gt_x: Optional[float] = None
    gt_y: Optional[float] = None
    target_gt_xy: Optional[List[float]] = None
    pos_error_thresh_cm: float = 80.0
    scenario4_not_found_policy: str = "fail"


# -----------------------------
# 공통(Scenario 1)
# -----------------------------
FUSION_TIME_LINE_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[KATECH\]\s*FUSION_TIME:\s*(?P<ms>[0-9]*\.?[0-9]+)\s*ms\.?\b",
    re.IGNORECASE,
)
# Fusion start/stop with timestamp (float after wall-clock time)
FUSION_START_RE = re.compile(
    r"^\s*\d+\s+\d{4}/\d{2}/\d{2}\s+\d{2}:\d{2}:\d{2}\.\d+\s+(?P<ts>[0-9]*\.?[0-9]+).*?\[F(?P<frame>\d+)\].*?\[KATECH\]\s*FUSION_START\b",
    re.IGNORECASE,
)
FUSION_STOP_RE = re.compile(
    r"^\s*\d+\s+\d{4}/\d{2}/\d{2}\s+\d{2}:\d{2}:\d{2}\.\d+\s+(?P<ts>[0-9]*\.?[0-9]+).*?\[F(?P<frame>\d+)\].*?\[KATECH\]\s*FUSION_STOP\b",
    re.IGNORECASE,
)
# Backward-compat (old format): a separate [FUSION_TIME] line + frame marker line
FUSION_RE = re.compile(r"\[FUSION_TIME\].*?:\s*(\d+)\s*ms\b")
FRAME_START_RE = re.compile(r"(\d+)\s*번째\s*로컬\s*mapdata\s*전송\s*시작")


# -----------------------------
# Scenario 2: 작업영역 범위 분석
# -----------------------------
FRAME_TAG_RE = re.compile(r"\[F(?P<frame>\d+)\]")
MAP_SIZE_RE = re.compile(r"\[WORKINFO\]\s*MAP_SIZE:\s*\(\s*(?P<w>\d+)\s*,\s*(?P<h>\d+)\s*\)")
REL_BEFORE_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[RELATIVETOMAP\]\s*장애물\s*class\s*(?P<cls>\d+)\s*좌표변환\s*before\s*\(\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\)"
)
REL_AFTER_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[RELATIVETOMAP\]\s*장애물\s*class\s*(?P<cls>\d+)\s*좌표변환\s*after\s*\(\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\)"
)
REL_REMOVE_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[RELATIVETOMAP\]\s*맵\s*범위\s*밖\s*장애물\s*제거:\s*class=\s*(?P<cls>\d+)\s*,\s*pos=\(\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\)"
)


# -----------------------------
# Scenario 3: 장애물/점유셀 파싱
# -----------------------------
OBST_CORNERS_RE = re.compile(
    r"장애물\s*ID:\s*(\d+)\s*4개\s*꼭지점\s*좌표:\s*"
    r"LU\(\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\)\s*,\s*"
    r"RU\(\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\)\s*,\s*"
    r"RL\(\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\)\s*,\s*"
    r"LL\(\s*([-\d\.]+)\s*,\s*([-\d\.]+)\s*\)"
)

OBST_CELL_RE = re.compile(
    r"장애물\s*ID\s*(\d+)\s*위치\s*인덱스\s*추가:\s*\[\s*(\d+)\s*,\s*(\d+)\s*\]"
)


# New log format (2026-01-27):
#   ... [F36] [OCCUPANCY] obstacle 4 ( 42 ) corners LU= ... RU= ... RL= ... LL= ...
#   ... [F36] [OCCUPANCY] obstacle 4 ( 42 ) : index ( 186 , 409 )
OCC_CORNERS_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[OCCUPANCY\]\s*obstacle\s*(?P<oid>\d+)\s*\(\s*\d+\s*\)\s*corners\s*"
    r"LU=\s*(?P<lux>[-\d\.]+)\s*,\s*(?P<luy>[-\d\.]+)\s*"
    r"RU=\s*(?P<rux>[-\d\.]+)\s*,\s*(?P<ruy>[-\d\.]+)\s*"
    r"RL=\s*(?P<rlx>[-\d\.]+)\s*,\s*(?P<rly>[-\d\.]+)\s*"
    r"LL=\s*(?P<llx>[-\d\.]+)\s*,\s*(?P<lly>[-\d\.]+)"
)
OCC_INDEX_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[OCCUPANCY\]\s*obstacle\s*(?P<oid>\d+)\s*\(\s*\d+\s*\)\s*:\s*index\s*\(\s*(?P<x>-?\d+)\s*,\s*(?P<y>-?\d+)\s*\)"
)


# -----------------------------
# Scenario 4: FINAL_OBS 위치 파싱
# -----------------------------
FINAL_OBS_RE = re.compile(
    r"\[FINAL_OBS\].*?id=\s*(\d+)\s*,.*?pos=\(\s*([\-\d\.]+)\s*,\s*([\-\d\.]+)\s*,"
)

# Scenario 4 (NEW): MERGELIST ID 할당 파싱
MERGELIST_ASSIGN_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\[MERGELIST\].*?ID\s*할당\s*(?P<id>\d+)\s*:\s*\[\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\]"
)
# [MERGELIST] 최종 융합 장애물 리스트 출력: 아래 줄에 나오는 ID 라인
MERGELIST_FINAL_ID_RE = re.compile(
    r"\[F(?P<frame>\d+)\].*?\bID\s*(?P<id>\d+)\s*\(\s*\d+\s*\)\s*:\s*\[\s*(?P<x>[-\d\.]+)\s*,\s*(?P<y>[-\d\.]+)\s*\]"
)


def _normalize_target_ids(v: Any) -> Optional[List[int]]:
    """YAML/JSON에서 들어오는 target_obstacle_ids를 안전하게 int 리스트로 정규화."""
    if v is None:
        return None
    if isinstance(v, (list, tuple)):
        out: List[int] = []
        for x in v:
            try:
                out.append(int(x))
            except Exception:
                continue
        return out if out else None
    try:
        return [int(v)]
    except Exception:
        return None


def load_config(path: str) -> Config:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config not found: {p}")

    text = p.read_text(encoding="utf-8")
    if p.suffix.lower() in (".yaml", ".yml"):
        try:
            import yaml  # pip install pyyaml
        except ImportError:
            raise RuntimeError("YAML config를 쓰려면 `pip install pyyaml`가 필요합니다. JSON으로도 가능합니다.")
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)

    fusion_count = data.get("fusion_count", data.get("frame_count", None))
    fusion_count = int(fusion_count) if fusion_count is not None else None

    return Config(
        scenario=int(data["scenario"]),
        log_path=str(data["log_path"]),
        fusion_count=fusion_count,
        only_module=str(data.get("only_module", "DAFU")),
        cell_size=float(data.get("cell_size", 1.0)),
        max_list_cells=int(data.get("max_list_cells", 30)),
        target_obstacle_ids=_normalize_target_ids(data.get("target_obstacle_ids", None)),
        target_obstacle_id=(int(data.get('target_obstacle_id')) if data.get('target_obstacle_id') is not None else None),
        gt_x=(
            float(data.get('gt_x'))
            if data.get('gt_x') is not None
            else (
                float(data.get('gt')[0])
                if isinstance(data.get('gt'), (list, tuple)) and len(data.get('gt')) > 0
                else (
                    float(data.get('target_gt_xy')[0])
                    if isinstance(data.get('target_gt_xy'), (list, tuple)) and len(data.get('target_gt_xy')) > 0
                    else None
                )
            )
        ),
        gt_y=(
            float(data.get('gt_y'))
            if data.get('gt_y') is not None
            else (
                float(data.get('gt')[1])
                if isinstance(data.get('gt'), (list, tuple)) and len(data.get('gt')) > 1
                else (
                    float(data.get('target_gt_xy')[1])
                    if isinstance(data.get('target_gt_xy'), (list, tuple)) and len(data.get('target_gt_xy')) > 1
                    else None
                )
            )
        ),
        target_gt_xy=(
            [float(data.get('target_gt_xy')[0]), float(data.get('target_gt_xy')[1])]
            if isinstance(data.get('target_gt_xy'), (list, tuple)) and len(data.get('target_gt_xy')) > 1
            else None
        ),
        pos_error_thresh_cm=float(data.get('pos_error_thresh_cm', data.get('pos_error_threshold_cm', 80.0))),
        scenario4_not_found_policy=str(data.get("scenario4_not_found_policy", data.get("not_found_policy", "fail"))),
    )


# -----------------------------
# 공통: 프레임 목록 유틸
# -----------------------------
def _get_frame_tags_in_order(
    log_path: str, only_module: str = "DAFU", max_frames: Optional[int] = None
) -> List[int]:
    """로그의 [F#] 태그를 순서대로 모아서 반환 (중복 제거, 등장 순서 유지)."""
    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    frames: List[int] = []
    seen: Set[int] = set()
    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue
            m = FRAME_TAG_RE.search(line)
            if not m:
                continue
            fr = int(m.group("frame"))
            if fr in seen:
                continue
            seen.add(fr)
            frames.append(fr)
            if max_frames is not None and len(frames) >= max_frames:
                break
    return frames


# -----------------------------
# Scenario 1
# -----------------------------
def parse_fusion_times_by_frame(log_path: str, only_module: str = "DAFU") -> Dict[int, float]:
    """Scenario 1 parser.

    Supports *new* log format:
      ... [F1] [KATECH] FUSION_TIME: 10.29085100 ms.

    And keeps backward-compat with the old format (FUSION_RE + FRAME_START_RE pairing).
    """
    fusion_by_frame: Dict[int, float] = {}

    # old-format state
    last_fusion_ms_int: Optional[int] = None

    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            # module filter (DAFU만)
            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue

            # New format: directly contains frame + ms
            m_new = FUSION_TIME_LINE_RE.search(line)
            if m_new:
                fr = int(m_new.group("frame"))
                ms = float(m_new.group("ms"))
                fusion_by_frame[fr] = ms
                continue

            # Old format: [FUSION_TIME] ... : <int> ms  (then later a frame marker line)
            m_old = FUSION_RE.search(line)
            if m_old:
                last_fusion_ms_int = int(m_old.group(1))
                continue

            m_fr = FRAME_START_RE.search(line)
            if m_fr and last_fusion_ms_int is not None:
                frame = int(m_fr.group(1))
                fusion_by_frame[frame] = float(last_fusion_ms_int)
                last_fusion_ms_int = None

    return fusion_by_frame

def parse_fusion_times_in_order(
    log_path: str, only_module: str = "DAFU"
) -> List[Tuple[int, float, Optional[float], Optional[float]]]:
    """Scenario 1 parser (ordered list).

    로그 등장 순서대로 (frame, ms, fusion_start_ts, fusion_end_ts) 리스트를 만든다.
    """
    rows: List[Tuple[int, float, Optional[float], Optional[float]]] = []
    last_fusion_ms_int: Optional[int] = None
    fusion_start_by_frame: Dict[int, float] = {}
    fusion_stop_by_frame: Dict[int, float] = {}

    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue

            m_start = FUSION_START_RE.search(line)
            if m_start:
                fr = int(m_start.group("frame"))
                fusion_start_by_frame[fr] = float(m_start.group("ts"))
                continue

            m_stop = FUSION_STOP_RE.search(line)
            if m_stop:
                fr = int(m_stop.group("frame"))
                fusion_stop_by_frame[fr] = float(m_stop.group("ts"))
                continue

            m_new = FUSION_TIME_LINE_RE.search(line)
            if m_new:
                fr = int(m_new.group("frame"))
                ms = float(m_new.group("ms"))
                rows.append(
                    (fr, ms, fusion_start_by_frame.get(fr), fusion_stop_by_frame.get(fr))
                )
                continue

            m_old = FUSION_RE.search(line)
            if m_old:
                last_fusion_ms_int = int(m_old.group(1))
                continue

            m_fr = FRAME_START_RE.search(line)
            if m_fr and last_fusion_ms_int is not None:
                frame = int(m_fr.group(1))
                rows.append(
                    (
                        frame,
                        float(last_fusion_ms_int),
                        fusion_start_by_frame.get(frame),
                        fusion_stop_by_frame.get(frame),
                    )
                )
                last_fusion_ms_int = None

    return rows

# -----------------------------
# Scenario 3
# -----------------------------
Corners = Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]  # LU,RU,RL,LL
Cell = Tuple[int, int]


def parse_scenario3_by_frame(
    log_path: str, only_module: str = "DAFU"
) -> Tuple[Dict[int, Dict[int, Dict[str, object]]], List[int]]:
    """Scenario 3 parser.

    Supports BOTH formats:

    (A) Old format (legacy):
      - '장애물 ID: ... 4개 꼭지점 좌표: LU(...), RU(...), RL(...), LL(...)'
      - '장애물 ID ... 위치 인덱스 추가: [x,y]'
      - '<N>번째 로컬 mapdata 전송 시작'  (frame marker)

    (B) New format (2026-01-27~):
      - '[F36] [OCCUPANCY] obstacle 4 ( 42 ) corners LU= ... RU= ... RL= ... LL= ...'
      - '[F36] [OCCUPANCY] obstacle 4 ( 42 ) : index ( 186 , 409 )'
      - frame marker is embedded as [F#] on every line.
    """
    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    frames: Dict[int, Dict[int, Dict[str, object]]] = {}
    frame_order: List[int] = []
    frame_order_seen: Set[int] = set()

    # --------
    # New-format accumulators (frame+oid)
    # --------
    new_corners: Dict[Tuple[int, int], Corners] = {}
    new_cells: Dict[Tuple[int, int], Set[Cell]] = {}

    # --------
    # Old-format state (frame marker comes later)
    # --------
    pending_corners: Dict[int, Corners] = {}
    pending_cells: Dict[int, Set[Cell]] = {}

    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            # module filter (DAFU만)
            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue

            # =========================
            # (B) New format first
            # =========================
            m_nc = OCC_CORNERS_RE.search(line)
            if m_nc:
                fr = int(m_nc.group("frame"))
                oid = int(m_nc.group("oid"))
                lu = (float(m_nc.group("lux")), float(m_nc.group("luy")))
                ru = (float(m_nc.group("rux")), float(m_nc.group("ruy")))
                rl = (float(m_nc.group("rlx")), float(m_nc.group("rly")))
                ll = (float(m_nc.group("llx")), float(m_nc.group("lly")))
                new_corners[(fr, oid)] = (lu, ru, rl, ll)
                # Important: the log can contain multiple OCCUPANCY blocks for the
                # same (frame, obstacle). We want the most recent block only, so
                # we reset the cell accumulator when a new corners line appears.
                new_cells[(fr, oid)] = set()
                if fr not in frame_order_seen:
                    frame_order_seen.add(fr)
                    frame_order.append(fr)
                continue

            m_ni = OCC_INDEX_RE.search(line)
            if m_ni:
                fr = int(m_ni.group("frame"))
                oid = int(m_ni.group("oid"))
                ix = int(m_ni.group("x"))
                iy = int(m_ni.group("y"))
                new_cells.setdefault((fr, oid), set()).add((ix, iy))
                continue

            # =========================
            # (A) Old format fallback
            # =========================
            m_corner = OBST_CORNERS_RE.search(line)
            if m_corner:
                oid = int(m_corner.group(1))
                lu = (float(m_corner.group(2)), float(m_corner.group(3)))
                ru = (float(m_corner.group(4)), float(m_corner.group(5)))
                rl = (float(m_corner.group(6)), float(m_corner.group(7)))
                ll = (float(m_corner.group(8)), float(m_corner.group(9)))
                pending_corners[oid] = (lu, ru, rl, ll)
                pending_cells.setdefault(oid, set())
                continue

            m_cell = OBST_CELL_RE.search(line)
            if m_cell:
                oid = int(m_cell.group(1))
                ix = int(m_cell.group(2))
                iy = int(m_cell.group(3))
                pending_cells.setdefault(oid, set()).add((ix, iy))
                continue

            m_fr = FRAME_START_RE.search(line)
            if m_fr:
                frame = int(m_fr.group(1))
                frame_obs: Dict[int, Dict[str, object]] = {}
                for oid, corners in pending_corners.items():
                    frame_obs[oid] = {
                        "corners": corners,
                        "log_cells": set(pending_cells.get(oid, set())),
                    }
                frames[frame] = frame_obs
                pending_corners = {}
                pending_cells = {}
                if frame not in frame_order_seen:
                    frame_order_seen.add(frame)
                    frame_order.append(frame)

    # --------
    # Commit new-format data into frames dict
    # (Only obstacles with corners are comparable, so we only add those)
    # --------
    for (fr, oid), corners in new_corners.items():
        frames.setdefault(fr, {})
        frames[fr][oid] = {
            "corners": corners,
            "log_cells": set(new_cells.get((fr, oid), set())),
        }

    return frames, frame_order



def _fmt_cells(cells: Set[Cell], max_list: int) -> str:
    arr = sorted(cells)
    head = arr[:max_list]
    s = ", ".join([f"[{x},{y}]" for x, y in head])
    if len(arr) > max_list:
        s += f", ... (+{len(arr) - max_list})"
    return s if s else "(none)"


def run_scenario3(cfg: Config) -> None:
    frames, frame_order = parse_scenario3_by_frame(cfg.log_path, only_module=cfg.only_module)

    if cfg.fusion_count is not None:
        target_frames = frame_order[: cfg.fusion_count]
    else:
        target_frames = list(frame_order)

    if not target_frames:
        raise RuntimeError("scenario3 프레임 데이터를 찾지 못했습니다.")

    eval_total_frames = len(target_frames)
    target_frame_set = set(target_frames)
    frames = {fr: v for fr, v in frames.items() if fr in target_frame_set}

    print("=" * 92)
    print(" Scenario 3 : Obstacle Occupancy Cell Verification (Rasterize vs Log)")
    print("=" * 92)
    print(f" Log File             : {cfg.log_path}")
    if cfg.fusion_count is not None:
        print(f" Fusion Count         : {cfg.fusion_count} (use first N in log)")
        print(f" Frames(selected)     : {len(target_frames)}")
    print(f" Frame Span           : {target_frames[0]} ~ {target_frames[-1]}")
    print(f" Frames(found in log) : {len(target_frames)}")
    print(f" only_module          : {cfg.only_module}")
    print(f" cell_size            : {cfg.cell_size} (coord units per cell)")
    print(" rasterizer           : SAT polygon∩cell (global, no alignment)")
    print(f" target_obstacle_ids  : {cfg.target_obstacle_ids if cfg.target_obstacle_ids is not None else 'ALL'}")
    print("")

    # per-obstacle frame stats
    # - comparable: corners+log_cells가 모두 있는 프레임 (계산/비교 가능)
    # - match: aligned_cells == log_cells 인 프레임 (완전 일치)
    per_comp: Dict[int, int] = {}
    per_match: Dict[int, int] = {}
    per_no_data: Dict[int, int] = {}      # 프레임에 obstacle 자체가 없거나(corners 없어서 frames에 안 담김) 비교 불가
    per_mismatch: Dict[int, int] = {}     # comparable 중 불일치

    # cell-level overall stats(참고용)
    total_obs_compared = 0
    total_match_cells = 0
    total_log_cells = 0
    total_calc_cells = 0

    # 선택 장애물 목록을 "평가구간 내 프레임 퍼센트" 계산 기준으로 만들기
    selected_oids: Optional[List[int]] = cfg.target_obstacle_ids

    for fr in target_frames:
        obs = frames.get(fr, {})  # 프레임이 없으면 빈 dict

        # 이번 프레임에서 처리할 장애물 ID 목록
        if selected_oids is None:
            obs_ids = sorted(obs.keys())
        else:
            obs_ids = [oid for oid in selected_oids if oid in obs]

            # selected_oids 기준으로 '해당 프레임에 데이터가 없으면' no_data 카운트
            for oid in selected_oids:
                if oid not in obs:
                    per_no_data[oid] = per_no_data.get(oid, 0) + 1

        if not obs_ids:
            continue

        # 프레임 출력(선택 장애물만)
        print("-" * 92)
        print(f"[Frame {fr}] obstacles(selected)={len(obs_ids)}")
        print("-" * 92)

        for oid in obs_ids:
            item = obs[oid]
            corners: Corners = item["corners"]  # 이 파서에서는 corners가 없는 oid는 frames에 안 들어감
            log_cells: Set[Cell] = item["log_cells"]

            poly = [corners[0], corners[1], corners[2], corners[3]]
            if rasterize_polygon_to_cells_sat is None:
                raise RuntimeError(
                    'rasterize_polygon_cells.py import failed. Place rasterize_polygon_cells.py next to this script.'
                )
            # v3: exact rasterization in global grid (polygon ∩ cell AABB via SAT). No alignment.
            calc_cells = rasterize_polygon_to_cells_sat(
                poly,
                cell_size=cfg.cell_size,
                include_touch=False,
            )
            aligned_cells, (dx, dy) = (calc_cells, (0, 0))

            inter = aligned_cells & log_cells
            missing = log_cells - aligned_cells
            extra = aligned_cells - log_cells

            # cell-level stats
            total_obs_compared += 1
            total_match_cells += len(inter)
            total_log_cells += len(log_cells)
            total_calc_cells += len(aligned_cells)

            # frame-level PASS stats (완전 동일해야 match)
            per_comp[oid] = per_comp.get(oid, 0) + 1
            if aligned_cells == log_cells:
                per_match[oid] = per_match.get(oid, 0) + 1
            else:
                per_mismatch[oid] = per_mismatch.get(oid, 0) + 1

            print(f"  - Obstacle ID {oid}")
            print(f"    corners(LU,RU,RL,LL): {corners[0]} {corners[1]} {corners[2]} {corners[3]}")
            print(f"    log_cells : {len(log_cells)}")
            print(f"    calc_cells: {len(calc_cells)}  (aligned dx={dx}, dy={dy}) -> {len(aligned_cells)}")
            print(f"    match_cells : {len(inter)}")
            print(f"    missing     : {len(missing)}")
            print(f"    extra       : {len(extra)}")
            print(f"    log list     : {_fmt_cells(log_cells, cfg.max_list_cells)}")
            print(f"    calc(global) : {_fmt_cells(aligned_cells, cfg.max_list_cells)}")
            print(f"    missing list : {_fmt_cells(missing, cfg.max_list_cells)}")
            print(f"    extra list   : {_fmt_cells(extra, cfg.max_list_cells)}")
            print("")

    print("=" * 92)
    print(" Summary (cells-level reference)")
    print("=" * 92)
    # ===== 요청사항: '일치 프레임 수'를 '실제 해당 장애물이 존재하는 프레임' 대비 퍼센트로 출력 =====
    if selected_oids is None:
        # selected가 없으면, 실제로 비교된 장애물들에 대해서만 출력
        oids_to_report = sorted(set(per_comp.keys()))
    else:
        oids_to_report = selected_oids

    for oid in oids_to_report:
        match_n = per_match.get(oid, 0)
        comp_n = per_comp.get(oid, 0)
        nodata_n = per_no_data.get(oid, 0)
        mismatch_n = per_mismatch.get(oid, 0)

        # 1) 실제 해당 장애물이 존재하는 프레임(comp_n) 대비 퍼센트(요청)
        pct_exist = (match_n / comp_n * 100.0) if comp_n > 0 else 0.0

        print(f" Obstacle ID {oid}")
        print(f"   match_frames               : {match_n}")
        print(f"   match_frames / exist_frames: {match_n}/{comp_n} = {pct_exist:.2f}%  <-- requested")
        print(f"   comparable_frames          : {comp_n} (frames where obstacle exists)")
        print(f"   mismatch_frames            : {mismatch_n}")
        print(f"   no_data_frames             : {nodata_n} (frames without this obstacle in log)")
        print("")

    print("=" * 92)


# -----------------------------
# Scenario 4
# -----------------------------

def parse_mergelist_target_positions(
    log_path: str,
    target_obstacle_id: int,
    only_module: str = "DAFU",
    frame_filter: Optional[Set[int]] = None,
) -> Tuple[Dict[int, Tuple[float, float]], Set[int], Dict[str, int]]:
    """MERGELIST의 'ID 할당 <id> : [x,y]'에서 target id의 프레임별 좌표를 파싱한다.

    - DAFU 라인만 처리
    - 동일 프레임에 여러 번 등장하면 "마지막 값"을 사용 (덮어쓰기)
    - 프레임 태그 존재 여부도 함께 수집
    """
    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    positions_by_frame: Dict[int, Tuple[float, float]] = {}
    seen_frame_tags: Set[int] = set()
    stats: Dict[str, int] = {
        "lines_total": 0,
        "lines_module_filtered": 0,
        "frame_tags_in_range": 0,
        "mergelist_matches_in_range": 0,      # ID 할당 라인
        "mergelist_final_matches_in_range": 0, # 최종 리스트 ID 라인
        "target_matches_in_range": 0,
    }

    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            stats["lines_total"] += 1

            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue
            stats["lines_module_filtered"] += 1

            m_frame = FRAME_TAG_RE.search(line)
            if m_frame:
                fr = int(m_frame.group("frame"))
                if frame_filter is not None:
                    if fr in frame_filter:
                        if fr not in seen_frame_tags:
                            stats["frame_tags_in_range"] += 1
                        seen_frame_tags.add(fr)
                else:
                    if fr not in seen_frame_tags:
                        stats["frame_tags_in_range"] += 1
                    seen_frame_tags.add(fr)

            m_merge = MERGELIST_ASSIGN_RE.search(line)
            if m_merge:
                fr = int(m_merge.group("frame"))
                if (fr in frame_filter) if frame_filter is not None else True:
                    stats["mergelist_matches_in_range"] += 1
                    oid = int(m_merge.group("id"))
                    if oid == target_obstacle_id:
                        x = float(m_merge.group("x"))
                        y = float(m_merge.group("y"))
                        positions_by_frame[fr] = (x, y)  # 마지막 값으로 덮어씀
                        stats["target_matches_in_range"] += 1
                continue

            # 최종 융합 장애물 리스트 출력 아래의 ID 라인도 반영
            m_final = MERGELIST_FINAL_ID_RE.search(line)
            if not m_final:
                continue

            fr = int(m_final.group("frame"))
            if frame_filter is not None:
                if fr not in frame_filter:
                    continue

            stats["mergelist_final_matches_in_range"] += 1

            oid = int(m_final.group("id"))
            if oid != target_obstacle_id:
                continue

            x = float(m_final.group("x"))
            y = float(m_final.group("y"))
            positions_by_frame[fr] = (x, y)  # 마지막 값으로 덮어씀
            stats["target_matches_in_range"] += 1

    return positions_by_frame, seen_frame_tags, stats


def run_scenario4(cfg: Config) -> None:
    if cfg.target_obstacle_id is None:
        raise RuntimeError("scenario4는 config에 target_obstacle_id가 필요합니다.")
    if cfg.gt_x is None or cfg.gt_y is None:
        raise RuntimeError(
            "scenario4는 config에 GT 좌표가 필요합니다. (gt_x/gt_y 또는 target_gt_xy: [x,y])"
        )

    not_found_policy = (cfg.scenario4_not_found_policy or "fail").strip().lower()
    if not_found_policy not in {"fail", "exclude"}:
        raise RuntimeError(
            f"scenario4_not_found_policy는 'fail' 또는 'exclude'만 지원합니다. (입력: {cfg.scenario4_not_found_policy})"
        )

    if cfg.fusion_count is not None:
        target_frames = _get_frame_tags_in_order(
            cfg.log_path, only_module=cfg.only_module, max_frames=cfg.fusion_count
        )
    else:
        target_frames = _get_frame_tags_in_order(cfg.log_path, only_module=cfg.only_module)

    if not target_frames:
        raise RuntimeError("scenario4 프레임 태그를 찾지 못했습니다.")

    eval_total_frames = len(target_frames)
    frame_filter = set(target_frames)
    thresh_cm = float(cfg.pos_error_thresh_cm)
    thresh_dm = thresh_cm / 10.0  # 80cm => 8dm

    positions_by_frame, seen_frame_tags, stats = parse_mergelist_target_positions(
        cfg.log_path,
        target_obstacle_id=int(cfg.target_obstacle_id),
        only_module=cfg.only_module,
        frame_filter=frame_filter,
    )

    found_frames = sorted(positions_by_frame.keys())
    frames_with_tag_in_range = sorted([fr for fr in target_frames if fr in seen_frame_tags])
    found_tag_count = len(frames_with_tag_in_range)
    missing_tag_count = max(0, eval_total_frames - found_tag_count)
    not_found_frames: List[int] = []
    no_frame_tag_frames: List[int] = []
    errors_dm: List[float] = []
    fail_over_frames: List[Tuple[int, float]] = []

    print("=" * 92)
    print(" Scenario 4 : Target Obstacle Position Error vs GT (MERGELIST / DAFU)")
    print("=" * 92)
    print(f" Log File              : {cfg.log_path}")
    if cfg.fusion_count is not None:
        print(f" Fusion Count          : {cfg.fusion_count} (use first N in log)")
    print(f" Frame Span            : {target_frames[0]} ~ {target_frames[-1]} (total={eval_total_frames})")
    print(f" only_module           : {cfg.only_module}")
    print(f" target_obstacle_id    : {cfg.target_obstacle_id}")
    print(f" GT (x,y) [dm]         : ({cfg.gt_x}, {cfg.gt_y})")
    print(f" threshold             : {thresh_dm:.1f} dm ({thresh_cm:.1f} cm)")
    print(f" not_found_policy      : {not_found_policy}")
    if frames_with_tag_in_range:
        print(
            f" frames_with_tag       : {found_tag_count} "
            f"(range in log: {frames_with_tag_in_range[0]} ~ {frames_with_tag_in_range[-1]})"
        )
    else:
        print(" frames_with_tag       : 0")
    print(f" frames_with_target_id : {len(found_frames)}")
    print("")

    print("-" * 92)
    print(" Per-frame Errors")
    print("-" * 92)

    gt_x = float(cfg.gt_x)
    gt_y = float(cfg.gt_y)

    for fr in target_frames:
        pos = positions_by_frame.get(fr)
        if pos is None:
            if fr not in seen_frame_tags:
                no_frame_tag_frames.append(fr)
            not_found_frames.append(fr)
            print(
                f" Frame=F{fr:>4} id={cfg.target_obstacle_id} "
                f"est=NOT_FOUND gt={_fmt_xy(gt_x, gt_y)} err=- status=NOT_FOUND"
            )
            continue

        x, y = pos
        dx = x - gt_x
        dy = y - gt_y
        err_dm = math.hypot(dx, dy)
        err_cm = err_dm * 10.0
        err_m = err_dm * 0.1
        errors_dm.append(err_dm)

        ok = err_dm <= thresh_dm
        status = "OK" if ok else "FAIL"
        if not ok:
            fail_over_frames.append((fr, err_dm))

        print(
            f" Frame=F{fr:>4} id={cfg.target_obstacle_id} "
            f"est={_fmt_xy(x, y)} gt={_fmt_xy(gt_x, gt_y)} "
            f"err={err_dm:.3f}dm ({err_cm:.1f}cm, {err_m:.3f}m) status={status}"
        )

    max_err_dm = max(errors_dm) if errors_dm else 0.0
    avg_err_dm = (sum(errors_dm) / len(errors_dm)) if errors_dm else 0.0
    max_err_cm = max_err_dm * 10.0
    avg_err_cm = avg_err_dm * 10.0

    not_found_count = len(not_found_frames)
    found_count = len(found_frames)
    missing_count = max(0, eval_total_frames - found_count)

    # PASS/FAIL 판정
    fail_reasons: List[str] = []
    if fail_over_frames:
        top_fr, top_err = max(fail_over_frames, key=lambda t: t[1])
        fail_reasons.append(
            f"threshold 초과 프레임 존재 (worst: F{top_fr}={top_err*10.0:.1f}cm)"
        )
    if not_found_policy == "fail" and not_found_frames:
        fail_reasons.append(f"NOT_FOUND 프레임 {not_found_count}건 (policy=fail)")

    result = "PASS" if not fail_reasons else "FAIL"

    print("")
    print("=" * 92)
    print(" Summary")
    print("=" * 92)
    print(f" evaluated_frames     : {eval_total_frames}")
    print(f" found_frames         : {found_count}")
    print(f" not_found_frames     : {not_found_count}")
    if no_frame_tag_frames:
        print(f" no_frame_tag_frames  : {len(no_frame_tag_frames)} (e.g., {no_frame_tag_frames[:10]})")
    print(f" max_error_dm         : {max_err_dm:.3f}")
    print(f" max_error_cm         : {max_err_cm:.1f}")
    print(f" avg_error_dm         : {avg_err_dm:.3f}")
    print(f" avg_error_cm         : {avg_err_cm:.1f}")
    if fail_over_frames:
        over_list = ", ".join([f"F{fr}({err*10.0:.1f}cm)" for fr, err in fail_over_frames[:15]])
        print(f" fail_over_frames     : {over_list}")
    if not_found_frames:
        nf_list = ", ".join([f"F{fr}" for fr in not_found_frames[:20]])
        print(f" not_found_list       : {nf_list}")
    print(f" RESULT               : {result}")
    if fail_reasons:
        for reason in fail_reasons:
            print(f" FAIL reason          : {reason}")
    print("")
    print("=" * 92)


def run_scenario1(cfg: Config) -> None:
    ordered_rows = parse_fusion_times_in_order(cfg.log_path, only_module=cfg.only_module)
    if cfg.fusion_count is not None:
        rows = ordered_rows[: cfg.fusion_count]
    else:
        rows = ordered_rows

    if not rows:
        raise RuntimeError("FUSION_TIME 데이터를 찾지 못했습니다.")

    ms_values = [ms for _, ms, _, _ in rows]
    avg_ms = sum(ms_values) / len(ms_values)
    max_ms = max(ms_values)

    print("=" * 60)
    print(" Scenario 1 : Map Data Fusion Time Analysis")
    print("=" * 60)
    print(f" Log File    : {cfg.log_path}")
    if cfg.fusion_count is not None:
        print(f" Fusion Count: {cfg.fusion_count} (use first N in log)")
    print(f" Total Frames(found): {len(rows)}")
    print("")

    print("-" * 120)
    print(
        f"{'Index':>6}   {'FUSION_TIME (ms)':>18}   {'FUSION_START (ts)':>18}   {'FUSION_STOP (ts)':>17}   {'FUSION_END - FUSION_START (ms)':>32}"
    )
    print("-" * 120)

    def fmt_ts(ts: Optional[float]) -> str:
        return "-" if ts is None else f"{ts:.4f}"

    def fmt_actual_ms(start_ts: Optional[float], end_ts: Optional[float]) -> str:
        if start_ts is None or end_ts is None:
            return "-"
        return f"{(end_ts - start_ts) * 1000.0:.3f}"

    for idx, (_, ms, start_ts, end_ts) in enumerate(rows, start=1):
        print(
            f"{idx:>6}   {ms:>18.3f}   {fmt_ts(start_ts):>13}   {fmt_ts(end_ts):>13}   {fmt_actual_ms(start_ts, end_ts):>20}"
        )

    print("")
    print("-" * 120)
    print(" Summary")
    print("-" * 120)
    print(f" Avg FUSION_TIME (ms): {avg_ms:.3f}")
    print(f" Max FUSION_TIME (ms): {max_ms:.3f}")
    print("=" * 60)


def _fmt_xy(x: float, y: float) -> str:
    return f"({x:.8f}, {y:.8f})"


def _init_scenario2_frame() -> Dict[str, Any]:
    return {
        "before": [],            # List[Dict[class:int, before_pos:(x,y), after_pos:Optional[(x,y)], removed:bool, note:str]]
        "removed": [],           # List[Dict[class:int, removed_pos:(x,y)]]
        "pending": defaultdict(deque),  # class -> deque[index into before]
        "orphan_after": [],      # after가 먼저 나오거나 매칭 실패한 경우
        "unparsed_rel_lines": 0,
    }


def parse_scenario2_frames(
    log_path: str,
    only_module: str = "DAFU",
    debug_sample_limit: int = 20,
    frame_filter: Optional[Set[int]] = None,
) -> Tuple[Optional[Tuple[int, int]], Dict[int, Dict[str, Any]], Dict[str, Any]]:
    """Scenario 2 parser (streaming).

    - DAFU 로그만 대상으로 MAP_SIZE 및 프레임별 before/after/remove를 파싱
    - 큰 로그를 고려해 start~end 프레임만 저장
    - 파싱 실패 RELATIVETOMAP 라인은 debug 정보로 수집
    """
    p = Path(log_path)
    if not p.exists():
        raise FileNotFoundError(f"Log not found: {p}")

    map_size: Optional[Tuple[int, int]] = None
    frames: Dict[int, Dict[str, Any]] = {}

    debug_info: Dict[str, Any] = {
        "lines_total": 0,
        "lines_module_filtered": 0,
        "lines_no_frame": 0,
        "rel_lines_unparsed": 0,
        "rel_unparsed_samples": [],
    }

    def _add_debug_sample(line: str) -> None:
        samples: List[str] = debug_info["rel_unparsed_samples"]
        if len(samples) < debug_sample_limit:
            samples.append(line.rstrip("\n"))

    with p.open("r", encoding="utf-8", errors="replace") as f:
        for line in f:
            debug_info["lines_total"] += 1

            # module filter (DAFU만)
            if only_module and only_module.strip():
                if f" {only_module} " not in f" {line} ":
                    continue
            debug_info["lines_module_filtered"] += 1

            # MAP_SIZE는 프레임 태그가 없어도 전역으로 보관
            m_map = MAP_SIZE_RE.search(line)
            if m_map:
                map_size = (int(m_map.group("w")), int(m_map.group("h")))

            # 프레임 태그가 없으면 프레임 데이터에는 귀속하지 않음
            m_frame = FRAME_TAG_RE.search(line)
            if not m_frame:
                debug_info["lines_no_frame"] += 1
                continue

            fr = int(m_frame.group("frame"))
            if frame_filter is not None:
                if fr not in frame_filter:
                    continue

            frame_data = frames.setdefault(fr, _init_scenario2_frame())

            m_before = REL_BEFORE_RE.search(line)
            if m_before:
                cls = int(m_before.group("cls"))
                x = float(m_before.group("x"))
                y = float(m_before.group("y"))
                idx = len(frame_data["before"])
                frame_data["before"].append(
                    {
                        "class": cls,
                        "before_pos": (x, y),
                        "after_pos": None,
                        "removed": False,
                        "note": "",
                    }
                )
                frame_data["pending"][cls].append(idx)
                continue

            m_after = REL_AFTER_RE.search(line)
            if m_after:
                cls = int(m_after.group("cls"))
                x = float(m_after.group("x"))
                y = float(m_after.group("y"))
                pend = frame_data["pending"][cls]
                if pend:
                    # 로그는 before -> (after|remove)가 바로 뒤에 붙는 경향이 있으므로
                    # 같은 class의 "가장 최근 before"에 매칭한다.
                    idx = pend.pop()
                    frame_data["before"][idx]["after_pos"] = (x, y)
                    if not frame_data["before"][idx]["note"]:
                        frame_data["before"][idx]["note"] = "맵 내 잔존(좌표변환 after 확인)"
                else:
                    frame_data["orphan_after"].append({"class": cls, "after_pos": (x, y)})
                continue

            m_remove = REL_REMOVE_RE.search(line)
            if m_remove:
                cls = int(m_remove.group("cls"))
                x = float(m_remove.group("x"))
                y = float(m_remove.group("y"))
                frame_data["removed"].append({"class": cls, "removed_pos": (x, y)})
                pend = frame_data["pending"][cls]
                if pend:
                    # 제거 라인도 같은 class의 "가장 최근 before"에 매칭
                    idx = pend.pop()
                    frame_data["before"][idx]["removed"] = True
                    frame_data["before"][idx]["note"] = "작업영역 외부로 삭제(직후 제거 로그)"
                continue

            # RELATIVETOMAP 중에서도 "장애물" 관련 라인만 디버그 수집
            # (차량 위치/heading 등은 의도적으로 무시)
            if "[RELATIVETOMAP]" in line and "장애물" in line:
                frame_data["unparsed_rel_lines"] += 1
                debug_info["rel_lines_unparsed"] += 1
                _add_debug_sample(line)

    # pending 자료구조는 출력에 필요 없으므로 정리
    for fr, data in frames.items():
        data.pop("pending", None)

    return map_size, frames, debug_info


def _classify_frame_scenario2(frame_data: Dict[str, Any]) -> Dict[str, Any]:
    """before/removed를 기반으로 삭제/잔존을 분류한다.

    로그 특성상 동일 class 다건이 있어 1:1 매칭이 어려울 수 있으므로,
    불확실한 경우 note에 명시적으로 표시한다.
    """
    before: List[Dict[str, Any]] = frame_data["before"]
    removed: List[Dict[str, Any]] = frame_data["removed"]

    remaining = [b for b in before if not b["removed"]]
    removed_marked = [b for b in before if b["removed"]]

    # 직전 before 기반 매칭이 실패한 제거/after를 명시적으로 알린다.
    uncertain_matches: List[str] = []
    before_count_by_class: Dict[int, int] = defaultdict(int)
    removed_marked_by_class: Dict[int, int] = defaultdict(int)
    removed_logged_by_class: Dict[int, int] = defaultdict(int)
    orphan_after_by_class: Dict[int, int] = defaultdict(int)

    for b in before:
        before_count_by_class[int(b["class"])] += 1
        if b["removed"]:
            removed_marked_by_class[int(b["class"])] += 1
    for r in removed:
        removed_logged_by_class[int(r["class"])] += 1
    for oa in frame_data.get("orphan_after", []):
        orphan_after_by_class[int(oa["class"])] += 1

    all_classes = set(before_count_by_class) | set(removed_logged_by_class) | set(orphan_after_by_class)
    per_class_stats: Dict[int, Dict[str, int]] = {}
    for cls in sorted(all_classes):
        bef_n = before_count_by_class.get(cls, 0)
        rem_logged = removed_logged_by_class.get(cls, 0)
        rem_marked = removed_marked_by_class.get(cls, 0)
        orphan_after_n = orphan_after_by_class.get(cls, 0)
        per_class_stats[cls] = {
            "before": bef_n,
            "removed_logged": rem_logged,
            "removed_matched": rem_marked,
            "orphan_after": orphan_after_n,
        }
        unmatched_removed = max(0, rem_logged - rem_marked)
        if unmatched_removed > 0:
            uncertain_matches.append(
                f"class={cls}: removed 로그 {rem_logged}건 중 {rem_marked}건만 직전 before와 매칭됨 (미매칭 {unmatched_removed}건)"
            )
        if orphan_after_n > 0:
            uncertain_matches.append(
                f"class={cls}: after 로그 {orphan_after_n}건이 직전 before 없이 등장"
            )

    return {
        "before_count": len(before),
        "removed_count": len(removed),
        "remaining_count": len(remaining),
        "remaining_min": len(remaining),
        "remaining_max": len(remaining),
        "remaining_display": f"{len(remaining)}",
        "remaining": remaining,
        "removed_marked": removed_marked,
        "uncertain_matches": uncertain_matches,
        "per_class_stats": per_class_stats,
        "orphan_after": frame_data.get("orphan_after", []),
        "unparsed_rel_lines": int(frame_data.get("unparsed_rel_lines", 0)),
    }


def run_scenario2(cfg: Config) -> None:
    if cfg.fusion_count is not None:
        target_frames_all = _get_frame_tags_in_order(
            cfg.log_path, only_module=cfg.only_module, max_frames=cfg.fusion_count
        )
    else:
        target_frames_all = _get_frame_tags_in_order(cfg.log_path, only_module=cfg.only_module)

    if not target_frames_all:
        raise RuntimeError("scenario2 프레임 태그를 찾지 못했습니다.")

    frame_filter = set(target_frames_all)
    map_size, frames, debug_info = parse_scenario2_frames(
        cfg.log_path,
        only_module=cfg.only_module,
        frame_filter=frame_filter,
    )
    eval_total_frames = len(target_frames_all)
    target_frames = [fr for fr in target_frames_all if fr in frames]
    found_frames = len(target_frames)
    missing_frames = max(0, eval_total_frames - found_frames)

    print("=" * 88)
    print(" Scenario 2 : Map Range / Obstacle Removal Analysis (DAFU only)")
    print("=" * 88)
    print(f" Log File        : {cfg.log_path}")
    if cfg.fusion_count is not None:
        print(f" Fusion Count    : {cfg.fusion_count} (use first N in log)")
    print(f" Frame Span      : {target_frames_all[0]} ~ {target_frames_all[-1]} (total={eval_total_frames})")
    if target_frames:
        print(f" Frames(found)   : {found_frames} (range in log: {target_frames[0]} ~ {target_frames[-1]})")
    else:
        print(f" Frames(found)   : {found_frames}")
    print(f" only_module     : {cfg.only_module}")
    print("")

    print("-" * 88)
    print(" Frame Summary")
    print("-" * 88)
    print(f"{'Frame':>6} | {'before':>6} {'removed':>7} {'remaining':>9} | notes")
    print("-" * 88)

    frame_reports: Dict[int, Dict[str, Any]] = {}
    for fr in target_frames:
        rep = _classify_frame_scenario2(frames[fr])
        frame_reports[fr] = rep
        notes: List[str] = []
        if rep["uncertain_matches"]:
            notes.append("매칭 불확실")
        if rep["orphan_after"]:
            notes.append(f"orphan_after={len(rep['orphan_after'])}")
        if rep["unparsed_rel_lines"] > 0:
            notes.append(f"unparsed_rel={rep['unparsed_rel_lines']}")
        note_str = ", ".join(notes) if notes else "-"
        print(
            f"{fr:>6} | {rep['before_count']:>6} {rep['removed_count']:>7} {rep['remaining_display']:>9} | {note_str}"
        )
    print("-" * 88)
    print("")

    # 프레임별 상세 출력
    for fr in target_frames:
        data = frames[fr]
        rep = frame_reports[fr]

        print("=" * 88)
        print(f" Frame F{fr}")
        print("=" * 88)

        # 1) 인지데이터(원본 before)
        print("[인지데이터: 좌표변환 before]")
        if data["before"]:
            for b in data["before"]:
                cls = int(b["class"])
                bx, by = b["before_pos"]
                line = f" - class={cls:>3} before={_fmt_xy(bx, by)}"
                if b["after_pos"] is not None:
                    ax, ay = b["after_pos"]
                    line += f" after={_fmt_xy(ax, ay)}"
                if b["removed"]:
                    line += f"  -> {b['note']}"
                print(line)
        else:
            print(" (none)")
        print("")

        # 2) 삭제된 장애물 목록 (removed 라인 기준)
        print("[삭제된 장애물: 맵 범위 밖 장애물 제거]")
        if data["removed"]:
            for r in data["removed"]:
                cls = int(r["class"])
                rx, ry = r["removed_pos"]
                print(f" - class={cls:>3} removed_pos={_fmt_xy(rx, ry)}  -> 작업영역 외부로 삭제")
        else:
            print(" (none)")
        print("")

        # 3) 남은 장애물 목록
        print("[남은 장애물]")
        if rep["remaining"]:
            for b in rep["remaining"]:
                cls = int(b["class"])
                bx, by = b["before_pos"]
                line = f" - class={cls:>3} before={_fmt_xy(bx, by)}"
                if b["after_pos"] is not None:
                    ax, ay = b["after_pos"]
                    line += f" after={_fmt_xy(ax, ay)}"
                print(line)
        else:
            print(" (none)")
        print("")

        # 불확실/디버그 정보는 숨기지 않고 명시
        if rep["uncertain_matches"] or rep["orphan_after"] or rep["unparsed_rel_lines"] > 0:
            print("[주의/디버그]")
            for msg in rep["uncertain_matches"]:
                print(f" - {msg}")
            # class별 직전 before 매칭 상태를 명시적으로 보여줌
            for cls, stats in sorted(rep["per_class_stats"].items()):
                bef_n = stats["before"]
                rem_logged = stats["removed_logged"]
                rem_matched = stats["removed_matched"]
                orphan_after_n = stats["orphan_after"]
                if rem_logged > 0 or orphan_after_n > 0:
                    print(
                        f" - class={cls}: before={bef_n}, removed_logged={rem_logged}, removed_matched={rem_matched}, orphan_after={orphan_after_n}"
                    )
            if rep["orphan_after"]:
                print(f" - orphan_after 존재: {len(rep['orphan_after'])}건")
            if rep["unparsed_rel_lines"] > 0:
                print(f" - RELATIVETOMAP 파싱 실패 라인: {rep['unparsed_rel_lines']}건")
            print("")

    samples: List[str] = debug_info["rel_unparsed_samples"]
    if samples:
        print(" rel_unparsed_samples  :")
        for s in samples[:10]:
            print(f"  - {s}")
    else:
        print(" rel_unparsed_samples  : (none)")
    print("=" * 88)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", required=True, help="config.yaml 또는 config.json 경로")
    args = ap.parse_args()

    cfg = load_config(args.config)

    if cfg.scenario == 1:
        run_scenario1(cfg)
    elif cfg.scenario == 2:
        run_scenario2(cfg)
    elif cfg.scenario == 3:
        run_scenario3(cfg)
    elif cfg.scenario == 4:
        run_scenario4(cfg)
    else:
        raise RuntimeError(f"현재 스크립트는 scenario=1,2,3,4만 지원합니다. (입력: {cfg.scenario})")


if __name__ == "__main__":
    main()
