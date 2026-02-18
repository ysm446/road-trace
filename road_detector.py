#!/usr/bin/env python3
"""
road_detector.py  —  ハイブリッド道路パス抽出アプリ

【パイプライン】
  1. OpenCV (LAB色空間) で道路ピクセルを検出 → 二値マスク
  2. scikit-image でスケルトン化 → 1px 幅の中心線
  3. 方向ガイドトレース: 分岐点で「最も曲がらない方向」を選んで長い一本道を生成
  4. 座標スムージング → Douglas-Peucker でウェイポイント削減
  5. 近傍エンドポイントを方向一致で結合 → さらに長い連続パスへ
  6. Gemma 3 (Ollama) で道路名・種別を付与
  7. road_network_editor.html 互換の JSON として出力
"""

import base64
import colorsys
import io
import json
import math
import random
import re
import tempfile
from typing import Optional

import cv2
import gradio as gr
import numpy as np
import requests
from PIL import Image, ImageDraw
from skimage.morphology import skeletonize as sk_skeletonize

# ── 定数 ──────────────────────────────────────────────────────────────────────
DEFAULT_OLLAMA_URL   = "http://localhost:11434"
DEFAULT_MODEL        = "gemma3:12b"
DEFAULT_WORLD_SCALE  = 1000.0
DEFAULT_SPEED        = 60.0
DEFAULT_WIDTH        = 3.0
DEFAULT_FRICTION     = 0.15

# CV パラメータ
DEFAULT_L_MIN        = 30
DEFAULT_L_MAX        = 170
DEFAULT_CHROMA_MAX   = 35.0
DEFAULT_CLOSE_SIZE   = 25
DEFAULT_MIN_AREA     = 3000
DEFAULT_MIN_PIXELS   = 100
DEFAULT_SIMPLIFY_EPS = 2.0   # 小さく → より細かいパス
DEFAULT_SMOOTH_WIN   = 9     # 座標スムージングウィンドウ
DEFAULT_MERGE_GAP    = 20    # エンドポイント接続距離 [px]
DEFAULT_MERGE_ANGLE  = 45.0  # 接続許容角度 [°]
DEFAULT_MAX_IMG_SIZE = 1024


# ── ユーティリティ ─────────────────────────────────────────────────────────────

def generate_road_id() -> str:
    return "".join(random.choices("0123456789abcdef", k=8))


def pil_to_bgr(image: Image.Image) -> np.ndarray:
    return cv2.cvtColor(np.array(image.convert("RGB")), cv2.COLOR_RGB2BGR)


def encode_image_to_base64(
    pil_image: Image.Image, max_size: int = DEFAULT_MAX_IMG_SIZE
) -> tuple[str, int, int]:
    img = pil_image.copy().convert("RGB")
    img.thumbnail((max_size, max_size), Image.LANCZOS)
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    b64 = base64.b64encode(buf.getvalue()).decode("utf-8")
    return b64, img.width, img.height


# ── 道路マスク検出 (OpenCV) ───────────────────────────────────────────────────

def detect_road_mask(
    image_bgr: np.ndarray,
    l_min: int        = DEFAULT_L_MIN,
    l_max: int        = DEFAULT_L_MAX,
    chroma_max: float = DEFAULT_CHROMA_MAX,
    close_size: int   = DEFAULT_CLOSE_SIZE,
    min_area: int     = DEFAULT_MIN_AREA,
) -> tuple[np.ndarray, Image.Image]:
    """LAB色空間でアスファルト（低彩度・中間輝度）ピクセルを検出する。"""
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB)
    L, a, b = cv2.split(lab)

    af = a.astype(np.float32) - 128.0
    bf = b.astype(np.float32) - 128.0
    chroma = np.sqrt(af ** 2 + bf ** 2)

    raw_mask = (
        (L >= l_min) & (L <= l_max) & (chroma <= chroma_max)
    ).astype(np.uint8) * 255

    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_size, close_size))
    k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    mask = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, k_close)
    mask = cv2.morphologyEx(mask,     cv2.MORPH_OPEN,  k_open)

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    filtered = np.zeros_like(mask)
    for i in range(1, num_labels):
        if stats[i, cv2.CC_STAT_AREA] >= min_area:
            filtered[labels == i] = 255

    vis = image_bgr.copy()
    green_layer = np.zeros_like(vis)
    green_layer[filtered > 0] = (0, 200, 0)
    vis = cv2.addWeighted(vis, 0.6, green_layer, 0.4, 0)
    mask_pil = Image.fromarray(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))

    return filtered, mask_pil


# ── パス抽出（方向ガイドトレース） ───────────────────────────────────────────

def _angle_diff(dx1: float, dy1: float, dx2: float, dy2: float) -> float:
    """2つのベクトル間の角度 [rad] を返す（0 = 同方向、π = 逆方向）。"""
    l1 = math.hypot(dx1, dy1)
    l2 = math.hypot(dx2, dy2)
    if l1 * l2 < 1e-9:
        return math.pi
    cos_a = (dx1 * dx2 + dy1 * dy2) / (l1 * l2)
    return math.acos(max(-1.0, min(1.0, cos_a)))


def _smooth_coords(
    pts: list[tuple[float, float]], window: int
) -> list[list[float]]:
    """スライディングウィンドウで座標を平均化してジャギーを除去する。"""
    n = len(pts)
    if n <= 2 or window < 3:
        return [[float(p[0]), float(p[1])] for p in pts]
    half = window // 2
    result = []
    for i in range(n):
        s = max(0, i - half)
        e = min(n, i + half + 1)
        win = pts[s:e]
        result.append([
            sum(p[0] for p in win) / len(win),
            sum(p[1] for p in win) / len(win),
        ])
    return result


def _merge_nearby_endpoints(
    paths: list[list[list[float]]],
    max_gap: float,
    max_angle_deg: float,
) -> list[list[list[float]]]:
    """
    近傍かつ方向が一致するパスのエンドポイントを結合し、連続パスを伸ばす。

    エンドポイントの「外向き方向」: パスの端から外側を向くベクトル。
    2つのエンドポイントが互いに向き合っていれば結合する。
    """
    max_angle = math.radians(max_angle_deg)

    # エンドポイントリスト: (path_idx, is_tail, x, y, outward_dx, outward_dy)
    endpoints: list[tuple] = []
    for i, p in enumerate(paths):
        if len(p) < 2:
            continue
        # 先頭: 外向き = p[0] → (p[0] - p[1]) 方向
        endpoints.append((i, False, p[0][0], p[0][1],
                          p[0][0] - p[1][0], p[0][1] - p[1][1]))
        # 末尾: 外向き = p[-1] → (p[-1] - p[-2]) 方向
        endpoints.append((i, True, p[-1][0], p[-1][1],
                          p[-1][0] - p[-2][0], p[-1][1] - p[-2][1]))

    # 距離×方向でソートした結合候補ペアを列挙
    pairs: list[tuple[float, int, int]] = []
    for ai, (pi, ie_a, ax, ay, adx, ady) in enumerate(endpoints):
        for bi in range(ai + 1, len(endpoints)):
            pj, ie_b, bx, by, bdx, bdy = endpoints[bi]
            if pi == pj:
                continue
            dist = math.hypot(ax - bx, ay - by)
            if dist > max_gap:
                continue
            to_b_x, to_b_y = bx - ax, by - ay
            ang_a = _angle_diff(adx, ady,  to_b_x,  to_b_y)
            ang_b = _angle_diff(bdx, bdy, -to_b_x, -to_b_y)
            if ang_a < max_angle and ang_b < max_angle:
                pairs.append((dist, ai, bi))
    pairs.sort()

    result = [list(p) for p in paths]
    path_ref = list(range(len(paths)))  # path_ref[i] = result の現在インデックス
    used_ep: set[int] = set()

    for _, ai, bi in pairs:
        if ai in used_ep or bi in used_ep:
            continue
        pi, ie_a = endpoints[ai][0], endpoints[ai][1]
        pj, ie_b = endpoints[bi][0], endpoints[bi][1]

        ri = path_ref[pi]
        rj = path_ref[pj]
        if ri == rj or result[ri] is None or result[rj] is None:
            continue

        p1, p2 = result[ri], result[rj]

        if ie_a and not ie_b:
            merged = p1 + p2
        elif not ie_a and ie_b:
            merged = p2 + p1
        elif ie_a and ie_b:
            merged = p1 + p2[::-1]
        else:
            merged = p1[::-1] + p2

        result[ri] = merged
        result[rj] = None
        for k in range(len(path_ref)):
            if path_ref[k] == rj:
                path_ref[k] = ri
        used_ep.add(ai)
        used_ep.add(bi)

    return [p for p in result if p is not None]


def skeleton_to_paths(
    mask: np.ndarray,
    min_pixels: int   = DEFAULT_MIN_PIXELS,
    simplify_eps: float = DEFAULT_SIMPLIFY_EPS,
    smooth_window: int  = DEFAULT_SMOOTH_WIN,
    merge_gap: float    = DEFAULT_MERGE_GAP,
    merge_angle_deg: float = DEFAULT_MERGE_ANGLE,
) -> list[list[list[float]]]:
    """
    二値マスク → スケルトン化 → 方向ガイドトレース → スムージング → 結合。

    【方向ガイドトレース】
    分岐点 (3+ 隣接) に到達したとき、最も角度変化が小さい（最も直進に近い）
    隣接ピクセルを選んで進み続ける。これにより道路の流れを壊さずに
    長い一本のパスとして抽出できる。
    """
    skel = sk_skeletonize(mask > 0).astype(np.uint8)
    ys, xs = np.where(skel > 0)
    if len(xs) == 0:
        return []

    pixel_set: set[tuple[int, int]] = set(zip(xs.tolist(), ys.tolist()))

    def get_nbrs(x: int, y: int) -> list[tuple[int, int]]:
        return [
            (x + dx, y + dy)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if (dx or dy) and (x + dx, y + dy) in pixel_set
        ]

    adj: dict[tuple, list] = {p: get_nbrs(*p) for p in pixel_set}
    endpoint_set = {p for p, n in adj.items() if len(n) == 1}

    global_visited: set[tuple[int, int]] = set()
    raw_paths: list[list[tuple[int, int]]] = []

    def trace(start: tuple[int, int]) -> list[tuple[int, int]]:
        """方向ガイドDFS: 常に最も直進に近い未訪問隣接ピクセルへ進む。"""
        if start in global_visited:
            return []
        path: list[tuple[int, int]] = [start]
        global_visited.add(start)
        prev_dir: Optional[tuple[float, float]] = None
        curr = start

        while True:
            unvisited = [n for n in adj[curr] if n not in global_visited]
            if not unvisited:
                break

            if prev_dir is None or len(unvisited) == 1:
                nxt = unvisited[0]
            else:
                # 現在の進行方向と各候補の角度差を計算し最小を選ぶ
                nxt = min(
                    unvisited,
                    key=lambda n: _angle_diff(
                        prev_dir[0], prev_dir[1],
                        n[0] - curr[0], n[1] - curr[1],
                    ),
                )

            prev_dir = (float(nxt[0] - curr[0]), float(nxt[1] - curr[1]))
            path.append(nxt)
            global_visited.add(nxt)
            curr = nxt

        return path

    # まず端点から出発（方向が決まりやすい）
    for start in endpoint_set:
        p = trace(start)
        if p:
            raw_paths.append(p)

    # 未訪問のループ（端点のない閉じたリング）を処理
    for px_ in sorted(pixel_set):
        if px_ not in global_visited:
            p = trace(px_)
            if p:
                raw_paths.append(p)

    # スムージング → Douglas-Peucker 簡略化 → 最小長フィルタ
    simplified: list[list[list[float]]] = []
    for raw in raw_paths:
        if len(raw) < min_pixels:
            continue

        smoothed = _smooth_coords(raw, smooth_window)

        pts_np = np.array(smoothed, dtype=np.float32).reshape(-1, 1, 2)
        dp = cv2.approxPolyDP(pts_np, simplify_eps, False)
        pts_list = [[float(p[0][0]), float(p[0][1])] for p in dp]

        if len(pts_list) >= 2:
            simplified.append(pts_list)

    # 近傍エンドポイントを結合してさらに連続化
    if simplified:
        simplified = _merge_nearby_endpoints(simplified, merge_gap, merge_angle_deg)

    return simplified


# ── 自動命名 ──────────────────────────────────────────────────────────────────

def auto_name_path(path: list, img_w: int, img_h: int, index: int) -> str:
    if len(path) < 2:
        return f"Road{index + 1}"
    mid = path[len(path) // 2]
    h_pos = "West" if mid[0] / img_w < 0.35 else ("East" if mid[0] / img_w > 0.65 else "")
    v_pos = "North" if mid[1] / img_h < 0.35 else ("South" if mid[1] / img_h > 0.65 else "")
    position = (v_pos + h_pos) or "Center"
    dx = path[-1][0] - path[0][0]
    dy = path[-1][1] - path[0][1]
    denom = abs(dx) + abs(dy) + 1e-9
    shape = "Straight" if abs(dx) / denom > 0.7 or abs(dy) / denom > 0.7 else "Curve"
    return f"{position}{shape}{index + 1}"


# ── AI ラベリング (Gemma 3) ───────────────────────────────────────────────────

LABELING_PROMPT = """You are analyzing an aerial/satellite image of a road network or racing circuit.
I have detected {n} road segments using computer vision. The image is {w}x{h} pixels.

Midpoint pixel coordinates of each detected segment:
{segments}

For each segment, assign a short descriptive name based on its position and shape.
Use names like: NorthStraight, SouthWestCurve, PitLane, MainStraight, Hairpin, Chicane, etc.

Return ONLY valid JSON (no markdown, no code fences):
[
  {{"index": 0, "name": "RoadName", "roadType": 0, "suggestedSpeed": 60}},
  ...
]

Road types: 0=normal, 1=highway, 2=race track. suggestedSpeed: integer km/h (20-200)."""


def label_paths_with_ai(
    pil_image: Image.Image,
    paths: list,
    img_w: int, img_h: int,
    model: str, ollama_url: str,
    max_size: int = DEFAULT_MAX_IMG_SIZE,
) -> dict[int, dict]:
    segments_lines = "\n".join(
        f"  Segment {i}: midpoint ({int(p[len(p)//2][0])}, {int(p[len(p)//2][1])})"
        for i, p in enumerate(paths)
    )
    prompt = LABELING_PROMPT.format(n=len(paths), w=img_w, h=img_h, segments=segments_lines)
    image_b64, _, _ = encode_image_to_base64(pil_image, max_size=max_size)
    payload = {
        "model": model, "prompt": prompt, "images": [image_b64],
        "stream": False, "options": {"temperature": 0.1, "num_predict": 2048},
    }
    resp = requests.post(f"{ollama_url}/api/generate", json=payload, timeout=120)
    resp.raise_for_status()
    try:
        items = extract_json_from_text(resp.json()["response"])
        return {int(it["index"]): it for it in items if isinstance(it, dict) and "index" in it}
    except Exception:
        return {}


# ── JSON パース ───────────────────────────────────────────────────────────────

def extract_json_from_text(raw: str) -> list[dict]:
    text = raw.strip()
    candidates = [text]
    m = re.search(r"```(?:json)?\s*(\[.*?\])\s*```", text, re.DOTALL)
    if m:
        candidates.append(m.group(1))
    s, e = text.find("["), text.rfind("]")
    if s != -1 and e > s:
        candidates.append(text[s:e + 1])

    for c in candidates:
        for attempt in [c, re.sub(r",\s*([}\]])", r"\1", c)]:
            try:
                data = json.loads(attempt)
                if isinstance(data, list):
                    return data
            except json.JSONDecodeError:
                continue
    raise ValueError(f"JSON 抽出失敗。応答先頭:\n{raw[:400]}")


# ── 座標変換・JSON 出力 ───────────────────────────────────────────────────────

def pixel_to_world(
    px: float, py: float,
    img_w: int, img_h: int,
    world_width_m: float,
    center_at_origin: bool = True,
) -> tuple[float, float]:
    world_h = world_width_m * (img_h / img_w)
    nx, nz = px / img_w, py / img_h
    if center_at_origin:
        return round((nx - 0.5) * world_width_m, 1), round((nz - 0.5) * world_h, 1)
    return round(nx * world_width_m, 1), round(nz * world_h, 1)


def format_road_json(data: list[dict]) -> str:
    raw = json.dumps(data, indent=2, ensure_ascii=False)
    for field in [
        "defaultTargetSpeed", "defaultFriction",
        "defaultWidthLaneLeft1", "defaultWidthLaneCenter", "defaultWidthLaneRight1",
        "curvatureRadius",
    ]:
        raw = re.sub(rf'("{field}": )(-?\d+)(?![\d.])', r"\g<1>\g<2>.0", raw)

    def fix_pos(m: re.Match) -> str:
        vals = [m.group(1), m.group(2), m.group(3)]
        return '"pos": [{}]'.format(", ".join(v if "." in v else v + ".0" for v in vals))

    return re.sub(
        r'"pos": \[\s*(-?[\d.]+),\s*(-?[\d.]+),\s*(-?[\d.]+)\s*\]', fix_pos, raw
    )


# ── 可視化 ────────────────────────────────────────────────────────────────────

def visualize_roads(
    original_image: Image.Image,
    roads: list[dict],
    line_width: int = 3,
) -> Image.Image:
    img_w, img_h = original_image.width, original_image.height
    vis = original_image.copy().convert("RGBA")
    overlay = Image.new("RGBA", vis.size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    n = len(roads)
    colors = [
        tuple(int(c * 255) for c in colorsys.hsv_to_rgb(i / max(n, 1), 0.9, 0.95))
        for i in range(n)
    ]
    for road, color in zip(roads, colors):
        pts = road["points"]
        ca, cs = color + (210,), color + (255,)
        if len(pts) >= 2:
            draw.line([(int(p[0]), int(p[1])) for p in pts], fill=ca, width=line_width)
        for px, py in pts:
            r = 5
            draw.ellipse([px-r, py-r, px+r, py+r], fill=cs, outline=(255,255,255,255), width=1)
        if pts:
            mid = pts[len(pts) // 2]
            lbl = road.get("name", "?")
            lw = len(lbl) * 7
            mx, my = int(mid[0]), int(mid[1])
            draw.rectangle([mx, my-12, mx+lw+4, my+4], fill=(0,0,0,160))
            draw.text((mx+2, my-11), lbl, fill=cs)
    return Image.alpha_composite(vis, overlay).convert("RGB")


# ── メイン処理パイプライン ────────────────────────────────────────────────────

def run_extraction(
    image: Optional[Image.Image],
    ollama_url: str, model_name: str,
    world_scale: float, center_at_origin: bool,
    default_speed: float, default_width: float,
    road_type: int, max_image_size: int,
    # CV パラメータ
    l_min: int, l_max: int, chroma_max: float,
    close_size: int, min_area: int,
    min_pixels: int, simplify_eps: float,
    smooth_window: int, merge_gap: float, merge_angle: float,
):
    """ハイブリッドパイプライン (Gradio generator)。"""
    if image is None:
        yield None, None, "画像をアップロードしてください。", "", gr.update(visible=False)
        return

    ollama_url = ollama_url.rstrip("/")
    img_w, img_h = image.width, image.height
    image_bgr = pil_to_bgr(image)

    try:
        # Step 1: 道路マスク
        yield None, None, "Step 1/4: 道路マスクを生成中...", "", gr.update(visible=False)
        road_mask, mask_pil = detect_road_mask(
            image_bgr, l_min, l_max, chroma_max, close_size, min_area
        )

        # Step 2: スケルトン化 → 方向ガイドトレース
        yield mask_pil, None, "Step 2/4: スケルトン化・方向ガイドトレース中...", "", gr.update(visible=False)
        paths = skeleton_to_paths(
            road_mask, min_pixels, simplify_eps,
            int(smooth_window), merge_gap, merge_angle,
        )

        if not paths:
            yield mask_pil, None, (
                "道路パスが検出されませんでした。\n"
                "輝度範囲・最大彩度・最小面積を調整してみてください。"
            ), "", gr.update(visible=False)
            return

        # Step 3: Gemma 3 で名前付与
        yield mask_pil, None, (
            f"Step 3/4: {len(paths)} パスを検出。Gemma 3 でラベリング中..."
        ), "", gr.update(visible=False)
        label_map = label_paths_with_ai(
            image, paths, img_w, img_h, model_name, ollama_url, int(max_image_size)
        )

        # Step 4: JSON 生成
        yield mask_pil, None, "Step 4/4: JSON を生成中...", "", gr.update(visible=False)

        road_objs: list[dict] = []
        for i, path in enumerate(paths):
            lbl = label_map.get(i, {})
            name = lbl.get("name") or auto_name_path(path, img_w, img_h, i)
            try:
                speed = max(20.0, min(200.0, float(lbl.get("suggestedSpeed", default_speed))))
            except (ValueError, TypeError):
                speed = default_speed
            rtype = lbl.get("roadType", int(road_type))

            pts_world = []
            for px, py in path:
                wx, wz = pixel_to_world(px, py, img_w, img_h, world_scale, center_at_origin)
                pts_world.append({
                    "pos": [wx, 0.0, wz],
                    "useCurvatureRadius": 0,
                    "curvatureRadius": 0.0,
                })

            road_objs.append({
                "id": generate_road_id(),
                "name": name,
                "roadType": int(rtype),
                "defaultTargetSpeed": float(speed),
                "defaultFriction": DEFAULT_FRICTION,
                "defaultWidthLaneLeft1": float(default_width),
                "defaultWidthLaneCenter": 0.0,
                "defaultWidthLaneRight1": float(default_width),
                "active": 1,
                "point": pts_world,
                "verticalCurve": [], "bankAngle": [], "laneSection": [],
            })

        json_str = format_road_json(road_objs)
        roads_for_vis = [{"name": r["name"], "points": p} for r, p in zip(road_objs, paths)]
        result_pil = visualize_roads(image, roads_for_vis)

        tmp = tempfile.NamedTemporaryFile(
            delete=False, suffix=".json", prefix="road_network_",
            mode="w", encoding="utf-8"
        )
        tmp.write(json_str)
        tmp.close()

        ai_note = f"AI ラベル {len(label_map)}/{len(paths)} 件" if label_map else "自動命名"
        n_pts = sum(len(p) for p in paths)
        status = (
            f"完了: {len(paths)} パス、{n_pts} ウェイポイント ({ai_note})\n"
            f"画像: {img_w}x{img_h}px  |  ワールドスケール: {world_scale}m 幅"
        )
        yield mask_pil, result_pil, status, json_str, gr.update(value=tmp.name, visible=True)

    except requests.exceptions.ConnectionError:
        yield None, None, (
            f"Ollama に接続できません ({ollama_url})。\n起動確認: ollama serve"
        ), "", gr.update(visible=False)
    except requests.exceptions.Timeout:
        yield None, None, (
            "Ollama がタイムアウト。「AI 送信時の最大画像サイズ」を小さくして再試行してください。"
        ), "", gr.update(visible=False)
    except requests.exceptions.HTTPError as e:
        yield None, None, (
            f"Ollama API エラー: {e}\nモデル名 '{model_name}' を確認: ollama list"
        ), "", gr.update(visible=False)
    except Exception as e:
        import traceback
        yield None, None, (
            f"エラー: {type(e).__name__}: {e}\n{traceback.format_exc()[-600:]}"
        ), "", gr.update(visible=False)


# ── Gradio UI ─────────────────────────────────────────────────────────────────

def build_ui() -> gr.Blocks:
    with gr.Blocks(title="Road Path Extractor") as demo:
        gr.Markdown(
            "# Road Path Extractor\n"
            "衛星・航空写真から道路パスを抽出 → `road_network_editor.html` 用 JSON を出力\n\n"
            "**方向ガイドトレース**: 分岐点で最も直進に近い方向を選んで連続パスを生成します。"
        )

        with gr.Row():
            # ── 左カラム: 入力 ───────────────────────────────────────────────
            with gr.Column(scale=1):
                image_input = gr.Image(type="pil", label="衛星・航空写真", height=360)

                with gr.Accordion("Ollama / 出力設定", open=False):
                    ollama_url    = gr.Textbox(value=DEFAULT_OLLAMA_URL, label="Ollama URL")
                    model_name    = gr.Textbox(value=DEFAULT_MODEL,      label="モデル名")
                    world_scale   = gr.Slider(100, 10000, DEFAULT_WORLD_SCALE, step=50,
                                              label="ワールドスケール（画像幅 [m]）")
                    center_origin = gr.Checkbox(value=True, label="ワールド原点を画像中央に配置")
                    default_speed = gr.Slider(20, 200, DEFAULT_SPEED, step=5,
                                              label="デフォルト速度 [km/h]")
                    default_width = gr.Slider(1.0, 10.0, DEFAULT_WIDTH, step=0.5,
                                              label="デフォルト車線幅 [m]")
                    road_type     = gr.Number(value=0, label="道路タイプ (roadType)", precision=0)
                    max_img_size  = gr.Slider(512, 2048, DEFAULT_MAX_IMG_SIZE, step=128,
                                              label="AI 送信時の最大画像サイズ [px]")

                with gr.Accordion("道路マスク（CV）パラメータ", open=True):
                    gr.Markdown("##### 検出色の調整（LAB色空間）")
                    with gr.Row():
                        l_min = gr.Slider(0, 120, DEFAULT_L_MIN, step=5,
                                          label="輝度下限 L_min",
                                          info="暗い影を除外")
                        l_max = gr.Slider(100, 255, DEFAULT_L_MAX, step=5,
                                          label="輝度上限 L_max",
                                          info="白線・明るい部分を除外")
                    with gr.Row():
                        chroma_max = gr.Slider(5, 80, DEFAULT_CHROMA_MAX, step=2,
                                               label="最大彩度 Chroma",
                                               info="小さい=純グレーのみ検出")
                        close_size = gr.Slider(5, 60, DEFAULT_CLOSE_SIZE, step=2,
                                               label="Close カーネル [px]",
                                               info="道路の隙間を埋める")
                    with gr.Row():
                        min_area   = gr.Slider(500, 20000, DEFAULT_MIN_AREA, step=500,
                                               label="最小面積 [px²]",
                                               info="小さなノイズを除去")
                        min_pixels = gr.Slider(20, 400, DEFAULT_MIN_PIXELS, step=10,
                                               label="最小パス長 [px]",
                                               info="短いパスを除外")

                    gr.Markdown("##### パス生成の調整")
                    with gr.Row():
                        simplify_eps  = gr.Slider(0.5, 10.0, DEFAULT_SIMPLIFY_EPS, step=0.5,
                                                  label="簡略化 ε [px]",
                                                  info="小さい=ウェイポイントが多い・精細")
                        smooth_window = gr.Slider(3, 21, DEFAULT_SMOOTH_WIN, step=2,
                                                  label="平滑化ウィンドウ [px]",
                                                  info="大きい=ジャギーが減る・なめらか")
                    with gr.Row():
                        merge_gap   = gr.Slider(0, 60, DEFAULT_MERGE_GAP, step=5,
                                                label="端点結合距離 [px]",
                                                info="近い端点を自動接続する距離")
                        merge_angle = gr.Slider(10, 90, DEFAULT_MERGE_ANGLE, step=5,
                                                label="端点結合角度 [°]",
                                                info="大きい=方向が多少ズレても結合")

                with gr.Row():
                    extract_btn = gr.Button("道路を検出", variant="primary", scale=3)
                    clear_btn   = gr.Button("クリア",    variant="secondary", scale=1)

            # ── 右カラム: 出力 ───────────────────────────────────────────────
            with gr.Column(scale=1):
                mask_output   = gr.Image(type="pil", label="道路マスク（緑＝検出領域）", height=260)
                result_output = gr.Image(type="pil", label="検出結果（パスオーバーレイ）", height=260)
                status_box    = gr.Textbox(label="ステータス", lines=3, interactive=False)
                json_output   = gr.Code(language="json", label="Road Network JSON", lines=10)
                download_btn  = gr.DownloadButton(
                    label="road_network.json をダウンロード", visible=False
                )

        extract_btn.click(
            fn=run_extraction,
            inputs=[
                image_input, ollama_url, model_name,
                world_scale, center_origin,
                default_speed, default_width, road_type, max_img_size,
                l_min, l_max, chroma_max, close_size, min_area,
                min_pixels, simplify_eps, smooth_window, merge_gap, merge_angle,
            ],
            outputs=[mask_output, result_output, status_box, json_output, download_btn],
        )

        clear_btn.click(
            fn=lambda: (None, None, None, "", "", gr.update(visible=False)),
            outputs=[image_input, mask_output, result_output, status_box, json_output, download_btn],
        )

    return demo


# ── エントリーポイント ────────────────────────────────────────────────────────

if __name__ == "__main__":
    demo = build_ui()
    demo.launch(
        server_name="0.0.0.0",
        show_error=True,
        inbrowser=True,
        theme=gr.themes.Soft(),
    )
