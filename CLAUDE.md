# road-trace プロジェクト

## 概要

衛星・航空写真から道路パスを抽出し、レーシングサーキットのネットワークデータを生成・編集するツール群。

## ファイル構成

| ファイル | 役割 |
|---------|------|
| `road_network_editor.html` | 道路ネットワークをブラウザ上で手動編集するエディタ（バニラ JS + Canvas 2D） |
| `road_detector.py` | 衛星写真から道路パスを自動抽出する Gradio アプリ（Python） |
| `requirements.txt` | Python 依存ライブラリ |
| `curve.md` | JSON フォーマット仕様書 |
| `curve_sample.json` | サンプル道路ネットワーク（26 道路） |
| `sample.jpg` | テスト用衛星写真 |

## road_detector.py の起動方法

```bash
pip install -r requirements.txt
python road_detector.py
# → http://localhost:7860 が自動で開く
```

## road_detector.py のパイプライン

1. **道路マスク生成** — OpenCV LAB色空間でアスファルト（低彩度グレー）ピクセルを検出
2. **スケルトン化** — scikit-image で 1px 幅の中心線に変換
3. **方向ガイドトレース** — 分岐点で「最も直進に近い方向」を選んで長い連続パスを生成
4. **スムージング + 簡略化** — スライディングウィンドウ平均 → Douglas-Peucker
5. **端点結合** — 近傍かつ方向一致のパス端点を自動接続
6. **AI ラベリング** — Ollama + Gemma 3（`gemma3:12b`）で道路名・種別・推奨速度を付与
7. **JSON 出力** — `road_network_editor.html` 互換フォーマット

## 出力 JSON フォーマット

`road_network_editor.html` の `loadJSON()` が受け付けるスキーマ（[curve.md](curve.md) 参照）:

```json
[{
  "id": "abc12345",
  "name": "RoadName",
  "roadType": 0,
  "defaultTargetSpeed": 60.0,
  "defaultFriction": 0.15,
  "defaultWidthLaneLeft1": 3.0,
  "defaultWidthLaneCenter": 0.0,
  "defaultWidthLaneRight1": 3.0,
  "active": 1,
  "point": [
    {"pos": [x, 0.0, z], "useCurvatureRadius": 0, "curvatureRadius": 0.0}
  ],
  "verticalCurve": [],
  "bankAngle": [],
  "laneSection": []
}]
```

**重要な制約:**
- `pos` は `[x, 0.0, z]` の順（Y は常に 0.0）
- 浮動小数点フィールドには必ず小数点を付ける（`3` ではなく `3.0`）
- 全フィールドが必須（`verticalCurve` / `bankAngle` / `laneSection` は空配列）

## 座標系

`road_network_editor.html` の座標系：
- **X+** = 右方向（東）
- **Z+** = 下方向（南）
- **Y** = 常に 0（平面）
- 単位: メートル、原点は画像中央に配置（デフォルト）

## Ollama 環境

- **使用モデル**: `gemma3:12b`（ビジョン対応）
- **エンドポイント**: `http://localhost:11434/api/generate`
- `stream: false` で同期呼び出し
- 画像は JPEG base64 エンコードで `images` フィールドに渡す

## CV パラメータの調整指針

| 状況 | 調整 |
|------|------|
| 道路が検出されない | `L_max` を上げる、`Chroma` を上げる |
| 建物・駐車場も検出される | `Chroma` を下げる、`最小面積` を上げる |
| 道路が途切れる | `Close カーネル` を大きくする |
| パスが細切れ | `端点結合距離` を大きくする |
| パスがジャギー | `平滑化ウィンドウ` を大きくする |
| ウェイポイントが多すぎ | `簡略化 ε` を大きくする |
