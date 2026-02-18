# Curve JSON 仕様

## 概要

curve JSONファイルは道路の線形データを定義する。
交差点から交差点までの線形を1本の道路（1オブジェクト）とする。
ポイント間はクロソイド曲線で接続される。

## 参考JSON
curve_sample.json を参照。

## ルール

- 1本の道路で円環（ループ）させてはならない
- 道路同士の接続は、始点・終点の座標を一致させることで表現する

## 道路オブジェクト

| フィールド | 型 | 説明 |
|---|---|---|
| id | string | UUID 8桁（例: `"7ce33863"`） |
| name | string | 固有の道路名（アルファベット） |
| roadType | int | 道路種別（基本 `0`） |
| defaultTargetSpeed | float | 通過想定速度（km/h） |
| defaultFriction | float | デフォルト摩擦係数（基本 `0.15`） |
| active | int | `1`: 使用中、`0`: 未使用・非表示 |
| point | array | ポイント座標の配列（後述） |
| verticalCurve | array | 縦断曲線（後述） |
| bankAngle | array | バンク角（後述） |
| laneSection | array | 車線セクション（後述） |

## point

ポイントは道路の線形を定義する制御点。ポイント間はクロソイド曲線で接続される。

```json
{
    "pos": [x, y, z],
    "useCurvatureRadius": 0,
    "curvatureRadius": 0.0
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| pos | [float, float, float] | 座標 `[x, y, z]`。yは標高 |
| useCurvatureRadius | int | 曲率半径の使用フラグ（基本 `0`） |
| curvatureRadius | float | 曲率半径（基本 `0.0`） |

## verticalCurve

縦断曲線の定義。道路の縦方向のカーブを制御する。

```json
{
    "u_coord": 0.5,
    "vcl": 50.0,
    "offset": 0.0
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| u_coord | float | 道路上の位置（0.0〜1.0、正規化座標） |
| vcl | float | 縦断曲線長 |
| offset | float | オフセット |

## bankAngle

バンク角（カント）の定義。

```json
{
    "u_coord": 0.5,
    "targetSpeed": 30.0,
    "overrideBank": 0,
    "bankAngle": 0.0
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| u_coord | float | 道路上の位置（0.0〜1.0） |
| targetSpeed | float | 対象速度（km/h） |
| overrideBank | int | バンク角の手動上書きフラグ |
| bankAngle | float | バンク角（度）。正が左傾き、負が右傾き |

## laneSection

車線構成の変化点を定義する。道路上の特定位置で車線幅や車線数を変更できる。

```json
{
    "u_coord": 0.5,
    "useLaneLeft2": 0,
    "widthLaneLeft2": 3.0,
    "useLaneLeft1": 1,
    "widthLaneLeft1": 3.0,
    "useLaneCenter": 1,
    "widthLaneCenter": 0.0,
    "useLaneRight1": 1,
    "widthLaneRight1": 3.0,
    "useLaneRight2": 0,
    "widthLaneRight2": 3.0,
    "offsetCenter": 0.0
}
```

| フィールド | 型 | 説明 |
|---|---|---|
| u_coord | float | 道路上の位置（0.0〜1.0） |
| useLaneLeft2 | int | 左第2車線の使用フラグ |
| widthLaneLeft2 | float | 左第2車線の幅（m） |
| useLaneLeft1 | int | 左第1車線の使用フラグ |
| widthLaneLeft1 | float | 左第1車線の幅（m） |
| useLaneCenter | int | 中央線の使用フラグ |
| widthLaneCenter | float | 中央線の幅（m） |
| useLaneRight1 | int | 右第1車線の使用フラグ |
| widthLaneRight1 | float | 右第1車線の幅（m） |
| useLaneRight2 | int | 右第2車線の使用フラグ |
| widthLaneRight2 | float | 右第2車線の幅（m） |
| offsetCenter | float | 中央線のオフセット（m） |

## コース生成時の注意

- 新規コース作成時、`verticalCurve`、`bankAngle`、`laneSection` は空配列 `[]` にする
- `useCurvatureRadius` は `0`、`curvatureRadius` は `0.0` とする
- `active` は基本 `1`
- 周回コースを作る場合、複数の道路に分割し、交差点の座標を一致させて接続する
