# RPLidar A1M8 制御システム

RPLidar A1M8を使用したLiDAR制御・可視化システムです。Mac環境で動作確認済みです。

## 概要

このプロジェクトは、RPLidar A1M8センサーからのデータを取得し、リアルタイムで可視化するPythonアプリケーションです。

## 動作環境

- **OS**: macOS
- **Python**: 3.7以上
- **デバイス**: RPLidar A1M8
- **接続**: USB経由のシリアル通信

## 主な機能

### 1. 2D可視化 (`2dplot.py`)
- RPLidar A1M8からの360度距離データをリアルタイムで極座標表示
- 高度な設定オプション付きMatplotlibアニメーション
- **設定可能な機能**：
  - シリアルポート設定 (`SERIAL_PORT`, `BAUDRATE`)
  - 更新レート調整 (`UPDATE_RATE`: デフォルト20fps)
  - 角度解像度設定 (`ANGLE_RESOLUTION`: 0.5°〜1.0°)
  - 距離に基づくカラーマップ表示 (`COLORMAP`: jet, viridis, plasma等)
  - 水平ミラー機能 (`MIRROR_HORIZONTALLY`: 左右反転)
  - リアルタイム統計情報表示（点数、平均/最小/最大距離）
- デバイス情報とヘルス状態の表示

### 2. 3D可視化 (`3dplot.py`)
- Plotly Dashを使用したインタラクティブな3D表示
- マウスによる視点変更が可能
- リアルタイムデータ更新

## セットアップ

### 1. 必要なライブラリのインストール

```bash
# 仮想環境の作成（推奨）
python -m venv venv
source venv/bin/activate  # macOS/Linux

# 必要なライブラリのインストール
pip install rplidar numpy matplotlib plotly dash pyserial
```

### 2. RPLidar A1M8の接続確認

デバイスを接続した後、シリアルポートを確認します：

```bash
ls /dev/tty.*
```

通常、`/dev/tty.usbserial-XXXX`形式で表示されます。

### 3. 設定の変更

#### 2dplot.py の設定

ファイル冒頭の設定セクションで各種パラメータを調整できます：

```python
# シリアルポート設定
SERIAL_PORT = '/dev/tty.usbserial-0001'  # 実際のポート番号に変更
BAUDRATE = 115200

# 表示設定
UPDATE_RATE = 50  # ミリ秒 (20 fps)
POINT_SIZE = 1    # データ点のサイズ
MAX_DISTANCE = 5000  # 最大表示距離 (mm)
ANGLE_RESOLUTION = 0.5  # 角度解像度 (度)

# カラーマップとミラー設定
COLORMAP = 'jet'  # カラーマップ
MIRROR_HORIZONTALLY = False  # 水平反転
```

## 使用方法

### 2D表示の実行

```bash
python 2dplot.py
```

実行すると以下の情報が表示されます：
- デバイス情報（モデル、ファームウェア、ハードウェア、シリアル番号）
- ヘルス状態とエラーコード
- 設定情報（更新レート、解像度、カラーマップ等）
- リアルタイム極座標プロット（統計情報付き）

### 3D表示の実行

```bash
python 3dplot.py
```

3D表示の場合、ブラウザが自動的に開き、`http://localhost:8050`でアクセスできます。

## RPLidar SDK

`rplidar_sdk/`フォルダには公式SDKが含まれています：

- **サンプルアプリケーション**: `app/`フォルダ内
- **ライブラリ**: `sdk/`フォルダ内
- **ドキュメント**: `docs/`フォルダ内

### SDK の使用

```bash
cd rplidar_sdk
make
```

コンパイル後、`output/Darwin/Release/`フォルダに実行ファイルが生成されます。

## トラブルシューティング

### 接続エラーの場合

1. シリアルポートの確認：
   ```bash
   ls /dev/tty.*
   ```

2. 権限の確認：
   ```bash
   sudo chmod 666 /dev/tty.usbserial-XXXX
   ```

3. デバイスの再接続

### 動作が重い場合

- データ取得頻度の調整
- 表示する点数の制限
- 仮想環境の使用

## 注意事項

- Mac環境でのみテスト済み
- USB接続が必要
- 適切なドライバーが必要な場合があります
- 初回実行時は権限の設定が必要な場合があります

## ライセンス

このプロジェクトは公式RPLidar SDKを含んでいます。詳細は`rplidar_sdk/LICENSE`を参照してください。

## 技術情報

- **通信プロトコル**: シリアル通信 (115200 baud)
- **データ形式**: バイナリ形式でのスキャンデータ
- **座標系**: 極座標 (角度, 距離) から直交座標への変換