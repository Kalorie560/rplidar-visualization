#!/usr/bin/env python3
"""
RPLIDAR A1 3Dインタラクティブ可視化プログラム（修正版）
Plotly Dashを使用してマウスで視点を変更可能な3D表示
"""

import serial
import struct
import numpy as np
import plotly.graph_objects as go
from dash import Dash, dcc, html, State
from dash.dependencies import Input, Output
import time
from threading import Thread, Lock
import collections

class RPLidarA1:
    """RPLIDAR A1用の簡易ドライバ"""

    SYNC_BYTE = 0xA5
    SYNC_BYTE2 = 0x5A

    GET_INFO = 0x50
    GET_HEALTH = 0x52
    STOP = 0x25
    RESET = 0x40
    SCAN = 0x20

    DESCRIPTOR_LEN = 7
    INFO_LEN = 20
    HEALTH_LEN = 3

    def __init__(self, port, baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None

    def connect(self):
        """シリアルポートに接続"""
        self.serial = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS
        )

    def disconnect(self):
        """接続を閉じる"""
        if self.serial:
            self.serial.close()

    def _send_command(self, cmd):
        """コマンドを送信"""
        req = struct.pack('<BB', self.SYNC_BYTE, cmd)
        self.serial.write(req)

    def _read_descriptor(self):
        """ディスクリプタを読み取り"""
        descriptor = self.serial.read(self.DESCRIPTOR_LEN)
        if len(descriptor) != self.DESCRIPTOR_LEN:
            raise Exception("ディスクリプタの読み取りに失敗")
        return descriptor

    def get_info(self):
        """デバイス情報を取得"""
        self._send_command(self.GET_INFO)
        descriptor = self._read_descriptor()
        data = self.serial.read(self.INFO_LEN)

        if len(data) == self.INFO_LEN:
            model, firmware_minor, firmware_major, hardware, serial_number = struct.unpack('<BBBB16s', data)
            return {
                'model': model,
                'firmware': f"{firmware_major}.{firmware_minor}",
                'hardware': hardware,
                'serial_number': serial_number.hex()
            }
        return None

    def start_scan(self):
        """スキャンを開始"""
        self._send_command(self.SCAN)
        descriptor = self._read_descriptor()
        if descriptor[0] != self.SYNC_BYTE or descriptor[1] != self.SYNC_BYTE2:
            raise Exception("スキャン開始に失敗")

    def stop_scan(self):
        """スキャンを停止"""
        self._send_command(self.STOP)
        time.sleep(0.1)
        self.serial.reset_input_buffer()

    def read_scan_data(self):
        """スキャンデータを読み取り（ジェネレータ）"""
        while True:
            data = self.serial.read(5)
            if len(data) == 5:
                quality = data[0] >> 2
                angle = ((data[2] << 8) | data[1]) >> 1
                angle = angle / 64.0  # 度に変換
                distance = (data[4] << 8) | data[3]
                distance = distance / 4.0  # mmに変換

                if quality > 0 and distance > 0:
                    yield {
                        'angle': angle,
                        'distance': distance,
                        'quality': quality
                    }


# グローバル変数（データ共有用）
lidar_data = {
    'angles': collections.deque(maxlen=1800),  # 最大1800点（半分に削減）
    'distances': collections.deque(maxlen=1800),
    'qualities': collections.deque(maxlen=1800)
}
data_lock = Lock()
lidar_instance = None
scan_thread = None
running = True

def scan_worker():
    """別スレッドでスキャンデータを読み取る"""
    global running
    while running:
        try:
            data = next(lidar_instance.read_scan_data())
            with data_lock:
                lidar_data['angles'].append(data['angle'])
                lidar_data['distances'].append(data['distance'])
                lidar_data['qualities'].append(data['quality'])
        except:
            break

# Dashアプリケーションの作成
app = Dash(__name__)

app.layout = html.Div([
    html.Div([
        # 左側：コントロールパネル
        html.Div([
            html.H3('設定', style={'textAlign': 'center', 'marginBottom': '20px'}),

            html.Div([
                html.Label('更新レート (秒):', style={'marginBottom': '5px'}),
                dcc.Slider(
                    id='update-rate-slider',
                    min=0.1,
                    max=2.0,
                    step=0.1,
                    value=0.5,
                    marks={i/10: str(i/10) for i in range(1, 21, 5)},
                    tooltip={"placement": "bottom", "always_visible": True}
                )
            ], style={'marginBottom': '30px'}),

            html.Div([
                html.Label('プロット点数:', style={'marginBottom': '5px'}),
                dcc.Slider(
                    id='points-slider',
                    min=180,
                    max=1440,
                    step=180,
                    value=720,
                    marks={i: str(i) for i in range(180, 1441, 360)},
                    tooltip={"placement": "bottom", "always_visible": True}
                )
            ], style={'marginBottom': '30px'}),

            html.Div([
                html.Button('停止', id='stop-button', n_clicks=0,
                           style={'backgroundColor': '#ff4444', 'color': 'white',
                                  'fontSize': '16px', 'padding': '10px 20px',
                                  'border': 'none', 'borderRadius': '5px',
                                  'cursor': 'pointer', 'width': '100%'}),
                html.Div(id='status', style={'marginTop': '10px', 'textAlign': 'center'})
            ], style={'marginBottom': '30px'}),

            html.Div(id='stats', style={'fontSize': '12px', 'lineHeight': '1.5'})

        ], style={'width': '250px', 'padding': '20px', 'backgroundColor': '#f0f0f0',
                  'height': '100vh', 'overflowY': 'auto', 'position': 'fixed', 'left': 0}),

        # 右側：グラフ
        html.Div([
            html.H1('RPLIDAR A1 - 3Dリアルタイム可視化', style={'textAlign': 'center', 'marginBottom': '10px'}),
            dcc.Graph(id='3d-scatter', style={'height': 'calc(100vh - 80px)'})
        ], style={'marginLeft': '250px', 'padding': '10px'})
    ]),

    dcc.Interval(id='interval-component', interval=500, n_intervals=0),

    # 隠しDiv（停止状態を保持）
    html.Div(id='stopped-state', style={'display': 'none'}, children='false')
])

# 更新レート変更のコールバック
@app.callback(
    Output('interval-component', 'interval'),
    [Input('update-rate-slider', 'value')]
)
def update_interval(rate):
    return rate * 1000  # 秒をミリ秒に変換

# 停止ボタンのコールバック
@app.callback(
    [Output('stopped-state', 'children'), Output('stop-button', 'children'),
     Output('stop-button', 'style'), Output('status', 'children')],
    [Input('stop-button', 'n_clicks')],
    [State('stopped-state', 'children')]
)
def toggle_stop(n_clicks, stopped):
    if n_clicks == 0:
        return 'false', '停止', {'backgroundColor': '#ff4444', 'color': 'white',
                                'fontSize': '16px', 'padding': '10px 20px',
                                'border': 'none', 'borderRadius': '5px',
                                'cursor': 'pointer'}, ''

    is_stopped = stopped == 'true'
    if is_stopped:
        # 再開
        return 'false', '停止', {'backgroundColor': '#ff4444', 'color': 'white',
                                'fontSize': '16px', 'padding': '10px 20px',
                                'border': 'none', 'borderRadius': '5px',
                                'cursor': 'pointer'}, '動作中'
    else:
        # 停止
        return 'true', '再開', {'backgroundColor': '#44ff44', 'color': 'black',
                               'fontSize': '16px', 'padding': '10px 20px',
                               'border': 'none', 'borderRadius': '5px',
                               'cursor': 'pointer'}, '停止中'

# メイングラフ更新のコールバック
@app.callback(
    [Output('3d-scatter', 'figure'), Output('stats', 'children')],
    [Input('interval-component', 'n_intervals'), Input('points-slider', 'value')],
    [State('3d-scatter', 'figure'), State('stopped-state', 'children'),
     State('3d-scatter', 'relayoutData')]
)
def update_graph(n, max_points, current_figure, stopped, relayout_data):
    """グラフを更新"""
    # 停止中の場合は更新しない
    if stopped == 'true':
        return current_figure or go.Figure(), "データ取得停止中"

    with data_lock:
        if len(lidar_data['angles']) == 0:
            # データがない場合は空のグラフを返す
            fig = go.Figure(data=[go.Scatter3d(x=[], y=[], z=[])])
        else:
            # データをコピー
            angles = list(lidar_data['angles'])
            distances = list(lidar_data['distances'])
            qualities = list(lidar_data['qualities'])

            # データを間引き（UIで設定された点数に基づく）
            step = max(1, len(angles) // max_points)
            angles = angles[::step]
            distances = distances[::step]
            qualities = qualities[::step]

            # 3D座標に変換
            angles_rad = np.radians(angles)
            x = distances * np.cos(angles_rad)
            y = distances * np.sin(angles_rad)
            # Z軸は0（2D LiDARなので高さ情報なし）
            z = np.zeros_like(angles)

            # 3Dプロット作成
            fig = go.Figure()

            # LiDARデータの点群
            fig.add_trace(go.Scatter3d(
                x=x, y=y, z=z,
                mode='markers',
                name='LiDARデータ',
                marker=dict(
                    size=1,  # 点を小さく
                    color=distances,  # 距離で色分け
                    colorscale='Turbo',  # 見やすいカラースケール
                    colorbar=dict(title="距離 (mm)"),
                    showscale=True,
                    cmin=0,
                    cmax=5000
                )
            ))

            # 中心点（LiDAR位置）を追加
            fig.add_trace(go.Scatter3d(
                x=[0], y=[0], z=[0],
                mode='markers',
                name='中心',
                marker=dict(
                    size=1,
                    color='black'
                ),
                showlegend=False
            ))

    # レイアウト設定（カメラ位置を保持）
    camera = dict(eye=dict(x=0.5, y=0.5, z=2.5))  # デフォルトは上から見下ろすビュー

    # relayoutDataから最新のカメラ位置を取得
    if relayout_data and 'scene.camera' in relayout_data:
        camera = relayout_data['scene.camera']
    # 既存のfigureからカメラ位置を取得（relayoutDataがない場合）
    elif current_figure and 'layout' in current_figure and 'scene' in current_figure['layout']:
        if 'camera' in current_figure['layout']['scene']:
            camera = current_figure['layout']['scene']['camera']

    fig.update_layout(
        scene=dict(
            xaxis=dict(title="X (mm)", range=[-5000, 5000]),
            yaxis=dict(title="Y (mm)", range=[-5000, 5000]),
            zaxis=dict(title="Z (mm)", range=[-100, 100]),
            camera=camera,
            dragmode='orbit'  # ドラッグモードを明示的に設定
        ),
        margin=dict(l=0, r=0, t=0, b=0),
        uirevision='constant',  # UIの状態を保持
        showlegend=False  # 凡例を非表示
    )

    # 統計情報
    if len(lidar_data['distances']) > 0:
        avg_dist = np.mean(list(lidar_data['distances']))
        min_dist = np.min(list(lidar_data['distances']))
        max_dist = np.max(list(lidar_data['distances']))
        stats = html.Div([
            html.P(f"ポイント数: {len(lidar_data['distances'])}", style={'margin': '5px 0'}),
            html.P(f"平均距離: {avg_dist:.0f} mm", style={'margin': '5px 0'}),
            html.P(f"最小距離: {min_dist:.0f} mm", style={'margin': '5px 0'}),
            html.P(f"最大距離: {max_dist:.0f} mm", style={'margin': '5px 0'})
        ])
    else:
        stats = html.Div("データ待機中...")

    return fig, stats


def main():
    """メイン関数"""
    global lidar_instance, scan_thread, running

    # LIDARに接続
    lidar_instance = RPLidarA1('/dev/tty.usbserial-0001')

    try:
        print("RPLIDARに接続中...")
        lidar_instance.connect()

        # デバイス情報を表示
        info = lidar_instance.get_info()
        if info:
            print(f"モデル: {info['model']}")
            print(f"ファームウェア: {info['firmware']}")

        # スキャンを開始
        print("\nスキャンを開始します...")
        lidar_instance.start_scan()

        # スキャンスレッドを開始
        scan_thread = Thread(target=scan_worker)
        scan_thread.start()

        # Webサーバーを起動
        print("\nブラウザで http://127.0.0.1:8050 を開いてください")
        print("マウスでドラッグ: 回転")
        print("マウスホイール: ズーム")
        print("右クリック+ドラッグ: パン")
        print("Ctrl+Cで終了")

        # Dashアプリを実行
        app.run(debug=False)

    except KeyboardInterrupt:
        print("\n終了します...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        # クリーンアップ
        running = False
        if scan_thread:
            scan_thread.join()
        lidar_instance.stop_scan()
        lidar_instance.disconnect()
        print("接続を閉じました")


if __name__ == "__main__":
    main()
