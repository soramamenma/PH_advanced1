# AI Safety Navigation

Physical AI 最終課題の移動ロボット版。OmniCampusアカウントは **moyashimame**。
Gazeboの差動二輪ロボットが、LiDARと既知の目標情報から速度を推論して移動します。
模倣学習モデル、模範制御、独立した安全シールド、要求と出力を表示するGUIを実装しました。

```text
LiDAR → 12方向の最小距離 ─┐
自己位置＋既知の目標 → 距離・方向 ─┴→ CPU MLP → 速度指令
LiDAR＋情報の鮮度 ───────────→ 安全シールド → 車両
```

## 再現環境と準備

Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、Python 3.12。WindowsではWSL2を使用します。
CPUで学習・推論し、有料APIやGPUは使いません。
ROS/GazeboとPython依存関係の準備は [SETUP](docs/SETUP.md) を参照してください。
以下はこのリポジトリのルートをカレントディレクトリとして実行します。

```bash
source scripts/env.sh
python3 scripts/check_submission.py --runtime
python3 -m unittest discover -s tests -v
```

上の確認はシミュレーターを起動せず、走行指令も送りません。
学習済みモデルと正規化情報を同梱しているため、再学習せず走行を試せます。

## AIの走行

```bash
source scripts/env.sh
python3 -m pai_ai_navigation.episode --mode ai-shield --scenario standard --timeout 80 --gui
```

Gazeboを初期化してAI走行を開始します。標準のゴールは (3.8, -1.0) mです。
終了はその端末で Ctrl+C。Gazeboも閉じる場合は、走行終了後に次を実行します。

```bash
python3 -m pai_ai_navigation.simulation stop
```

ROSドメイン42とGazebo通信区画 `pai_navigation` を使用します。同じ設定の別コピーを同時に走らせないでください。
管理処理は記録した専用プロセスだけを操作し、他のGazeboやWSL全体を終了しません。

## GUIと任意開始

[操作手順](docs/MONITOR.md)に、WindowsのネイティブGUIと録画に合わせた操作を説明しています。
「走行準備」で停止待機し、録画後に「走行開始」、途中終了は「停止」です。
アプリを開くだけでは走行しません。GUIのボタンは車両用で、録画は録画アプリで操作します。

## 模範制御と学習

模範制御は手動運転ではなく、人が設計した規則で進路と速度を選ぶ局所制御です。
車体の大きさと測距の粗さを考慮し、目標方向と通行余裕を評価します。
MLP (14 → 32 → 32 → 2、Tanh) はその入出力を模倣学習します。

```bash
source scripts/env.sh
# 同梱の8エピソード、3,007件を使用。既存モデルを上書きしない。
python3 -m pai_ai_navigation.train --data data --output models_v2 --epochs 400 --seed 42
# 新たな模範データの収集はシミュレーターを起動し走行させる。
python3 -m pai_ai_navigation.collect
```

学習1,966件・検証1,041件にエピソード単位で分割し、正規化は学習側のみから算出しました。
同梱CSVと学習済みモデルの説明は [データとモデル](data/README.md) を参照してください。

## 安全シールド

AIや模範制御の指令を車両へ送る前に、距離・情報の新しさ・異常値を監視する別ROSノードです。
進路は制御器が選び、シールドは必要に応じて指令を減速・停止へ制限します。

| 条件 | 動作 |
|---|---|
| 全方向の最小測距が0.42 m未満 | 前進・旋回とも停止 |
| 前方測距が0.50〜0.95 m | 前進速度を制限 |
| スキャン・自己位置・時計の古さ、指令途絶、NaNなど | 前進・旋回とも停止 |
| 目標距離0.22 m以内 | 制御器が到達として停止 |

距離はLiDAR原点基準です。安全シールド自身を強制終了したときの自動停止は保証しません。
通常は `episode` の終了処理で制御器を止め、ゼロ速度を送ります。
**これはシミュレーション上の制御制限の実験で、安全性の証明や実機用の完成システムではありません。**

## 実測結果と制約

2026年10月31日の同じ地図・開始位置・目標で、3方式を各3回比較しました。

| 方式 | 到達 | 平均シミュレーション時間 | 車体接触検出 |
|---|---:|---:|---:|
| 模範制御 | 3/3 | 26.45秒 | 0回 |
| AIのみ（鮮度・異常値監視あり） | 3/3 | 26.87秒 | 0回 |
| AIと安全シールド | 3/3 | 26.34秒 | 0回 |

通常走行では近接理由のシールド介入はなく、衝突率の改善を示した結果ではありません。
安全停止は条件を設定した別試験です。接触計測はbase_linkのみで、全車輪を網羅しません。

11月1日にゴールを (3.8, 0.0) mへ移した追加確認は、AIとシールドが80秒で未到達、模範制御が29.45秒で到達でした（各1回）。
この追加結果は上の9試行に混ぜていません。既知のゴール座標を利用しており、画像から隠れたゴールを探す機能は未実装です。
詳細と生の位置・指令ログは [RESULTS](docs/RESULTS.md) に保存しています。

## 計測と再生

```bash
# 新しい9試行を実際に走らせる。過去の結果は上書きしない。
python3 -m pai_ai_navigation.evaluate --trials 3
# 同梱の実測ログから補助的な2D再生を生成する。Gazebo録画とは別。
mkdir -p runtime
python3 -m pai_ai_navigation.replay docs/results/standard_runs/ai-shield_1 --output runtime/replay.html
```

既成の [2D再生例](docs/demo_replay.html) もあります。ブラウザーで開く補助資料です。
[追加の停止・接触検証](docs/SETUP.md#追加の制御試験) は実際にシミュレーターを操作する別試験です。

## ファイル構成

| 場所 | 内容 |
|---|---|
| `pai_ai_navigation/` | ROSノード、模範制御、学習・推論、安全監視、計測 |
| `worlds/` / `config/` / `launch/` | Gazeboワールド、通信ブリッジ、起動定義 |
| `models/` / `data/` | 採用モデル・正規化・学習記録、実測の教師データ |
| `scripts/` | 環境設定、GUI起動、確認、ビルド |
| `tests/` | 前処理・停止条件・排他・手動開始・配置などのテスト |
| `docs/results/` | 保存した実験結果と位置・指令の生ログ |

提出PDFとYouTube動画は別途提出する成果物です。このコードのコミットだけでは提出完了になりません。
ライセンスと参照資料は [LICENSE](LICENSE) と [参照元](docs/ATTRIBUTION.md) を参照してください。
