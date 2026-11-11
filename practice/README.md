# practice：Stretch URDF × TRC 醫院三關練習場

`trc_sim` 是一個純前端的網頁模擬器：載入 Hello Robot Stretch 的 URDF，在 TRC 醫療照護組三關的場景裡，用 stretch_ai `RobotClient` 同名的介面自動跑完三關並計分。目的是在 rover 做出來之前，先練流程、檢查可達範圍、找出 stretch_ai 缺的功能。

## 準備

模擬器會讀取兩個第三方 repo 的檔案，它們不在這個 repo 裡，要自己抓到 `practice/` 底下：

```powershell
cd practice
git clone --depth 1 https://github.com/hello-robot/stretch_ai.git
git clone --depth 1 https://github.com/hello-robot/stretch_urdf.git
```

- `stretch_ai`：只用到 `src/stretch/config/urdf/stretch.urdf`。完整 clone 會下載約 110 MB 的 LFS 檔，不需要的話先設 `$env:GIT_LFS_SKIP_SMUDGE=1`。
- `stretch_urdf`：用到 `stretch_urdf/SE3/meshes` 的 37 個網格。

不需要 Node。three.js 0.160.0（MIT）與 urdf-loader 0.12.3（Apache-2.0）已放在 `trc_sim/vendor`。

## 執行

```powershell
python -m http.server 8765 --bind 127.0.0.1 --directory practice
```

開 <http://127.0.0.1:8765/trc_sim/>，按「開始自動過三關」。

網址參數：`seed=N` 場景種子、`speed=1|4|10|30`、`autorun=1`、`view=follow|top|free`、`tab=tScore|tLog|tUrdf|tCap`。`headless=1`、`shotAt=秒`、`report=URL` 是自動測試用的。

## 檔案

| 檔案 | 內容 |
| --- | --- |
| `trc_sim/js/robot.js` | 載入 URDF；模擬版 `RobotClient`（方法名、預設姿態、速度上限取自 stretch_ai） |
| `trc_sim/js/planner.js` | 格點 A*，結構對照 stretch_ai 的 `motion/algo/a_star.py` |
| `trc_sim/js/scene.js` | 醫院三關場景、模擬感知、夾取與抽屜按鈕、裁判計分 |
| `trc_sim/js/tasks.js` | 自動過三關的任務腳本 |
| `trc_sim/js/main.js` | 畫面、模擬迴圈、面板 |

## 解讀結果時要注意

- 只有運動學。感知是「在視野、距離內且沒被擋住就給正確答案」，沒有夾持力、打滑或辨識錯誤。滿分代表流程與可達性走得通，不代表實機成功率。
- 場景是為了能過關而設計並調整過的（例如紅籤的一具人型障礙原本會封死 Bed 1 的通道，已移開；見 `scene.js` 內註解）。
- 配分依 `trc_plan.html` 的整理；每盒取藥 30 分、每個巡檢情境 25 分是由關卡總分反推的。20 種器具名稱是示意。
- Stretch 原機俯抓最高 95.3 cm，搆不到 96 cm 的器具桌，模擬裡改用斜持放下；桌面升到 135.5 cm 時完全搆不到。
