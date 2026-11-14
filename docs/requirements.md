# Mini-VIO

## Purpose

- 這個專案的目的是學習 VIO 與 Visual SLAM，不是交付產品。
- 取捨原則： 程式碼的可讀性與「每一步都看得到、說得出原理」優先於精度與效能的極致。
- 開發方式、分工與各階段的學習重點記錄在 [decisions.md](decisions.md)。

## Scope & Non-Goals

- 我們要做什麼 (In-Scope)：
  - 感測器配置： 強烈建議採用 雙目 + IMU (Stereo-Inertial)。雙目能直接透過左右眼 Baseline 算出真實物理尺度（Metric Scale），因此視覺慣性對齊（Visual-Inertial Alignment）不必像單目一樣估計尺度，只需解出重力方向、速度與陀螺儀 Bias，可以在載具移動中完成初始化，不需要開機靜止。
  - 核心演算法： 緊耦合（Tightly-coupled）滑動視窗優化，狀態向量包含 $$X_i \in \text{SE}(3)$$、$V_i \in \mathbb{R}^3$、$B_i \in \mathbb{R}^6$。
  - 執行模式： 優先做 離線資料集播放模式（Dataset Player），直接吃硬碟裡的 EuRoC 檔案，確定演算法收斂後再包成 ROS 2 Node。
  - Landmark 表示法： 後端採用 Explicit Landmark，每個 3D 點都是因子圖中的變數，觀測以 GTSAM 的 GenericStereoFactor 搭配 Huber Robust Noise Model 表示。不使用 Smart Factor。
- 我們不做什麼 (Out-of-Scope / Non-Goals)：
  - 不寫 Loop Closure（迴路檢測）與全域 Pose Graph： 專注在純 Odometry 的局部精度與穩定性。Loop Closure 留到第二版，作為學習 Visual SLAM 的延伸；Keyframe 與 Landmark 的資料結構須保留擴充的空間。
  - 不手刻邊緣化（Marginalization）的 Schur Complement： 100% 委派給 GTSAM 的 IncrementalFixedLagSmoother (iSAM2) 處理。
  - 不自幹 GUI 視窗： 拋棄老舊且容易卡執行緒的 Pangolin，視覺化直接輸出成標準軌跡檔（TUM 格式）供 evo 評估，並透過輕量的 Rerun C++ SDK 渲染。
  - 第一版不支援 KITTI： KITTI 的 odometry benchmark 沒有可直接使用的高頻同步 IMU，且所有 KPI 都以 EuRoC 定義。
  - 第一版不做多執行緒： 先以單一執行緒完成並通過 KPI，前後端分離的多執行緒模式留到後續版本。DatasetReader 的 Concept 仍須保留，日後可加入其他資料集。

## Functional Requirements

以下數值皆為預設值，必須可由設定檔調整。

FR-1 資料集讀取 (DatasetReader)
  - 讀取 EuRoC ASL 格式：cam0、cam1 的影像清單與影像檔、imu0 的量測，以及各感測器 sensor.yaml 中的內參、畸變係數、外參與 IMU 雜訊參數。
  - 依時間戳順序逐筆輸出左、右影像與 IMU 量測；左右影像的配對由 SensorSync 負責（FR-2）。
  - 支援設定起始時間與播放長度，僅供除錯使用（例如只重跑某一段）。驗收一律從序列的第一筆資料開始，不跳過任何片段。

FR-2 感測器同步 (SensorSync)
  - 以時間戳配對左右影像，無法配對的影像直接丟棄。
  - 對任意兩個影像時刻，以 std::span<const ImuData> 提供涵蓋該區間的 IMU 序列，區間兩端以線性內插補齊。

FR-3 雙目前端 (FeatureDetector / Tracker)
  - 特徵偵測： 以網格均勻分布的方式補點，維持約 150 個追蹤中的特徵。
  - 時間追蹤： 在左影像上以 KLT 光流追蹤前一幀的特徵。
  - 雙目匹配： 以 KLT 光流在右影像找出對應點。
  - 外點剔除： 前後向光流一致性檢查、時間追蹤的 RANSAC，以及雙目匹配的極線檢查。
  - 輸出： 每個特徵的唯一 ID 與校正（Rectified）座標下的雙目量測 (uL, uR, v)。

FR-4 關鍵幀選擇 (Keyframe Selection)
  - 符合任一條件即建立 Keyframe：平均視差超過門檻、追蹤中的特徵數低於門檻、距離上一個 Keyframe 超過最長間隔。
  - 目標頻率約 3–5 Hz，使 3.0 秒視窗內維持 10–15 個 Keyframe。

FR-5 IMU 預積分
  - 使用 GTSAM 的預積分與 CombinedImuFactor，在相鄰 Keyframe 之間建立約束，並包含 Bias 的隨機漫步。
  - IMU 雜訊參數取自資料集的 sensor.yaml，可由設定檔覆寫。

FR-6 初始化
  - 動態初始化（EuRoC 的正式路徑）： 不假設開機靜止，從序列的第一筆資料開始執行。
    - 先以純視覺雙目後端估計最初數個 Keyframe 的姿態（雙目直接提供真實尺度）。
    - 與相鄰 Keyframe 之間的 IMU 預積分對齊，解出陀螺儀 Bias、重力方向與各 Keyframe 的速度；加速度計 Bias 初始值為零。
    - 以重力大小與對齊殘差檢查結果。不通過時捨棄最舊的 Keyframe 繼續嘗試，不強行初始化。
    - 開機靜止是動態初始化的特例，同樣必須能處理。
    - 初始化完成前不輸出姿態。輸出從第一筆資料到初始化完成所經過的時間。
  - 靜止初始化（只用於模擬資料，階段 4）：
    - 偵測開頭的靜止區段（加速度計變異低於門檻，持續約 1 秒）。
    - 由加速度計平均值求重力方向，決定初始 roll 與 pitch；由陀螺儀平均值求初始陀螺儀 Bias；初始速度為零。
    - 找不到靜止區段時回報錯誤並結束。

FR-7 滑動視窗後端
  - 使用 IncrementalFixedLagSmoother，視窗長度 3.0 秒。
  - 變數： 每個 Keyframe 的姿態 X、速度 V、Bias B，以及每個 Landmark L。
  - 因子： 第一個狀態的 Prior、相鄰 Keyframe 間的 CombinedImuFactor、每筆觀測一個掛載 Huber 的 GenericStereoFactor。
  - Landmark 在至少被 2 個 Keyframe 觀測到後才加入，初始值由雙目三角化取得。
  - 視差低於門檻或距離超過門檻的 Landmark 不加入。
  - Landmark 隨視窗一起邊緣化，不得殘留在圖中。
  - （選配）偵測到靜止時加入 ZUPT 約束。

FR-8 輸出
  - 以 TUM 格式輸出軌跡檔，內容為每個 Keyframe 建立當下的最新估計值（IMU 座標系在世界座標系下的姿態）。
  - 輸出前端與後端每次處理的耗時統計，用來驗證效能 KPI。
  - 透過 Rerun 顯示軌跡、Landmark 與特徵追蹤結果。從第一個開發階段就提供，結果寫成記錄檔，在主機上開啟檢視。

FR-9 執行方式與設定
  - 提供單一執行檔，以命令列指定設定檔、資料集路徑與輸出路徑。
  - 所有可調參數集中在一份 YAML 設定檔。
  - 提供評估腳本，對兩個驗收序列執行 evo_ape 並回報 ATE RMSE。

## Non-Functional & Modern C++ Requirements

A. C++20 語言特性的實戰應用
  - 編譯期多型 (concepts & requires)： 定義 FeatureDetector、Tracker 與 DatasetReader 的 Concept。
    - Use KLT for version 1.
    - 例如把 KLT 光流換成 SuperPoint + LightGlue 時，完全不需要 virtual 函式表（vtable）開銷，編譯器還能做跨模組最佳化。
  - 零拷貝視圖 (std::span & std::string_view)： 在 SensorSync 擷取兩個 Keyframe 之間的 IMU 序列傳給預積分器時，使用 std::span<const ImuData> 直接傳遞連續記憶體區段，杜絕 std::vector 的頻繁複製。
  - 現代化並行管理 (std::jthread & Lock-free Queue)： （後續版本，第一版不實作）若開啟多執行緒模式（前端 20Hz 即時追蹤、後端 5–10Hz 非同步優化），使用 C++20 的 std::jthread 搭配 std::stop_token 優雅管理執行緒生命週期，確保按下 Ctrl+C 時不會發生 Segfault 或 Deadlock。

B. 數值穩定性與容錯需求 (Robustness)
  - 退化場景保護 (Degeneracy Handling)： 當載具完全靜止（例如等紅綠燈或無人機懸停前）時，視覺三角化會因為零視差導致資訊矩陣奇異。雙目的單次觀測已能提供深度，因此靜止本身不會造成奇異；真正的弱點是遠處、視差（Disparity）極小的點。後端必須以最小視差或最大距離門檻濾除這類 Landmark。Zero-Velocity Update (ZUPT) 約束列為選配。
  - 追丟處理 (Tracking Loss)： 初始化完成後，前端追丟時以 IMU 推算姿態並持續輸出，等前端恢復後由新的視覺觀測修正，不重新初始化。只靠 IMU 推算超過時間上限時才重新初始化。追丟期間不得因每一幀都成為 Keyframe 而讓後端狀態數暴增。
  - 外點免疫 (Outlier Rejection)： 在非線性優化中，除了前端的 RANSAC，後端必須在每個 GenericStereoFactor 上掛載 Huber Robust Noise Model（gtsam::noiseModel::mEstimator::Huber），防止單一錯誤匹配的光流點把整個 IMU Bias 拉飛。

## Acceptance Criteria / KPI

1. 精度指標 (Accuracy)：
  - 在 MH_01_easy 與 V1_01_easy 的完整序列上（不設起始時間），使用 evo_ape 對齊評估，絕對軌跡誤差（ATE RMSE）小於 0.15 公尺（純 Odometry 無 Loop Closure 的合理優秀水準）。
  - 這項指標是努力目標，不是驗收門檻：專案以學習為目的，未達標不阻擋後續階段。
2. 效能指標 (Performance)：
  - 在單一 CPU 執行緒下，前端處理耗時小於 10 ms（支援 30+ FPS）。
  - 滑動視窗大小設為 $\Delta t = 3.0\text{ s}$（約 10–15 個 Keyframes）時，後端每次 iSAM2 增量更新耗時小於 35 ms。
3. 工程品質 (Code Quality)：
  - 核心庫（mini_vio_core）總程式碼行數控制在 2,000 到 3,000 行內，維持極簡、高可讀性。
  - 透過 Podman 容器化建置環境，達成一行指令編譯與執行測試。

### Dataset

- 資料集： EuRoC MAV Dataset（ASL 格式），第一版唯一支援的資料集。
- 驗收序列： MH_01_easy 與 V1_01_easy。
- 感測器： 雙目灰階相機（20 Hz、752×480、全域快門）與 IMU（200 Hz）。實際參數一律以各序列的 sensor.yaml 為準。
- 影像未經校正： 原始影像帶有畸變，使用前必須依 sensor.yaml 完成去畸變與雙目校正。
- 起始片段： MH_01_easy 前約 21 秒都在手持移動（其中 0–6 秒劇烈晃動），之後才出現第一段靜止。這段刻意不跳過：真實使用時無法保證靜止開機，系統必須以動態初始化（FR-6）在移動中完成初始化，並撐過開頭的晃動。
- 真值（Ground Truth）： 使用各序列 state_groundtruth_estimate0 的資料，由 evo 讀取後與輸出軌跡對齊評估。
- 存放方式： 資料集不放進版本庫，放在主機上並以唯讀方式掛載進 Podman 容器，路徑由命令列參數指定。
