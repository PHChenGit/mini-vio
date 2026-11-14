# Mini-VIO 決策紀錄

記錄專案的目的、已做出的決定與理由。需求本身見 [requirements.md](requirements.md)。

最後更新：2026-11-14

## 專案目的

學習 VIO 與 Visual SLAM。所有取捨都以「學得到東西」為準：

- 可讀性優先於精度與效能的極致。
- 每個階段都要有看得到的結果，並說得出背後的原理。
- 精度 KPI（ATE RMSE 小於 0.15 公尺）是努力目標，不是驗收門檻。

## 技術決定

| 項目 | 決定 | 理由 |
|---|---|---|
| 感測器 | 雙目 + IMU | 雙目直接提供尺度，視覺慣性對齊不必估計尺度，可以在移動中完成初始化 |
| 初始化 | EuRoC 用動態初始化，驗收用完整序列；靜止初始化只用於模擬資料 | 真實情況無法保證靜止開機，MH_01_easy 前約 21 秒都在移動。不用起始時間跳過開頭，而是把它當作要克服的挑戰 |
| 資料集 | 第一版只支援 EuRoC | KITTI 的 odometry benchmark 沒有可直接使用的高頻同步 IMU，且 KPI 都以 EuRoC 定義 |
| Landmark 表示法 | Explicit Landmark + GenericStereoFactor + Huber | 直接滿足 Huber 的需求，與 IncrementalFixedLagSmoother 搭配單純，Landmark 可以畫出來檢查，容易學習與除錯 |
| 不採用 Smart Factor | — | 內建的外點剔除是「接受或拒絕」，沒有 Huber 的位置；在滑動視窗邊界的處理也比較麻煩。若後端耗時超過 35 ms 再重新評估 |
| 退化保護 | 最小視差或最大距離門檻，ZUPT 為選配 | 雙目的單次觀測已有深度，靜止不會造成奇異，弱點是遠處視差極小的點 |
| 執行緒 | 第一版單一執行緒 | KPI 都以單一執行緒定義，也能控制程式碼行數 |
| 初始化的最小平方 | Ceres Solver 2.2.0（apt 套件） | 動態初始化的殘差由自己寫、Jacobian 用自動微分，對照 GTSAM 把殘差與 Jacobian 都包在因子裡的做法；可用 SphereManifold 直接處理「重力大小固定」的限制。滑動視窗後端仍用 GTSAM。apt 版本與 GTSAM、OpenCV 共用系統 Eigen |
| 程式風格 | Google C++ Style，每行上限 120 字元，副檔名 `.h`/`.cc`；命名照 Google，但座標轉換與矩陣保留數學記號（`T_W_B`、`R1`） | 用現成、完整的規範，clang-format 與 clang-tidy 能自動檢查；數學記號讓座標系一眼可辨（design.md 第 2 節），換成 `t_w_b` 反而容易搞混。由 git pre-commit hook 自動排版與檢查 |
| 邊緣化 | 交給 GTSAM 的 IncrementalFixedLagSmoother | 不手刻 Schur Complement，原理改以說明投影片補足 |
| 視覺化 | Rerun，從第一階段就使用 | 看到追蹤、Landmark 與軌跡，比只看誤差數字學得更多 |
| 評分用的軌跡 | 每個 Keyframe 建立當下的估計值 | 符合 Odometry 的即時定義，不使用事後平滑的結果 |
| 影像處理 | 每幀先將左右影像完整校正 | 校正後雙目匹配只需沿同一列搜尋，程式最單純 |
| Loop Closure | 留到第二版 | 這是 Visual SLAM 的核心，適合在 Odometry 完成後作為延伸 |

## 開發方式

採用「先視覺、後 IMU」的順序，切成五個階段，每個階段只加入一個新概念：

| 階段 | 建置內容 | 學習重點 | 看得到的結果 |
|---|---|---|---|
| 1 | 資料集讀取、影像校正 | 相機模型、雙目幾何 | Rerun 中的校正影像對 |
| 2 | KLT 雙目前端 | 特徵追蹤、外點剔除 | Rerun 中的特徵追蹤軌跡 |
| 3 | 純視覺後端 | 因子圖、Landmark、Huber、滑動視窗 | 第一條可用 evo 評分的軌跡 |
| 4 | 在模擬資料上加入 IMU | 預積分、座標系、Bias、靜止初始化 | 模擬軌跡與真值的比對 |
| 5 | 完整系統跑 EuRoC | 動態初始化（視覺慣性對齊）、調參、評估 | 兩個驗收序列完整序列的 ATE RMSE、初始化完成時間 |

階段 4 使用模擬資料的理由：IMU 的座標系與正負號錯誤，在真實資料上很難看出來，有已知真值才容易確認。

## 分工

- Claude 負責周邊：建置系統、Podman 容器、資料集讀取、評估腳本。
- 階段 2 到 5 的核心程式由使用者自己寫：Claude 先說明原理，使用者實作，Claude 再審查並協助除錯。
- 每個階段附一份說明投影片（放在 `docs/slides`），解釋該階段的原理，以及 GTSAM 在底層做了什麼。

理由：自己寫後端，比閱讀別人寫好的程式學得深。

## 每個階段的實驗

每個階段除了功能之外，安排二到三個實驗，觀察「拿掉某個機制會發生什麼事」。例如：

- 關閉 Huber，並注入錯誤的匹配。
- 把滑動視窗縮短到 1 秒。
- 在序列中途停用 IMU。

各階段的具體實驗在進入該階段時再訂定。

## 額外練習

不屬於五個階段的主線，也不計入核心程式碼行數，時間另外安排。

- **用 Ceres 手寫 Bundle Adjustment**（狀態：待安排，建議在階段 3 之前或期間）
  - 內容：在模擬資料（或 EuRoC 的一小段雙目 Keyframe）上，自己寫雙目重投影殘差，變數為各 Keyframe 的姿態與 3D 點，
    固定第一個姿態，加上 Huber loss，用 Ceres 求解。
  - 對照：同一個問題用 GTSAM 的 `GenericStereoFactor` 再解一次，比較結果與耗時。
  - 學習重點：重投影誤差與 Jacobian、Gauge Freedom（為什麼要固定第一個姿態）、旋轉的參數化與 Manifold、
    問題的稀疏結構與 Schur Complement（Ceres 的 `SPARSE_SCHUR`）、Robust Loss 的效果。

## 尚未解決的事項

- 資料集在主機的 `~/Documents/datasets/EuRoC`：MH_01_easy 已解壓縮；V1_01_easy 只有內層的 `V1_01_easy.zip`，尚未解壓縮出 `mav0/`。
