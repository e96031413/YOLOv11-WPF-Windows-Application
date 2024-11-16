# YOLOv11-WPF-Windows-Application

![image](https://github.com/user-attachments/assets/d42970fa-b1cf-4b97-a5d0-3d89744a2ec8)

這是一個基於WPF開發的物件偵測應用程式，使用YOLOv11模型進行即時物件偵測。應用程式提供了直覺的使用者介面，可以輕鬆地對圖片進行物件偵測並查看結果。

## 功能特點

- 📷 支援多種圖片格式（JPG、PNG、BMP）
- 🔍 使用YOLOv11進行高精度物件偵測
- 📊 即時顯示偵測結果和信心度
- 💾 可匯出偵測結果
- 📁 支援批次處理多張圖片
- 📈 物件統計分析功能

## 系統需求

- Windows 10 或更新版本
- .NET 9.0 或更新版本
- 建議：具備GPU加速功能的顯示卡

## 安裝步驟

1. 下載最新版本的應用程式
2. 解壓縮檔案到指定目錄
3. 執行 `ObjectDetectionApp.exe`

## 使用方法

1. 啟動應用程式
2. 點擊「選擇圖片」按鈕選擇要偵測的圖片
3. 點擊「開始偵測」進行物件偵測
4. 在右側面板查看偵測結果
5. 可選擇「儲存結果」將結果匯出

### 批次處理

1. 點擊「批次處理」按鈕
2. 選擇多張要處理的圖片
3. 等待處理完成
4. 查看並匯出結果

## 技術架構

- 框架：WPF (.NET 9.0)
- 物件偵測：YOLOv11n
- 影像處理：OpenCV Sharp
- UI元件：原生WPF控制項

## 相依套件

```xml
- OpenCvSharp4.Windows (4.8.0.20230708)
- OpenCvSharp4.WpfExtensions (4.8.0.20230708)
- YoloSharp (6.0.0)
```

## 開發配置

如果您想要自行編譯專案：

1. 克隆專案：
```bash
git clone [repository-url]
```

2. 使用Visual Studio Code開啟專案

3. 還原NuGet套件：
```bash
dotnet restore
```

4. 編譯專案：
```bash
dotnet build
```

## 注意事項

- 首次執行時需要下載YOLO模型檔案
- 確保系統有足夠的記憶體空間
- 建議使用GPU進行加速
- 支援的圖片格式：JPG、PNG、BMP
- 記得將[yolo11n.onnx](https://github.com/dme-compunet/YoloSharp/blob/main/Source/Assets/models/yolo11n.onnx)放置於專案目錄的Models下

## 常見問題

Q: 為什麼第一次執行較慢？
A: 首次執行需要載入AI模型，這是正常現象。

Q: 如何提升偵測效能？
A: 建議使用支援GPU加速的顯示卡。

## 授權說明

本專案採用 MIT 授權條款。詳見 [LICENSE](LICENSE) 檔案。

## 聯絡方式

如有任何問題或建議，歡迎提出 Issue。

## 更新日誌

### v1.0.0 (2024-11-16)
- 初始版本發布
- 支援基本物件偵測功能
- 批次處理功能
- 結果匯出功能
