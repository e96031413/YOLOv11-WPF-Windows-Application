using Microsoft.Win32;
using OpenCvSharp;
using OpenCvSharp.WpfExtensions;
using System;
using System.Collections.ObjectModel;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Compunet.YoloSharp;
using ObjectDetectionApp.Models;
using System.Linq;
using System.Diagnostics;
using System.Windows.Input;
using System.ComponentModel;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Controls;

namespace ObjectDetectionApp
{
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        private YoloPredictor? _predictor;
        private string? _currentImagePath;
        private ObservableCollection<DetectionResult> _detectionResults;
        private ObservableCollection<ObjectStatistic> _objectStats;
        private bool _isLabelSortAscending = true;
        private bool _isConfidenceSortAscending = true;

        public event PropertyChangedEventHandler? PropertyChanged;

        public bool IsLabelSortAscending
        {
            get => _isLabelSortAscending;
            set
            {
                if (_isLabelSortAscending != value)
                {
                    _isLabelSortAscending = value;
                    OnPropertyChanged(nameof(IsLabelSortAscending));
                }
            }
        }

        public bool IsConfidenceSortAscending
        {
            get => _isConfidenceSortAscending;
            set
            {
                if (_isConfidenceSortAscending != value)
                {
                    _isConfidenceSortAscending = value;
                    OnPropertyChanged(nameof(IsConfidenceSortAscending));
                }
            }
        }

        protected virtual void OnPropertyChanged(string propertyName)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(propertyName));
        }

        public ICommand SortByLabelCommand { get; }
        public ICommand SortByConfidenceCommand { get; }

        private readonly Dictionary<string, Scalar> _classColors = new Dictionary<string, Scalar>();
        private readonly Random _random = new Random(42);  // 使用固定種子以確保顏色一致性

        public class ObjectStatistic
        {
            public string ClassName { get; set; }
            public int Count { get; set; }

            public ObjectStatistic(string className, int count)
            {
                ClassName = className;
                Count = count;
            }
        }

        public MainWindow()
        {
            InitializeComponent();
            InitializeModel();
            _detectionResults = new ObservableCollection<DetectionResult>();
            _objectStats = new ObservableCollection<ObjectStatistic>();
            
            // 使用 lambda 表達式來確保命令總是在 UI 線程上執行
            SortByLabelCommand = new RelayCommand(() => Dispatcher.Invoke(SortByLabel));
            SortByConfidenceCommand = new RelayCommand(() => Dispatcher.Invoke(SortByConfidence));
            
            DataContext = this;
            lvDetectionResults.ItemsSource = _detectionResults;
            lvObjectStats.ItemsSource = _objectStats;

            btnSelectImage.Click += BtnSelectImage_Click;
            btnStartDetection.Click += BtnStartDetection_Click;
            btnSaveResult.Click += BtnSaveResult_Click;
            btnBatchProcess.Click += BtnBatchProcess_Click;
        }

        private void InitializeModel()
        {
            try
            {
                var modelPath = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Models", "yolo11n.onnx");
                if (!File.Exists(modelPath))
                {
                    MessageBox.Show("找不到模型檔案：yolo11n.onnx\n請確保模型檔案存在於 Models 目錄中。", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                _predictor = new YoloPredictor(modelPath);

                // 嘗試進行一次測試偵測以確保模型正常運作
                try
                {
                    using var testImage = new Mat(1, 1, MatType.CV_8UC3, Scalar.All(0));
                    var testResult = _predictor.Detect(testImage.ToBytes());
                    if (testResult == null)
                    {
                        throw new Exception("模型測試偵測失敗");
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"模型初始化測試失敗：{ex.Message}", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                    _predictor = null;
                    return;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"初始化模型時發生錯誤：{ex.Message}", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                _predictor = null;
            }
        }

        private void BtnSelectImage_Click(object sender, RoutedEventArgs e)
        {
            var openFileDialog = new OpenFileDialog
            {
                Filter = "圖片檔案|*.jpg;*.jpeg;*.png;*.bmp|所有檔案|*.*"
            };

            if (openFileDialog.ShowDialog() == true)
            {
                _currentImagePath = openFileDialog.FileName;
                
                // 載入並顯示原始圖片
                using var image = new Mat(_currentImagePath);
                imgMain.Source = image.ToBitmapSource();
                
                // 清空之前的結果
                _detectionResults.Clear();
                _objectStats.Clear();
                
                // 更新保存按鈕狀態
                UpdateSaveButtonState();
            }
        }

        private async void BtnStartDetection_Click(object sender, RoutedEventArgs e)
        {
            if (_currentImagePath == null)
            {
                MessageBox.Show("請先選擇圖片", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            if (_predictor == null)
            {
                MessageBox.Show("YOLO 模型尚未初始化", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            try
            {
                btnStartDetection.IsEnabled = false;
                Mouse.OverrideCursor = Cursors.Wait;

                // 清空之前的結果
                _detectionResults.Clear();
                _objectStats.Clear();

                var result = await Task.Run(() => _predictor.Detect(_currentImagePath));
                if (result == null)
                {
                    MessageBox.Show("偵測失敗：未能取得結果", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                // 使用 OpenCV 顯示結果
                using var image = new Mat(_currentImagePath);

                foreach (var detection in result)
                {
                    var bbox = detection.Bounds;
                    var rect = new System.Windows.Rect(
                        bbox.X,
                        bbox.Y,
                        bbox.Width,
                        bbox.Height);

                    // 清理物件名稱
                    var objectName = detection.Name.ToString().Split(':').Last().Trim('"');
                    var color = GetColorForClass(objectName);  // 使用清理過的名稱獲取顏色

                    var detectionItem = new DetectionResult(
                        objectName,
                        detection.Confidence,
                        rect);

                    _detectionResults.Add(detectionItem);

                    // 在圖片上繪製邊界框
                    Cv2.Rectangle(image,
                        new OpenCvSharp.Point((int)bbox.X, (int)bbox.Y),
                        new OpenCvSharp.Point((int)(bbox.X + bbox.Width), 
                                            (int)(bbox.Y + bbox.Height)),
                        color, 1);

                    // 計算文字大小以繪製背景
                    var labelText = $"{objectName} {detection.Confidence:P0}";
                    var fontScale = 0.5;
                    var thickness = 1;
                    var font = HersheyFonts.HersheySimplex;
                    var textSize = Cv2.GetTextSize(labelText, font, fontScale, thickness, out var baseline);

                    // 繪製文字背景矩形（在框內上方）
                    var rect_x = (int)bbox.X;
                    var rect_y = (int)bbox.Y + textSize.Height + baseline;
                    Cv2.Rectangle(image,
                        new OpenCvSharp.Point(rect_x, rect_y - textSize.Height - baseline),
                        new OpenCvSharp.Point(rect_x + textSize.Width, rect_y),
                        color,
                        -1);

                    // 添加白色文字
                    Cv2.PutText(image,
                        labelText,
                        new OpenCvSharp.Point(rect_x, rect_y - baseline),
                        font,
                        fontScale,
                        Scalar.White,
                        thickness);
                }

                // 更新圖片顯示
                imgMain.Source = image.ToBitmapSource();

                // 更新物件統計
                UpdateObjectStatistics();
                
                // 更新保存按鈕狀態
                UpdateSaveButtonState();
            }
            catch (Exception ex)
            {
                MessageBox.Show($"偵測失敗: {ex.Message}", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
            }
            finally
            {
                btnStartDetection.IsEnabled = true;
                Mouse.OverrideCursor = null;
            }
        }

        private void UpdateObjectStatistics()
        {
            var objectCounts = _detectionResults
                .GroupBy(r => r.Label)
                .Select(g => new ObjectStatistic(g.Key, g.Count()))
                .OrderByDescending(s => s.Count)
                .ThenBy(s => s.ClassName);

            _objectStats.Clear();
            foreach (var stat in objectCounts)
            {
                _objectStats.Add(stat);
            }
        }

        private void UpdateSaveButtonState()
        {
            btnSaveResult.IsEnabled = _currentImagePath != null && _detectionResults.Count > 0;
        }

        private Scalar GetColorForClass(string className)
        {
            className = className.ToLower();  // 轉換為小寫以確保一致性
            
            if (_classColors.ContainsKey(className))
            {
                return _classColors[className];
            }

            // 使用 HSV 色彩空間生成新顏色
            double hue = _random.NextDouble() * 360;  // 色相 0-360
            double saturation = 0.7 + (_random.NextDouble() * 0.3);  // 飽和度 70-100%
            double value = 0.8 + (_random.NextDouble() * 0.2);  // 亮度 80-100%

            // 轉換 HSV 到 RGB
            double c = value * saturation;
            double x = c * (1 - Math.Abs(((hue / 60) % 2) - 1));
            double m = value - c;

            double r, g, b;
            if (hue < 60)
            {
                r = c; g = x; b = 0;
            }
            else if (hue < 120)
            {
                r = x; g = c; b = 0;
            }
            else if (hue < 180)
            {
                r = 0; g = c; b = x;
            }
            else if (hue < 240)
            {
                r = 0; g = x; b = c;
            }
            else if (hue < 300)
            {
                r = x; g = 0; b = c;
            }
            else
            {
                r = c; g = 0; b = x;
            }

            Scalar color = new Scalar(
                (r + m) * 255,
                (g + m) * 255,
                (b + m) * 255
            );

            _classColors[className] = color;
            return color;
        }

        private void SortByLabel()
        {
            try
            {
                var items = _detectionResults.ToList();
                var direction = IsLabelSortAscending ? 1 : -1;
                items.Sort((x, y) => direction * string.Compare(x.Label, y.Label, StringComparison.Ordinal));
                
                _detectionResults.Clear();
                foreach (var item in items)
                {
                    _detectionResults.Add(item);
                }
                
                IsLabelSortAscending = !IsLabelSortAscending;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"排序錯誤: {ex.Message}");
            }
        }

        private void SortByConfidence()
        {
            try
            {
                var items = _detectionResults.ToList();
                var direction = IsConfidenceSortAscending ? 1 : -1;
                items.Sort((x, y) => direction * x.Confidence.CompareTo(y.Confidence));
                
                _detectionResults.Clear();
                foreach (var item in items)
                {
                    _detectionResults.Add(item);
                }
                
                IsConfidenceSortAscending = !IsConfidenceSortAscending;
            }
            catch (Exception ex)
            {
                Debug.WriteLine($"排序錯誤: {ex.Message}");
            }
        }

        private async void BtnSaveResult_Click(object sender, RoutedEventArgs e)
        {
            if (_currentImagePath == null || _detectionResults.Count == 0)
            {
                MessageBox.Show("請先進行物件偵測", "提示", MessageBoxButton.OK, MessageBoxImage.Information);
                return;
            }

            try
            {
                var saveFileDialog = new SaveFileDialog
                {
                    Filter = "PNG 圖片|*.png|所有檔案|*.*",
                    Title = "儲存偵測結果",
                    DefaultExt = "png"
                };

                if (saveFileDialog.ShowDialog() == true)
                {
                    btnSaveResult.IsEnabled = false;
                    Mouse.OverrideCursor = Cursors.Wait;

                    // 保存圖片
                    var imagePath = saveFileDialog.FileName;
                    var csvPath = Path.ChangeExtension(imagePath, "csv");
                    
                    // 保存當前顯示的圖片
                    var imageSource = imgMain.Source as BitmapSource;
                    if (imageSource != null)
                    {
                        // 在 UI 執行緒中創建一個凍結的 BitmapSource
                        BitmapSource frozenBitmap = null;
                        
                        // 如果圖片還沒有被凍結，創建一個凍結的副本
                        if (!imageSource.IsFrozen && imageSource.CanFreeze)
                        {
                            frozenBitmap = imageSource.Clone();
                            frozenBitmap.Freeze();
                        }
                        else
                        {
                            frozenBitmap = imageSource;
                        }
                        
                        await Task.Run(() =>
                        {
                            using var fileStream = new FileStream(imagePath, FileMode.Create);
                            var encoder = new PngBitmapEncoder();
                            encoder.Frames.Add(BitmapFrame.Create(frozenBitmap));
                            encoder.Save(fileStream);
                        });
                    }

                    // 保存偵測結果到 CSV
                    await Task.Run(() =>
                    {
                        using var writer = new StreamWriter(csvPath, false, Encoding.UTF8);
                        // 寫入標題
                        writer.WriteLine("物件,信心度,位置X,位置Y,寬度,高度");
                        
                        // 寫入偵測結果
                        foreach (var result in _detectionResults)
                        {
                            writer.WriteLine($"{result.Label},{result.Confidence:F4},{result.BoundingBox.X:F1},{result.BoundingBox.Y:F1},{result.BoundingBox.Width:F1},{result.BoundingBox.Height:F1}");
                        }
                    });

                    // 寫入統計資訊
                    var statsPath = Path.ChangeExtension(imagePath, "stats.csv");
                    await Task.Run(() =>
                    {
                        using var statsWriter = new StreamWriter(statsPath, false, Encoding.UTF8);
                        statsWriter.WriteLine("物件類別,數量");
                        foreach (var stat in _objectStats)
                        {
                            statsWriter.WriteLine($"{stat.ClassName},{stat.Count}");
                        }
                    });

                    MessageBox.Show($"已儲存:\n圖片: {Path.GetFileName(imagePath)}\n偵測結果: {Path.GetFileName(csvPath)}\n統計資料: {Path.GetFileName(statsPath)}", 
                        "儲存成功", MessageBoxButton.OK, MessageBoxImage.Information);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"儲存失敗: {ex.Message}", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
            }
            finally
            {
                btnSaveResult.IsEnabled = true;
                Mouse.OverrideCursor = null;
            }
        }

        private async void BtnBatchProcess_Click(object sender, RoutedEventArgs e)
        {
            if (_predictor == null)
            {
                MessageBox.Show("YOLO 模型尚未初始化", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            try
            {
                var dialog = new OpenFileDialog
                {
                    Multiselect = true,
                    Filter = "圖片檔案|*.jpg;*.jpeg;*.png;*.bmp|所有檔案|*.*"
                };

                if (dialog.ShowDialog() != true || dialog.FileNames.Length == 0)
                    return;

                var firstFile = dialog.FileNames[0];
                var directoryPath = Path.GetDirectoryName(firstFile);
                if (string.IsNullOrEmpty(directoryPath))
                {
                    MessageBox.Show("無法取得檔案路徑", "錯誤", MessageBoxButton.OK, MessageBoxImage.Error);
                    return;
                }

                // 建立輸出資料夾
                var outputDir = Path.Combine(directoryPath, "批次處理結果_" + DateTime.Now.ToString("yyyyMMdd_HHmmss"));
                Directory.CreateDirectory(outputDir);

                // 顯示進度條
                progressBar.Visibility = Visibility.Visible;
                progressText.Visibility = Visibility.Visible;
                progressBar.Maximum = dialog.FileNames.Length;
                progressBar.Value = 0;

                // 禁用按鈕
                btnSelectImage.IsEnabled = false;
                btnBatchProcess.IsEnabled = false;
                btnStartDetection.IsEnabled = false;
                btnSaveResult.IsEnabled = false;

                // 建立彙總統計CSV
                var summaryPath = Path.Combine(outputDir, "彙總統計.csv");
                using (var writer = new StreamWriter(summaryPath, false, Encoding.UTF8))
                {
                    writer.WriteLine("檔案名稱,物件總數,偵測物件類別數,類別統計");
                }

                // 用於追蹤所有出現過的類別
                var allClassesSet = new HashSet<string>();

                try
                {
                    foreach (var (file, index) in dialog.FileNames.Select((f, i) => (f, i)))
                    {
                        // 更新進度
                        progressBar.Value = index + 1;
                        progressText.Text = $"處理中... ({index + 1}/{dialog.FileNames.Length}) {Path.GetFileName(file)}";
                        await Task.Delay(1); // 讓UI更新

                        try
                        {
                            // 載入並處理圖片
                            var image = await Task.Run(() => Cv2.ImRead(file));
                            if (image.Empty())
                            {
                                MessageBox.Show($"無法載入圖片：{Path.GetFileName(file)}", "警告", MessageBoxButton.OK, MessageBoxImage.Warning);
                                continue;
                            }

                            // 執行偵測
                            var results = await Task.Run(() => _predictor.Detect(file));
                            if (results == null)
                            {
                                MessageBox.Show($"偵測失敗：{Path.GetFileName(file)}", "警告", MessageBoxButton.OK, MessageBoxImage.Warning);
                                continue;
                            }

                            // 更新檢測結果
                            _detectionResults.Clear();
                            
                            // 用於統計每個類別的數量
                            var classCount = new Dictionary<string, int>();

                            foreach (var detection in results)
                            {
                                var bbox = detection.Bounds;
                                var rect = new OpenCvSharp.Rect(
                                    (int)bbox.X,
                                    (int)bbox.Y,
                                    (int)bbox.Width,
                                    (int)bbox.Height
                                );

                                // 清理物件名稱
                                var objectName = detection.Name.ToString().Split(':').Last().Trim('"');
                                
                                // 更新類別統計
                                if (!classCount.ContainsKey(objectName))
                                    classCount[objectName] = 0;
                                classCount[objectName]++;
                                
                                // 添加到全域類別集合
                                allClassesSet.Add(objectName);

                                _detectionResults.Add(new DetectionResult(
                                    objectName,
                                    detection.Confidence,
                                    new System.Windows.Rect(rect.X, rect.Y, rect.Width, rect.Height)));
                            }

                            // 在圖片上繪製結果
                            foreach (var result in _detectionResults)
                            {
                                var color = GetColorForLabel(result.Label);
                                var rect = new OpenCvSharp.Rect(
                                    (int)result.BoundingBox.X,
                                    (int)result.BoundingBox.Y,
                                    (int)result.BoundingBox.Width,
                                    (int)result.BoundingBox.Height
                                );

                                image.Rectangle(rect, color, 2);
                                image.PutText($"{result.Label} ({result.Confidence:P1})",
                                    new OpenCvSharp.Point(rect.X, rect.Y - 5),
                                    HersheyFonts.HersheySimplex, 0.8, color, 2);
                            }

                            // 儲存結果
                            var baseName = Path.GetFileNameWithoutExtension(file);
                            
                            // 儲存標註後的圖片
                            var imagePath = Path.Combine(outputDir, baseName + "_標註.png");
                            await Task.Run(() => image.ImWrite(imagePath));

                            // 儲存偵測結果
                            var csvPath = Path.Combine(outputDir, baseName + "_偵測結果.csv");
                            await Task.Run(() =>
                            {
                                using var writer = new StreamWriter(csvPath, false, Encoding.UTF8);
                                writer.WriteLine("物件類別,信心度,X,Y,寬度,高度");
                                foreach (var result in _detectionResults)
                                {
                                    writer.WriteLine($"{result.Label},{result.Confidence:F3},{result.BoundingBox.X:F0},{result.BoundingBox.Y:F0},{result.BoundingBox.Width:F0},{result.BoundingBox.Height:F0}");
                                }
                            });

                            // 更新彙總統計
                            using (var writer = new StreamWriter(summaryPath, true, Encoding.UTF8))
                            {
                                // 建立類別統計字串
                                var classStats = string.Join("; ", classCount.Select(kvp => $"{kvp.Key}:{kvp.Value}"));
                                
                                writer.WriteLine($"{Path.GetFileName(file)},{_detectionResults.Count},{classCount.Count},\"{classStats}\"");
                            }

                            // 釋放圖片資源
                            image.Dispose();
                        }
                        catch (Exception ex)
                        {
                            MessageBox.Show($"處理檔案 {Path.GetFileName(file)} 時發生錯誤: {ex.Message}", "錯誤",
                                MessageBoxButton.OK, MessageBoxImage.Warning);
                        }
                    }

                    MessageBox.Show($"批次處理完成！\n結果已儲存至：{outputDir}", "完成",
                        MessageBoxButton.OK, MessageBoxImage.Information);
                }
                finally
                {
                    // 恢復UI狀態
                    progressBar.Visibility = Visibility.Collapsed;
                    progressText.Visibility = Visibility.Collapsed;
                    btnSelectImage.IsEnabled = true;
                    btnBatchProcess.IsEnabled = true;
                    btnStartDetection.IsEnabled = true;
                    btnSaveResult.IsEnabled = true;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"批次處理時發生錯誤: {ex.Message}", "錯誤",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private Scalar GetColorForLabel(string label)
        {
            if (!_classColors.TryGetValue(label, out var color))
            {
                // 使用標籤的雜湊值來生成 HSV 顏色
                var hash = label.GetHashCode();
                var hue = Math.Abs(hash % 360); // 0-360
                var saturation = 0.7 + (Math.Abs((hash >> 8) % 30) / 100.0); // 0.7-1.0
                var value = 0.8 + (Math.Abs((hash >> 16) % 20) / 100.0); // 0.8-1.0

                // 轉換 HSV 到 RGB
                var rgb = HsvToRgb(hue, saturation, value);
                color = new Scalar(rgb.B, rgb.G, rgb.R); // OpenCV 使用 BGR 格式
                _classColors[label] = color;
            }
            return color;
        }

        private (byte R, byte G, byte B) HsvToRgb(double h, double s, double v)
        {
            var c = v * s;
            var x = c * (1 - Math.Abs(((h / 60) % 2) - 1));
            var m = v - c;

            double r = 0, g = 0, b = 0;
            if (h < 60) { r = c; g = x; b = 0; }
            else if (h < 120) { r = x; g = c; b = 0; }
            else if (h < 180) { r = 0; g = c; b = x; }
            else if (h < 240) { r = 0; g = x; b = c; }
            else if (h < 300) { r = x; g = 0; b = c; }
            else { r = c; g = 0; b = x; }

            return (
                R: (byte)((r + m) * 255),
                G: (byte)((g + m) * 255),
                B: (byte)((b + m) * 255)
            );
        }

        protected override void OnClosed(EventArgs e)
        {
            base.OnClosed(e);
            _predictor?.Dispose();
        }

        private class RelayCommand : ICommand
        {
            private readonly Action _execute;
            
            public RelayCommand(Action execute)
            {
                _execute = execute ?? throw new ArgumentNullException(nameof(execute));
            }

            public event EventHandler? CanExecuteChanged
            {
                add { CommandManager.RequerySuggested += value; }
                remove { CommandManager.RequerySuggested -= value; }
            }

            public bool CanExecute(object? parameter) => true;

            public void Execute(object? parameter)
            {
                _execute();
            }
        }
    }
}