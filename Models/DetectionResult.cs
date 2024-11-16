using System.Windows;

namespace ObjectDetectionApp.Models
{
    public class DetectionResult
    {
        public string Label { get; }
        public float Confidence { get; }
        public Rect BoundingBox { get; }

        public DetectionResult(string label, float confidence, Rect boundingBox)
        {
            Label = label;
            Confidence = confidence;
            BoundingBox = boundingBox;
        }

        public override string ToString()
        {
            return $"{Label} ({Confidence:P1})";
        }
    }
}
