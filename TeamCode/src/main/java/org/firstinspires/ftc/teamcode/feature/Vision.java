package org.firstinspires.ftc.teamcode.feature;


import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Config
class VisionConstants{
    public static final Point EATER_POS = new Point(0,0);

    public static int lowY = 75;
    public static int highY = 105;
    // Red
    public static int lowR = 105;
    public static int highR = 120;
    // Blue
    public static int lowB1 = 0;
    public static int highB1 = 30;
    public static int lowB2 = 160;
    public static int highB2 = 179;
}

public class Vision {

    private OpenCvCamera webcam;
    private boolean isTurnedOn;
    private SampleDetectionPipeline pipeline;

    public Vision(OpenCvCamera webcam){
        this.webcam = webcam;
    }

    public void turnOn() {
        this.pipeline = new SampleDetectionPipeline();

        this.webcam.setPipeline(pipeline);
        this.webcam.openCameraDevice();
        this.webcam.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
        isTurnedOn = true;
    }

    public void turnOff(){
        isTurnedOn = false;
        this.webcam.stopStreaming();
        this.webcam.closeCameraDevice();
        this.webcam.stopRecordingPipeline();
    }

    // detect the closest sample from the eater
    public Sample detectTarget(SampleColor color){
        if(!isTurnedOn) return null;

        this.pipeline.setTargetColor(color);
        return this.pipeline.getTargetSample();
    }


    public static class Sample {
        public SampleColor color;
        public Point center;
        public double angle;

        public Sample(SampleColor color, Point center, double angle){
            this.color = color;
            this.center = center;
            this.angle = angle;
        }
    }

    public enum SampleColor {YELLOW, BLUE, RED};

    public static class SampleDetectionPipeline extends OpenCvPipeline {

        public ArrayList<Sample> rects = new ArrayList<>();
        public Sample targetSample;
        public SampleColor targetColor;

        // Main loop
        public Mat processFrame(Mat input) {

            // Preprocess the frame to detect regions
            ArrayList<Mat> Masks = preprocessFrame(input);
            Mat hierarchy = new Mat();
            rects.clear();

            Mat Mask = Masks.get(0);

            // Find mask of target color
            switch (targetColor){
                case YELLOW: Mask = Masks.get(0);
                case BLUE: Mask = Masks.get(1);
                case RED: Mask = Masks.get(2);
            }

            // Find contours of the detected mask
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(Mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find rectangles that contours fit in
            Pair<List<Sample>, List<RotatedRect>> results = findRotatedRect(input, contours, targetColor);

            rects.addAll(results.first);

            // FOR DEBUG
            // Draw rectangles on Mask
            drawRotatedRect(input, results.second);

            targetSample = findClosestSample(rects, targetColor);

            return input;
        }

        // Set target color
        public void setTargetColor(SampleColor color){
            this.targetColor = color;
        }

        // Get target sample
        public Sample getTargetSample(){
            return this.targetSample;
        }

        /// help functions
        // Set color masks
        private ArrayList<Mat> preprocessFrame(Mat frame) {
            ArrayList<Mat> result = new ArrayList<Mat>();
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Mat yellowMask = new Mat();
            Mat blueMask = new Mat();
            Mat redMask = new Mat();

            // Yellow
            Core.inRange(hsvFrame, new Scalar(VisionConstants.lowY, 25, 25), new Scalar(VisionConstants.highY, 255, 255), yellowMask);
            Mat kernel1 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel1);

            result.add(yellowMask);


            // Blue
            Mat blueMask1 = new Mat();
            Mat blueMask2 = new Mat();
            Core.inRange(hsvFrame, new Scalar(VisionConstants.lowB1, 25, 25), new Scalar(VisionConstants.highB1, 255, 255), blueMask1);
            Core.inRange(hsvFrame, new Scalar(VisionConstants.lowB2, 25, 25), new Scalar(VisionConstants.highB2, 255, 255), blueMask2);

            Core.bitwise_or(blueMask1,blueMask2,blueMask);

            Mat kernel2 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, kernel2);

            result.add(blueMask);


            // Red
            Core.inRange(hsvFrame, new Scalar(VisionConstants.lowR, 25, 25), new Scalar(VisionConstants.highR, 25, 25), redMask);
            Mat kernel3 = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel3);

            result.add(redMask);

            return result;
        }

        // Find minimum rectangle on samples
        // Return List of Samples and their minRects
        private Pair<List<Sample>, List<RotatedRect>> findRotatedRect(Mat input, List<MatOfPoint> contours, SampleColor color_name){
            List<Sample> samples = new ArrayList<>();
            List<RotatedRect> rects = new ArrayList<>();

            int AREA_THRESHOLD = 20000;

            for (MatOfPoint contour : contours){
                double area = Imgproc.contourArea(contour);
                if (area > AREA_THRESHOLD) {
                    MatOfPoint2f cnt = new MatOfPoint2f(contour.toArray());
                    RotatedRect rect = Imgproc.minAreaRect(cnt);
                    samples.add(new Sample(color_name, rect.center, rect.angle));
                }
            }
            return new Pair<List<Sample>, List<RotatedRect>>(samples, rects);
        }

        // Draw rectangles of samples
        private void drawRotatedRect(Mat input, List<RotatedRect> rects){

            for(RotatedRect rect : rects) {
                // draw rect
                Point[] box = new Point[4];
                rect.points(box);

                List<Point> boxList = new ArrayList<>();
                for (Point p : box) {
                    boxList.add(new Point(Math.round(p.x), Math.round(p.y)));
                }
                MatOfPoint boxMat = new MatOfPoint();
                boxMat.fromList(boxList);

                Imgproc.drawContours(input, Arrays.asList(boxMat), 0, new Scalar(0,255,0), 2);

                Imgproc.putText(
                        input,
                        "angle : " + rect.angle,
                        new Point(rect.center.x - 50, rect.center.y - 15),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0,255,0),
                        2
                );
            }
        }

        // Find the closest sample of the color from the servo
        private Sample findClosestSample(List<Sample> samples, SampleColor color){
            Point ORIGIN = new Point(100,100);

            int MIN_distance = 1000000;
            Sample target_sample = null;

            int distance = 0;

            for(Sample sample : samples){
                if(sample == null){
                    continue;
                }
                if(sample.color != color){
                    continue;
                }

                distance = dist(sample.center);
                if(distance < MIN_distance){
                    target_sample = sample;
                    MIN_distance = distance;
                }
            }

            return target_sample;
        }

        private int dist(Point p1){
            return (int) Math.sqrt( (p1.x - VisionConstants.EATER_POS.x)*(p1.x - VisionConstants.EATER_POS.x) + (p1.y - VisionConstants.EATER_POS.y)*(p1.y - VisionConstants.EATER_POS.y) );
        }

    }

}
