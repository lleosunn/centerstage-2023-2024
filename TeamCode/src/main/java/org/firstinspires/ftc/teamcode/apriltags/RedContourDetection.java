package org.firstinspires.ftc.teamcode.apriltags;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;  // Add this line
import org.opencv.imgproc.Imgproc;


@TeleOp
public class RedContourDetection extends LinearOpMode {
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        RedContourPipeline pipeline = new RedContourPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                // Handle camera open error
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            if (pipeline.getDetected()) {
                telemetry.addData("Contour Position", "x: %d, y: %d", pipeline.getRectX(), pipeline.getRectY());
                telemetry.addData("Contour Size", "width: %d, height: %d", pipeline.getRectWidth(), pipeline.getRectHeight());
            } else {
                telemetry.addData("Status", "No Contour Detected");
            }
            telemetry.update();
            sleep(20);
        }
    }

    static class RedContourPipeline extends OpenCvPipeline {
        Scalar lowerRed1 = new Scalar(0, 128, 51);
        Scalar upperRed1 = new Scalar(5, 255, 255);
        Scalar lowerRed2 = new Scalar(160, 128, 51);
        Scalar upperRed2 = new Scalar(179, 255, 255);

        Mat mask = new Mat();
        Mat hierarchy = new Mat();

        boolean detected = false;
        int rectX, rectY, rectWidth, rectHeight;

        @Override
        public Mat processFrame(Mat input) {
            detected = false;
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Apply Gaussian Blur
            Imgproc.GaussianBlur(hsv, hsv, new Size(5, 5), 0);

            // Thresholding
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, lowerRed1, upperRed1, mask1);
            Core.inRange(hsv, lowerRed2, upperRed2, mask2);

            Core.addWeighted(mask1, 1.0, mask2, 1.0, 0.0, mask);
            mask1.release();
            mask2.release();

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                Rect boundingRect = Imgproc.boundingRect(contour);
                Imgproc.rectangle(input, boundingRect.tl(), boundingRect.br(), new Scalar(50, 205, 50), 4);

                rectX = boundingRect.x;
                rectY = boundingRect.y;
                rectWidth = boundingRect.width;
                rectHeight = boundingRect.height;
                detected = true;

                // Break after drawing the first contour for simplicity
                break;
            }

            return input;
        }

        boolean getDetected() {
            return detected;
        }

        int getRectX() {
            return rectX;
        }

        int getRectY() {
            return rectY;
        }

        int getRectWidth() {
            return rectWidth;
        }

        int getRectHeight() {
            return rectHeight;
        }
    }
}
