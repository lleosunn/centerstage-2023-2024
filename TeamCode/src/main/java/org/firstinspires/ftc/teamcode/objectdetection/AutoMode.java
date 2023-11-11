package org.firstinspires.ftc.teamcode.objectdetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp
public class AutoMode extends LinearOpMode {
    OpenCvCamera camera;
    SkystoneDetector detector;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        int width = 720; // Adjust the width as per your camera's resolution
        detector = new SkystoneDetector(width);
        camera.setPipeline(detector);

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
            SkystoneDetector.SkystoneLocation location = detector.getLocation();
            if (location != SkystoneDetector.SkystoneLocation.NONE) {
                telemetry.addData("Skystone Location", location);
            } else {
                telemetry.addData("Status", "Skystone not detected");
            }
            telemetry.update();
            sleep(20);
        }
    }
}
