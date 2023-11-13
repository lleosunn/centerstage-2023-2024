package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.File;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class odometry implements Runnable {
    //Odometry wheels
    private DcMotor encoderLeft, encoderRight, encoderAux;

    BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation lastAngles = new Orientation();
    double globalAngle;
    public void imuinit() {
        parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    //Thead run condition
    private boolean isRunning = true;

    //Position variables used for storage and calculations
    double currentRightPos = 0, currentLeftPos = 0, currentAuxPos = 0, currentIMU = 0,  orientationChange = 0;
    private double robotGlobalXCoordinatePosition = 0, robotGlobalYCoordinatePosition = 0, robotOrientationRadians = 0;
    private double oldRightPos = 0, oldLeftPos = 0, oldAuxPos = 0, oldIMU = 0;

    //Algorithm constants
    private double robotEncoderWheelDistance;
    private double horizontalEncoderTickPerDegreeOffset;

    //Sleep time interval (milliseconds) for the position update thread
    private int sleepTime;

    public odometry(DcMotor encoderLeft, DcMotor encoderRight, DcMotor encoderAux, int threadSleepDelay, BNO055IMU imu){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderAux = encoderAux;
        this.imu = imu;
        sleepTime = threadSleepDelay;
    }

    public void globalCoordinatePositionUpdate(){
        oldLeftPos = currentLeftPos;
        oldRightPos = currentRightPos;
        oldAuxPos = currentAuxPos;
        oldIMU = currentIMU;

        currentLeftPos = encoderLeft.getCurrentPosition();
        currentRightPos = -encoderRight.getCurrentPosition();
        currentAuxPos = -encoderAux.getCurrentPosition();
        currentIMU = getAngle();

        double leftChange = currentLeftPos - oldLeftPos;
        double rightChange = currentRightPos - oldRightPos;
        double auxChange = currentAuxPos - oldAuxPos;
        double IMUChange = currentIMU - oldIMU;
//
//        orientationChange = (rightChange - leftChange) / 27703.414;
//        robotOrientationRadians = ((robotOrientationRadians + orientationChange)); //using odometry
        orientationChange = Math.toRadians(IMUChange);
        robotOrientationRadians = Math.toRadians(currentIMU); //using imu

        double horizontalChange = auxChange - (orientationChange * 1860); //168.4 10695 10000

        double p = ((rightChange + leftChange) / 2);
        double n = horizontalChange;

        robotGlobalXCoordinatePosition = robotGlobalXCoordinatePosition + (n*Math.cos(robotOrientationRadians) - p*Math.sin(robotOrientationRadians));
        robotGlobalYCoordinatePosition = robotGlobalYCoordinatePosition + (n*Math.sin(robotOrientationRadians) + p*Math.cos(robotOrientationRadians));
    }

    public double x(){ return robotGlobalXCoordinatePosition; }

    public double y(){ return robotGlobalYCoordinatePosition; }

    public double h(){ return Math.toDegrees(robotOrientationRadians); }

    public void stop(){ isRunning = false; }

    @Override
    public void run() {
        while(isRunning) {
            globalCoordinatePositionUpdate();
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}