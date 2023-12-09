package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import java.math.*;


@TeleOp(name="Duo", group="Linear Opmode")

public class Duo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor arm = null;

    Servo wrist;
    Servo leftclaw;
    Servo rightclaw;

    CRServo lefthang;

    CRServo righthang;

    IMU imu;

    private double globalAngle;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        arm = hardwareMap.get(DcMotor.class, "arm");

        wrist = hardwareMap.get(Servo.class, "wrist");
        leftclaw = hardwareMap.get(Servo.class, "leftclaw");
        rightclaw = hardwareMap.get(Servo.class, "rightclaw");

        lefthang = hardwareMap.get(CRServo.class, "lefthang");
        righthang = hardwareMap.get(CRServo.class, "righthang");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();
        double modifier = 1;
        imu.resetYaw();

        while (opModeIsActive()) {

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

            /*
            DRIVER 1 YEAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
             */

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.toRadians(orientation.getYaw(AngleUnit.DEGREES)) + (Math.PI/2);
//
//            fl.setPower(modifier*(y + x + turn));
//            fr.setPower(modifier*(y - x - turn));
//            bl.setPower(modifier*(y - x + turn));
//            br.setPower(modifier*(y + x - turn));

            if (gamepad1.right_trigger > 0){
                modifier = 0.3;
            } else modifier = 1;


            double flH = x * Math.sin(theta - (Math.PI/4));
            double frH = x * Math.cos(theta - (Math.PI/4));
            double blH = x * Math.cos(theta - (Math.PI/4));
            double brH = x * Math.sin(theta - (Math.PI/4));

            double flV = y * Math.sin(theta + (Math.PI/4));
            double frV = y * Math.cos(theta + (Math.PI/4));
            double blV = y * Math.cos(theta + (Math.PI/4));
            double brV = y * Math.sin(theta + (Math.PI/4));

            fl.setPower(flV - flH + turn);
            fr.setPower(frV - frH - turn);
            bl.setPower(blV - blH + turn);
            br.setPower(brV - brH - turn);

            if (gamepad1.right_stick_button) {
                imu.resetYaw();
            }


            /*
            DRIVER 2 YEAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH
             */

            if(gamepad2.x) { //reset arm
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
            }
            if (gamepad2.a) { //deposit position
                arm.setTargetPosition(1300);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);
                wrist.setPosition(0.45);
            }
            if (gamepad2.b) { //intake position
                arm.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.4);
                wrist.setPosition(0.9);
            }
            if (gamepad2.y) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(0.8);
            }
            if (gamepad2.left_trigger > 0){ //left claw close
                leftclaw.setPosition(0.4);
            }
            if (gamepad2.left_bumper){ //left claw open
                leftclaw.setPosition(0.55);
            }
            if (gamepad2.right_trigger > 0){ //right claw close
                rightclaw.setPosition(0.49);
            }
            if (gamepad2.right_bumper){ //right claw open
                rightclaw.setPosition(0.34);
            }
            if (gamepad2.dpad_left){
                lefthang.setPower(-1);
                righthang.setPower(1);
            }
            else if (gamepad2.dpad_right){
                lefthang.setPower(1);
                righthang.setPower(-1);
            }
            else {
                lefthang.setPower(0);
                righthang.setPower(0);
            }
            if (gamepad2.left_stick_y > 0.5) { //stick pointing down
                arm.setTargetPosition(arm.getCurrentPosition() - 35);
                arm.setPower(1);
            }
            if (gamepad2.left_stick_y < -0.5) {
                arm.setTargetPosition(arm.getCurrentPosition() + 35);
                arm.setPower(1);
            }
            if (gamepad2.right_stick_y > 0.5) { //stick pointing down
                wrist.setPosition(wrist.getPosition() + 0.0005);
            }
            if (gamepad2.right_stick_y < -0.5) {
                wrist.setPosition(wrist.getPosition() - 0.0005);
            }





        }
    }
}
