package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;


@TeleOp(name="WOODduo", group="Linear Opmode")

public class duo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;

    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;

    Servo claw;
    Servo wrist;
    Servo guider;

    private DistanceSensor clawDistance;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        lift1 = hardwareMap.get(DcMotor.class, "lift1");
        lift2 = hardwareMap.get(DcMotor.class, "lift2");
        arm = hardwareMap.get(DcMotor.class, "arm");

        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        guider = hardwareMap.get(Servo.class, "guider");

        clawDistance = hardwareMap.get(DistanceSensor.class, "clawDistance");

        //robot hardware
        RobotHardware robot = new RobotHardware(fl, fr, bl, br, lift1, lift2, arm, claw, wrist, guider);
        robot.innitHardwareMap();

        waitForStart();
        runtime.reset();
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        arm.setTargetPosition(0);

        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift1.setPower(0);
        lift2.setPower(0);
        arm.setPower(0);

        double distance = 0;
        double modifier = 0.5;
        double turnmod = 0.7;

        while (opModeIsActive()) {
            distance = clawDistance.getDistance(DistanceUnit.MM);

            telemetry.addData("lift1", lift1.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("distance", distance);
            telemetry.update();

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            fl.setPower(modifier * (y + x + turnmod*turn));
            fr.setPower(modifier * (y - x - turnmod*turn));
            bl.setPower(modifier * (y - x + turnmod*turn));
            br.setPower(modifier * (y + x - turnmod*turn));

            if (gamepad1.right_trigger > 0) {
                modifier = 0.3;
                turnmod = 1;
            } else if (gamepad1.left_trigger > 0) {
                modifier = 1;
                turnmod = 1;
            } else {
                modifier = 0.5;
                turnmod = 0.7;
            }

            if (gamepad2.right_bumper) {
                if (distance < 30) {
                    robot.clawClose();
                }
            }

            if (gamepad2.left_stick_button) {
                lift1.setTargetPosition(0);
                lift2.setTargetPosition(0);
                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift1.setPower(0);
                lift2.setPower(0);
            }
            if (gamepad2.dpad_up) {
                robot.setLift(880, 1);
            }
            if (gamepad2.dpad_right) {
                robot.setLift(300, 1);
            }
            if (gamepad2.dpad_left) {
                robot.setArm(720, 0.8);
            }
            if (gamepad2.dpad_down) {
                robot.setLift(0, 0.5);
            }
            if (gamepad2.right_stick_button) {
                arm.setTargetPosition(0);
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
            }
            if (gamepad2.a) {
                robot.setArm(600, 0.8);
            }
            if (gamepad2.b) {
                robot.setArm(0, 0.5);
                robot.setLift(0, 0.5);
                robot.wristReset();
                robot.clawClose();
            }
            if (gamepad2.left_stick_y > 0.5) {
                lift1.setTargetPosition(lift1.getCurrentPosition() - 50);
                lift2.setTargetPosition(lift2.getCurrentPosition() - 50);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad2.left_stick_y < -0.5) {
                lift1.setTargetPosition(lift1.getCurrentPosition() + 100);
                lift2.setTargetPosition(lift2.getCurrentPosition() + 100);
                lift1.setPower(1);
                lift2.setPower(1);
            }
            if (gamepad2.right_stick_x < -0.5) {
                arm.setTargetPosition(arm.getCurrentPosition() - 50);
                arm.setPower(1);
            }
            if (gamepad2.right_stick_x > 0.5) {
                arm.setTargetPosition(arm.getCurrentPosition() + 50);
                arm.setPower(1);
            }
            if (gamepad2.right_trigger > 0.5) {
                robot.clawClose();
            }
            if (gamepad2.left_trigger > 0.5 || gamepad1.y) {
                robot.clawOpen();
            }
            if (arm.getCurrentPosition() > 710) {
                robot.guiderFlat();
            } else robot.guiderBack();
            if (gamepad2.x) {
                robot.wristTurn();
            }
            if (gamepad2.y) {
                robot.wristReset();
            }

        }
    }
}

