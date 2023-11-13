package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Solo", group="Linear Opmode")

public class Solo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor bl = null;
    private DcMotor br = null;
    private DcMotor lift1 = null;
    private DcMotor lift2 = null;
    private DcMotor arm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
//        lift1 = hardwareMap.get(DcMotor.class, "lift1");
//        lift2 = hardwareMap.get(DcMotor.class, "lift2");
//        arm = hardwareMap.get(DcMotor.class, "arm");
//
//        Servo lclaw = hardwareMap.get(Servo.class, "lclaw");
//        Servo rclaw = hardwareMap.get(Servo.class, "rclaw");

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
//        lift1.setDirection(DcMotor.Direction.REVERSE);
//        lift2.setDirection(DcMotorSimple.Direction.REVERSE);
//        arm.setDirection(DcMotor.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        runtime.reset();

        while (opModeIsActive()) {
//            telemetry.addData("lift1", lift1.getCurrentPosition());
//            telemetry.addData("lift2", lift2.getCurrentPosition());
//            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.update();

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double modifier = 1;

            fl.setPower(modifier*(y + x - turn));
            fr.setPower(modifier*(y - x + turn));
            bl.setPower(modifier*(y - x - turn));
            br.setPower(modifier*(y + x + turn));

//            if(gamepad1.y) {
//                lift1.setTargetPosition(0);
//                lift2.setTargetPosition(0);
//                lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift1.setPower(0.5);
//                lift2.setPower(0.5);
//            }
//            if(gamepad1.x) {
//                arm.setTargetPosition(0);
//                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                arm.setPower(0);
//            }
//            if (gamepad1.a) { //deposit position
//                arm.setTargetPosition(600);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.4);
//                lclaw.setPosition(0.5);
//                rclaw.setPosition(0.5);
//            }
//            if (gamepad1.b) { //intake position
//                arm.setTargetPosition(38);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(0.3);
//                lclaw.setPosition(0.5);
//                rclaw.setPosition(0.5);
//            }
//            if (gamepad1.right_trigger > 0){ //claw close
//                lclaw.setPosition(0.5);
//                rclaw.setPosition(0.5);            }
//            if (gamepad1.left_trigger > 0){ //claw open
//                lclaw.setPosition(0.25);
//                rclaw.setPosition(0.75);            }
//
//            if(gamepad1.dpad_up) {
//                lclaw.setPosition(0.5);
//                rclaw.setPosition(0.5);
//                lift1.setTargetPosition(900);
//                lift2.setTargetPosition(900);
//                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift1.setPower(1);
//                lift2.setPower(1);
//            }
//            if(gamepad1.dpad_down) {
//                lift1.setTargetPosition(0);
//                lift2.setTargetPosition(0);
//                lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                lift1.setPower(0.5);
//                lift2.setPower(0.5);
//            }
//
//            if (gamepad1.dpad_left){
//                arm.setTargetPosition(arm.getCurrentPosition()-25);
//                arm.setPower(0.5);
//            }
//            if (gamepad1.dpad_right){
//                arm.setTargetPosition(arm.getCurrentPosition()+25);
//                arm.setPower(0.5);
//            }
//            if(gamepad1.left_bumper){
//                lift1.setTargetPosition(lift1.getCurrentPosition()-50);
//                lift2.setTargetPosition(lift2.getCurrentPosition()-50);
//                lift1.setPower(0.5);
//                lift2.setPower(0.5);
//            }
//            if(gamepad1.right_bumper){
//                lift1.setTargetPosition(lift1.getCurrentPosition()+50);
//                lift2.setTargetPosition(lift2.getCurrentPosition()+50);
//                lift1.setPower(0.5);
//                lift2.setPower(0.5);
//            }




        }
    }
}
