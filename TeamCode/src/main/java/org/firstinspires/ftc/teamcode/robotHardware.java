package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class robotHardware {

    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    DcMotor lift1;
    DcMotor lift2;
    DcMotor arm;

    Servo claw;
    Servo wrist;
    Servo guider;

    public robotHardware(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, DcMotor lift1, DcMotor lift2, DcMotor arm, Servo claw, Servo wrist, Servo guider) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
        this.lift1 = lift1;
        this.lift2 = lift2;
        this.arm = arm;
        this.claw = claw;
        this.wrist = wrist;
        this.guider = guider;
    }

    public void innitHardwareMap() {
        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift1.setDirection(DcMotor.Direction.FORWARD);
        lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        lift1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setLift(int height, double power){
        lift1.setTargetPosition(height);
        lift2.setTargetPosition(height);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift1.setPower(power);
        lift2.setPower(power);
    }
    public void setArm(int angle, double power) {
        arm.setTargetPosition(angle);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(power);
    }

    public void clawOpen() {
        claw.setPosition(0.4);
    }

    public void clawClose() {
        claw.setPosition(0.515);
    }

    public void wristTurn() {
        wrist.setPosition(0.79);
    }

    public void wristReset() {
        wrist.setPosition(0.13);
    }

    public void guiderBack() {
        guider.setPosition(0.4);
    }

    public void guiderSet() {
        guider.setPosition(0.68);
    }

    public void guiderFlat() {
        guider.setPosition(1);
    }


}
