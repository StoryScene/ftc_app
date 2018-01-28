package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Copy pasted from a file created by Kit Caldwell on 11/10/2017.
 * :)
 */
@TeleOp
public class VeryTemporaryQuestionMark extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor intake1;
    DcMotor intake2;
    //DcMotor hold1, hold2;
    DcMotor slide, score;
    DcMotor relic;

    Servo arm;

    CRServo turn;
    CRServo grab;

    @Override
    public void init() {

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        intake1 = hardwareMap.dcMotor.get("in1");
        intake2 = hardwareMap.dcMotor.get("in2");
        //hold1 = hardwareMap.dcMotor.get("hold1");
        //hold2 = hardwareMap.dcMotor.get("hold2");
        slide = hardwareMap.dcMotor.get("slide");
        score = hardwareMap.dcMotor.get("score");
        relic = hardwareMap.dcMotor.get("relic");

        arm = hardwareMap.servo.get("arm");
        turn = hardwareMap.crservo.get("turn");
        grab = hardwareMap.crservo.get("grab");


        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        //hold2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {


        leftDrive.setPower(discreteDrive(gamepad1.left_stick_y));
        rightDrive.setPower(discreteDrive(gamepad1.right_stick_y));


        if (gamepad1.left_bumper){
            intake1.setPower(-.5);
            intake2.setPower(-.5);
        }
        else if (gamepad1.right_bumper) {
            intake1.setPower(.5);
            intake2.setPower(.5);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        /*
        if (gamepad1.dpad_up) {
            hold1.setPower(0.3);
            hold2.setPower(0.3);
        }
        else if (gamepad1.dpad_down) {
            hold1.setPower(-0.3);
            hold2.setPower(-0.3);
        }
        else {
            hold1.setPower(0);
            hold2.setPower(0);
        }
        */

        if(gamepad1.dpad_up) {
            leftDrive.setPower(0.6);
            rightDrive.setPower(0.6);
        }
        else if (gamepad1.dpad_down){
            leftDrive.setPower(-0.6);
            rightDrive.setPower(-0.6);
        }
        else if (gamepad1.dpad_left) {
            leftDrive.setPower(-0.6);
            rightDrive.setPower(0.6);
        }

        relic.setPower(gamepad1.right_trigger);

        slide.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));
        score.setPower(Range.clip(gamepad2.right_stick_y, -0.5, 0.5));

        if (gamepad2.b) {
            arm.setPosition(Range.clip(arm.getPosition() - 0.02,-1,1));
        }
        else if (gamepad2.y) {
            arm.setPosition(Range.clip(arm.getPosition() + 0.02,-1,1));
        }

        if (gamepad2.a){
            turn.setPower(.5);
        }
        else if (gamepad2.x){
            turn.setPower(-.5);
        }
        else{
            turn.setPower(0);
        }

        if (gamepad2.left_bumper){
            grab.setPower(.5);
        }
        else if(gamepad2.right_bumper){
            grab.setPower(-.5);
        }
        else{
            grab.setPower(0);
        }




        telemetry.addData("left stick 1", gamepad1.left_stick_y);
        telemetry.addData("right stick 1", gamepad1.right_stick_y);
        telemetry.addData("right bumper",gamepad1.right_bumper);
        telemetry.addData("left bumper",gamepad1.left_bumper);
        telemetry.addData("right stick 2",gamepad2.right_stick_y);
        telemetry.addData("actual arm pos", arm.getPosition());

        //telemetry.addData("hold1: ", hold1.getPower());
        telemetry.addData("intake1: ", intake1.getPower());
        telemetry.update();
    }

    private double discreteDrive(double x){
        x = Range.clip(x, -1, 1);
        if (Math.abs(x) < 0.1){
            return 0;
        }
        if (Math.abs(x) < 0.4){
            return Math.signum(x) * 0.25;
        }
        if (Math.abs(x) < 0.9){
            return Math.signum(x) * 0.6;
        }
        return Math.signum(x) * 1;
    }

}
