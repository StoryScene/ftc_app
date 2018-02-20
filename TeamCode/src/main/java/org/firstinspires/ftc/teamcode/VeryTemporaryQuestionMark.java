package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Copy pasted from a file created by Kit Caldwell on 11/10/2017.
 * :)
 */
@TeleOp
public class VeryTemporaryQuestionMark extends OpMode {

    DcMotor lf, rf, lb, rb;
    DcMotor intake1;
    DcMotor intake2;
    //DcMotor hold1, hold2;
    DcMotor slide, score;
    //DcMotor relic;

    Servo arm;

    public GamepadV2 pad1 = new GamepadV2();
    //CRServo turn;
    //CRServo grab;

    @Override
    public void init() {

        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake1 = hardwareMap.dcMotor.get("in1");
        intake2 = hardwareMap.dcMotor.get("in2");
        //hold1 = hardwareMap.dcMotor.get("hold1");
        //hold2 = hardwareMap.dcMotor.get("hold2");

        slide = hardwareMap.dcMotor.get("slide");
        score = hardwareMap.dcMotor.get("score");
        //relic = hardwareMap.dcMotor.get("relic");

        arm = hardwareMap.servo.get("arm");
        //turn = hardwareMap.crservo.get("turn");
        //grab = hardwareMap.crservo.get("grab");

        intake2.setDirection(DcMotorSimple.Direction.REVERSE);
        //hold2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        mechanumLoop();

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
            setPowers(0,0.6,0);
        }
        else if (gamepad1.dpad_down){
            setPowers(0,-0.6,0);

        }
        else if (gamepad1.dpad_left) {
            setPowers(-0.6,0,0);
        }
        else if (gamepad1.dpad_left) {
            setPowers(0.6,0,0);
        }

        /*
        if (gamepad2.dpad_up){
            relic.setPower(0.6);
        }
        else if (gamepad2.dpad_down) {
            relic.setPower(-0.6);
        }
        else {
            relic.setPower(0);
        }
        */


        slide.setPower(Range.clip(gamepad2.left_stick_y, -1, 1));
        score.setPower(Range.clip(gamepad2.right_stick_y, -0.5, 0.5));

        if (gamepad2.b) {
            arm.setPosition(Range.clip(arm.getPosition() - 0.02,-1,1));
        }
        else if (gamepad2.y) {
            arm.setPosition(Range.clip(arm.getPosition() + 0.02,-1,1));
        }

        /*

        if (gamepad2.a){
            turn.setPower(.5);
        }
        else if (gamepad2.x){
            turn.setPower(-.5);
        }
        else{
            turn.setPower(0);
        }
        */



        /*
        if (gamepad2.left_bumper){
            grab.setPower(.5);
        }
        else if(gamepad2.right_bumper){
            grab.setPower(-.5);
        }
        else{
            grab.setPower(0);
        }



        telemetry.addData("Power of turn:", turn.getPower());
        telemetry.addData("Power of grab:", grab.getPower());
        telemetry.addData("gamepad2.a:", gamepad2.a);
        telemetry.addData("relic power:", relic.getPower());

        */


        telemetry.addData("left stick 1", gamepad1.left_stick_y);
        telemetry.addData("right stick 1", gamepad1.right_stick_y);
        telemetry.addData("right bumper 2",gamepad2.right_bumper);
        telemetry.addData("left bumper 2",gamepad2.left_bumper);
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

    private void mechanumLoop() {
        pad1.update(gamepad1);
        double x = Range.clip(gamepad1.left_stick_x, -1, 1);
        double y = - Range.clip(gamepad1.left_stick_y, -1, 1);

        x = discreteDrive(x);
        y = discreteDrive(y);

        double rot = discreteDrive(Range.clip(gamepad1.right_stick_x, -1, 1));

        double r = Math.hypot(x, y);
        double angle = 0.0;

        double POW = Math.max(Math.hypot(x, y), Math.abs(rot));


        if (r > 0.1){
            angle = Math.atan2(y,x) - Math.PI / 4;
        }

        telemetry.addData("angle: ", angle);
        telemetry.addData("radius: ", r);
        telemetry.addData("rotate: ", rot);


        double vlf = r * Math.cos(angle) + rot;
        double vrf = r * Math.sin(angle) - rot;
        double vlb = r * Math.sin(angle) + rot;
        double vrb = r * Math.cos(angle) - rot;

        double maxPower = maxPow(vlf, vrf, vlb, vrb);

        vlf /= maxPower;
        vrf /= maxPower;
        vlb /= maxPower;
        vrb /= maxPower;

        lf.setPower(Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(-Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(-Math.pow(POW,2) * Range.clip(vrb, -1, 1));


        telemetry.addData("maxPower: ", maxPower);
    }

    private void setPowers(double xx, double yy, double rotation) {
        double x = xx;
        double y = - yy;

        x = discreteDrive(x);
        y = discreteDrive(y);

        double rot = rotation;
        double r = Math.hypot(x, y);
        double angle = 0.0;

        double POW = Math.max(Math.hypot(x, y), Math.abs(rot));

        if (r > 0.1){
            angle = Math.atan2(y,x) - Math.PI / 4;
        }

        telemetry.addData("angle: ", angle);
        telemetry.addData("radius: ", r);
        telemetry.addData("rotate: ", rot);


        double vlf = r * Math.cos(angle) + rot;
        double vrf = r * Math.sin(angle) - rot;
        double vlb = r * Math.sin(angle) + rot;
        double vrb = r * Math.cos(angle) - rot;

        double maxPower = maxPow(vlf, vrf, vlb, vrb);

        vlf /= maxPower;
        vrf /= maxPower;
        vlb /= maxPower;
        vrb /= maxPower;

        lf.setPower(Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(-Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(-Math.pow(POW,2) * Range.clip(vrb, -1, 1));


        telemetry.addData("maxPower: ", maxPower);
    }


    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
    }

}
