package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Kit Caldwell on 11/14/2017.
 */
@TeleOp
public class FourServosOrTwoServos extends OpMode{

    Servo one, two;
    CRServo three, four;
    DcMotor lf, rf, lb, rb;
    DcMotor lift;
    public GamepadV2 pad1 = new GamepadV2();


    @Override
    public void init() {

        one = hardwareMap.servo.get("one");
        two = hardwareMap.servo.get("two");
        three = hardwareMap.crservo.get("three");
        four = hardwareMap.crservo.get("four");

        lift = hardwareMap.dcMotor.get("lift");

        mechanumInit();

    }
//push
    @Override
    public void loop() {

        if (gamepad2.a) {
            one.setPosition(.5);
            two.setPosition(-.5);
        }
        if (gamepad2.b) {
            one.setPosition(-.5);
            two.setPosition(.5);
        }
        if (gamepad2.x) {
            three.setPower(.5);
            four.setPower(-.5);
        }
        else if (gamepad2.y){
            three.setPower(-.5);
            four.setPower(.5);
        }
        else{
            three.setPower(0);
            four.setPower(0);
        }
        if (gamepad2.dpad_down){
            lift.setPower(-.5);
        }
        else if (gamepad2.dpad_up){
            lift.setPower(.5);
        }
        else{
            lift.setPower(0);
        }

        telemetry.addData("position 1", one.getPosition());
        telemetry.addData("position 2", two.getPosition());

        mechanumLoop();
        telemetry.addData("Power of lf, rf, lb, rb:\n", lf.getPower() + "\n" + rf.getPower()
                + "\n" + lb.getPower() + "\n" + rb.getPower());
    }

    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
    }

    private void mechanumInit() {
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    private void mechanumLoop() {
        pad1.update(gamepad1);
        double x = Range.clip(gamepad1.left_stick_x, -1, 1);
        double y = - Range.clip(gamepad1.left_stick_y, -1, 1);

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        double rot = Range.clip(gamepad1.right_stick_x, -1, 1);

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
}
