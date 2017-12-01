package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kit Caldwell on 11/14/2017.
 */
@TeleOp
public class FourServosOrTwoServos extends OpMode{

    Servo one, two, three, four;
    DcMotor lf, rf, lb, rb;
    DcMotor lift;

    @Override
    public void init() {

        one = hardwareMap.servo.get("one");
        two = hardwareMap.servo.get("two");
        three = hardwareMap.servo.get("three");
        four = hardwareMap.servo.get("four");

        lift = hardwareMap.dcMotor.get("lift");

        mechanumInit();

    }
//push
    @Override
    public void loop() {

        if (gamepad1.a) {
            one.setPosition(1);
            two.setPosition(1);
        }
        if (gamepad1.b) {
            one.setPosition(0);
            two.setPosition(0);
        }
        if (gamepad1.x) {
            three.setPosition(1);
            four.setPosition(1);
        }
        if (gamepad1.y){
            three.setPosition(0);
            four.setPosition(0);
        }
        if (gamepad1.dpad_down){
            lift.setPower(.5);
        }
        if (gamepad1.dpad_up){
            lift.setPower(-.5);
        }

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
    }


    private void mechanumLoop() {
        double x = Range.clip(gamepad1.left_stick_x, -1, 1);
        double y = Range.clip(gamepad1.left_stick_y, -1, 1);
        double rot = Range.clip(gamepad1.right_stick_x, -1, 1);

        double r = Math.hypot(x, y);
        double angle = 0.0;


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

        lf.setPower(Range.clip(vlf, -1, 1));
        rf.setPower(Range.clip(vrf, -1, 1));
        lb.setPower(Range.clip(vlb, -1, 1));
        rb.setPower(Range.clip(vrb, -1, 1));

        telemetry.addData("maxPower: ", maxPower);
    }
}
