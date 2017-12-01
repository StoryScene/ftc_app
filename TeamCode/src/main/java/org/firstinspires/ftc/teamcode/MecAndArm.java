package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Kit Caldwell on 11/14/2017.
 */

public class MecAndArm extends OpMode {

    DcMotor lf, rf, lb, rb;
    DcMotor motorOne, motorTwo, motorTres;
    Servo servo;

    @Override
    public void init() {
        motorOne = hardwareMap.dcMotor.get("one");
        motorTwo = hardwareMap.dcMotor.get("two");
        motorTres = hardwareMap.dcMotor.get("three");
        servo = hardwareMap.servo.get("servo");

        mechanumInit();

    }
    //push

    @Override
    public void loop() {
        mechanumLoop();
        telemetry.addData("Power of lf, rf, lb, rb:\n", lf.getPower() + "\n" + rf.getPower()
                + "\n" + lb.getPower() + "\n" + rb.getPower());

        motorOne.setPower(gamepad2.left_stick_y);

        motorTwo.setPower(gamepad2.right_stick_y);

        if (gamepad2.a){
            motorTres.setPower(.3);
        }

        else if (gamepad2.b){
            motorTres.setPower(-.25);
        }
        else{
            motorTres.setPower(0);
        }
        if (gamepad2.x){
            servo.setPosition(1);
        }
        if (gamepad2.y){
            servo.setPosition(0);
        }


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

