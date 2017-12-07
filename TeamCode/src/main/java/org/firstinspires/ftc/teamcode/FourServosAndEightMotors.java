package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Kit Caldwell on 12/06/2017.
 */
@TeleOp
public class FourServosAndEightMotors extends OpMode{

    Servo one, two;
    CRServo three, four;
    DcMotor lf, rf, lb, rb;
    DcMotor lift;
    DcMotor motorOne;
    DcMotor motorTwo;
    DcMotor motorTres;
    //Servo servo;
    GamepadV2 pad1 = new GamepadV2();




    @Override
    public void init() {

        one = hardwareMap.servo.get("one");
        two = hardwareMap.servo.get("two");
        three = hardwareMap.crservo.get("three");
        four = hardwareMap.crservo.get("four");

        lift = hardwareMap.dcMotor.get("lift");

        mechanumInit();

        motorOne = hardwareMap.dcMotor.get("one");
        motorTwo = hardwareMap.dcMotor.get("two");
        motorTres = hardwareMap.dcMotor.get("three");

    }
    //push
    @Override
    public void loop() {

        double one_pos = one.getPosition();

        if (gamepad2.a) {
            one.setPosition(one_pos+.5);
            two.setPosition(-.5);
        }
        if (gamepad2.b) {
            one.setPosition(one_pos-.5);
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

        //The wheels and grabber part

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


        //The three motor part

        pad1.update(gamepad1);

        motorOne.setPower(pad1.left_stick_y_exponential(.5));

        motorTwo.setPower(pad1.right_stick_y_exponential(.5));

        if (gamepad1.a){
            motorTres.setPower(.3);
        }

        else if (gamepad1.b){
            motorTres.setPower(-.25);
        }
        else{
            motorTres.setPower(0);
        }

        //if (gamepad1.left_bumper){
        //servo.setPosition(1);
        //}
        //if (gamepad1.right_bumper){
        //servo.setPosition(0);
        //}

        telemetry.addData("right stick",gamepad1.right_stick_y);
        telemetry.addData("left stick",gamepad1.left_stick_y);
        telemetry.addData("a",gamepad1.a);
        telemetry.addData("b",gamepad1.b);

    }
}
