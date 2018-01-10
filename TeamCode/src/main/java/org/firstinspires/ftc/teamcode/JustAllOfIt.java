/**package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;


 * Created by Kit Caldwell on 12/8/2017.


@TeleOp
public class JustAllOfIt extends OpMode{
    Servo tR, bL, tL, bR;
    DcMotor lf, rf, lb, rb;
    DcMotor lift;
    DcMotor r1, r2, r3;
    public GamepadV2 pad1 = new GamepadV2();

    int targetPosForLift = 0;
    boolean wasChangingTargetPos = false;

    @Override
    public void init() {
        tR= hardwareMap.servo.get("tR");
        tL = hardwareMap.servo.get("tL");
        bR = hardwareMap.servo.get("bL");
        bL = hardwareMap.servo.get("bR");

        r1 = hardwareMap.dcMotor.get("r1");
        r2 = hardwareMap.dcMotor.get("r2");
        r3 = hardwareMap.dcMotor.get("r3");

        lift = hardwareMap.dcMotor.get("lift");

        mechanumInit();

    }

    @Override
    public void loop() {

        //servos
        if (gamepad1.b){
            bL.setPosition(1);
            bR.setPosition(-.75);
        }
        else if (gamepad1.a){
            bL.setPosition(-.75);
            bR.setPosition(.75);
        }

        if (gamepad1.x){
            tR.setPosition(-1);
            tL.setPosition(.75);
        }
        else if (gamepad1.y){
            tR.setPosition(.75);
            tL.setPosition(-.75);
        }
        //extend
        if (gamepad2.dpad_down){
            r1.setPower(-.5);
        }
        else if (gamepad2.dpad_up){
            r1.setPower(.5);
        }
        else{
            r1.setPower(0);
        }
        //lift robot
        if (gamepad2.b){
            r2.setPower(.5);
        }
        else if (gamepad2.y){-
            r2.setPower(-.5);
        }
        else{
            r2.setPower(0);
        }
        //spool
        r3.setPower(gamepad2.left_stick_y);
        //glyph lift
        lift.setPower(-1*gamepad2.right_stick_y);

        telemetry.addData("position 1", tR.getPosition());
        telemetry.addData("position 2", tL.getPosition() );

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
        double x = -Range.clip(gamepad1.left_stick_x, -1, 1);
        double y =  Range.clip(gamepad1.left_stick_y, -1, 1);

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

        lf.setPower(0.4*Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(0.4*-Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(0.4*Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(0.4 *-Math.pow(POW,2) * Range.clip(vrb, -1, 1));


        telemetry.addData("maxPower: ", maxPower);
    }

}
 */