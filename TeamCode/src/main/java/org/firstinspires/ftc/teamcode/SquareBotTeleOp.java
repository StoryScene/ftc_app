package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Emma on 12/11/17.
 */

@TeleOp
public class SquareBotTeleOp extends OpMode {

    //CRServo arm;
    //ColorSensor color;
    DcMotor lf, rf, lb, rb;

    GamepadV2 pad1 = new GamepadV2();

    final double POWER = 0.9;



    @Override
    public void init() {
        mechanumInit();
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);

/*
        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");
*/
    }

    @Override
    public void loop() {

        mechanumLoop();
        /*
        if (pad1.left_bumper){
            arm.setPower(POWER);
        }
        else if (pad1.right_bumper){
            arm.setPower(-POWER);
        }
        else {
            arm.setPower(0);
        }
        telemetry.addData("Arm power: ", arm.getPower());
        */
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

        lf.setPower(0.5*Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(0.5*Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrb, -1, 1));


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
