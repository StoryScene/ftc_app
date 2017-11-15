package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Emma on 11/13/17.
 */
@TeleOp
public class MechanumDrive extends OpMode {

    DcMotor lf, rf, lb, rb;


    @Override
    public void init() {
        mechanumInit();
    }

    @Override
    public void loop() {
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
