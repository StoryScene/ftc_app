package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;


/**
 * Created by Emma on 11/29/17.
 */

public class Wheels {

    //Currently these should be passed in as rf, lf, lb, rb
    DcMotor lf, rf, lb, rb;
    public GamepadV2 pad1 = new GamepadV2();
    private double maxP = 0, rot = 0, dir = 0;

    public Wheels(DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        lf = leftFront;
        rf = rightFront;
        lb = leftBack;
        rb = rightBack;
    }

    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
    }

    public void initialize() {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //Pass in directly
    public double[] move(double xx, double yy, double rotation) {
        double x = Range.clip(xx, -1, 1);
        double y = - Range.clip(yy, -1, 1);

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        rot = rotation;


        double r = Math.hypot(x, y);
        double angle = 0.0;

        double POW = Math.max(Math.hypot(x, y), Math.abs(rot));


        if (r > 0.1){
            angle = Math.atan2(y,x) - Math.PI / 4;
        }

        dir = angle;

        double vlf = r * Math.cos(angle) + rot;
        double vrf = r * Math.sin(angle) - rot;
        double vlb = r * Math.sin(angle) + rot;
        double vrb = r * Math.cos(angle) - rot;

        double maxPower = maxPow(vlf, vrf, vlb, vrb);

        maxP = maxPower;

        vlf /= maxPower;
        vrf /= maxPower;
        vlb /= maxPower;
        vrb /= maxPower;

        /*
        lf.setPower(Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(-Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(-Math.pow(POW,2) * Range.clip(vrb, -1, 1));
        */

        double[] result = new double[4];
        result[0] = Math.pow(POW,2) * Range.clip(vlf, -1, 1);
        result[1] = Math.pow(POW,2) * Range.clip(vrf, -1, 1);
        result[2] = Math.pow(POW,2) * Range.clip(vlb, -1, 1);
        result[3] = Math.pow(POW,2) * Range.clip(vrb, -1, 1);

        return result;

    }

    public double getMaxP(){
        return maxP;
    }

    public double getRot(){
        return rot;
    }

    public double getDir(){
        return dir;
    }

}
