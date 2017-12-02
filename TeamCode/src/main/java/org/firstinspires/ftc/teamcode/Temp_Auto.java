package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.StateMachine;

import java.util.Timer;

/**
 * Created by Emma on 12/2/17.
 */

@Autonomous
public class Temp_Auto extends OpMode {

    DcMotor lf, rf, lb, rb;
    Wheels wheels;
    double startTime;


    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        /*
        wheels = new Wheels(lf, rf, lb, rb);

        wheels.initialize();
        */
        startTime = getRuntime();

    }

    @Override
    public void loop() {
        telemetry.addData("Wheels power: ", lf.getPower() + "\n" + rf.getPower() + "\n" + lb.getPower() + "\n" + rb.getPower());
        if (getRuntime() - startTime < 2) {
            lf.setPower(-1);
            rf.setPower(-1);
            lb.setPower(-1);
            rb.setPower(-1);
        }
        else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }

    }
}
