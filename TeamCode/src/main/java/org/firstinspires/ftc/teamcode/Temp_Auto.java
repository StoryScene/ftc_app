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


    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        wheels = new Wheels(lf, rf, lb, rb);

        wheels.initialize();
        resetStartTime();
    }

    @Override
    public void loop() {
        telemetry.addData("Wheels power: ", lf.getPower() + "\n" + rf.getPower() + "\n" + lb.getPower() + "\n" + rb.getPower());
        if (getRuntime() < 1.8) {
            wheels.move(0,1,0);
        }
        else {
            wheels.move(0,0,0);
        }

    }
}
