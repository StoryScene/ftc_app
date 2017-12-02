package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Emma on 12/2/17.
 */

public class Temp_ColorfulAuto extends OpMode {

    //Blue
    DcMotor lf, rf, lb, rb;
    Wheels wheels;

    Servo arm;
    ColorSensor color;

    double oPos, startTime;
    final double ARMDOWNTIME = 0.5, SHORTERMOVE = 0.3, LONGERMOVE = 0.7;

    boolean isTheShorterWay = true;

    state currentState = state.ARM;

    enum state {
        ARM, DECIPHER, MOVE, PUTGLYPH
    }

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        wheels = new Wheels(lf, rf, lb, rb);
        wheels.initialize();

        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        oPos = arm.getPosition();


    }

    @Override
    public void loop() {
        telemetry.addData("Wheels power: ", lf.getPower() + "\n" + rf.getPower() + "\n" + lb.getPower() + "\n" + rb.getPower());

        telemetry.addData("Current State: ", currentState);
        switch (currentState){
            case ARM:

                if (getRuntime() < ARMDOWNTIME) {
                    arm.setPosition(oPos + 0.3);
                    telemetry.addData("Arm Position: ", arm.getPosition());
                    if (color.blue()/2 > color.red()) {
                        wheels.move(0,-1,0);
                        isTheShorterWay = false;
                    }

                    if (color.blue() < color.red()/2) {
                        wheels.move(0,1,0);
                        isTheShorterWay = true;
                    }
                    else{
                        wheels.move(0,0,0);
                    }
                }
                else if (getRuntime() < 2*ARMDOWNTIME) {
                    wheels.move(0,0,0);
                    arm.setPosition(oPos);
                    telemetry.addData("Arm Position: ", arm.getPosition());
                }
                else{
                    currentState = state.MOVE;
                    startTime = getRuntime();
                    break;
                }
            case MOVE:
                double runningFor;
                if (isTheShorterWay) {
                    runningFor = SHORTERMOVE;
                }
                else {
                    runningFor = LONGERMOVE;
                }

                if (getRuntime() - startTime < runningFor) {
                        wheels.move(0,1,0);
                }
                else {
                    wheels.move(0,0,0);
                    break;
                }

        }


    }
}
