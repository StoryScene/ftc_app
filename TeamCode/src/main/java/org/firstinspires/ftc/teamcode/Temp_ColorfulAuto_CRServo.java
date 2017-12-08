package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by Emma on 12/2/17.
 */

@Autonomous
public class Temp_ColorfulAuto_CRServo extends OpMode {

    //Blue
    DcMotor lf, rf, lb, rb;

    CRServo arm;
    ColorSensor color;

    Wheels wheels;

    double oPow, startTime;
    final double ARMDOWNTIME = 1, HITBALL = 0.5, SHORTERMOVE = 0.3, LONGERMOVE = 0.7;

    boolean isTheShorterWay = true;


    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        wheels = new Wheels(lf,rf,lb,rb);

        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        oPow = arm.getPower();


    }

    @Override
    public void loop() {
        telemetry.addData("Wheels power: ", lf.getPower() + "\n" + rf.getPower() + "\n" + lb.getPower() + "\n" + rb.getPower());
        if (getRuntime() < ARMDOWNTIME) {
            arm.setPower(oPow + 0.3);
            telemetry.addData("Arm Position: ", arm.getPower());
            if (color.blue()/2 > color.red()) {
                isTheShorterWay = false;
            }

            if (color.blue() < color.red()/2) {
                isTheShorterWay = true;
            }
        }
        else if (getRuntime() < ARMDOWNTIME+HITBALL) {
            if (isTheShorterWay) {
            telemetry.addData("Arm Power: ", arm.getPower());
            lf.setPower(1);
            rf.setPower(1);
            lb.setPower(1);
            rb.setPower(1);
            }

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
            }

        }


    }
}
