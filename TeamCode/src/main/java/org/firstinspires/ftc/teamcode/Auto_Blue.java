package org.firstinspires.ftc.teamcode;

/**
 *  This current autonomous will simply knock off the jewel (30 points)
 *  and park in the safe zone (10 points)
 */

//main

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous

public class Auto_Blue extends LinearOpMode {

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    DcMotor lS;
    Servo left;
    Servo right;
    Servo arm;
    ColorSensor color;

    double dPosition = 0.3;
    double oPosition = 1;

    public void runOpMode() throws InterruptedException {

        fL = hardwareMap.dcMotor.get("frontLeft");
        bL = hardwareMap.dcMotor.get("backLeft");
        fR = hardwareMap.dcMotor.get("frontRight");
        bR = hardwareMap.dcMotor.get("backRight");
        lS = hardwareMap.dcMotor.get("linearSlide");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        fL.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(Servo.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        double POWER = .5;

        arm.setPosition(dPosition);
        left.setPosition(0);
        right.setPosition(0);

        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());

        waitForStart();

        while (opModeIsActive()) {

            if (color.blue()/2 > color.red()) {
                fL.setPower(-POWER);
                bL.setPower(-POWER);
                fR.setPower(-POWER);
                bR.setPower(-POWER);
                sleep(200);
                arm.setPosition(oPosition);
            }

            if (color.blue() < color.red()/2) {
                fL.setPower(POWER);
                bL.setPower(POWER);
                fR.setPower(POWER);
                bR.setPower(POWER);
                sleep(200);
                arm.setPosition(oPosition);
            }

            else{
                fL.setPower(0);
                bL.setPower(0);
                fR.setPower(0);
                bR.setPower(0);
            }

        }
    }
}