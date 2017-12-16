package org.firstinspires.ftc.teamcode;

/**
 *  This current autonomous will simply knock off the jewel (30 points)
 *  and park in the safe zone (10 points)
 */

//main

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous

public class Auto_Red extends LinearOpMode {

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    //DcMotor lS;
    //Servo left;
    //Servo right;
    CRServo arm;
    ColorSensor color;



    public void runOpMode() throws InterruptedException {

        fL = hardwareMap.dcMotor.get("leftF");
        bL = hardwareMap.dcMotor.get("leftB");
        fR = hardwareMap.dcMotor.get("rightF");
        bR = hardwareMap.dcMotor.get("rightB");
        //lS = hardwareMap.dcMotor.get("linearSlide");
        //left = hardwareMap.servo.get("left");
        //right = hardwareMap.servo.get("right");
        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        fL.setDirection(DcMotor.Direction.REVERSE);
        //left.setDirection(Servo.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        double POWER = .5;

        //left.setPosition(0);
        //right.setPosition(0);

        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());

        waitForStart();

        while (opModeIsActive()) {


            arm.setPower(0.5);
            telemetry.addData("Current arm power: ", arm.getPower());


            if (color.blue()/2 > color.red()) {
                arm.setPower(0);
                telemetry.addData("Current arm power: ", arm.getPower());
                setPowers(0,0.8,0);
                sleep(1000);
                telemetry.addData("Here :) ", arm.getPower());

                setPowers(0,0,0);
                arm.setPower(-0.5);
                telemetry.addData("HELLO: ", arm.getPower());
                sleep(2000);
                arm.setPower(0);
                setPowers(0,-0.8,0);
                sleep(8000);
                setPowers(0,0,0);
                sleep(30000);
            }
            if (color.blue() < color.red()/2) {
                arm.setPower(0);
                telemetry.addData("Current arm power: ", arm.getPower());
                setPowers(0,-0.8,0);
                sleep(1000);
                telemetry.addData("Here :) ", arm.getPower());
                setPowers(0,0,0);
                arm.setPower(-0.5);
                telemetry.addData("HELLO: ", arm.getPower());
                sleep(2000);
                arm.setPower(0);
                setPowers(0,-0.8,0);
                sleep(6000);
                setPowers(0,0,0);
                sleep(30000);
            }
            else{
                arm.setPower(0);
                setPowers(0,0,0);
            }

        }
    }

    public void setPowers(double xx, double yy, double rotation) {
        double x = Range.clip(xx, -1, 1);
        double y = - Range.clip(yy, -1, 1);

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        double rot = Range.clip(rotation, -1, 1);

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

        fL.setPower(0.5*Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        fR.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        bL.setPower(0.5*Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        bR.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrb, -1, 1));


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