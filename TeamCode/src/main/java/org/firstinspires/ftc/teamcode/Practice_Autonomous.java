package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous practice")

public class Practice_Autonomous extends LinearOpMode {

    DcMotor motorLeftFront = null;
    DcMotor motorLeftBack = null;
    DcMotor motorRightFront = null;
    DcMotor motorRightBack = null;

    public void runOpMode() throws InterruptedException {
        motorLeftFront = hardwareMap.dcMotor.get("frontLeft");
        motorLeftBack = hardwareMap.dcMotor.get("backLeft");
        motorRightFront = hardwareMap.dcMotor.get("frontRight");
        motorRightBack = hardwareMap.dcMotor.get("backRight");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        //motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        double POWER = .5;

        waitForStart();

        while (opModeIsActive()) {
            //This SHOULD drive forward.
            motorLeftFront.setPower(POWER);
            motorLeftBack.setPower(POWER);
            motorRightFront.setPower(POWER);
            motorRightBack.setPower(POWER);
            sleep(1000);

            //This SHOULD drive backward.
            motorLeftFront.setPower(-POWER);
            motorLeftBack.setPower(-POWER);
            motorRightFront.setPower(-POWER);
            motorRightBack.setPower(-POWER);
            sleep(1000);

            //This SHOULD drive sideway.
            motorLeftFront.setPower(POWER);
            motorLeftBack.setPower(-POWER);
            motorRightFront.setPower(POWER);
            motorRightBack.setPower(-POWER);
            sleep(1000);

            //This SHOULD stop.
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
            sleep(1000);
        }
    }

}
