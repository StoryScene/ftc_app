package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Emma on 11/10/17.
 */
//push

public class FiveMotors extends OpMode {

    DcMotor motorLeft;
    DcMotor motorRight;
    DcMotor grabLeft;
    DcMotor grabRight;
    DcMotor arm;

    @Override
    public void init(){
        motorLeft = hardwareMap.dcMotor.get("left");
        motorRight = hardwareMap.dcMotor.get("right");
        grabLeft = hardwareMap.dcMotor.get("grab1");
        grabRight = hardwareMap.dcMotor.get("grab2");
        arm = hardwareMap.dcMotor.get("arm");

        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        motorLeft.setPower(gamepad1.left_stick_y);
        motorRight.setPower(gamepad1.right_stick_y);

        if (gamepad1.x){
            grabLeft.setPower(.5);
            grabRight.setPower(-.5);
        }
        else if (gamepad1.y){
            grabLeft.setPower(-.5);
            grabRight.setPower(.5);
        }
        else{
            grabLeft.setPower(0);
            grabRight.setPower(0);
        }
        if (gamepad1.left_bumper){
            arm.setPower(.5);
        }
        if (gamepad1.right_bumper){
            arm.setPower(0);
        }

        telemetry.addData("right bumper",gamepad1.right_bumper);
        telemetry.addData("left bumper",gamepad1.left_bumper);
        telemetry.addData("a",gamepad1.a);
        telemetry.addData("b",gamepad1.b);
        telemetry.addData("x",gamepad1.x);
        telemetry.addData("y",gamepad1.y);
        telemetry.addData("right stick", gamepad1.right_stick_y);
        telemetry.addData("left stick", gamepad1.left_stick_y);

    }
}
