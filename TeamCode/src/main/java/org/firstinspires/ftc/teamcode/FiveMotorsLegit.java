package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Kit Caldwell on 11/10/2017.
 */
@TeleOp
public class FiveMotorsLegit extends OpMode {

    DcMotor leftDrive;
    DcMotor rightDrive;
    DcMotor intake1;
    DcMotor intake2;
    DcMotor lift;

    @Override
    public void init() {

        leftDrive = hardwareMap.dcMotor.get("left");
        rightDrive = hardwareMap.dcMotor.get("right");
        intake1 = hardwareMap.dcMotor.get("in1");
        intake2 = hardwareMap.dcMotor.get("in2");
        lift = hardwareMap.dcMotor.get("lift");

        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        leftDrive.setPower(gamepad1.left_stick_y);
        rightDrive.setPower(gamepad1.right_stick_y);

        if (gamepad2.right_bumper){
            intake1.setPower(.5);
            intake2.setPower(.5);
        }
        else if (gamepad2.left_bumper) {
            intake1.setPower(-.5);
            intake2.setPower(-.5);
        }
        else {
            intake1.setPower(0);
            intake2.setPower(0);
        }

        lift.setPower(gamepad2.right_stick_y);

        telemetry.addData("left stick 1", gamepad1.left_stick_y);
        telemetry.addData("right stick 1", gamepad1.right_stick_y);
        telemetry.addData("right bumper",gamepad2.right_bumper);
        telemetry.addData("left bumper",gamepad2.left_bumper);
        telemetry.addData("right stick 2",gamepad2.right_stick_y);
    }
}