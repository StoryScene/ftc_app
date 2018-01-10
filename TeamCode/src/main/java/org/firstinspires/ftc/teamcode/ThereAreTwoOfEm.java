package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
/**
 * Created by Kit Caldwell on 1/9/2018.
 */

@TeleOp
public class ThereAreTwoOfEm extends OpMode{

    DcMotor leftMotor;
    DcMotor rightMotor;

    public void init(){
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void loop(){

        leftMotor.setPower(gamepad1.left_stick_y);
        rightMotor.setPower(gamepad1.right_stick_y);

    }
}
