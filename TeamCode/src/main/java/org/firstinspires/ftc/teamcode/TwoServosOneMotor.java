package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 1/16/2018.
 */

@TeleOp
public class TwoServosOneMotor extends OpMode{

    DcMotor motor;
    CRServo servo1;
    Servo servo2;


    @Override
    public void init() {

        motor = hardwareMap.dcMotor.get("motor");
        servo1 = hardwareMap.crservo.get("one");
        servo2 = hardwareMap.servo.get("two");
    }

    @Override
    public void loop() {

        motor.setPower(gamepad1.left_stick_y);

        if (gamepad1.left_bumper){

            servo1.setPower(.5);
            servo2.setPosition(0);
        }
        else if (gamepad1.right_bumper){
            servo1.setPower(-.5);
            servo2.setPosition(1);
        }
        else{
            servo1.setPower(0);
        }


        }
    }