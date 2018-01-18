package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 1/17/2018.
 */

@TeleOp
public class TwoCR extends OpMode {

    CRServo servo1;
    CRServo servo2;

    @Override
    public void init() {

        servo1 = hardwareMap.crservo.get("one");
        servo2 = hardwareMap.crservo.get("two");

    }

    @Override
    public void loop() {

        if (gamepad1.x){
            servo1.setPower(.5);
        }
        else if (gamepad1.y){
            servo1.setPower(-.5);
        }
        else{
            servo1.setPower(0);
        }

        if (gamepad1.a){
            servo2.setPower(.5);
        }
        else if (gamepad1.b){
            servo2.setPower(-.5);
        }
        else{
            servo2.setPower(0);
        }
    }
}
