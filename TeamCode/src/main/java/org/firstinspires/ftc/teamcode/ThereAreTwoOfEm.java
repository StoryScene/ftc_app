package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 1/9/2018.
 */

@TeleOp
public class ThereAreTwoOfEm extends OpMode{

    Servo servo1;
    Servo servo2;

    public void init(){
        servo1 = hardwareMap.servo.get("one");
        servo2 = hardwareMap.servo.get("two");

    }

    public void loop(){

        servo1.setPosition(1);
        servo2.setPosition(-1);
    }
}
