package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 12/14/2017.
 */
@TeleOp
public class ServoTest extends OpMode {

    Servo serv;

    @Override
    public void init() {
        serv = hardwareMap.servo.get("s");
    }

    @Override
    public void loop() {
        if (gamepad1.a){
            serv.setPosition(1);
        }
        else if (gamepad1.b){
            serv.setPosition(-1);
        }
    }
}
