package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 12/14/2017.
 */
@TeleOp
public class CRServoTest extends OpMode {

    CRServo cr;

    @Override
    public void init() {
        cr = hardwareMap.crservo.get("cr");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            cr.setPower(1);
        }
        else if (gamepad1.b){
            cr.setPower(-1);
        }
        else{
            cr.setPower(0);
        }

    }
}
