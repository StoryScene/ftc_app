package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Kit Caldwell on 11/14/2017.
 */
@TeleOp
public class FourServosOrTwoServos extends OpMode{

    Servo one, two, three, four;

    @Override
    public void init() {

        one = hardwareMap.servo.get("one");
        two = hardwareMap.servo.get("two");
        three = hardwareMap.servo.get("three");
        four = hardwareMap.servo.get("four");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            one.setPosition(1);
        }
        if (gamepad1.b) {
            one.setPosition(0);
        }
        if (gamepad1.left_bumper){
            two.setPosition(1);
        }
        if (gamepad1.right_bumper){
            two.setPosition(0);
        }
        if (gamepad1.x) {
            three.setPosition(1);
        }
        if (gamepad1.y){
            three.setPosition(0);
        }
        if (gamepad1.dpad_down){
            four.setPosition(1);
        }
        if (gamepad1.dpad_up){
            four.setPosition(0);
        }
    }
}
