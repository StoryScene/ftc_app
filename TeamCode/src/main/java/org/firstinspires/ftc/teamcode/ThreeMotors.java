package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Kit Caldwell on 10/24/2017.
 */
@TeleOp
public class ThreeMotors extends OpMode {
    DcMotor motorOne;
    DcMotor motorTwo;
    DcMotor motorTres;
    //Servo servo;
    GamepadV2 pad1 = new GamepadV2();


    @Override
    public void init() {
        motorOne = hardwareMap.dcMotor.get("tL");
        motorTwo = hardwareMap.dcMotor.get("tR");
        motorTres = hardwareMap.dcMotor.get("bL");
        //servo = hardwareMap.servo.get("servo");

        motorOne.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        pad1.update(gamepad1);

        motorOne.setPower(pad1.left_stick_y_exponential(.5));

        motorTwo.setPower(pad1.left_stick_y_exponential(.5));

        motorTres.setPower(pad1.right_stick_y_exponential(.6));

        //if (gamepad1.left_bumper){
            //servo.setPosition(1);
        //}
        //if (gamepad1.right_bumper){
            //servo.setPosition(0);
        //}

        telemetry.addData("right stick",gamepad1.right_stick_y);
        telemetry.addData("left stick",gamepad1.left_stick_y);
        telemetry.addData("a",gamepad1.a);
        telemetry.addData("b",gamepad1.b);

    }
}
