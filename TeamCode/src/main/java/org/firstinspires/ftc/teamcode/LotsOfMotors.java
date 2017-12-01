package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Emma on 12/1/17.
 */

public class LotsOfMotors extends OpMode {

    Wheels wheels;
    DcMotor motor;

    public GamepadV2 pad1 = new GamepadV2();

    @Override
    public void init() {


        DcMotor lf = hardwareMap.dcMotor.get("frontLeft");
        DcMotor rf = hardwareMap.dcMotor.get("frontRight");
        DcMotor lb = hardwareMap.dcMotor.get("backLeft");
        DcMotor rb = hardwareMap.dcMotor.get("backRight");

        wheels = new Wheels(rf, lf, lb, rb);
        wheels.initialize();

    }

    @Override
    public void loop() {
        wheels.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);


    }
}
