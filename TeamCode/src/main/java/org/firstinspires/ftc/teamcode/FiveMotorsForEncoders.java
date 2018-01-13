package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;

/**
 * Created by Emma on 1/12/18.
 */

@TeleOp(name = "Encoders Test")
public class FiveMotorsForEncoders extends OpMode {

    DcMotor lWheel, rWheel;
    DcMotor transparent;
    DcMotor grabLeft, grabRight;
    CRServo arm;
    ColorSensor color;

    public GamepadV2 pad1 = new GamepadV2();

    @Override
    public void init() {

        lWheel = hardwareMap.dcMotor.get("leftWheel");
        rWheel = hardwareMap.dcMotor.get("rightWheel");
        grabLeft = hardwareMap.dcMotor.get("grab1");
        grabRight = hardwareMap.dcMotor.get("grab2");

        //transparent = hardwareMap.dcMotor.get("transparent");

        /*
        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");
        */
        lWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        double y = Range.clip(gamepad1.left_stick_y, -1, 1);
        double rot = Range.clip(gamepad1.right_stick_y, -1, 1);
        setPowers(y, rot);

        telemetry.addData("lWheel Encoder:", lWheel.getCurrentPosition());
        telemetry.addData("rWheel Encoder:", rWheel.getCurrentPosition());

        if (gamepad1.a){
            lWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.x){
            grabLeft.setPower(.5);
            grabRight.setPower(-.5);
        }
        else if (gamepad1.y){
            grabLeft.setPower(-.5);
            grabRight.setPower(.5);
        }
        else{
            grabLeft.setPower(0);
            grabRight.setPower(0);
        }

        /*
        if (gamepad1.left_bumper){
            transparent.setPower(.5);
        }
        if (gamepad1.right_bumper){
            transparent.setPower(-.5);
        }
        else {
            transparent.setPower(0);
        }
        */

        telemetry.update();

    }


    public void setPowers(double yy, double rotation) {
        double y = Range.clip(yy, -1, 1);

        if (rotation == 0) {
            lWheel.setPower(y);
            rWheel.setPower(y);
        } else {
            lWheel.setPower(rotation);
            rWheel.setPower(-rotation);
        }

        telemetry.addData("Actual powers: ", lWheel.getPower()+ " " +  rWheel.getPower());
    }

    private int getSign(double x) {
        if (x > 0.001) {
            return 1;
        }
        if (x < -0.001) {
            return -1;
        }
        return 0;
    }
}
