package org.firstinspires.ftc.teamcode; /**
 * Created by billcipher1344 on 10/24/2016.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.GamepadV2;
import org.firstinspires.ftc.teamcode.modules.MecanumDrive1617;

@TeleOp
public class MecanumTest1617 extends OpMode {

    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public Servo servo1, servo2, servo4;
    public CRServo servo3;
    public GamepadV2 pad1 = new GamepadV2();

    public void init(){
        //hi
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        servo1 = hardwareMap.servo.get("tL");
        servo2 = hardwareMap.servo.get("tR");
        servo3 = hardwareMap.crservo.get("bL");
        servo4 = hardwareMap.servo.get("bR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop(){

        pad1.update(gamepad1);
        MecanumDrive1617.loop(frontLeft, frontRight, backLeft, backRight, pad1);

        if (pad1.x){
            servo1.setPosition(1);
            servo2.setPosition(0);
        }
        if (pad1.y){
            servo1.setPosition(.5);
            servo2.setPosition(.5);
        }
        if (pad1.a){
            servo3.setPower(.5);
            servo4.setPosition(0);
        }
        if (pad1.b){
            servo3.setPower(.5);
            servo4.setPosition(.5);
        }
    }
}
