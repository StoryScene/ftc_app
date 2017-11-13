package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HardwareVortex {
    // TAGS
    public final static String
        DRIVE_MOTOR_TAG = "DRIVE",
        GENERIC_MOTOR_TAG = "MOTORS",
        ENCODER_TAG = "ENCODERS",
        SERVO_TAG = "SERVOS";

    // MOTORS
    public DcMotor frontLeft, frontRight;
    public DcMotor grab1, grab2;

    // SERVOS
    //public Servo pusherLeft, pusherRight;
    //public Servo leftJewel, rightJewel;

    // SENSORS
    public LynxI2cColorRangeSensor colorRange;
    public BNO055IMU imu;
    public I2cAddr ColorNumber = I2cAddr.create7bit(0x39);

    // CONSTANTS
    public final static double SHOOTER_POWER = .6;

    public final static double LEFT_LIFT_INIT = 220/255.0, LEFT_LIFT_PUSH = 70/255.0;
    public final static double RIGHT_LIFT_INIT = 0/255.0, RIGHT_LIFT_PUSH = 65/255.0;

    public HardwareVortex() {
    }

    public void init(HardwareMap hardwareMap) {
        // Initialize the motors
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        //backLeft = hardwareMap.dcMotor.get("BL");
        //backRight = hardwareMap.dcMotor.get("BR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        grab1 = hardwareMap.dcMotor.get("grab1");
        grab2 = hardwareMap.dcMotor.get("grab2");

        grab2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize the servos
        //leftJewel = hardwareMap.servo.get("leftJewel");
        //rightJewel = hardwareMap.servo.get("rightJewel");

        //leftJewel.setPosition(LEFT_LIFT_INIT);
        //rightJewel.setPosition(RIGHT_LIFT_INIT);

        //pusherLeft = hardwareMap.servo.get("pusherLeft");
        //pusherRight = hardwareMap.servo.get("pusherRight");

        //pusherLeft.setPosition(0.5);
        //pusherRight.setPosition(0.5);


        // Initialize the sensors
        colorRange = (LynxI2cColorRangeSensor)hardwareMap.i2cDevice.get("sensorColorRange");
        ColorNumber = (new I2cAddr(0x39));
    }

    public void resetEncoders(){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void generateTelemetry(Telemetry telemetry, Boolean encoders) {
        telemetry.addData(DRIVE_MOTOR_TAG, "FL: %f FR: %f, BL: %f, BR: %f", frontLeft.getPower(), frontRight.getPower());
        telemetry.addData(ENCODER_TAG, "FL: %d FR: %d BL: %d BR: %d", frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
        telemetry.addData(GENERIC_MOTOR_TAG, "grab1: %f grab2: %f lift: %f flipper: %f", grab1.getPower(), grab2.getPower());
        //telemetry.addData(SERVO_TAG, "L: %f R: %f PL: %f PR: %f", leftJewel.getPosition(), rightJewel.getPosition(), pusherLeft.getPosition(), pusherRight.getPosition());
    }
}
