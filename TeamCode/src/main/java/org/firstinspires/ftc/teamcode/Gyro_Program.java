package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;



@Autonomous

public class Gyro_Program extends LinearOpMode {

    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    BNO055IMU imu;

    GyroSensor sensorGyro; //configurating GyroSensor
    ModernRoboticsI2cGyro mrGyro; //

    @Override

    public void runOpMode() throws InterruptedException

    {
        // Initialize stuff
        frontleft = hardwareMap.dcMotor.get("frontleft");
        frontright = hardwareMap.dcMotor.get("frontright");
        backleft = hardwareMap.dcMotor.get("backleft");
        backright = hardwareMap.dcMotor.get("backright");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        //Set the encoders
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Turn robot to 15% power.
        double turnspeed = 0.15;
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;
        int zAccumilated; //Total rotation left/right

        sleep(1000);
        mrGyro.calibrate(); // Sensor callibration

        waitForStart();


        while (mrGyro.isCalibrating()) { //Ensures the robot is callibrated

        }
        while (opModeIsActive()) {
            zAccumilated = mrGyro.getIntegratedZValue(); //Set variables to gyro readings

            if (zAccumilated > 0) {
                frontleft.setPower(turnspeed);
                frontright.setPower(-turnspeed);
                backleft.setPower(turnspeed);
                backright.setPower(-turnspeed);
            }
            if (zAccumilated < 0) {
                frontleft.setPower(-turnspeed);
                frontright.setPower(turnspeed);
                backleft.setPower(-turnspeed);
                backright.setPower(turnspeed);
            }
        }

        zAccumilated = mrGyro.getIntegratedZValue();

        telemetry.addData("1.accu", String.format("@03d", zAccumilated));

        sleep(25);
    }

}