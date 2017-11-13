package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Precision;
import org.firstinspires.ftc.teamcode.modules.State;
import org.firstinspires.ftc.teamcode.modules.StateMachine;

/**
 * Created by Kit Caldwell on 11/11/2017.
 */
@Autonomous
public class BasicAuto extends OpMode {
    private Precision precision;

    private HardwareVortex robot = new HardwareVortex();

    private DcMotor[] leftDrive, rightDrive, driveMotors;

    private StateMachine main;
    //delay
    private double delay = 0;

    public void init() {
        precision = new Precision();


        //robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive = new DcMotor[]{
                robot.frontLeft
        };
        rightDrive = new DcMotor[]{
                robot.frontRight
        };
        driveMotors = new DcMotor[]{
                robot.frontLeft,
                robot.frontRight,
        };

        main = new StateMachine(
                new State("stop") {

                    @Override
                    public void run() {
                        move(0);
                    }
                },
                new State("drive to box") {
                    @Override
                    public void run() {
                        if (reachedDestination(400, 500, 0.4)) {
                            robot.resetEncoders();
                            changeState("stop");
                        }
                    }
                }
        ).start();
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            delay = 0;
        }
        else if (gamepad1.b) {
            delay = 2.5;
        }
        else if (gamepad1.x) {
            delay = 5;
        }
        else if (gamepad1.y) {
            delay = 7.5;
        }
    }

    @Override
    public void loop() {

        telemetry.addData("avgEncoder", main.getDouble("encoder"));
        robot.generateTelemetry(telemetry, true);
    }
    private void move(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
    }
    private boolean reachedDestination(int target, int timeout, double power) {
        return precision.destinationReached(driveMotors, power, Math.signum(power)*0.12, target, 2.0, 10, timeout);
    }

}

