/**package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.modules.Precision;
import org.firstinspires.ftc.teamcode.modules.State;
import org.firstinspires.ftc.teamcode.modules.StateMachine;


public class AutoJewel extends OpMode{
    private Precision precision;

    private HardwareVortex robot = new HardwareVortex();

    private DcMotor[] leftDrive, rightDrive, driveMotors;

    private LynxI2cColorRangeSensor colorRange;

    private StateMachine main;
    //delay
    private double delay = 0;
    //team color (side)
    private boolean blue = true;
    //start location (corner to left == true)
    private boolean cornerStart = false;
    //motion (parked or moving)
    private boolean park = false;

    @Override
    public void init() {
        precision = new Precision();

        robot.init(hardwareMap);

        //0grobot.limit.setPosition(HardwareVortex.LIMIT_OFF);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDrive = new DcMotor[]{
                robot.frontLeft,
                robot.backLeft
        };
        rightDrive = new DcMotor[]{
                robot.frontRight,
                robot.backRight
        };
        driveMotors = new DcMotor[]{
                robot.frontRight,
                robot.frontLeft,
                robot.backRight,
                robot.backLeft
        };

        main = new StateMachine(
                new State("stop") {
                    @Override
                    public void run() {
                        move(0);
                    }
                },

                new State("drive to jewels") {
                    @Override
                    public void run() {
                        if (elapsedTime(getDouble("start")) > delay) {
                            sendData("shooter start", time);
                            if (cornerStart) {
                                changeState("drive to center line");
                            } else {
                                changeState("middle - reach vortex");
                            }
                        }
                    }
                }
        ).start();

    }

    public void init_loop() {

    }
    public void start() {

    }
    public void loop() {

    }

    public void stop() {

    }

    //Functions vvv

    private void move(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }
    private double elapsedTime(double startTime){
        return time - startTime;
    }
    private boolean reachedDestination(int target, int timeout, double power){
        return precision.destinationReached(driveMotors,power,Math.signum(power)*0.12,target,2.0,10,timeout);
    }
    private boolean turnedDegrees(double degrees, int timeout, double power){
        return precision.distanceTurned(leftDrive,rightDrive,power,0.42,(int)((-degrees/180.0)*1200),2.0,20,timeout);
    }
    private void updateSensors(){
        telemetry.addData("State", main.getActiveState());
        telemetry.addData("Color","R: %d G: %d B: %d A: %d",colorRange.red(),colorRange.green(),colorRange.blue(),colorRange.alpha());
        telemetry.addData("Light", "Left: %f Right: %f", robot.colorRange.getLightDetected());
    }
    private boolean isBlue(LynxI2cColorRangeSensor colorRange){
        int red = colorRange.red();
        int blue = colorRange.blue();

        if (red >= 100 && blue >= red + 50) return true;
        if (Math.abs(red - blue) <= 10) return false;
        if (red - blue >= 100) return true;

        return false;
    }

    private int boolToSign(boolean bool) {return bool?1:-1;}

}
*/