package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Created by Emma on 01/12/18.
 */

@Autonomous(name = "Red Without Encoders")
public class AutoWithoutEncoders extends LinearOpMode {


    // Largely copied from Auto Red
    // Wait no it won't because um idk how to put down glyphs

    DcMotor lWheel;
    DcMotor rWheel;
    DcMotor score;
    Servo arm;
    ColorSensor color;

    final double[] targetCoords = {0.1,0,-400};

    final double maxPower = 0.6;

    final int HITBALL = 100, ROTATE_NINETY = 600, LAST_PUSH = 500, DRIVE_FIRST = 0;

    int closer = 0;
    final int DIFF = 2*HITBALL;


    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;


    private int distance = 800;




    @Override
    public void runOpMode() throws InterruptedException {

        lWheel = hardwareMap.dcMotor.get("left");
        rWheel = hardwareMap.dcMotor.get("right");

        lWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        score = hardwareMap.dcMotor.get("score");

        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        VuforiaTrackable relicImage = setUpVuforia();


        waitForStart();
        boolean prepared = false;
        //State 0 or 1 means not prepared (goes to readjust), but 0 goes to move ball and 1 goes to driving
        //State 2 is move ball and state 3 is driving
        int state = 0;
        setPowers(0,0);
        double yy=0, rot=0;
        RelicRecoveryVuMark vu = RelicRecoveryVuMark.UNKNOWN;

        while (opModeIsActive()) {
            if (state == 0 || state == 1){
                ArrayList vuMarkAndPos = lookForRelicImage(relicImage);
                if (vu == RelicRecoveryVuMark.UNKNOWN) {
                    vu = (RelicRecoveryVuMark) vuMarkAndPos.get(0);
                }
                float[] pos = (float[]) vuMarkAndPos.get(1);

                // While only pos[2], pos[12] and pos[14] are useful.
                // 2 is the angle  (looking to the left is positive)
                // 12 is the distance (left/right)  (left is negative)
                // 14 is the distance (front/back)  (away from photo is negative)

                float[] usefulCoords = {pos[2], pos[12], pos[14]};

                /*
                if (state == 1 && Math.abs(usefulCoords[1] - targetCoords[1]) > 10){
                    yy = maxPower*Math.signum(usefulCoords[1] - targetCoords[1]);
                    rot = 0;
                    setPowers(yy,rot);
                } else {

                    // K I'm not sure about any of these but thinking about them is difficult hh
                    state += 2;
                    if (vu == vu.RIGHT) {
                        distance = 3000;
                    } else if (vu == vu.CENTER) {
                        distance = 4000;
                    } else if (vu == vu.LEFT) {
                        distance = 5000;
                    } else {
                        state -= 2;
                    }
                }
                */

                state += 2;

                if (vu == vu.LEFT) {
                    distance = 800;
                } else if (vu == vu.CENTER) {
                    distance = 700;
                } else if (vu == vu.RIGHT) {
                    distance = 600;
                }
                telemetry.addData("State: ", state);
                telemetry.update();
            }

            else if (state == 2) {

                arm.setPosition(1);
                setPowers(0,0);
                sleep(1000);
                telemetry.addData("Current arm position: ", arm.getPosition());

                if (color.blue()/2 > color.red()) {
                    setPowers(0,0.8);
                    sleep(HITBALL);
                    setPowers(0,0);

                    arm.setPosition(0);
                    sleep(1000);

                    setPowers(0,-0.7);
                    sleep(HITBALL);

                    setPowers(-0.6, 0);
                    sleep(DRIVE_FIRST);
                    setPowers(0,0);

                    closer = 1;
                    state = 1;

                }

                if (color.blue() < color.red()/2) {
                    setPowers(0,-0.7);
                    sleep(HITBALL);
                    setPowers(0,0);

                    arm.setPosition(0);
                    sleep(1000);

                    setPowers(0,0.8);
                    sleep(HITBALL);

                    setPowers(-0.6, 0);
                    sleep(DRIVE_FIRST);
                    setPowers(0,0);

                    state = 1;
                }

                else{
                    setPowers(0,0);
                }

                sleep(500);

                ArrayList vuMarkAndPos = lookForRelicImage(relicImage);
                if (vu == RelicRecoveryVuMark.UNKNOWN) {
                    vu = (RelicRecoveryVuMark) vuMarkAndPos.get(0);
                }
                float[] pos = (float[]) vuMarkAndPos.get(1);

                // While only pos[2], pos[12] and pos[14] are useful.
                // 2 is the angle  (looking to the left is positive)
                // 12 is the distance (left/right)  (left is negative)
                // 14 is the distance (front/back)  (away from photo is negative)

                float[] usefulCoords = {pos[2], pos[12], pos[14]};

                if (vu == vu.LEFT) {
                    distance = 800;
                } else if (vu == vu.CENTER) {
                    distance = 700;
                } else if (vu == vu.RIGHT) {
                    distance = 600;
                }

                telemetry.addData("State: ", state);
                telemetry.addData("Dis: ", distance);
                telemetry.addData("Actual powers: ", lWheel.getPower()+ " " +  rWheel.getPower());
                telemetry.update();

            }
            else {
                telemetry.addData("State: ", state);
                telemetry.update();

                setPowers(-0.6, 0);
                telemetry.addData("Code Loc:", "0");
                telemetry.addData("Driving distance:", distance + closer * DIFF);
                telemetry.update();
                sleep(distance + closer * DIFF - DRIVE_FIRST);

                setPowers( 0, -0.5);
                telemetry.addData("Code Loc:", "1");
                telemetry.update();
                sleep(ROTATE_NINETY);
                setPowers(0,0);
                sleep(200);

                setPowers(-0.6, 0);
                telemetry.addData("Code Loc:", "2");
                telemetry.update();
                sleep(2*LAST_PUSH);
                setPowers( 0, 0);
                sleep(200);

                score.setPower(0.5);
                telemetry.addData("Dis:", distance);
                telemetry.addData("Code Loc:", "3");
                telemetry.update();
                sleep(800);
                score.setPower(0);
                score.setPower(-0.5);
                sleep(800);
                score.setPower(0);
                sleep(300);

                setPowers(-0.6, 0);
                sleep(LAST_PUSH);

                setPowers(0.6, 0);
                sleep(LAST_PUSH/2);
                setPowers(0,0);
                sleep(20000);
            }

        }
    }


    VuforiaLocalizer vuforia;

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public VuforiaTrackable setUpVuforia() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();


        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "AYYlzbv/////AAAAGVuIwy4xcUzHo8C0G/OeRbRJnsxEWV4lE/p6iB2hM6zRKwfeHOCORAknKPwAGkr9HfMdkpKwrL3xwnyWZQch+X8DhIBW9JD9CZA7nWOLzfSOSAW71/6ZmKc+9Lcu5FxekvCQcWx7Ghj/Un7ULh/PbVqK2Qt+FLjc8RB6D+vd9M4EW3GA7NQcWocUn8BdSLKJDZwsOab3p7MB+HcYTWEEk/8fB8qbYtGsUd+YzU+2U/dOOzI/fQkI7wcIBBl6RVxgdLjmNVyi8Q2NgQV/0ETxhizPql6/T8kCutUwQDEbjpbAg19N9++WazmIYktjmXSbDWtpASQl9CVW40pPX6GVs27W2v44Of92NoaP3faSY7FF";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         *  in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();
        return relicTemplate;
    }

    public ArrayList<Object> lookForRelicImage(VuforiaTrackable relicTemplate) {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        float[] loc = new float[16];
        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            telemetry.addData("VuMark", "%s visible", vuMark);


                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
            telemetry.addData("Pose", format(pose));
            loc = pose.getData();
            telemetry.addData("3, 13, 15: ", loc[2] + " " + loc[12] + " " + loc[14]);


                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();
                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                double tX = trans.get(0);
                double tY = trans.get(1);
                double tZ = trans.get(2);

                // Extract the rotational components of the target relative to the robot
                double rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;
            }
        }
        else {
            telemetry.addData("VuMark", "not visible");
        }

        ArrayList<Object> result = new ArrayList<>();
        result.add(vuMark);
        result.add(loc);
        return result;
    }


    public void setPowers(double yy, double rotation) {
        double y = - Range.clip(yy, -1, 1);

        if (rotation == 0) {
            lWheel.setPower(y);
            rWheel.setPower(-y);
        } else {
            lWheel.setPower(rotation);
            rWheel.setPower(rotation);
        }

        telemetry.addData("Actual powers: ", lWheel.getPower()+ " " +  rWheel.getPower());
    }

    /*
    private void setAllPowersAndDistance(int distance, double yy, double rotation){
        int[] signs = computeSigns(yy, rotation);

        lWheel.setTargetPosition(signs[0]*distance);
        rWheel.setTargetPosition(signs[1]*distance);

        lWheel.setPower(signs[0]*maxPower);
        rWheel.setPower(signs[1]*maxPower);

    }
    */

    private int[] computeSigns(double yy, double rotation){
        double y = - Range.clip(yy, -1, 1);

        int[] result = new int[2];
        if (rotation == 0) {
            result[0] = getSign(y);
            result[1] = getSign(-y);
        } else {
            result[0] = getSign(rotation);
            result[1] = getSign(rotation);
        }

        return result;
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

    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
    }
}

