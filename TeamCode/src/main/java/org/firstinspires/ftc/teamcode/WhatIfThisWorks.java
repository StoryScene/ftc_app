package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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

/**
 * Created by Emma on 12/15/17.
 */

@TeleOp
public class WhatIfThisWorks extends LinearOpMode {

    // Largely copied from Auto Red
    // Wait no it won't because um idk how to \put down glyphs

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    //DcMotor lS;
    //Servo left;
    //Servo right;
    CRServo arm;
    ColorSensor color;



    public void runOpMode() throws InterruptedException {

        fL = hardwareMap.dcMotor.get("leftF");
        bL = hardwareMap.dcMotor.get("leftB");
        fR = hardwareMap.dcMotor.get("rightF");
        bR = hardwareMap.dcMotor.get("rightB");
        //lS = hardwareMap.dcMotor.get("linearSlide");
        //left = hardwareMap.servo.get("left");
        //right = hardwareMap.servo.get("right");
        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");


        double POWER = .5;

        //left.setPosition(0);
        //right.setPosition(0);

        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());

        waitForStart();

        while (opModeIsActive()) {


            arm.setPower(0.5);
            telemetry.addData("Current arm power: ", arm.getPower());


            if (color.blue()/2 > color.red()) {
                arm.setPower(0);
                setPowers(0,-0.8,0);
                sleep(1000);
                setPowers(0,0,0);

                arm.setPower(-0.5);
                sleep(3000);
                arm.setPower(0);
                sleep(1000);

                VuforiaTrackable relicImage = setUpVuforia();
                RelicRecoveryVuMark goal = lookForRelicImage(relicImage);


                double RUNAWAY = 2;
                if (goal.equals(RelicRecoveryVuMark.LEFT)){
                    RUNAWAY = 2;
                }
                else if (goal.equals(RelicRecoveryVuMark.CENTER)) {
                    RUNAWAY = 3;
                }
                else if (goal.equals(RelicRecoveryVuMark.RIGHT)) {
                    RUNAWAY = 4;
                }

                setPowers(0,0.6,0);
                sleep((int)(1000*(RUNAWAY+8)));

                setPowers(0,0,0.8);
                sleep(2000);

                setPowers(0,1,0);
                sleep(1000);

                setPowers(0,0,0);
                sleep(30000);
            }

            if (color.blue() < color.red()/2) {
                arm.setPower(0);
                setPowers(0,0.8,0);
                sleep(1000);
                setPowers(0,0,0);

                arm.setPower(-0.5);
                sleep(3000);
                arm.setPower(0);
                sleep(1000);

                VuforiaTrackable relicImage = setUpVuforia();
                RelicRecoveryVuMark goal = lookForRelicImage(relicImage);


                double RUNAWAY = 2;
                if (goal.equals(RelicRecoveryVuMark.LEFT)){
                    RUNAWAY = 2;
                }
                else if (goal.equals(RelicRecoveryVuMark.CENTER)) {
                    RUNAWAY = 3;
                }
                else if (goal.equals(RelicRecoveryVuMark.RIGHT)) {
                    RUNAWAY = 4;
                }

                setPowers(0,0.6,0);
                sleep((int)(1000*(RUNAWAY+8)));

                setPowers(0,0,0.8);
                sleep(2000);

                setPowers(0,1,0);
                sleep(1000);

                setPowers(0,0,0);
                sleep(30000);
            }

            else{
                arm.setPower(0);
                setPowers(0,0,0);
            }

        }
    }

    public void setPowers(double xx, double yy, double rotation) {
        double x = Range.clip(xx, -1, 1);
        double y = - Range.clip(yy, -1, 1);

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        double rot = Range.clip(rotation, -1, 1);

        double r = Math.hypot(x, y);
        double angle = 0.0;

        double POW = Math.max(Math.hypot(x, y), Math.abs(rot));


        if (r > 0.1){
            angle = Math.atan2(y,x) - Math.PI / 4;
        }

        telemetry.addData("angle: ", angle);
        telemetry.addData("radius: ", r);
        telemetry.addData("rotate: ", rot);


        double vlf = r * Math.cos(angle) + rot;
        double vrf = r * Math.sin(angle) - rot;
        double vlb = r * Math.sin(angle) + rot;
        double vrb = r * Math.cos(angle) - rot;

        double maxPower = maxPow(vlf, vrf, vlb, vrb);

        vlf /= maxPower;
        vrf /= maxPower;
        vlb /= maxPower;
        vrb /= maxPower;

        fL.setPower(0.5*Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        fR.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        bL.setPower(0.5*Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        bR.setPower(-0.5*Math.pow(POW,2) * Range.clip(vrb, -1, 1));


        telemetry.addData("maxPower: ", maxPower);
    }


    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
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
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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

    public RelicRecoveryVuMark lookForRelicImage(VuforiaTrackable relicTemplate) {
        /**
         * See if any of the instances of {@link relicTemplate} are currently visible.
         * {@link RelicRecoveryVuMark} is an enum which can have the following values:
         * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
         * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
         */
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
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

        telemetry.update();
        return vuMark;
    }




}
