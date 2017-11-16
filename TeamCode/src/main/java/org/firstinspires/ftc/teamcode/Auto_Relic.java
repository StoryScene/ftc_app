package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
 * Created by Emma on 11/14/17.
 */

@Autonomous(name="Test Vuforia", group ="Concept")
public class Auto_Relic extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    double tStartArm = 2, tStartMoving = 8, tPlaceRelic;
    int state = 0;
    double startTime, runTime;

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    DcMotor lS;
    Servo left;
    Servo right;
    Servo arm;
    ColorSensor color;

    double dPosition = 0.3;
    double oPosition = 1;


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        double POW = motorInit();
        VuforiaTrackable relicImage = setUpVuforia();
        startTime = getRuntime();

        while (opModeIsActive()) {
            runTime = getRuntime() - startTime;
            if (runTime < tStartArm) {
                moveArmLoop(POW);
            }
            else if (runTime < tStartMoving+2){
                RelicRecoveryVuMark goal = lookForRelicImage(relicImage);
                if (goal.equals(RelicRecoveryVuMark.LEFT)){
                    state = 2;
                }
                else if (goal.equals(RelicRecoveryVuMark.CENTER)) {
                    state = 3;
                }
                else if (goal.equals(RelicRecoveryVuMark.RIGHT)) {
                    state = 4;
                }
                tPlaceRelic = state + 8; // or something idk
            }
            else if (runTime < tPlaceRelic){
                fL.setPower(POW);
                bL.setPower(-POW);
                fR.setPower(-POW);
                bR.setPower(POW);
            }
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



    public double motorInit(){
        fL = hardwareMap.dcMotor.get("frontLeft");
        bL = hardwareMap.dcMotor.get("backLeft");
        fR = hardwareMap.dcMotor.get("frontRight");
        bR = hardwareMap.dcMotor.get("backRight");
        lS = hardwareMap.dcMotor.get("linearSlide");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        fL.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(Servo.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        double POWER = .5;

        arm.setPosition(dPosition);
        left.setPosition(0);
        right.setPosition(0);

        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());

        return POWER;
    }

    public void moveArmLoop(double POWER){
        if (color.blue()/2 > color.red()) {
            arm.setPosition(oPosition);
            fL.setPower(-POWER);
            bL.setPower(POWER);
            fR.setPower(POWER);
            bR.setPower(-POWER);

        }

        if (color.blue() < color.red()/2) {
            fL.setPower(POWER);
            bL.setPower(-POWER);
            fR.setPower(-POWER);
            bR.setPower(POWER);
            sleep(200);
            arm.setPosition(oPosition);
        }

        else{
            fL.setPower(0);
            bL.setPower(0);
            fR.setPower(0);
            bR.setPower(0);
        }
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
