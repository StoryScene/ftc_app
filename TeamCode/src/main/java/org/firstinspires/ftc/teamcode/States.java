
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import org.firstinspires.ftc.teamcode.modules.GamepadV2;
import org.firstinspires.ftc.teamcode.modules.State;
import org.firstinspires.ftc.teamcode.modules.StateMachine;

/**
 * Created by Emma on 12/5/17.
 */

@Autonomous
public class States extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    private final double absoluteMaxPower = 0.5, ARMTARGETPOS = 0.3;
    private double tPlaceRelic, POWER = 0.5;
    private int state = 0;

    manyStates multipleStates = manyStates.STATE_MOVEARM;

    DcMotor lf, lb, rf, rb;

    CRServo arm;
    ColorSensor color;

    private enum manyStates {
        STATE_REST,
        STATE_MOVEARM,
        STATE_IMAGE,
        STATE_DRIVEFAR,
        STATE_TURN,
    }


    @Override
    public void runOpMode() {
        motorInit();
        VuforiaTrackable relicImage = setUpVuforia();

        while (opModeIsActive()){
            telemetry.addData("Cuurent State: ",  multipleStates);
            switch (multipleStates) {
                case STATE_REST:
                    setPowers(0,0,0);
                case STATE_MOVEARM:
                    sleep(1000);
                    moveArmLoop();
                    sleep(5000);
                    multipleStates = manyStates.STATE_IMAGE;
                    break;
                case STATE_IMAGE:
                    look(relicImage);
                    sleep(3000);
                    multipleStates = manyStates.STATE_DRIVEFAR;
                    break;
                case STATE_DRIVEFAR:
                    setPowers(0,absoluteMaxPower,0);
                    multipleStates = manyStates.STATE_TURN;
                    sleep((int)(tPlaceRelic*1000));
                    break;
                case STATE_TURN:
                    setPowers(0,0,absoluteMaxPower);
                    sleep(1000);
                    multipleStates = manyStates.STATE_REST;
                    break;
            }


        }
    }

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }

    public void motorInit(){
        lf = hardwareMap.dcMotor.get("leftF");
        rf = hardwareMap.dcMotor.get("rightF");
        lb = hardwareMap.dcMotor.get("leftB");
        rb = hardwareMap.dcMotor.get("rightB");

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm = hardwareMap.crservo.get("arm");
        color = hardwareMap.colorSensor.get("color");

        //three = hardwareMap.crservo.get("three");
        //four = hardwareMap.crservo.get("four");

        //lift = hardwareMap.dcMotor.get("lift");



    }


    public void moveArmLoop(){
        arm.setPower(-POWER);
        telemetry.addData("Red: ", color.red());
        telemetry.addData("Blue: ", color.blue());
        telemetry.addData("Arm power: ", arm.getPower());
        if (color.blue()/2 > color.red()) {
            setPowers(0,absoluteMaxPower,0);
            arm.setPower(POWER/2);
        }

        if (color.blue() < color.red()/2) {
            setPowers(0,-absoluteMaxPower,0);
            arm.setPower(POWER/2);
        }
        else{
            setPowers(0,0,0);
        }


    }


    public double look(VuforiaTrackable relicImage){
        RelicRecoveryVuMark goal = lookForRelicImage(relicImage);
        telemetry.addData("Goal: ", goal);
        sleep(100);
        if (goal.equals(RelicRecoveryVuMark.LEFT)){
            state = 2;
        }
        else if (goal.equals(RelicRecoveryVuMark.CENTER)) {
            state = 3;
        }
        else if (goal.equals(RelicRecoveryVuMark.RIGHT)) {
            state = 4;
        }
        telemetry.addData("Did it recognize: ", state);
        tPlaceRelic = state + 8; // or something idk
        telemetry.addData("time: ", tPlaceRelic);
        return tPlaceRelic;
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


    public VuforiaTrackable setUpOtherCamera() {

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


    public void setPowers(double xx, double yy, double rotation) {
        double x = Range.clip(xx, -1, 1);
        double y = - Range.clip(yy, -1, 1);

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }

        double rot = rotation;


        double r = Math.hypot(x, y);
        double angle = 0.0;

        double POW = Math.max(Math.hypot(x, y), Math.abs(rot));


        if (r > 0.1){
            angle = Math.atan2(y,x) - Math.PI / 4;
        }

        double vlf = r * Math.cos(angle) + rot;
        double vrf = r * Math.sin(angle) - rot;
        double vlb = r * Math.sin(angle) + rot;
        double vrb = r * Math.cos(angle) - rot;

        double maxPower = maxPow(vlf, vrf, vlb, vrb);

        vlf /= maxPower;
        vrf /= maxPower;
        vlb /= maxPower;
        vrb /= maxPower;


        lf.setPower(Math.pow(POW,2) * Range.clip(vlf, -1, 1));
        rf.setPower(-Math.pow(POW,2) * Range.clip(vrf, -1, 1));
        lb.setPower(Math.pow(POW,2) * Range.clip(vlb, -1, 1));
        rb.setPower(-Math.pow(POW,2) * Range.clip(vrb, -1, 1));

    }

    private double maxPow(double x, double y, double z, double w) {
        x = Math.abs(x);
        y = Math.abs(y);
        z = Math.abs(z);
        w = Math.abs(w);
        return Math.max(Math.max(x,y), Math.max(z,w));
    }


}
