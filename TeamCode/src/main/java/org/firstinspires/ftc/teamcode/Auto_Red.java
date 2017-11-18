package org.firstinspires.ftc.teamcode;

/**
 *  This current autonomous will knock off the jewel (30 points),
 *  Decipher the pictograph and place the block in the respective column (45 points),
 *  Drive into the safe zone (10 points)
 *  For a total of 85 points (the maximum possible)
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous

public class Auto_Red extends LinearOpMode {

    DcMotor fL;
    DcMotor bL;
    DcMotor fR;
    DcMotor bR;
    DcMotor lS;
    Servo left;
    Servo right;
    Servo arm;
    ColorSensor color;
    state state358;
    VuforiaLocalizer vuforia;

    double dPosition = 0.3;
    double oPosition = 1;

    enum state {
        DECIPHER, JEWEL, SAFEZONE, GLYPH, STOP
    }

    public void runOpMode() throws InterruptedException {

        fL = hardwareMap.dcMotor.get("frontLeft");
        bL = hardwareMap.dcMotor.get("backLeft");
        fR = hardwareMap.dcMotor.get("frontRight");
        bR = hardwareMap.dcMotor.get("backRight");
        lS = hardwareMap.dcMotor.get("linearSlide");
        left = hardwareMap.servo.get("left");
        right = hardwareMap.servo.get("right");
        arm = hardwareMap.servo.get("arm");
        color = hardwareMap.colorSensor.get("color");
        state358 = state.DECIPHER;

        fL.setDirection(DcMotor.Direction.REVERSE);
        left.setDirection(Servo.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);

        double POWER = .5;

        arm.setPosition(dPosition);
        left.setPosition(0);
        right.setPosition(0);

        //Setting up Camera and Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXzW9CD/////AAAAGTPAtr9HRUXZmowtd9p0AUwuXiBVONS/c5x1q8OvjMrQ8/XJGxEp0TP9Kl8PvqSzeXOWIvVa3AeB6MyAQboyW/Pgd/c4a4U/VBs1ouUsVBkEdbaq1iY7RR0cjYr3eLwEt6tmI37Ugbwrd5gmxYvOBQkGqzpbg2U2bVLycc5PkOixu7PqPqaINGZYSlvUzEMAenLOCxZFpsayuCPRbWz6Z9UJfLeAbfAPmmDYoKNXRFll8/jp5Ie7iAhSQgfFggWwyiqMRCFA3GPTsOJS4H1tSiGlMjVzbJnkusPKXfJ0dK3OH9u7ox9ESpi91T0MemXw3nn+/6QRvjGtgFH+wMDuQX7ta89+yW+wqdXX9ZQu8BzY";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();

        waitForStart();

        while (opModeIsActive()) {
            switch(state358) {

                //INCOMPLETE: Assign Value to L, M, R perhaps?
                case DECIPHER:
                    state358 = state.JEWEL;
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    while (vuMark != RelicRecoveryVuMark.LEFT || vuMark != RelicRecoveryVuMark.CENTER || vuMark != RelicRecoveryVuMark.RIGHT) {
                        telemetry.addData("VuMark", "not visible");
                    }
                    telemetry.addData("VuMark", "%s visible", vuMark);
                    break;

                case JEWEL:
                    state358 = state.SAFEZONE;
                    if (color.blue()/2 > color.red()) {
                        fL.setPower(POWER);
                        bL.setPower(POWER);
                        fR.setPower(POWER);
                        bR.setPower(POWER);
                        sleep(200);
                        arm.setPosition(oPosition);
                    }

                    if (color.blue() < color.red()/2) {
                        fL.setPower(-POWER);
                        bL.setPower(-POWER);
                        fR.setPower(-POWER);
                        bR.setPower(-POWER);
                        sleep(200);
                        arm.setPosition(oPosition);
                    }

                    else {
                        fL.setPower(0);
                        bL.setPower(0);
                        fR.setPower(0);
                        bR.setPower(0);
                    }
                    break;

                case SAFEZONE:
                    state358 = state.GLYPH;
                    fL.setPower(POWER);
                    bL.setPower(POWER);
                    fR.setPower(POWER);
                    bR.setPower(POWER);
                    sleep(100);
                    break;

                case GLYPH:
                    state358 = state.STOP;
                    fL.setPower(0);
                    bL.setPower(0);
                    fR.setPower(0);
                    bR.setPower(0);
                    sleep(200);
                    break;

                case STOP:
                    fL.setPower(0);
                    bL.setPower(0);
                    fR.setPower(0);
                    bR.setPower(0);

            }
        }
    }
}