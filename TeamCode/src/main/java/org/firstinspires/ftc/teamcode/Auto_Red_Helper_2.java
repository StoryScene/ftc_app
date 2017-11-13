package org.firstinspires.ftc.teamcode;

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
public class Auto_Red_Helper_2 extends LinearOpMode {
    Servo jewel;
    ColorSensor color;
    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;
    int position = 0;
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {
        jewel = hardwareMap.servo.get("jewel");
        color = hardwareMap.colorSensor.get("color");
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        //setting up camera and vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AXzW9CD/////AAAAGTPAtr9HRUXZmowtd9p0AUwuXiBVONS/c5x1q8OvjMrQ8/XJGxEp0TP9Kl8PvqSzeXOWIvVa3AeB6MyAQboyW/Pgd/c4a4U/VBs1ouUsVBkEdbaq1iY7RR0cjYr3eLwEt6tmI37Ugbwrd5gmxYvOBQkGqzpbg2U2bVLycc5PkOixu7PqPqaINGZYSlvUzEMAenLOCxZFpsayuCPRbWz6Z9UJfLeAbfAPmmDYoKNXRFll8/jp5Ie7iAhSQgfFggWwyiqMRCFA3GPTsOJS4H1tSiGlMjVzbJnkusPKXfJ0dK3OH9u7ox9ESpi91T0MemXw3nn+/6QRvjGtgFH+wMDuQX7ta89+yW+wqdXX9ZQu8BzY";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        //Params LEFT CENTER RIGHT into integers
        while (vuMark != RelicRecoveryVuMark.LEFT || vuMark != RelicRecoveryVuMark.CENTER || vuMark != RelicRecoveryVuMark.RIGHT) {
            telemetry.addData("VuMark", "not visible");
        }

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            position = 1;
        }
        if (vuMark == RelicRecoveryVuMark.CENTER) {
            position = 2;
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            position = 3;
        }

        telemetry.addData("VuMark", "%s visible", vuMark);
        telemetry.addData("RED: ", color.red());
        telemetry.addData("BLUE: ", color.blue());
        telemetry.addData("Pos: ", position);
    }

}