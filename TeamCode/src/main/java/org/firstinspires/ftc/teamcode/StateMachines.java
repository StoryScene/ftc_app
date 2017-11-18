package org.firstinspires.ftc.teamcode;

/**
 * Created by lawrencemao on 10/8/17.
 */
/**
public class StateMachines extends OpMode{
    private enum state{
        STATE_INITIAL, //initialize
        STATE_VUFORIA_SERVO, //phone decrypts and servo dropped down
        STATE_COLOR, //servo sees color
        STATE_TURN, //turn and adjust
        STATE_DRIVE_TO_CRYPT, //drive to crypt and turn right 90 degrees
        STATE_GLYPH, //drive forward and place glyph into cryptobox
        STATE_BALANCE, //drive back and balance onto balancing stone
        STATE_STOP,
    }

    //Define driving paths as pairs of relative wheel movements in inches (left,right,speed)
    //Note: The current values are incorrect and will have to be adjusted
    final PathSeg[] mTurn = {
            new PathSeg(  0.0,  3.0,  0.2),  //Left
            new PathSeg( 60.0, 60.0, 0.9),   //Forward
            new PathSeg(  1.0,  0.0,  0.2),  //Right
    };

    final double COUNTS_PER_INCH = 240;      //Number of encoder counts per inch of wheel travel

    final double WHITE_THRESHOLD = 0.5;      //When seeing white line on the ground
    final double RANGE_THRESHOLD = 0.5;      //When going to hit wall with optical distance sensor
    final double SERVO_DEPOLY    = 1.0;      //Servo deploys
    final double SERV_RETRACT    = 0.1;      //Servo retracts

    //----------------------------------------------------------
    // Robot device objects
    //----------------------------------------------------------
    public DcMotor lfMotor;
    public DcMotor lbMotor;
    public DcMotor rfMotor;
    public DcMotor rbMotor;
    public LightSensor mLight;
    public OpticalDistanceSensor mDistance;



} **/