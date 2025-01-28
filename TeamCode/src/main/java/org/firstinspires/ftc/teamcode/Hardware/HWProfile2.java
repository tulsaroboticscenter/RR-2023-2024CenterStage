package org.firstinspires.ftc.teamcode.Hardware;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class HWProfile2 {


    /*
     * Constants
     */
    public final double PIVOT_SPEED = 0.5;
    public final double COUNTS_PER_ROTATION = 28;
    public final double GB_COUNTS_PER_ROTATION = 28;    // goBilda encoder value
    public final double MIN_PIDROTATE_POWER = 0.10;

    public double GOING_UP_SPEED = 0.1;
    public double GOING_DOWN_SPEED = -0.1;

    /*
     *  Constants & variables for wheel parameters
     */
    public final double DRIVE_TICKS_PER_INCH = 32;

    public final double STRAFE_FACTOR = 0.9;

    public final int WINCH_PICKUP_POSITION = 0;
    public final int WINCH_SCORE_POSITION = 560;
    public final int WINCH_DUCK_POSITION = 800;

    public final int HANG_RESET = 0;
    public final int HANG_READY = 4150;
    public final int HANG_POSITION = 2000;

    public final double DRONE_LOAD = 0.5;
    public final double DRONE_FIRE = 1;

    public final int LIFT_RESET = 0;
    public final int LIFT_MIN_LOW = 0;
    public final int LIFT_MAX_HIGH = -2040;

    public final int LIFT_PURPLE_RELEASE = -500;
    public final int LIFT_LOW_JUNCTION = -1000;
    public final int LIFT_MID_JUNCTION = -1500;
    public final int LIFT_HIGH_JUNCTION = -2000;

    public final double LIFT_POWER = 1;
    public final double TOP_CLAW_OPEN = 0.95;
    public final double TOP_CLAW_CLOSE = 0.85;

    public final double BOTTOM_CLAW_OPEN = 0;
    public final double BOTTOM_CLAW_CLOSE = 0.12;



    public final int ARM_FLIP_POSITION = 1900;

    public final double RIGHT_DRIVE_CORRECTION_MULTIPLIER = 1.4;
    public final double LEFT_DRIVE_CORRECTION_MULTIPLIER = 1.2;

    public final double MAX_DRIVING_POWER = 1;

    public double MIN_STRAFE_POWER = 0.35;

    public final double PID_Kp = 0.08;
    public final double PID_Ki = 0.01;
    public final double PID_Kd = 0.000001;
    public final double PID_MIN_SPEED = 0.05;
    public final double PID_ROTATE_ERROR = 1;

    public final double DRIVE_Kp = 0.05;
    public final double DRIVE_Ki = 0.01;
    public final double DRIVE_Kd = 0.31;

    public final double ARM_PICKUP = 0.92;    // TODO: Set this to the correct value
    public final double ARM_OUT = 0.09;      // TODO: Set this to the correct value
    public final double ARM_SCORE = 0.6;      // TODO: Set this to the correct value

    /*
     * Hardware devices
     */

    public RevIMU imu = null;


    public DcMotorEx motorLF;
    public DcMotorEx motorLR;
    public DcMotorEx motorRF;
    public DcMotorEx motorRR;

    //public DcMotorEx hangMotor;
    //public DcMotorEx winchMotor;
    //public DcMotor liftMotorLeft = null;

    public Motor liftMotorLeft;
    public Motor liftMotorRight;

    // grab the internal DcMotor object


    //public DcMotorEx liftMotorRight = null;

    public Servo clawServoTop;
    public Servo clawServoBottom;
    public Servo armServo;
    //public Servo droneServo;

    public CRServo intakeServo;

    public DcMotorEx intakeMotor;



//    public MecanumDrive mecanum = null;

    HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public HWProfile2() {
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        motorLF = ahwMap.get(DcMotorEx.class,"leftFront");
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLF.setPower(0);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorLR = ahwMap.get(DcMotorEx.class,"leftRear");
        motorLR.setDirection(DcMotor.Direction.REVERSE);
        motorLR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorLR.setPower(0);
        motorLR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRF = ahwMap.get(DcMotorEx.class,"rightFront");
        motorRF.setDirection(DcMotor.Direction.FORWARD);
        motorRF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRF.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRF.setPower(0);
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorRR = ahwMap.get(DcMotorEx.class,"rightRear");
        motorRR.setDirection(DcMotor.Direction.FORWARD);
        motorRR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorRR.setPower(0);
        motorRR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorRight = new Motor(ahwMap, "liftMotorR");
        liftMotorRight.setInverted(false);
        liftMotorRight.resetEncoder();
        liftMotorRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        liftMotorLeft = new Motor(ahwMap, "liftMotorL");
        liftMotorLeft.setInverted(true);
        liftMotorLeft.resetEncoder();
        liftMotorLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        MotorGroup Slides = new MotorGroup(liftMotorRight, liftMotorLeft);


        //drivebase init
//        mecanum = new MecanumDrive(motorLF, motorRF, motorLR, motorRR);

        imu = new RevIMU(ahwMap);
        imu.init();

        clawServoTop = ahwMap.get(Servo.class, "clawServoTop");
        clawServoBottom = ahwMap.get(Servo.class, "clawServoBottom");
//        clawServo.setPosition(SERVO_GRAB_OPEN);

        armServo = ahwMap.get(Servo.class, "armServo");

        intakeServo = ahwMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
        liftMotorLeft = ahwMap.get(DcMotorEx.class,"liftMotorL");
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setTargetPosition(0);
        liftMotorLeft.setPower(0);
        liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorRight = ahwMap.get(DcMotorEx.class,"liftMotorR");
        liftMotorRight.setDirection(DcMotor.Direction.FORWARD);
        liftMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setTargetPosition(0);
        liftMotorRight.setPower(0);
        liftMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        **/


        intakeMotor = ahwMap.get(DcMotorEx.class,"intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setPower(0);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/**
        winchMotor = ahwMap.get(DcMotorEx.class,"winchMotor");
        winchMotor.setDirection(DcMotor.Direction.REVERSE);
        winchMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        winchMotor.setTargetPosition(0);
        winchMotor.setPower(0);
        winchMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        winchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hangMotor.setPower(1);
        hangMotor.setTargetPosition(HANG_RESET);

        winchMotor.setPower(0.5);
        winchMotor.setTargetPosition(WINCH_PICKUP_POSITION);
        **/

        /* Webcam device will go here */
//        webcam = hwMap.get(WebcamName.class, "Webcam 1");
    }


    /**
     * The RunMode of the motor.
     */
    public enum RunMode {
        VelocityControl, PositionControl, RawPower
    }
}
