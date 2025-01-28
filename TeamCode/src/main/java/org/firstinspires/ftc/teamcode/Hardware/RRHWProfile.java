package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class RRHWProfile {


    public CSAutoParams params;

    /*
     * Hardware devices
     */

//    public RevIMU imu = null;

    public DcMotorEx liftMotorL = null;
    public DcMotorEx liftMotorR = null;

    public Servo clawServoTop;
    public Servo clawServoBottom;

    public Servo armServo;

    public HardwareMap hwMap;

    /*
     * Declare Odometry hardware
     */

    /* Constructor */
    public RRHWProfile(CSAutoParams myParams) {
        params = myParams;
    }

    public void init(HardwareMap ahwMap) {

        params = new CSAutoParams();
        hwMap = ahwMap;

        clawServoTop = ahwMap.get(Servo.class, "clawServoTop");
//        clawServoTop.setPosition(params.SERVO_GRAB_OPEN);

        clawServoBottom = ahwMap.get(Servo.class, "clawServoBottom");
//        clawServoBottom.setPosition(params.SERVO_GRAB_OPEN);

        armServo = ahwMap.get(Servo.class, "armServo");
//        armServo.setPosition(0.03);

        liftMotorL = ahwMap.get(DcMotorEx.class,"liftMotorL");
        liftMotorL.setDirection(DcMotor.Direction.REVERSE);
        liftMotorL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorL.setTargetPosition(0);
        liftMotorL.setPower(0);
        liftMotorL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorR = ahwMap.get(DcMotorEx.class,"liftMotorR");
        liftMotorR.setDirection(DcMotor.Direction.FORWARD);
        liftMotorR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setTargetPosition(0);
        liftMotorR.setPower(0);
        liftMotorR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
