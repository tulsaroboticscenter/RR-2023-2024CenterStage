package org.firstinspires.ftc.teamcode.Hardware;

public class CSAutoParams {
    /*

    PARAMETER MAP - PLEASE READ BEFORE EDITING

    This file contains all the parameters for RRAuto.
    This file also contains all Lift encoder values for cone retrieval from the stack.

    If making a new auto, include "AutoParams params = new AutoParams();" within the class.

    All headings are converted to radians here, so there is no need to convert them in the main autonomous programs.

    All values are based on BlueTerminalAuto and are negated as needed in RedTerminalAuto
     */
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

    public final int WINCH_END_AUTO_POSITION = -50;
    public final int WINCH_PICKUP_POSITION = 0;
    public final int WINCH_SCORE_POSITION = 600;
    public final int WINCH_DUCK_POSITION = 800;

    public final int HANG_RESET = 0;
    public final int HANG_READY = 4250;
    public final int HANG_POSITION = 2000;

    public final int LIFT_RESET = 0;
    public final int LIFT_MIN_LOW = 0;
    public final int LIFT_MAX_HIGH = -2040;
    public final int LIFT_LOW_JUNCTION = -1000;
    public final int LIFT_SCORE_JUNCTION = -500;
    public final int LIFT_MID_JUNCTION = -1500;
    public final int LIFT_HIGH_JUNCTION = -2000;
    public final double LIFT_POWER = 1;

    public final double SERVO_GRAB_BOTTOM_OPEN = 0.0;
    public final double SERVO_GRAB_TOP_OPEN = 0.95;
    public final double SERVO_GRAB_BOTTOM_CLOSE = 0.25;
    public final double SERVO_GRAB_TOP_CLOSE = 0.75;

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

    public final double ARM_RESET = 0.5;    // TODO: Set this to the correct value
    public final double ARM_OUT = 0.09;      // TODO: Set this to the correct value
    public final double ARM_SCORE = 0.1;      // TODO: Set this to the correct value

}