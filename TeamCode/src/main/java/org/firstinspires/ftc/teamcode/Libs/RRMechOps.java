package org.firstinspires.ftc.teamcode.Libs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Hardware.RRHWProfile;

public class RRMechOps{

    public RRHWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;
    private HWProfile2 robot2 = new HWProfile2();

    private DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot2, opMode);

    /*
     * Constructor method
     */
    public RRMechOps(RRHWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close DriveMecanum constructor Method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */


    public void closeTopClaw(){
        robot.clawServoTop.setPosition(params.SERVO_GRAB_TOP_CLOSE);
    }   // end of closeClaw method

    public void closeBottomClaw(){
        robot.clawServoBottom.setPosition(params.SERVO_GRAB_BOTTOM_CLOSE);
    }   // end of closeClaw method

    public void openTopClaw(){
        robot.clawServoTop.setPosition(params.SERVO_GRAB_TOP_OPEN);
    }   // end of openClaw method

    public void openBottomClaw(){
        robot.clawServoBottom.setPosition(params.SERVO_GRAB_BOTTOM_OPEN);
    }   // end of openClaw method



    public void raiseArm(double position){
        robot.armServo.setPosition(position);
    }

    public void resetArm(){
        closeBottomClaw();
        closeTopClaw();
        opMode.sleep(100);
        robot.armServo.setPosition(params.ARM_RESET);
    }

    public void deployArm(){
        resetArm();
        liftLowJunction();
        opMode.sleep(300);
        robot.armServo.setPosition(params.ARM_SCORE);
    }

    public void liftReset(){
        resetArm();
        opMode.sleep(200);
        robot.liftMotorL.setPower(params.LIFT_POWER);
        robot.liftMotorR.setPower(params.LIFT_POWER);
        robot.liftMotorL.setTargetPosition(params.LIFT_RESET);
        robot.liftMotorR.setTargetPosition(params.LIFT_RESET);

        openBottomClaw();
        openTopClaw();
    }   // end of liftReset method

    public void liftLowJunction(){
        robot.liftMotorL.setPower(params.LIFT_POWER);
        robot.liftMotorR.setPower(params.LIFT_POWER);
        robot.liftMotorL.setTargetPosition(params.LIFT_LOW_JUNCTION);
        robot.liftMotorR.setTargetPosition(params.LIFT_LOW_JUNCTION);
    }   // end of liftLowJunction method

    public void liftScore(){
        robot.liftMotorL.setPower(params.LIFT_POWER);
        robot.liftMotorR.setPower(params.LIFT_POWER);
        robot.liftMotorL.setTargetPosition(params.LIFT_SCORE_JUNCTION);
        robot.liftMotorR.setTargetPosition(params.LIFT_SCORE_JUNCTION);
    }   // end of liftLowJunction method

    public void liftMidJunction(){
        robot.liftMotorL.setPower(params.LIFT_POWER);
        robot.liftMotorR.setPower(params.LIFT_POWER);
        robot.liftMotorL.setTargetPosition(params.LIFT_MID_JUNCTION);
        robot.liftMotorR.setTargetPosition(params.LIFT_MID_JUNCTION);
    }   // end of liftMidJunction method

    public void liftHighJunction(){
        robot.liftMotorL.setPower(params.LIFT_POWER);
        robot.liftMotorR.setPower(params.LIFT_POWER);
        robot.liftMotorL.setTargetPosition(params.LIFT_HIGH_JUNCTION);
        robot.liftMotorR.setTargetPosition(params.LIFT_HIGH_JUNCTION);
    }   // end of liftHighJunction method

    public void lift(int position) {
        drive.closeClaw();
        robot.liftMotorL.setPower(1);
        robot.liftMotorR.setPower(1);
        opMode.sleep(200);
        robot.liftMotorL.setTargetPosition(position);
        robot.liftMotorR.setTargetPosition(position);
        opMode.sleep(500);
        robot.armServo.setPosition(robot2.ARM_SCORE);
    }

    public void yellowPixelRelease() {
        lift(robot2.LIFT_LOW_JUNCTION);
        opMode.sleep(500);
        robot.clawServoTop.setPosition(robot2.TOP_CLAW_OPEN);
        opMode.sleep(100);
    }

    public void purplePixelRelease() {
        lift(robot2.LIFT_PURPLE_RELEASE);
        opMode.sleep(500);
        robot.clawServoBottom.setPosition(robot2.BOTTOM_CLAW_OPEN);
        opMode.sleep(100);
    }



}   // close the RRMechOps class