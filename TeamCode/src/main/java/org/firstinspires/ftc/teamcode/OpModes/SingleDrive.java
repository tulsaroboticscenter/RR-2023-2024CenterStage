package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;
import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.tensorflow.lite.task.core.TaskJniUtils;


/**
#########################################

                               CONTROLS MAPPING

#########################################

 WINCH                            ##
 pickup = pad 2: dpad up
 score = pad 2: dpad left
 duck = pad 2: dpad down

 VIPER SLIDE                   ##
 floor = pad 2: A
 low section = pad 2: B
 middle section = pad 2: X
 high section = pad 2: Y

 HANG HOOK                ##


 ARM


**/
@TeleOp(name="SingleDrive", group="LinearOpMode")

/**

This is the DriveOpMode. This is the OpMode that is used for the driver-controlled portion, and
is also sometimes used for testing.

notes:

The armPosition for the high level of the team shipping hub is -0.588.

**/

public class SingleDrive extends LinearOpMode {

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    private final boolean pad2input = true;

    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;
    private int liftPosition = 0;

    private void moveLift() {
        robot.liftMotorLeft.setTargetPosition(liftPosition);
        robot.liftMotorLeft.setPower(1);

        robot.liftMotorRight.setTargetPosition(liftPosition);
        robot.liftMotorRight.setPower(1);
    }


    private ElapsedTime buttonPress = new ElapsedTime();

    private boolean IsOverrideActivated = false;

    public void runOpMode() {
        robot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        drive.pickupArm();
        drive.openClaw();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        drive.haltandresetencoders();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        buttonPress.reset();
        double stickDrive = 0;
        double turn = 0;
        double strafe = 0;
        double leftPower = 0;
        double rightPower = 0;
        int dpadCount = 0;
//        double armUpDown;


        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * StrafeSpeed;

            drive.StrafeDrive(stickDrive, turn, strafe);

            // LIFT

            if (this.gamepad1.a) {
                new Thread(() -> {
                    try {
                        drive.closeClaw();
                        Thread.sleep(200);
                        drive.pickupArm();
                        liftPosition = robot.LIFT_RESET;
                        moveLift();
                        drive.openClaw();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                }).start();


            } else if (this.gamepad1.b) {
                new Thread(() -> {
                    try {
                        drive.closeClaw();
                        Thread.sleep(200);
                        liftPosition = robot.LIFT_LOW_JUNCTION;
                        moveLift();
                        Thread.sleep(500);
                        robot.armServo.setPosition(robot.ARM_SCORE);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();

            } else if (this.gamepad1.x) {
                new Thread(() -> {
                    try {
                        drive.closeClaw();
                        Thread.sleep(200);
                        liftPosition = robot.LIFT_MID_JUNCTION;
                        moveLift();
                        Thread.sleep(500);
                        robot.armServo.setPosition(robot.ARM_SCORE);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
            } else if (this.gamepad1.y) {
                new Thread(() -> {
                    try {
                        drive.closeClaw();
                        Thread.sleep(200);
                        liftPosition = robot.LIFT_HIGH_JUNCTION;
                        moveLift();
                        Thread.sleep(500);
                        robot.armServo.setPosition(robot.ARM_SCORE);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }).start();
            }

            // INTAKE


            if (gamepad1.right_trigger > 0.05) {
                robot.intakeMotor.setPower(0.4);
                robot.intakeServo.setPower(1);
            } else if (gamepad1.left_trigger > 0.05) {
                robot.intakeMotor.setPower(-gamepad1.left_trigger);
                robot.intakeServo.setPower(-1);
            } else {
                robot.intakeMotor.setPower(0);
                robot.intakeServo.setPower(0);
            }


            // CLAW

            if (gamepad1.dpad_down) {
                robot.clawServoBottom.setPosition(robot.BOTTOM_CLAW_OPEN);
            }

            if (gamepad1.dpad_up) {
                robot.clawServoTop.setPosition(robot.TOP_CLAW_OPEN);
            }

            // DRIVE POWER

            if (gamepad1.left_bumper) {
                DriveSpeed = 1;
                StrafeSpeed = 1;
                TurnSpeed = 1;
            } else {
                DriveSpeed = 0.5;
                StrafeSpeed = 0.5;
                TurnSpeed = 0.5;
            }

            if (liftPosition > 0) {
                liftPosition = 0;
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("elevator = ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}

