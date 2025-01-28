package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;


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
@TeleOp(name="__MecanumWheelDrive__", group="LinearOpMode")

/**

 This is the DriveOpMode. This is the OpMode that is used for the driver-controlled portion, and
 is also sometimes used for testing.

 notes:

 The armPosition for the high level of the team shipping hub is -0.588.

 **/

public class __MecanumWheelDrive__ extends LinearOpMode
{

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    private DistanceSensor sensorColorRange;
    private Servo servoTest;
    private final boolean pad2input = true;

    private double DriveSpeed = 1;
    private double TurnSpeed = 1;
    private double StrafeSpeed = 1;

    private boolean IsOverrideActivated = false;

    public void runOpMode()
    {
        robot.init(hardwareMap);
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//        drive.haltandresetencoders();
        //runtime.reset();

        // run until the end of the match (driver presses STOP)
        double stickDrive = 0;
        double turn = 0;
        double strafe = 0;
        double leftPower = 0;
        double rightPower = 0;
//        double armUpDown;
        int armPosition = 0;
        int hangPosition = 0;

        while (opModeIsActive()) {
            stickDrive = this.gamepad1.left_stick_y * DriveSpeed;
            turn = this.gamepad1.right_stick_x * TurnSpeed;
            strafe = this.gamepad1.left_stick_x * StrafeSpeed;

           drive.StrafeDrive(stickDrive, turn, strafe);

           if (gamepad1.y) {

           } else if (gamepad1.a) {

           } else if (gamepad1.x) {

           } else if (gamepad1.b) {

           } else if (gamepad1.a) {

           }


            if(this.gamepad2.a) {
                armPosition = robot.LIFT_RESET;


                robot.armServo.setPosition(robot.ARM_PICKUP);

            } else if(this.gamepad2.b){
                armPosition = robot.LIFT_LOW_JUNCTION;
                sleep(300);
                robot.armServo.setPosition(robot.ARM_SCORE);

            } else if(this.gamepad2.x) {
                armPosition = robot.LIFT_MID_JUNCTION;
                sleep(300);
                robot.armServo.setPosition(robot.ARM_SCORE);
            }

            /**
            if (gamepad2.dpad_up){
                robot.droneServo.setPosition(robot.DRONE_FIRE);
            }

            if(gamepad2.dpad_down){
                robot.droneServo.setPosition(robot.DRONE_LOAD);
            }
            **/

            if (gamepad1.left_bumper) {
                DriveSpeed = 1;
                StrafeSpeed = 1;
                TurnSpeed = 1;
            } else {
                DriveSpeed = 0.5;
                StrafeSpeed = 0.5;
                TurnSpeed = 0.5;
            }

            if (gamepad1.dpad_up) {
                hangPosition = robot.HANG_READY;
            } else if (gamepad1.dpad_down){
                hangPosition = robot.HANG_RESET;
            } else if (gamepad1.dpad_left || gamepad1.dpad_right) {
                hangPosition = robot.HANG_POSITION;
            }

            if(this.gamepad2.right_stick_button && !IsOverrideActivated) {
                IsOverrideActivated = true;
                sleep(200);
            } else if(this.gamepad2.right_stick_button && IsOverrideActivated) {
                robot.liftMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                IsOverrideActivated = false;
                sleep(200);
            }

            if (gamepad2.left_trigger > 0.5) {
                drive.openClaw();
            } else if (gamepad2.right_trigger > 0.5) {
                drive.closeClaw();
            }

            robot.liftMotorLeft.setPower(1);
            robot.liftMotorLeft.setTargetPosition(armPosition);



//            armUpDown = this.gamepad2.left_stick_y * ArmUpDownSpeed;

//            robot.elevatorMotor.setPower(1);
//            scout.elevatorMotor.setTargetPosition(armPosition);

            //drive.StrafeDrive(stickDrive, turn, strafe);
            //telemetry.addData("bruhdatderepositionrightdere = ", trapdoor);
            telemetry.addData("Status", "Running");
            telemetry.addData("elevator = ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();

        }
    }
}
