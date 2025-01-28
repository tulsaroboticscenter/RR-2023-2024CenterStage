package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;

@Autonomous(name = "Blue Auto - Backdrop", group = "Competition")
@Disabled
public class BlueAutoDetect extends LinearOpMode {

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;

    FtcDashboard dashboard;

    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public BlueAutoDetect(){

    }

    //    @Override
    public void runOpMode() {
//        State runState = State.HIGH_JUNCTION_1;
        State runState = State.DETECT;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();


        drive.resetArm();
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup

        while (opModeIsActive()) {
            switch(runState){
                case DETECT:

                    // close the claw
                    drive.closeClaw();
                    sleep(500);

                    // drive forward to detect prop
                    drive.driveDistance(0, 33.5);
                    sleep(1000);

                    // check to see if the prop is on the left or right
//                    if(robot.sensorLeft.getDistance(DistanceUnit.CM) < 15) {
//                        runState = State.LEFT;
//                    } else if(robot.sensorRight.getDistance(DistanceUnit.CM) < 15) {
//                        runState = State.RIGHT;
//                    } else runState = State.CENTER;

                    break;

                case LEFT:
                    // Back away to avoid the prop
                    drive.driveDistance(180, 10);

                    // rotate towards left line
                    drive.pidRotate(90, 0.5);

                    // drive toward the backboard to place the purple pixel
                    drive.driveDistance(180, 15);

                    // strafe to place the purple pixel
                    drive.driveDistance(90, 5);

                    // Lift the arm
                    drive.raiseArm(robot.ARM_OUT);
                    sleep(1000);

                    // drive forward to knock the pixel over
                    robot.motorLF.setPower(0.1);
                    robot.motorLR.setPower(0.1);
                    robot.motorRR.setPower(0.1);
                    robot.motorRF.setPower(0.1);
                    sleep(50);

                    // back up towards backdrop
                    drive.driveDistance(180, 18);

                    // strafe into scoring position for the LEFT side
//                    drive.driveDistance(-90, 10);

                    // back into backdrop
//                    drive.driveDistance(180,5);

                    // drop the pixel
                    drive.liftLowJunction();

                    drive.raiseArm(robot.ARM_SCORE);
                    sleep(1000);
                    drive.openClaw();
                    sleep(1000);

                    // reset the arm
                    drive.resetArm();
                    sleep(1000);
                    drive.liftReset();
                    sleep(1000);


                    // park near the wall
                    drive.driveDistance(-90, 30);
                    sleep(1000);

                    runState = State.PARK;
                    break;

                case CENTER:
                    // drive forward to place the purple pixel
                    drive.driveDistance(180, 2);

                    // Lift the arm
                    drive.raiseArm(robot.ARM_OUT);
                    sleep(400);

                    // drive forward to knock the pixel over
                    robot.motorLF.setPower(0.1);
                    robot.motorLR.setPower(0.1);
                    robot.motorRR.setPower(0.1);
                    robot.motorRF.setPower(0.1);
                    sleep(50);
//                    drive.driveDistance(0, 1);

                    // back away from the pixel
                    drive.driveDistance(180, 5);

                    // rotate towards backdrop to score
                    drive.pidRotate(90, 0.5);

                    // back up towards backdrop
                    drive.driveDistance(180, 24);

                    // strafe into scoring position for the CENTER side
                    drive.driveDistance(90, 4);

                    // back into backdrop
                    drive.driveDistance(180,5);

                    // drop the pixel
                    drive.liftLowJunction();

                    drive.raiseArm(robot.ARM_SCORE);
                    sleep(2000);
                    drive.openClaw();
                    sleep(2000);

                    // reset the arm
                    drive.resetArm();
                    sleep(2000);
                    drive.liftReset();
                    sleep(2000);

                    // park near the wall
                    drive.driveDistance(-90, 30);

                    sleep(2000);

                    runState = State.PARK;
                    break;

                case RIGHT:
                    // back up to score on the right side
                    drive.driveDistance(180, 5);

                    // rotate towards backdrop to score
                    drive.pidRotate(90, 0.5);

                    // drive forward to place the purple pixel
                    drive.driveDistance(0, 4);

                    // Lift the arm
                    drive.raiseArm(robot.ARM_OUT);

                    // drive forward to place the purple pixel
                    drive.driveDistance(0, 1);

                    // back up towards backdrop
                    drive.driveDistance(180, 30);

                    // strafe into scoring position for the RIGHT side
                    drive.driveDistance(90, 4);

                    // back into backdrop
                    drive.driveDistance(180,5);

                    // drop the pixel
                    drive.liftLowJunction();

                    drive.raiseArm(robot.ARM_SCORE);
                    sleep(2000);
                    drive.openClaw();
                    sleep(2000);

                    // reset the arm
                    drive.resetArm();
                    sleep(1000);
                    drive.liftReset();
                    sleep(1000);

                    // park near the wall
                    drive.driveDistance(-90, 30);

                    sleep(1000);

                    runState = State.PARK;
                    break;

                case PARK:

                    drive.driveDistance(180, 4);
                    // wait for all resets to complete
                    sleep(4000);

                    runState = State.HALT;

                    break;


                case HALT:

                    // shut down all motors
                    drive.motorsHalt();

                    requestOpModeStop();    // request stoppage of the program

                    break;
            }   // end of switch(state)
        }   // end of while(opModeIsActive)

        requestOpModeStop();

        telemetry.addData("Path", "Complete");
        telemetry.update();

    } // end of opmode

    /*
     * Enumerate the states of the machine
     */
    enum State {
        DETECT, LEFT, RIGHT, CENTER, PARK, HALT
    }   // end of enum State


    /**
     * Initialize the TensorFlow Object Detection engine.
     */

}       //End Linear Op Mode