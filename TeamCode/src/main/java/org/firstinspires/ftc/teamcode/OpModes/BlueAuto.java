package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;

@Autonomous(name = "Blue (left)", group = "Competition")

public class BlueAuto extends LinearOpMode {

    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;

    FtcDashboard dashboard;

    private static final String TFOD_MODEL_ASSET = "PP_Generic_SS.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "circle",
            "star",
            "triangle"
    };

    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);
    int position = 3;

    public BlueAuto(){

    }

    //    @Override
    public void runOpMode() {
//        State runState = State.HIGH_JUNCTION_1;
        State runState = State.TEST;

        telemetry.addData("Robot State = ", "NOT READY");
        telemetry.update();

        robot.init(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        TelemetryPacket dashTelemetry = new TelemetryPacket();

        /*
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.

         */


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        waitForStart();

        if(isStopRequested()) requestOpModeStop();   // user requested to abort setup

        while (opModeIsActive()) {
            switch(runState){
                case TEST:

                    drive.closeClaw();
                    sleep(500);
                    drive.driveDistance(90,48);
                    sleep(500);
                    drive.openClaw();

                    runState = State.HALT;
                    break;

                case LEVEL_ADJUST:
                    runState = State.HIGH_JUNCTION_1;
                    break;

                case HIGH_JUNCTION_1:
                    // starting from start position, close claw

                    break;

                case PARK:

                    robot.MIN_STRAFE_POWER = 0.45;


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
        TEST, ALLIANCE_SELECT, HIGH_JUNCTION_1, CONE_2, LOW_JUNCTION_2, CONE_3, LOW_JUNCTION_3, MID_JUNCTION_3, LEVEL_ADJUST, PARK, HALT, SET_DISTANCES
    }   // end of enum State


    /**
     * Initialize the TensorFlow Object Detection engine.
     */

}       //End Linear Op Mode