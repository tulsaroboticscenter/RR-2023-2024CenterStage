/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Libs.DriveMecanumFTCLib;
import org.firstinspires.ftc.teamcode.Hardware.HWProfile2;


@TeleOp(name="__System Test", group="Test Mode")

public class SystemTest extends LinearOpMode {

    // Declare OpMode members.
    private final static HWProfile2 robot = new HWProfile2();
    private final LinearOpMode opMode = this;
    public DriveMecanumFTCLib drive = new DriveMecanumFTCLib(robot, opMode);

    private final ElapsedTime runtime = new ElapsedTime();
    private final double clawPositionValue = robot.TOP_CLAW_OPEN;
    private final int elevatorPosition = 0;

    private final int hangPosition = 0;

    private int liftPosition = 0;

    private final int winchPosition = 0;

    private double topClawPosition = 0.1;
    private double bottomClawPosition = 0.9;

    /**

     VALUES

     Claw top: 0.15 close, 0 open
     claw bottom: 0.85 close, 1 open
     arm: pickup 0, score 0.3

     **/

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        /**leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");**/

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        /**leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);**/

        double strafe;
        double tempPosition = 0.3;
        // Wait for the game to start (driver presses PLAY)

        robot.clawServoTop.setPosition(topClawPosition);
        robot.clawServoBottom.setPosition(bottomClawPosition);
        robot.armServo.setPosition(1);


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.dpad_up) {
                robot.motorLF.setPower(1);
            } else {
                robot.motorLF.setPower(0);
            }

            if(gamepad1.dpad_down) {
                robot.motorLR.setPower(1);
            } else {
                robot.motorLR.setPower(0);
            }

            if(gamepad1.dpad_left) {
                robot.motorRF.setPower(1);
            } else {
                robot.motorRF.setPower(0);
            }

            if(gamepad1.dpad_right) {
                robot.motorRR.setPower(1);
            } else {
                robot.motorRR.setPower(0);
            }

            if(gamepad1.y) {
                robot.motorLF.setPower(1);
                robot.motorLR.setPower(1);
                robot.motorRF.setPower(1);
                robot.motorRR.setPower(1);
            } else {
                robot.motorLF.setPower(0);
                robot.motorLR.setPower(0);
                robot.motorRF.setPower(0);
                robot.motorRR.setPower(0);
            }

            if (gamepad1.a) {
                liftPosition += 10;
            } else if (gamepad1.b) {
                liftPosition -= 10;
            }

            if (gamepad1.right_bumper) {
                robot.intakeMotor.setPower(1);
            }

            if (gamepad1.x) {
                robot.armServo.setPosition(0.7);

            }

            //robot.clawServoTop.setPosition(topClawPosition);
            //robot.clawServoBottom.setPosition(bottomClawPosition);

            if (liftPosition < 0) {
                liftPosition = 0;
            }

            robot.liftMotorLeft.setTargetPosition(liftPosition);
            robot.liftMotorLeft.setPower(1);

            robot.liftMotorRight.setTargetPosition(liftPosition);
            robot.liftMotorRight.setPower(1);

            robot.clawServoTop.setPosition(topClawPosition);
            robot.clawServoBottom.setPosition(bottomClawPosition);

//            robot.armServo.setPosition(0); // this was trying to go down at the end of the loop, regardless of if gamepad1.x was pressed

            telemetry.addData("liftPosition = ", liftPosition);
            telemetry.addData("Claw Bottom Position = ", robot.clawServoBottom.getPosition());
            telemetry.addData("Actual Claw Top Position = ", robot.clawServoTop.getPosition());
            telemetry.addData("Right Front Encoder = ", robot.motorRF.getCurrentPosition());
            telemetry.addData("Right Rear Encoder = ", robot.motorRR.getCurrentPosition());
            telemetry.addData("Left Front Encoder = ", robot.motorLF.getCurrentPosition());
            telemetry.addData("Left Rear Encoder = ", robot.motorLR.getCurrentPosition());
            telemetry.addData("Elevator Encoder = ", robot.liftMotorLeft.getCurrentPosition());
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}
