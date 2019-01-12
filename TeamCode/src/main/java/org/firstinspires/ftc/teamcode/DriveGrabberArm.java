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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Everything Arcade Drive", group="Linear Opmode")
//@Disabled
public class DriveGrabberArm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor midarm = null;
    private DcMotor lifter = null;
    private CRServo grabber1 = null;
    private CRServo grabber2 = null;
    //private DcMotor basearm = null;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        midarm  = hardwareMap.get(DcMotor.class, "mid_arm");
        lifter = hardwareMap.get(DcMotor.class, "lifter");
        grabber1 = hardwareMap.get(CRServo.class,  "grabber1");
        grabber2 = hardwareMap.get(CRServo.class,  "grabber2");
        //basearm = hardwareMap.get(DcMotor.class, "base_arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        midarm.setDirection(DcMotor.Direction.FORWARD);
        lifter.setDirection(DcMotor.Direction.FORWARD);
        grabber1.setDirection(CRServo.Direction.FORWARD);
        grabber2.setDirection(CRServo.Direction.REVERSE);

        /*leftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        //basearm.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower;
        double rightPower;
        double turn;
        double speedChange;//driving
        double armSpeedChange;//midarm
        double midarmPower = 0;
        double lifterPower = 0;
        double grabberPower;
        boolean useMidarm = FALSE;
        int starPositionArm = midarm.getCurrentPosition();
        int currentPositionArm = starPositionArm - midarm.getCurrentPosition();
        int targetPositionArm;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;
            //double midarmPower;
            //double basearmPower;
            midarmPower = 0;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
           // double turn  =  gamepad1.right_stick_x;
           // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            speedChange = 1-(gamepad1.right_trigger * 0.8);
            if(gamepad1.x) {
                leftPower = 1 * speedChange;
                rightPower = -1 * speedChange;
            } else if(gamepad1.b) {
                leftPower = -1 * speedChange;
                rightPower = 1 * speedChange;
            } else if(gamepad1.y) {
                leftPower = -1 * speedChange;
                rightPower = -1 * speedChange;
            } else if(gamepad1.a) {
                leftPower = 1 * speedChange;
                rightPower = 1 * speedChange;
            } else {
                turn = gamepad1.left_stick_x;
                leftPower  = (gamepad1.left_stick_y - turn) * speedChange ;
                rightPower = (gamepad1.left_stick_y + turn) * speedChange;
            }


            //midarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armSpeedChange = 1-(gamepad2.right_trigger * 0.4);
            if (gamepad2.b && gamepad2.y) {
                lifterPower = 0.9;//for raising I commented this out untill we get the lifter
            } else if(gamepad2.x && gamepad2.a){
                lifterPower = -0.8;//for lowering
            } else if(TRUE){
                lifterPower = 0.0;
            }
                midarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                midarmPower=(0.8 * gamepad2.left_stick_y) * armSpeedChange;//*0.5




            if (gamepad2.right_bumper) {
                grabberPower = 0.97;
            } else if (gamepad2.left_bumper) {
                grabberPower = -0.97;
            } else {
                grabberPower = 0;
            }









            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            //if(useMidarm == TRUE){
                midarm.setPower(midarmPower);
            //}
            lifter.setPower(lifterPower);
            grabber1.setPower(grabberPower);
            grabber2.setPower(grabberPower);
            //basearm.setPower(basearmPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Lifter", "lifter" + lifterPower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("MotosArm", "mid (%.2f)", midarmPower);
            telemetry.addData("Grabber Power", "mid (%.2f)", grabberPower);
            telemetry.addData("Drive", "Change: (%.2f)", speedChange);
            telemetry.update();
        }
    }
}
