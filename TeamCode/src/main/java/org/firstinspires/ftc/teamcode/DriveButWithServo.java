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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Boolean.FALSE;


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

@TeleOp(name="Servo Arcade Drive", group="Linear Opmode")
//@Disabled
public class DriveButWithServo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //private DcMotor midarm = null;
    private DcMotor colapser = null;
    private DcMotor extender2 = null;
    private DcMotor arm = null;

    private CRServo marker = null;
    private Servo leftGrabber = null;//left grabber when arm is in the crater
    private Servo rightGrabber = null;//right grabber when arm is in the crater

    /*private CRServo grabber1 = null;
    private CRServo grabber2 = null;*/
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
        //midarm  = hardwareMap.get(DcMotor.class, "mid_arm");
        colapser = hardwareMap.get(DcMotor.class, "lifter");
        extender2 = hardwareMap.get(DcMotor.class, "lifter2");
        arm = hardwareMap.get(DcMotor.class, "arm");

        marker = hardwareMap.get(CRServo.class, "marker");
        leftGrabber = hardwareMap.get(Servo.class,  "grabber1");
        rightGrabber = hardwareMap.get(Servo.class,  "grabber2");
        //basearm = hardwareMap.get(DcMotor.class, "base_arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        //midarm.setDirection(DcMotor.Direction.FORWARD);
        colapser.setDirection(DcMotor.Direction.FORWARD);
        extender2.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        marker.setDirection(CRServo.Direction.FORWARD);
        leftGrabber.setDirection(Servo.Direction.FORWARD);
        rightGrabber.setDirection(Servo.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        colapser.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //basearm.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower = 0;
        double rightPower = 0;
        double turn;
        double speedChange;//driving
        double armSpeedChange;//midarm
        double grabberPosition = -85.0;
        double colapsePower = 0;
        double extendPower = 0;
        double armPower = 0;
        double markerPower = 0;
        int startPosition = extender2.getCurrentPosition();
        int armStartPosition = arm.getCurrentPosition();


        //double grabberPower;
        //int starPositionArm = midarm.getCurrentPosition();
        //int currentPositionArm = starPositionArm - midarm.getCurrentPosition();
        //int targetPositionArm;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            //double leftPower;
            //double rightPower;
            //double midarmPower;
            //double basearmPower;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
           // double turn  =  gamepad1.right_stick_x;
           // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            speedChange = 1-(gamepad1.right_trigger * 0.8);
            if(gamepad1.x) {//turn right
                leftPower = 1 * speedChange;
                rightPower = -1 * speedChange;
            } else if(gamepad1.b) {//turn left
                leftPower = -1 * speedChange;
                rightPower = 1 * speedChange;
            } else if(gamepad1.y) {//drive backwards
                leftPower = -1 * speedChange;
                rightPower = -1 * speedChange;
            } else if(gamepad1.a) {//drive forward
                leftPower = 1 * speedChange;
                rightPower = 1 * speedChange;
            } else if(gamepad1.left_bumper) {//individually control the left wheel
                if(gamepad1.dpad_up) {
                    leftPower = 0.1 * speedChange;
                } else if(gamepad1.dpad_down) {
                    leftPower = -0.1 * speedChange;
                } else {
                    leftPower = 0;
                }
            } else if(gamepad1.right_bumper) {//individually control the right wheel
                if(gamepad1.dpad_up) {
                    rightPower = 0.1 * speedChange;
                } else if(gamepad1.dpad_down) {
                    rightPower = -0.1 * speedChange;
                } else {
                    rightPower = 0;
                }
            } else if(gamepad1.left_stick_y != 0 || gamepad1.left_stick_x != 0){//use the left stick to drive fast
                turn = gamepad1.left_stick_x;
                leftPower  = (gamepad1.left_stick_y - turn) * speedChange ;
                rightPower = (gamepad1.left_stick_y + turn) * speedChange;
            } else {//use the right sick to drive slow
                turn = gamepad1.right_stick_x;
                leftPower  = (gamepad1.right_stick_y - turn) * speedChange * 0.5;
                rightPower = (gamepad1.right_stick_y + turn) * speedChange * 0.5;
            }


            //midarm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armSpeedChange = 1-(gamepad2.right_trigger * 0.4);
            if (gamepad2.b && gamepad2.y) {
                colapsePower = -0.8 * armSpeedChange; //colapsePower = 0.32;//for lowering
                extendPower = 0.9 * armSpeedChange;//extendPower = -0.99;
            } else if(gamepad2.x && gamepad2.a){
                colapsePower = 1 * armSpeedChange;//colapsePower = -0.32;//for raising
                extendPower = -0.01 * armSpeedChange;//extendPower = 0.99;
            } else if(gamepad2.left_bumper){//individually control the two lifting motors
                colapsePower = gamepad2.right_stick_y;
                extendPower = gamepad2.left_stick_y;
            } else {
                colapsePower = 0;
                extendPower = 0;
            }

            if(gamepad2.left_bumper) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = 0;
            } else if(gamepad2.a && (gamepad2.x == FALSE))  {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = -0.65 * armSpeedChange;
            } else if(gamepad2.y && (gamepad2.b == FALSE)) {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = 0.65 * armSpeedChange;
            } else if((gamepad2.left_bumper == FALSE) && gamepad2.right_stick_button) {//if you press the button, the arm should go to its starting position
                if(arm.getCurrentPosition() != armStartPosition) {
                    arm.setTargetPosition(armStartPosition);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.9);
                } else {
                    arm.setPower(0.0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else if((gamepad2.left_bumper == FALSE) && gamepad2.right_stick_y < -0.6) {
                if(arm.getCurrentPosition() != armStartPosition + -177) {
                    arm.setTargetPosition(armStartPosition + -177);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.9);
                } else {
                    arm.setPower(0.0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else if((gamepad2.left_bumper == FALSE) && gamepad2.right_stick_y > 0.6) {
                if(arm.getCurrentPosition() != armStartPosition + -515) {
                    arm.setTargetPosition(armStartPosition + -515);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0.9);
                } else {
                    arm.setPower(0.0);
                    arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            } else {
                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armPower = gamepad2.left_stick_y * 0.8 * armSpeedChange;
            }


            if(gamepad2.left_trigger > 0.89) {
                armStartPosition = arm.getCurrentPosition();//reset start arm position
            }


            if(gamepad2.dpad_up) {
                markerPower = -0.6 * armSpeedChange;
            } else if(gamepad2.dpad_down) {
                markerPower = 0.6 * armSpeedChange;
            } else {
                markerPower = 0;
            }



                //midarmPower=(0.8 * gamepad2.left_stick_y) * armSpeedChange;//*0.5




            if(false) {//for individual grabber movement   was gamepad2.right_bumper
               /* if(gamepad2.x) {
                    grabberPowerR = 0.3 * armSpeedChange;
                    grabberPowerL = 0.0 * armSpeedChange;
                } else if(gamepad2.b) {
                    grabberPowerR = -0.3 * armSpeedChange;
                    grabberPowerL = 0.0 * armSpeedChange;
                } else if(gamepad2.dpad_left) {
                    grabberPowerL = -0.3 * armSpeedChange;
                    grabberPowerR = 0.0 * armSpeedChange;
                } else if(gamepad2.dpad_right) {
                    grabberPowerL = 0.3 * armSpeedChange;
                    grabberPowerR = 0.0 * armSpeedChange;
                } else {
                    grabberPowerL = 0.0;
                    grabberPowerR = 0.0;
                } */
            } else if(gamepad2.x && (gamepad2.a == FALSE)) {//grab something
                grabberPosition += 0.5*armSpeedChange;
            } else if(gamepad2.b && (gamepad2.y == FALSE)) {//release something
               grabberPosition -= 0.5*armSpeedChange;
            } else if(gamepad2.dpad_right) {//grab something
                grabberPosition += 0.5;
            } else if(gamepad2.dpad_left) {//release something
                grabberPosition -= 0.5;
            }
            if(grabberPosition > 88) {//make sure you don't stress your motors
                grabberPosition = 88;
            } else if(grabberPosition < 1) {
                grabberPosition = 1;
            }









            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            //midarm.setPower(midarmPower);
            colapser.setPower(colapsePower);
            extender2.setPower(extendPower);
            if (arm.isBusy() == FALSE) {//If it is not using the encoders set the power as normal
                arm.setPower(armPower);
            }
            marker.setPower(markerPower);
            leftGrabber.setPosition(grabberPosition);
            rightGrabber.setPosition(-grabberPosition);//It is opposite of the left grabber
            /*grabber1.setPower(grabberPower);
            grabber2.setPower(grabberPower);*/
            //basearm.setPower(basearmPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Grabber", "position: %.2f", grabberPosition);
            telemetry.addData("Arm", "position (%d)", arm.getCurrentPosition() - armStartPosition);
            telemetry.addData("Lifter", "lifter" + colapsePower);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Arm", "power (%.2f)", armPower);
            telemetry.addData("Extender", "Position (%d)", startPosition - extender2.getCurrentPosition());
            telemetry.addData("Marker", "power (%.2f)", markerPower);
            //telemetry.addData("MotosArm", "mid (%.2f)", midarmPower);
            //telemetry.addData("Grabber Power", "mid (%.2f)", grabberPower);
            telemetry.addData("Drive", "Change: (%.2f)", speedChange);
            /*telemetry.addData("Left", "click(%d)", leftDrive.getCurrentPosition());
            telemetry.addData("Right","click(%d)", rightDrive.getCurrentPosition());*/
            telemetry.update();
        }
    }
}
