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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


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

@TeleOp(name="3 Wheel Drive", group="Linear Opmode")
//@Disabled
public class HolonomiclDriving extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor backDrive = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    //private DcMotor midarm = null;

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
        backDrive = hardwareMap.get(DcMotor.class, "back_drive");
        //midarm  = hardwareMap.get(DcMotor.class, "mid_arm");
        /*grabber1 = hardwareMap.get(CRServo.class,  "grabber1");
        grabber2 = hardwareMap.get(CRServo.class,  "grabber2");*/
        //basearm = hardwareMap.get(DcMotor.class, "base_arm");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);//the motors are set to turn if all positive
        backDrive.setDirection(DcMotor.Direction.FORWARD);
        //midarm.setDirection(DcMotor.Direction.FORWARD);


        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //basearm.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower = 0;
        double rightPower = 0;
        double backPower = 0;
        double speedChange;//driving
        double angle;//This is used in one stick, any direction driving
        double max;//This is used in one stick, any direction driving
        double setA;//This is used in one stick, any direction driving
        double setB;//This is used in one stick, any direction driving
        double force;//This is used in one stick, any direction driving

        //double midarmPower = 0;



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
            if(gamepad1.x) {//move to the left
                backPower = -1 * speedChange;
                leftPower = 0.5 * speedChange;
                rightPower = 0.5 * speedChange;//when going to the left or to the right, the back wheel is going two times as fast
            } else if(gamepad1.b) {//move to the right
                backPower = 1 * speedChange;
                leftPower = -0.5 * speedChange;
                rightPower = -0.5 * speedChange;
            } else if(gamepad1.y) {//move forwards
                backPower = 0;
                leftPower = -1 * speedChange;
                rightPower = 1 * speedChange;
            } else if(gamepad1.a) {//move backwards
                backPower = 0;
                leftPower = 1 * speedChange;
                rightPower = -1 * speedChange;
            }  else if(gamepad1.left_bumper) {//turn clockwise
                backPower = 1 * speedChange;
                leftPower = 1 * speedChange;
                rightPower = 1 * speedChange;
            } else if(gamepad1.right_bumper) {//turn counterclockwise
                backPower = -1 * speedChange;
                leftPower = -1 * speedChange;
                rightPower = -1 * speedChange;
            } else {//This uses the left joystick to move the robot in any direction
                force = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.right_stick_y * gamepad1.right_stick_y);//This is used so you don't always have to go at full speed
                angle = Math.atan2(gamepad1.left_stick_x, -gamepad1.left_stick_y);//This figures out the angle that the robot is supposed to go
                setA = Math.cos(angle)/Math.cos(Math.toRadians(30));//This is an important formula for the calculations
                setB = setA*Math.sin(Math.toRadians(30)) - Math.sin(angle);//This is an important formula for the calculations
                backPower = setA;
                rightPower = -setB;
                leftPower = setB-setA;
                max = Math.max(Math.abs(backPower), Math.abs(rightPower));//This helps figure out the maximum absolute value
                max = Math.max(max, Math.abs(leftPower));//You can then divide all the powers by it to scale them so nothing is above 1
                backPower = backPower/max*force;
                rightPower = rightPower/max*force;
                leftPower = leftPower/max*force;
            }








            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            backDrive.setPower(backPower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), back(%.2f)", leftPower, rightPower, backPower);
            telemetry.addData("Drive", "Change: (%.2f)", speedChange);
            /*telemetry.addData("Left", "click(%d)", leftDrive.getCurrentPosition());
            telemetry.addData("Right","click(%d)", rightDrive.getCurrentPosition());*/
            telemetry.update();
        }
    }
}
