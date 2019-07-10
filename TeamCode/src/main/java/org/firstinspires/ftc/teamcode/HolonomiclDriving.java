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

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


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

    // The IMU sensor object
    BNO055IMU imu;//We will use the IMU later

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

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");//This is getting the IMU from the hub
        imu.initialize(parameters2);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        backDrive = hardwareMap.get(DcMotor.class, "back_drive");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);//the motors are set to turn if all positive
        backDrive.setDirection(DcMotor.Direction.FORWARD);


        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //basearm.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)

        telemetry.addData("Ready...", "Press Play to start");
        telemetry.update();

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
        double turn;//Used to store how much turn power we want to add/subtract
        double IMUOffset = getIMUAngle();//This is the offset IMU angle to start out with

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

            if(gamepad1.left_stick_button && gamepad1.left_trigger > 0.9) {
                IMUOffset=getIMUAngle();//You can reset the IMU starting offset by pressing down on the left joystick and pressing the left trigger
            }

            speedChange = 1-(gamepad1.right_trigger * 0.8);
            if(gamepad1.x) {//move to the left
                backPower = -1;
                leftPower = 0.5;
                rightPower = 0.5;//when going to the left or to the right, the back wheel is going two times as fast
            } else if(gamepad1.b) {//move to the right
                backPower = 1;
                leftPower = -0.5;
                rightPower = -0.5;
            } else if(gamepad1.y) {//move forwards
                backPower = 0;
                leftPower = -1;
                rightPower = 1;
            } else if(gamepad1.a) {//move backwards
                backPower = 0;
                leftPower = 1;
                rightPower = -1;
            }  else if(gamepad1.left_bumper) {//turn counterclockwise(was clockwise)
                backPower = 1;
                leftPower = 1;
                rightPower = 1;
            } else if(gamepad1.right_bumper) {//turn clockwise(was counterclockwise)
                backPower = -1;
                leftPower = -1;
                rightPower = -1;
            } else if(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) > 0) {//This uses the left joystick to move the robot in any direction
                force = Math.sqrt(gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y);//This is used so you don't always have to go at full speed
                turn = gamepad1.right_stick_x * 0.5;//You can turn the robot with the x-axis of the other stick
                //The IMU is used to keep the robot going in the same direction
                angle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (IMUOffset - getIMUAngle());//This was x, y.  Android studios uses Radians.  This figures out the angle that the robot is supposed to go
                setA = Math.cos(angle)/Math.cos(Math.toRadians(30));//This is an important formula for the calculations
                setB = setA*Math.sin(Math.toRadians(30)) - Math.sin(angle);//This is an important formula for the calculations
                backPower = setA - turn;
                rightPower = -setB - turn;
                leftPower = setB-setA - turn;
                max = Math.max(Math.abs(backPower), Math.abs(rightPower));//This helps figure out the maximum absolute value
                max = Math.max(max, Math.abs(leftPower));//You can then divide all the powers by it to scale them so nothing is above 1
                backPower = backPower/max*force;
                rightPower = rightPower/max*force;
                leftPower = leftPower/max*force;
            } else {//Arcade Drive with right joystick
                turn = gamepad1.right_stick_x;
                leftPower  = -(gamepad1.right_stick_y - turn);
                rightPower = (gamepad1.right_stick_y + turn);
                backPower = 0.0;
            }






            // Send calculated power to wheels
            leftDrive.setPower(leftPower * speedChange);
            rightDrive.setPower(rightPower * speedChange);
            backDrive.setPower(backPower * speedChange);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors without speedChange", "left (%.2f), right (%.2f), back(%.2f)", leftPower, rightPower, backPower);
            telemetry.addData("Drive", "Change: (%.2f)", speedChange);
            /*telemetry.addData("Left", "click(%d)", leftDrive.getCurrentPosition());
            telemetry.addData("Right","click(%d)", rightDrive.getCurrentPosition());*/
            telemetry.update();

            sleep(3);//Sleep a little to give the phones a tiny break
        }
    }

    public double getIMUAngle() {//This is a function that reads the IMU angle
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
}


