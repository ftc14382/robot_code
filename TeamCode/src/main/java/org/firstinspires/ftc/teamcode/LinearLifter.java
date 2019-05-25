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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


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

@TeleOp(name="Lifter Arcade Drive", group="Linear Opmode")
//@Disabled
public class LinearLifter extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    Orientation angles;
    Acceleration gravity;
    Double  distanceFront;
    Double  distanceRight;
    Double  distanceLeft;

    HardwarePushbot robot;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot = new HardwarePushbot();
        robot.init(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        //leftDrive  = hardwareMap.get(DcMotor.class, "driveleft");
        //rightDrive = hardwareMap.get(DcMotor.class, "driveright");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        double leftPower = 0;
        double rightPower = 0;
        double drive;
        double turn;
        double speedChange = 0.0;//driving

        double grabberPowerL = 0.0;
        double grabberPowerR = 0.0;
        double colapsePower = 0;
        double extendPower = 0;
        double armPower = 0;
        double markerPower = 0;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            drive =  gamepad1.left_stick_y;
            turn  =  gamepad1.left_stick_x;

            if (gamepad1.a) {
                robot.webcam.detector.saveSnapshot();
            }

            if (Math.abs(turn) < 0.02) {
                turn = 0.0;
            }

            if (Math.abs(drive) < 0.02) {
                drive = 0.0;
            }


            leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;


            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
            distanceFront = robot.sensorFront.getDistance(DistanceUnit.INCH);
            distanceRight = robot.sensorRight.getDistance(DistanceUnit.INCH);
            distanceLeft  = robot.sensorLeft.getDistance(DistanceUnit.INCH);

            // Send calculated power to wheels
            robot.leftDrive.setPower(leftPower);
            robot.rightDrive.setPower(-1.0 * rightPower);
            //midarm.setPower(midarmPower);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

            telemetry.addData("Gamepad1:", "%s",gamepad1.toString());

            telemetry.addLine().addData("Heading: ", "%s", formatAngle(angles.angleUnit, angles.firstAngle))
                    .addData("Roll:", "%s", formatAngle(angles.angleUnit, angles.secondAngle))
                    .addData("Pitch:", "%s", formatAngle(angles.angleUnit, angles.thirdAngle));

            telemetry.addLine()
                    .addData("distance (l,f,r)", "(%.3f, %.3f, %.3f)",
                                    distanceLeft, distanceFront, distanceRight);


            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
