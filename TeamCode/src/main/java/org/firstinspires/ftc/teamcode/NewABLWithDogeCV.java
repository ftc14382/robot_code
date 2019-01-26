/* Copyright (c) 2018 FIRST. All rights reserved.
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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


/**
 * This 2018-2019 OpMode illustrates the basics of using the Vuforia localizer to determine
 * positioning and orientation of robot on the FTC field.
 * The code is structured as a LinearOpMode
 *
 * Vuforia uses the phone's camera to inspect it's surroundings, and attempt to locate target images.
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code than combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * This example assumes a "square" field configuration where the red and blue alliance stations
 * are on opposite walls of each other.
 *
 * From the Audience perspective, the Red Alliance station is on the right and the
 * Blue Alliance Station is on the left.

 * The four vision targets are located in the center of each of the perimeter walls with
 * the images facing inwards towards the robots:
 *     - BlueRover is the Mars Rover image target on the wall closest to the blue alliance
 *     - RedFootprint is the Lunar Footprint target on the wall closest to the red alliance
 *     - FrontCraters is the Lunar Craters image target on the wall closest to the audience
 *     - BackSpace is the Deep Space image target on the wall farthest from the audience
 *
 * A final calculation then uses the location of the camera on the robot to determine the
 * robot's location and orientation on the field.
 *
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */

@Autonomous(name="NewBlueLeftCV", group ="NewDogeCV")
//@Disabled
public class NewABLWithDogeCV extends LinearOpMode {
public static final String Tag = "OurLog";
    HardwarePushbot robot       = new HardwarePushbot();


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "ASiIF9r/////AAABmbB85zU3k0g3qzF1DLbC7GUnvGVHWKDgtHLp6I/mzHMkcRm8A0oZl2woG1jqog81fIG7hAfVTp50Fj3sgLTQCqJ/sy9mZ/SQzMh2E3EBTIqS4ndxzRR0KGqW62bmVqQN69a7cuamH1QC4y3yiTaEDha8JoQF7kS3K32S6bziY2MYoBO8PCegD6dsnhtAH4VnAwIeiM/dCvhDXh1FuLFfLZmoExZGKasu20D3hqlvVRFoa7jUIIdzEEbuCM70asfMyzHk1ZdqgpBAqFOtxoyVgF0/ackncBT+hYFqfBbPkFGwiLiFED/8OBiMWRLVm4raAYo9NIgXqDFJhghNXqL8OMPwyuYYJuhZfqeg0z39M3fr";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here

    private static final float mmPerInch        = 25.4f;
    private static final float mmFTCFieldWidth  = (12*6) * mmPerInch;       // the width of the FTC field (from the center point to the outer panels)
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Select which camera you want use.  The FRONT camera is the one on the same side as the screen.
    // Valid choices are:  BACK or FRONT
    //private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;

    private OpenGLMatrix lastLocation = null;
    boolean targetVisible;//was private
    Dogeforia vuforia;

    private enum TurnDirection {RIGHT,LEFT}
    private enum DriveDirection {FORWARD, BACKWARD}
    private enum Quad {RED_LEFT, BLUE_LEFT, RED_RIGHT, BLUE_RIGHT}

    private static final Quad  startQuad =  Quad.BLUE_LEFT;//This determines the starting position

    private Position Depot = new Position();
    private Position crater = new Position();
    private Position transfer = new Position();
    private Position cube1 = new Position();
    private Position cube2 = new Position();
    private Position cube3 = new Position();
    private Position cube1Found = new Position();
    private Position cube3Found = new Position();


    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    //VuforiaLocalizer vuforia;
    WebcamName webcamName;

    List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
    private ElapsedTime runtime = new ElapsedTime();


    // DogeCV detector
    GoldAlignDetector detector;

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;//was 4     // For figuring circumference!!!!!!!!!!!!!!!!!!!!!!!!!!
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;//was 0.4

    @Override public void runOpMode() {
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        if(startQuad == Quad.BLUE_LEFT || startQuad == Quad.BLUE_RIGHT) {
            Depot.x = -55;//-47
            Depot.y = 56;//58//64

            transfer.x = 0;
            transfer.y = 60;

            crater.x = 46;
            crater.y = 61;//52.5
        } else {
            Depot.x = 55;//47
            Depot.y = -55;//58//64

            transfer.x = 0;
            transfer.y = -60;

            crater.x = -46;
            crater.y = -61;//52.5
        }

        cube1.x = -25.5;
        cube1.y = 45.5;
        cube1Found.x = -27;
        cube1Found.y = 50;

        cube2.x = -35.5;
        cube2.y = 35.5;

        cube3.x = -45.5;
        cube3.y = 25.5;
        cube3Found.x = -50;
        cube3Found.y = 30;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot.init(hardwareMap);
        //leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        //rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //leftDrive.setDirection(DcMotor.Direction.FORWARD);
        //rightDrive.setDirection(DcMotor.Direction.REVERSE);
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();//should this have View ID??


        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.fillCameraMonitorViewParent = true;

        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        //parameters.cameraDirection   = CAMERA_CHOICE;
        parameters.cameraName = webcamName;
// Create Dogeforia object
        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        vuforia.showDebug();


        //  Instantiate the Vuforia engine
     //vuforia = ClassFactory.getInstance().createVuforia(parameters);//!!!!!!!!!!!!!!!!!!!

        // Load the data sets that for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");



        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);



        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * This Rover Ruckus sample places a specific target in the middle of each perimeter wall.
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        /**
         * To place the BlueRover target in the middle of the blue perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Then, we translate it along the Y axis to the blue perimeter wall.
         */
        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        /**
         * To place the RedFootprint target in the middle of the red perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 180 around the field's Z axis so the image is flat against the red perimeter wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative Y axis to the red perimeter wall.
         */
        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        /**
         * To place the FrontCraters target in the middle of the front perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it 90 around the field's Z axis so the image is flat against the front wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the negative X axis to the front perimeter wall.
         */
        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        /**
         * To place the BackSpace target in the middle of the back perimeter wall:
         * - First we rotate it 90 around the field's X axis to flip it upright.
         * - Second, we rotate it -90 around the field's Z axis so the image is flat against the back wall
         *   and facing inwards to the center of the field.
         * - Then, we translate it along the X axis to the back perimeter wall.
         */
        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        /**
         * Create a transformation matrix describing where the phone is on the robot.
         *
         * The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
         * pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
         * camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.
         *
         * If using the rear (High Res) camera:
         * We need to rotate the camera around it's long axis to bring the rear camera forward.
         * This requires a negative 90 degree rotation on the Y axis
         *
         * If using the Front (Low Res) camera
         * We need to rotate the camera around it's long axis to bring the FRONT camera forward.
         * This requires a Positive 90 degree rotation on the Y axis
         *
         * Next, translate the camera lens to where it is on the robot.
         * In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */

        final int CAMERA_FORWARD_DISPLACEMENT  = 226;   // eg: Camera is 110 mm in front of robot center
        final int CAMERA_VERTICAL_DISPLACEMENT = 178;   // eg: Camera is 200 mm above ground
        final int CAMERA_LEFT_DISPLACEMENT     = 33;     // eg: Camera is ON the robot's center line


        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY,AngleUnit.DEGREES,
                        90, 90, 0));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, phoneLocationOnRobot);
        }



// Initialize the detector

        //Activate the targets
        targetsRoverRuckus.activate();


        //Initialize the detector
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;
        detector.alignPosOffset = 40;
        detector.alignSize = 60;


        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
        /** Wait for the game to begin */



        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();




        runtime.reset();
        RobotInfo robotInfo = new RobotInfo();

        //double startLeft = robot.leftDrive.getCurrentPosition();
        encoderDrive(TURN_SPEED, degreesToInches(90), degreesToInches(-90), 8);
        /*while(runtime.seconds()<10) {
            telemetry.addData("Cube Found: ", detector.isFound());
            telemetry.addData("Cube X: ", detector.getXPosition());
            telemetry.update();
            if(detector.isFound()) {
                if (detector.getAligned()) {
                    sleep(130);
                    if(detector.getAligned()) {
                        //robotInfo.degrees += (startLeft - robot.leftDrive.getCurrentPosition())/11.2;
                        break;
                    } else if(detector.getXPosition() > 360) {
                        robot.leftDrive.setPower(0.03);
                        robot.rightDrive.setPower(-0.03);
                    } else {
                        robot.rightDrive.setPower(0.03);
                        robot.leftDrive.setPower(-0.03);
                    }
                } else if (detector.getXPosition() > 360) {
                    robot.leftDrive.setPower(0.08);
                    robot.rightDrive.setPower(-0.08);
                } else {
                    robot.rightDrive.setPower(0.08);
                    robot.leftDrive.setPower(-0.08);
                }
            } else {
                encoderDrive(TURN_SPEED, 1.2, -1.2, 4);
                if(detector.isFound() == false) {
                    encoderDrive(TURN_SPEED, 1.2, -1.2, 4);
                    encoderDrive(TURN_SPEED, 1.2, -1.2, 4);
                }
            }
        }*/

        /*encoderDrive(TURN_SPEED, 20, 20, 5);

        encoderDrive(TURN_SPEED, degreesToInches(-170), degreesToInches(170), 5);
        if(startQuad == Quad.BLUE_LEFT || startQuad == Quad.RED_LEFT) {
            encoderDrive(DRIVE_SPEED,22,22,4.0);
            encoderDrive(TURN_SPEED, degreesToInches(75),degreesToInches(-75),2.0);  //Trying to turn
            encoderDrive(DRIVE_SPEED,11,11,5.0);
        } else {
            encoderDrive(DRIVE_SPEED,24,24,4.0);
            encoderDrive(TURN_SPEED, degreesToInches(-75),degreesToInches(75),2.0);  //Trying to turn
            encoderDrive(DRIVE_SPEED,12,12,5.0);
        }*/

        /** Start tracking the data sets we care about. */
        targetsRoverRuckus.activate();


        double leftPower;
        double rightPower;
        sleep(1000);
        // check all the trackable target to see which one (if any) is visible.
        targetVisible = false;
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                telemetry.addData("Visible Target", trackable.getName());
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            RobotLog.ii(Tag, "Target Visible");
            VectorF translation; // translation of robot center
            Orientation rotation; // rotation of robot
            // express position (translation) of robot in inches.
           translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);
            robotInfo.x = translation.get(0) / mmPerInch;
            robotInfo.y = translation.get(1) / mmPerInch;
            // express the rotation of the robot in degrees.
            rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
            robotInfo.degrees = rotation.thirdAngle;
        }
        else {
            if (startQuad == Quad.BLUE_LEFT) {
                robotInfo.x = -21;//-16
                robotInfo.y = 41;//36
                robotInfo.degrees = 60;
            } else if(startQuad == Quad.RED_LEFT){
                robotInfo.x = 19;//-16
                robotInfo.y = -44;//36
                robotInfo.degrees = -120;
            } else if(startQuad == Quad.RED_RIGHT){
                robotInfo.x = -23;//-16
                robotInfo.y = -46;//36
                robotInfo.degrees = -60;
            } else if(startQuad == Quad.BLUE_RIGHT){
                robotInfo.x = 21;//-16
                robotInfo.y = 41;//36
                robotInfo.degrees = 120;
            }
            telemetry.addData("Visible Target", "none");
        }
        telemetry.update();//End Viuforia
        //sleep(10000);
        turnTo(robotInfo, cube1);
        sleep(3000);
        if(detector.getAligned()) {
            driveTo(robotInfo, cube1);
            driveTo(robotInfo, cube1Found);
        } else {
            turnTo(robotInfo, cube2);
            sleep(8000);
            if(detector.getAligned()) {
                driveTo(robotInfo, cube2);
            } else {
                //turnTo(robotInfo, cube3);
                driveTo(robotInfo, cube3);
                driveTo(robotInfo, cube3Found);
            }
        }

        driveTo(robotInfo, Depot);




        /*if(startQuad == Quad.BLUE_LEFT || startQuad == Quad.RED_LEFT) {
            driveTo(robotInfo, Depot);
            encoderDrive(TURN_SPEED, degreesToInches(190), degreesToInches(-190), 6);
            robotInfo.degrees = robotInfo.degrees - 190;

            robot.marker.setPower(0.6);
            sleep(1100);
            robot.marker.setPower(-0.6);
            sleep(1050);
            robot.marker.setPower(0);

            driveTo(robotInfo, transfer);


            driveTo(robotInfo, crater);
        } else {
            driveTo(robotInfo, Depot);
            encoderDrive(TURN_SPEED, degreesToInches(190), degreesToInches(-190), 6);
            robotInfo.degrees = robotInfo.degrees - 190;

            robot.marker.setPower(0.6);
            sleep(1100);
            robot.marker.setPower(-0.6);
            sleep(1050);
            robot.marker.setPower(0);

            driveTo(robotInfo, transfer);
            driveTo(robotInfo, crater);
        }*/


    }


    public void turnTo(RobotInfo r, Position p) {
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        if (turn > 180) {
            turn -= 360;
        }
        if (turn < -180) {
            turn += 360;
        }
        encoderDrive(TURN_SPEED, -degreesToInches(turn), degreesToInches(turn), 5);
        r.degrees = Math.toDegrees(theta);
        telemetry.addData("Robot Heading", r.degrees);
    }

    public double degreesToInches(double degrees) {
        double Inches = (degrees * 11.2)/COUNTS_PER_INCH;
        return Math.round(Inches);
    }

    public void driveTo(RobotInfo r, Position p) {
        double deltaX = p.x - r.x;
        double deltaY = p.y - r.y;
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double theta = Math.atan2(deltaY, deltaX);
        double turn = Math.toDegrees(theta) - r.degrees;
        if (turn > 180) {
            turn -= 360;
        }
        if (turn < -180) {
            turn += 360;
        }
        encoderDrive(TURN_SPEED, -degreesToInches(turn), degreesToInches(turn), 5);
        encoderDrive(DRIVE_SPEED, distance, distance, 6);
        r.x = p.x;
        r.y = p.y;
        r.degrees = Math.toDegrees(theta);
        telemetry.addData("RobotX:", r.x);
        telemetry.addData("RobotY", r.y);
        telemetry.addData("Robot Heading", r.degrees);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            if(leftInches != rightInches) {
                robot.leftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.BRAKE);//set brake mode
                robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if(leftInches != rightInches) {
                robot.leftDrive.setZeroPowerBehavior( DcMotor.ZeroPowerBehavior.FLOAT);//set brake mode
                robot.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            //  sleep(250);   // optional pause after each move
        }
    }

}
