package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.Dogeforia;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;


public class Camsensor {

    private static final String VUFORIA_KEY = "ASiIF9r/////AAABmbB85zU3k0g3qzF1DLbC7GUnvGVHWKDgtHLp6I/mzHMkcRm8A0oZl2woG1jqog81fIG7hAfVTp50Fj3sgLTQCqJ/sy9mZ/SQzMh2E3EBTIqS4ndxzRR0KGqW62bmVqQN69a7cuamH1QC4y3yiTaEDha8JoQF7kS3K32S6bziY2MYoBO8PCegD6dsnhtAH4VnAwIeiM/dCvhDXh1FuLFfLZmoExZGKasu20D3hqlvVRFoa7jUIIdzEEbuCM70asfMyzHk1ZdqgpBAqFOtxoyVgF0/ackncBT+hYFqfBbPkFGwiLiFED/8OBiMWRLVm4raAYo9NIgXqDFJhghNXqL8OMPwyuYYJuhZfqeg0z39M3fr";

    public Dogeforia vuforia;
    public WebcamName webcamName;
    public int cameraMonitorViewId;
    public GoldAlignDetector detector;

    public void init(HardwareMap ahwMap, int forwardDisplacement, int verticalDisplacement, int leftDisplacement) {
	webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();//should this have View ID??
        parameters.fillCameraMonitorViewParent = true;
        parameters.vuforiaLicenseKey = VUFORIA_KEY ;
        parameters.cameraName = webcamName;

        vuforia = new Dogeforia(parameters);
        vuforia.enableConvertFrameToBitmap();
        vuforia.showDebug();

	// Load the data sets for vuforia tracking.
        VuforiaTrackables targetsRoverRuckus = this.vuforia.loadTrackablesFromAsset("RoverRuckus");
        VuforiaTrackable blueRover = targetsRoverRuckus.get(0);
        blueRover.setName("Blue-Rover");
        VuforiaTrackable redFootprint = targetsRoverRuckus.get(1);
        redFootprint.setName("Red-Footprint");
        VuforiaTrackable frontCraters = targetsRoverRuckus.get(2);
        frontCraters.setName("Front-Craters");
        VuforiaTrackable backSpace = targetsRoverRuckus.get(3);
        backSpace.setName("Back-Space");
	
	List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsRoverRuckus);

        OpenGLMatrix blueRoverLocationOnField = OpenGLMatrix
                .translation(0, mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        blueRover.setLocation(blueRoverLocationOnField);

        OpenGLMatrix redFootprintLocationOnField = OpenGLMatrix
                .translation(0, -mmFTCFieldWidth, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        redFootprint.setLocation(redFootprintLocationOnField);

        OpenGLMatrix frontCratersLocationOnField = OpenGLMatrix
                .translation(-mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0 , 90));
        frontCraters.setLocation(frontCratersLocationOnField);

        OpenGLMatrix backSpaceLocationOnField = OpenGLMatrix
                .translation(mmFTCFieldWidth, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        backSpace.setLocation(backSpaceLocationOnField);

        int CAMERA_FORWARD_DISPLACEMENT  = forwarDisplacement;   // eg: Camera is 110 mm in front of robot center
        int CAMERA_VERTICAL_DISPLACEMENT = verticalDisplacement;   // eg: Camera is 200 mm above ground
        int CAMERA_LEFT_DISPLACEMENT     = letDisplacement;     // eg: Camera is ON the robot's center line

        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XZY,AngleUnit.DEGREES,
                        90, 90, 0));

        //  Let all the trackable listeners know where the phone is. 
        for (VuforiaTrackable trackable : allTrackables)
        {
            ((VuforiaTrackableDefaultListener)trackable.getListener()).setCameraLocationOnRobot(parameters.cameraName, phoneLocationOnRobot);
        }

        targetsRoverRuckus.activate();

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext,CameraViewDisplay.getInstance(), 0, true);
        detector.useDefaults();
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.downscale = 0.8;
        detector.alignPosOffset = 0;
        detector.alignSize = 236;

        // Set the detector
        vuforia.setDogeCVDetector(detector);
        vuforia.enableDogeCV();
        vuforia.showDebug();
        vuforia.start();
    }
}
