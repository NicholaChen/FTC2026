package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;


    @Config
    public class CameraVision {
        private final AprilTagProcessor aprilTag;
        public static String CAMERA_NAME = "Webcam 1";

        private Telemetry telemetry;
        public CameraVision(HardwareMap hardwareMap, Telemetry telemetry_) {
            telemetry = telemetry_;

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, CAMERA_NAME), cameraMonitorViewId);
            FtcDashboard.getInstance().startCameraStream(camera, 0);

            aprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.

                    .build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            //aprilTag.setDecimation(3);

            // Create the vision portal by using a builder.
            VisionPortal.Builder builder = new VisionPortal.Builder();

            // Set the camera (webcam vs. built-in RC phone camera).
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

            // Choose a camera resolution. Not all cameras support all resolutions.
            //builder.setCameraResolution(new Size(640, 480));

            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            //builder.setAutoStopLiveView(false);


            // Set and enable the processor.
            builder.addProcessor(aprilTag);

            // Build the Vision Portal, using the above settings.
            VisionPortal visionPortal = builder.build();

            FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
            telemetry.addData("CameraVision", "Initialized");
        }
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detect() {
        return aprilTag.getDetections();
    }

    public int readObelisk() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (int i= 0; i<detections.size(); i++) {
            int tagID = detections.get(i).id;
            if (tagID == 21) {
                return 21;
            }
            if (tagID == 22) {
                return 22;
            }
            if (tagID == 23) {
                return 23;
            }
        }
        return 0;
    }
    public void aprilTagTelemetry(){
        List<AprilTagDetection> Detections = aprilTag.getDetections();
        for (AprilTagDetection CurrentDetection : Detections) {
            if (CurrentDetection.metadata != null) {
                telemetry.addLine(String.valueOf(CurrentDetection.id)+ "\n" +  String.valueOf(CurrentDetection.metadata));
                if (!CurrentDetection.metadata.name.contains("Obelisk")){
                    telemetry.addLine("X: " +  String.valueOf(CurrentDetection.robotPose.getPosition().x));
                    telemetry.addLine("Y: " + String.valueOf(CurrentDetection.robotPose.getPosition().y));
                    telemetry.addLine("Z: " + String.valueOf(CurrentDetection.robotPose.getPosition().z));
                    telemetry.addLine("Pitch: " + String.valueOf(CurrentDetection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES)));
                    telemetry.addLine("Yaw: " + String.valueOf(CurrentDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    telemetry.addLine("Roll: " + String.valueOf(CurrentDetection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES)));
                }
            }
        }
        // return 0 for nothing
        // return -1 for error
        // return 1-3 for each different ids

//    public Pose3D getPose() {
//
    }
}
