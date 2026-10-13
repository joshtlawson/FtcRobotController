package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AprilTagWebcam {
        private AprilTagProcessor aprilTagProcessor;
        private VisionPortal visionPortal;

        private ArrayList<AprilTagDetection> detectionTags;
        private Telemetry telemetry;

        public void init(HardwareMap hwMap, Telemetry telemetry) {
            this.telemetry = telemetry;

            aprilTagProcessor = new AprilTagProcessor.Builder()
                    .setDrawTagID(true)
                    .setDrawTagOutline(true)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                    .build();
            VisionPortal.Builder builder = new VisionPortal.Builder();
            builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
            builder.setCameraResolution(new Size(640, 480));
            builder.addProcessor(aprilTagProcessor);
            visionPortal = builder.build();
        }

        public void update(){
            detectionTags = aprilTagProcessor.getDetections();
        }

        public ArrayList<AprilTagDetection> getDetectedTags() {
            return detectionTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detection){

                if (detection != null) {
                    telemetry.addLine(String.format("\n ===== ID %d) %s",detection.id,detection.metadata.name));
                    //telemetry.addLine(String.format("XYZ = %6.1f %6.1f %6.1f (cm)",detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    //telemetry.addLine(String.format("PRY = %6.1f %6.1f %6.1f (deg)",detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE = %6.1f %6.1f %6.1f (cm, deg, cm)",detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
                else {
                    telemetry.addLine(String.format("\n NO TAG DETECTED"));
                }
//                else {
//                    telemetry.addLine(String.format("\n ===== ID %d) Unknown",detection.id));
//                    telemetry.addLine(String.format("Center = %6.0f %6.0f (cm, cm)",detection.center.x, detection.center.y));
//                }




    }
    public AprilTagDetection getTagBySpecified(int id) {
            for (AprilTagDetection detection : detectionTags){
                if (detection.id == id){
                    return detection;
                }
            }
            return null;
    }
    public void stop(){
            if (visionPortal != null){
                visionPortal.close();
            }
    }
}
