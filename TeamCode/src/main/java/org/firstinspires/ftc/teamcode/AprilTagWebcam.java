package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;


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

    public void displayDetectionTelemetry(AprilTagClusterDetection detection){

                if (detection != null) {
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", detection.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", detection.percentClusterFound));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
                else {
                    telemetry.addLine(String.format("\n NO TAG DETECTED"));
                }
    }
    public AprilTagDetection getTagBySpecified(int id) {
            for (AprilTagDetection detection : detectionTags){
                if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
                    if (singleDet.metadata.id == id){
                        return detection;
                    }
                }  else {
                    AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;
                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                }
            }
            return null;
    }
    public AprilTagClusterDetection getClusterDetection(RobotControl.AllianceColor Alliance) {
        for (AprilTagDetection detection : detectionTags){
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection clusterDet = (AprilTagClusterDetection) detection;

                if(clusterDet.metadata.name.charAt(0) == 'R' &&
                        Alliance == RobotControl.AllianceColor.RED &&
                        abs(clusterDet.ftcPose.roll) < 90) {
                        return clusterDet;
                }
                else if(clusterDet.metadata.name.charAt(0) == 'B' &&
                    Alliance == RobotControl.AllianceColor.BLUE &&
                    abs(clusterDet.ftcPose.roll) < 90) {

                    telemetry.addLine(String.format("\n==== Tag Cluster (%s)", clusterDet.metadata.name));
                    telemetry.addLine(String.format("Percent tags found: %d", clusterDet.percentClusterFound));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                    return clusterDet;
                }
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
