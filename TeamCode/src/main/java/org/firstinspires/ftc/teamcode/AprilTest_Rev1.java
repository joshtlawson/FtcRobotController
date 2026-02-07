package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTest_Rev1 extends OpMode {
    AprilTagWebcam aprilTagWebcam;
    @Override
    public void init(){
        aprilTagWebcam.init(hardwareMap,telemetry);
    }
    @Override
    public  void loop(){
        aprilTagWebcam.update();
        AprilTagDetection id20 = aprilTagWebcam.getTagBySpecified(20);
        aprilTagWebcam.displayDetectionTelemetry(id20);
    }
}
