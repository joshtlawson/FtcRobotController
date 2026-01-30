package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AUTORed3SingleMove (Blocks to Java)", preselectTeleOp = "TeleOp25_26Season")
public class AUTORed3SingleMove extends OpMode {

    private RobotControl Robo = new RobotControl();

    @Override
    public void init() {
        Robo.InitRobot();
    }
    @Override
    public void loop() {
        telemetry.addLine("Autonomous Active");
        telemetry.update();
        telemetry.update();
        Robo.OpenGate();
        // move to the launch line
        Robo.RobotMoveAtSpeed(-30, 0, 0, 1.4);
        // align with the goal
        Robo.RobotMoveAtSpeed(-20, -23, 0, 2.1);
        // launch
        Robo.LaunchBalls(2, 3, 2500,1000);
        // move the gate
        Robo.RobotMoveDistance(0, 45, 0, 5);
        // line up with the gate
        Robo.RobotMoveDistance(0, 0, -15, 5);
        // move toward the gate
        Robo.RobotMoveDistance(-20, 0, 0, 5);
    }
}