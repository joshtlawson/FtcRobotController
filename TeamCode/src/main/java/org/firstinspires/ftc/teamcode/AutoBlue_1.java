package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class AutoBlue_1 extends OpMode {
    private RobotControl Robo = new RobotControl();
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    /*
     * Code to run ONCE when the driver hits INIT
     */
    public void init() {
        Robo.InitRobot(hardwareMap,telemetry, RobotControl.AllianceColor.BLUE, gamepad1, gamepad2);

    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        Robo.RunTeleOp();
        // Action 1
        Robo.RobotMoveDistance(24,0,0,5);
        // Action 2
        Robo.RobotMoveDistance(0, 0, 24, 5);
        //Action 3
        Robo.RobotMoveDistance(96, 0, 0, 5);
        //Action 4
        Robo.RobotMoveDistance(0, 0, -24, 5);
        //Action 5
        Robo.RobotMoveDistance(-12, 0, 0, 5);
        //Action 6
        Robo.LaunchBalls(3,5,1000);
        //Action 7
        Robo.RobotMoveDistance(0, 0, 80, 5);
        //Action8
        Robo.RobotMoveDistance(36, 0, 0, 5);
}
}