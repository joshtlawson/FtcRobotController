package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class RobotControl extends OpMode{
    private DcMotorEx FrontRightDrive;
    private DcMotorEx FrontLeftDrive;
    private DcMotorEx BackLeftDrive;
    private DcMotorEx BackRightDrive;
    private DcMotorEx BallMover;
    private DcMotorEx Intake;
    private DcMotorEx LeftLauncher;
    private DcMotorEx RightLauncher;
    private Servo RightBallStop;
    private Servo LeftBallStop;
    double RotationRatio;
    double GearRatio;
    private float SteeringAggressiveness;
    private float ForwardAggressiveness;
    private float StrafingAggressiveness;
    private boolean InvertControllers;
    private float MaxLauncherSpeed;
    private float MaxIntakeVelocity;
    private float MaxBallMoverSpeed;
    private float LeftStopPosition;
    private float LeftOpenPosition;
    private float RightStopPosition;
    private float RightOpenPosition;
    private volatile Gamepad gamepad1 = null;

    private volatile Gamepad gamepad2 = null;
    private float RangeAutoAggressiveness;
    private float TargetingAggressiveness;
    private float TempRange_cm;
    private float LastGoalDetected_ms;
 private HardwareMap hardwareMap;
    ElapsedTime RunTimer_sec = new ElapsedTime();

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private Telemetry telemetry;
    public enum AllianceColor{
        RED,
        BLUE
    };

    private int goalTagID;
    public void InitRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance,Gamepad gamepad1,Gamepad gamepad2)
    {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        // Setup Hardware
        FrontRightDrive = (DcMotorEx)(hardwareMap.get(DcMotor.class, "FrontRightDrive"));
        FrontLeftDrive = (DcMotorEx)(hardwareMap.get(DcMotor.class, "FrontLeftDrive"));
        BackLeftDrive = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BackLeftDrive"));
        BackRightDrive = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BackRightDrive"));
        BallMover = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BallMover"));
        Intake = (DcMotorEx )(hardwareMap.get(DcMotor.class, "Intake"));
        LeftLauncher = (DcMotorEx )(hardwareMap.get(DcMotor.class, "LeftLauncher"));
        RightLauncher = (DcMotorEx )(hardwareMap.get(DcMotor.class, "RightLauncher"));
        RightBallStop = hardwareMap.get(Servo.class, "RightBallStop");
        LeftBallStop = hardwareMap.get(Servo.class, "LeftBallStop");

        // Initialize the Vision System
        aprilTagWebcam.init(hardwareMap,telemetry);
        if(alliance == RobotControl.AllianceColor.RED){
            goalTagID = 24;
        }
        else {
            goalTagID = 20;
        }

        // Setup the PID Tuning
        ((DcMotorEx) FrontRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) FrontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 7, MotorControlAlgorithm.PIDF));

        // Configure Movement Types
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BallMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        BallMover.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        LeftLauncher.setDirection(DcMotor.Direction.REVERSE);
        RightLauncher.setDirection(DcMotor.Direction.FORWARD);
        // Control Parameters
        LeftStopPosition = 0.75f;
        LeftOpenPosition = 0.5f;
        RightStopPosition = 0.5f;
        RightOpenPosition = 0.75f;
        TempRange_cm = 0; // Init to zero

        RotationRatio = 11.1;
        GearRatio = 41.25;

        // Control Parameters
        SteeringAggressiveness = 1500;
        TargetingAggressiveness = 50; // Control Rotation
        RangeAutoAggressiveness = 12.5f; // Ball Speed
        ForwardAggressiveness = 2500;
        StrafingAggressiveness = 2500;
        InvertControllers = false;
        MaxLauncherSpeed = 3000;
        MaxIntakeVelocity = 2000;
        MaxBallMoverSpeed = 3000;
    }
    public void StopRobot()
    {
        FrontLeftDrive.setVelocity(0);
        FrontRightDrive.setVelocity(0);
        BackLeftDrive.setVelocity(0);
        BackRightDrive.setVelocity(0);
        BallMover.setVelocity(0);
        Intake.setVelocity(0);
        LeftLauncher.setVelocity(0);
        RightLauncher.setVelocity(0);
    }
    public void RobotMoveAtSpeed(float ForwardVelocity, float CWRotationalVelocity, float RightVelocity, float MoveTime_sec) {
        telemetry.addLine("calledFunction");
        ((DcMotorEx) FrontLeftDrive).setVelocity((GearRatio * ForwardVelocity - GearRatio * RightVelocity) - RotationRatio * CWRotationalVelocity);
        ((DcMotorEx) BackLeftDrive).setVelocity((GearRatio * ForwardVelocity + GearRatio * RightVelocity) - RotationRatio * CWRotationalVelocity);
        ((DcMotorEx) FrontRightDrive).setVelocity(GearRatio * ForwardVelocity + GearRatio * RightVelocity + RotationRatio * CWRotationalVelocity);
        ((DcMotorEx) BackRightDrive).setVelocity((GearRatio * ForwardVelocity - GearRatio * RightVelocity) + RotationRatio * CWRotationalVelocity);
        RunTimer_sec = new ElapsedTime();
        RunTimer_sec.reset();
        telemetry.addLine("waiting");
        while (RunTimer_sec.seconds() < MoveTime_sec) {
        }
        telemetry.addLine("done");
    }
    public AprilTagDetection UpdateGoalLocation()
    {
        aprilTagWebcam.update();
        AprilTagDetection GoalID = aprilTagWebcam.getTagBySpecified(goalTagID);
        aprilTagWebcam.displayDetectionTelemetry(GoalID);
        return GoalID;
    }
    /**
     * Describe this function...
     */
    public void RobotMoveDistance(int Forward_in, int Rotate_cw_degrees, int Lateral_right_in, int waitTime_sec) {
        telemetry.addLine("calledFunction");
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setTargetPosition((int) (GearRatio * Forward_in + GearRatio * Lateral_right_in + RotationRatio * Rotate_cw_degrees));
        BackLeftDrive.setTargetPosition((int) ((GearRatio * Forward_in - GearRatio * Lateral_right_in) + RotationRatio * Rotate_cw_degrees));
        FrontRightDrive.setTargetPosition((int) ((GearRatio * Forward_in - GearRatio * Lateral_right_in) - RotationRatio * Rotate_cw_degrees));
        BackRightDrive.setTargetPosition((int) ((GearRatio * Forward_in + GearRatio * Lateral_right_in) - RotationRatio * Rotate_cw_degrees));
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RunTimer_sec = new ElapsedTime();
        RunTimer_sec.reset();
        telemetry.addLine("waiting");
        while (RunTimer_sec.seconds() < waitTime_sec && (FrontLeftDrive.isBusy() || FrontRightDrive.isBusy() || BackLeftDrive.isBusy() || BackRightDrive.isBusy())) {
        }
        telemetry.addLine("done");
    }

    /**
     * Describe this function...
     */
    public void LaunchBalls(int SpinUpTime_sec, int LaunchTime_sec, int LauncherSpeed) {
        // Spin up the Launcher
        ((DcMotorEx) LeftLauncher).setVelocity(LauncherSpeed);
        ((DcMotorEx) RightLauncher).setVelocity(LauncherSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < SpinUpTime_sec) {
        }
        // Open the Gates

        ((DcMotorEx) BallMover).setVelocity(MaxBallMoverSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < LaunchTime_sec) {
        }
        ((DcMotorEx) RightLauncher).setVelocity(0);
        ((DcMotorEx) LeftLauncher).setVelocity(0);
        ((DcMotorEx) BallMover).setVelocity(0);
    }
    public void LaunchBallsWithCamera(int TargetTime_s) {

        float ClockwiseVelocityCmd = 0;
        float LauncherSpeedCmd = MaxLauncherSpeed;
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < TargetTime_s) {

            AprilTagDetection GoalLocation = this.UpdateGoalLocation();
            if (GoalLocation != null) {
                ClockwiseVelocityCmd = -(float) (GoalLocation.ftcPose.bearing) * TargetingAggressiveness;
            }
            this.SetDriveMotionCmds(0, 0, ClockwiseVelocityCmd);


            if(GoalLocation != null) {
                LauncherSpeedCmd = 1600 + 3.61f * (TempRange_cm - 147);
            }
            // Spin up the Launcher
            ((DcMotorEx) LeftLauncher).setVelocity(LauncherSpeedCmd);
            ((DcMotorEx) RightLauncher).setVelocity(LauncherSpeedCmd);
        }
        // Spin up the Launcher
        ((DcMotorEx) LeftLauncher).setVelocity(LauncherSpeed);
        ((DcMotorEx) RightLauncher).setVelocity(LauncherSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < SpinUpTime_sec) {
        }
        // Open the Gates

        ((DcMotorEx) BallMover).setVelocity(MaxBallMoverSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < LaunchTime_sec) {
    }
        ((DcMotorEx) RightLauncher).setVelocity(0);
        ((DcMotorEx) LeftLauncher).setVelocity(0);
        ((DcMotorEx) BallMover).setVelocity(0);
    }
    // Manual TeleOp Commands
    public void OpenGate()
    {
        RightBallStop.setPosition(RightOpenPosition);
        LeftBallStop.setPosition(LeftOpenPosition);
    }
    public void CloseGate()
    {
        RightBallStop.setPosition(RightStopPosition);
        LeftBallStop.setPosition(LeftStopPosition);
    }
    public void SetDriveMotionCmds(float ForwardSpeed, float RightSpeed, float ClockwiseSpeed)
    {
        FrontLeftDrive.setVelocity(ForwardSpeed  - RightSpeed + ClockwiseSpeed);
        BackLeftDrive.setVelocity(ForwardSpeed + RightSpeed + ClockwiseSpeed);
        FrontRightDrive.setVelocity(-ForwardSpeed - RightSpeed + ClockwiseSpeed);
        BackRightDrive.setVelocity(ForwardSpeed - RightSpeed - ClockwiseSpeed);
    }
    public void SetLaunchControlCmds(float intakeVelocity, float BallMoverSpeedCmd, float LauncherSpeedCmd)
    {
        Intake.setVelocity(intakeVelocity);
        BallMover.setVelocity(BallMoverSpeedCmd);
        RightLauncher.setVelocity(LauncherSpeedCmd);
        LeftLauncher.setVelocity(LauncherSpeedCmd);
    }
    public void RunTeleOp()
    {

        AprilTagDetection GoalLocation = this.UpdateGoalLocation();
        // Invert As Needed
        if (gamepad1.right_bumper) {
            InvertControllers = true;
        } else {
            InvertControllers = false;
        }


        // Drive Logic -----------------------------------------------------------------------
        // Remote Control Forward Cmds
        float ForwardSpeedCmd = (-gamepad1.left_stick_y)*ForwardAggressiveness;
        if (InvertControllers)
            ForwardSpeedCmd = -ForwardSpeedCmd;

        // Remote Control Strafing Cmds
        float RightSpeedCmd ;
        if (gamepad1.left_trigger > 0) {
            RightSpeedCmd  = -(gamepad1.left_trigger * StrafingAggressiveness);
        } else if (gamepad1.right_trigger > 0) {
            RightSpeedCmd  = gamepad1.right_trigger * StrafingAggressiveness;
        } else {
            RightSpeedCmd  = 0;
        }
        if (InvertControllers)
            RightSpeedCmd = -RightSpeedCmd;
        float ClockwiseVelocityCmd = 0;
        if (gamepad1.left_bumper && GoalLocation != null)
        {
            gamepad1.rumble(200,200,200);
            gamepad2.rumble(200,200,200);
            telemetry.addLine("Targeting Status: ACTIVE");
            ClockwiseVelocityCmd = -(float)(GoalLocation.ftcPose.bearing) * TargetingAggressiveness;
        }
        else
        {
            gamepad1.stopRumble();
            gamepad2.stopRumble();
            if( GoalLocation == null){
                telemetry.addLine("Targeting Status: NOT READY");
            }
            else
            {
                telemetry.addLine("Targeting Status: READY");
            }
            // Remote Control Turning Cmds
            ClockwiseVelocityCmd = gamepad1.right_stick_x*SteeringAggressiveness;
        }


        this.SetDriveMotionCmds(ForwardSpeedCmd, RightSpeedCmd, ClockwiseVelocityCmd);


        // Intake Logic -----------------------------------------------------------------------
        float IntakeVelocityCmd;
        if (gamepad2.dpad_down) {
            IntakeVelocityCmd = -MaxIntakeVelocity;
        } else if (gamepad2.dpad_up) {
            IntakeVelocityCmd = MaxIntakeVelocity;
        } else {
            IntakeVelocityCmd = 0;
        }

        // Launcher
        float LauncherSpeedCmd;
        if(gamepad1.left_bumper && (GoalLocation != null || TempRange_cm !=0))
        {
            if(GoalLocation != null) {
                TempRange_cm = (float) (GoalLocation.ftcPose.range);
            }
            if(TempRange_cm < 147)
            {
                TempRange_cm = 147;
            }
            LauncherSpeedCmd = 1600 + 3.61f*(TempRange_cm - 147);
        }
        else {
            if (gamepad2.left_bumper) {
                LauncherSpeedCmd = MaxLauncherSpeed;
            } else if (gamepad2.right_bumper) {
                LauncherSpeedCmd = MaxLauncherSpeed * 0.54f;
            } else {
                LauncherSpeedCmd = MaxLauncherSpeed * gamepad2.right_trigger;
            }
        }
        // Ball mover
        float BallMoverSpeedCmd;
        if (gamepad2.cross) {
            BallMoverSpeedCmd = -MaxBallMoverSpeed;
        } else if (gamepad2.triangle) {
            BallMoverSpeedCmd = MaxBallMoverSpeed;
        } else {
            BallMoverSpeedCmd = -(MaxBallMoverSpeed * gamepad2.right_stick_y);
        }

        // Gate
        if (gamepad2.dpad_right) {
            this.OpenGate();
        } else {
            this.CloseGate();
        }
        // Output Commands
        this.SetLaunchControlCmds(IntakeVelocityCmd,BallMoverSpeedCmd,LauncherSpeedCmd);
        telemetry.addData("Launcher Velocity: (%.2f)",LauncherSpeedCmd);
        // Add some Telemetry
        // Show the elapsed game time and wheel power.
        telemetry.addData("Velocity Cmds", "Forward (%.2f), Rightward (%.2f), ClockWise (%.2f)",
                ForwardSpeedCmd, RightSpeedCmd,ClockwiseVelocityCmd);

    }
    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
