package org.firstinspires.ftc.teamcode;

import android.icu.text.CaseMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

public class RobotControl extends OpMode{
    private DcMotorEx FrontRightDrive;
    private DcMotorEx FrontLeftDrive;
    private DcMotorEx BackLeftDrive;
    private DcMotorEx BackRightDrive;
    private DcMotorEx BallMover;
    private DcMotorEx Intake;
    private DcMotorEx Launcher;
    private DcMotorEx TurretYawMotor;
    private Servo LauncherThroat;
    private Servo BallLifter;
    private Servo FlowerRamp;
    private Servo DozerLeftServo;
    private Servo DozerRightServo;
    private ColorSensor BallColor;
    private TouchSensor YawLimitSensor;
    private boolean IsFlowerDown;
    private boolean LastIsFlowerButtonPressed;
    double RotationRatio;
    double GearRatio;
    private float SteeringAggressiveness;
    private float ForwardAggressiveness;
    private float StrafingAggressiveness;
    private boolean InvertControllers;
    private float MaxLauncherSpeed;
    private float MaxIntakeVelocity;
    private float MaxBallMoverSpeed;
    private float MaxTurretYawSpeed;
    private float LifterUpperPosition;
    private float LifterLowerPosition;
    private float DozerPosition;
    private boolean LastXBtnState;
    private boolean LastYBtnState;
    private volatile Gamepad gamepad1 = null;

    private volatile Gamepad gamepad2 = null;
    private float RangeAutoAggressiveness;
    private float TargetingAggressiveness;
    private float TempRange_cm;
    private float LastGoalDetected_ms;
    private AllianceColor ActiveAlliance;
    private BallType LastSeenBall;
    private AprilTagClusterDetection GoalLocation;

 private HardwareMap hardwareMap;
    ElapsedTime RunTimer_sec = new ElapsedTime();

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private Telemetry telemetry;
    public enum AllianceColor{
        RED,
        BLUE
    };
    public enum BallType{
        UNKNOWN,
        NECTAR,
        POLLEN
    };

    private int goalTagID;
    public void InitRobot(HardwareMap hardwareMap, Telemetry telemetry, AllianceColor alliance,Gamepad gamepad1,Gamepad gamepad2)
    {
        ActiveAlliance = alliance;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        // Setup Hardware
        FrontRightDrive = (DcMotorEx)(hardwareMap.get(DcMotor.class, "FrontRightDrive"));
        FrontLeftDrive = (DcMotorEx)(hardwareMap.get(DcMotor.class, "FrontLeftDrive"));
        BackLeftDrive = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BackLeftDrive"));
        BackRightDrive = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BackRightDrive"));
        //BallMover = (DcMotorEx )(hardwareMap.get(DcMotor.class, "BallMover"));
        Intake = (DcMotorEx )(hardwareMap.get(DcMotor.class, "Intake"));
        Launcher = (DcMotorEx )(hardwareMap.get(DcMotor.class, "TurretLauncher"));
        LauncherThroat = hardwareMap.get(Servo.class, "LauncherThroat");
        BallLifter = hardwareMap.get(Servo.class, "BallLifter");
        FlowerRamp = hardwareMap.get(Servo.class, "FlowerRamp");

        DozerLeftServo = hardwareMap.get(Servo.class, "DozerLeft");
        DozerRightServo = hardwareMap.get(Servo.class, "DozerRight");

        TurretYawMotor = (DcMotorEx) (hardwareMap.get(DcMotor.class,"TurretYaw"));

        // Initialize the Vision System
        aprilTagWebcam.init(hardwareMap,telemetry);
        // Initialize Sensors
        BallColor = (ColorSensor)(hardwareMap.get(ColorSensor.class,"ColorSensor"));
        YawLimitSensor = (TouchSensor)(hardwareMap.get(TouchSensor.class,"YawLimitSW"));
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
        //BallMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        TurretYawMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        //BallMover.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Launcher.setDirection(DcMotor.Direction.FORWARD);
        TurretYawMotor.setDirection(DcMotor.Direction.FORWARD);
        // Control Parameters
        LifterUpperPosition = 0.75f;
        LifterLowerPosition = 0.5f;
        TempRange_cm = 0; // Init to zero
        DozerPosition = 0.5f;

        RotationRatio = 11.1;
        GearRatio = 41.25;

        // Control Parameters
        SteeringAggressiveness = 1500;
        TargetingAggressiveness = 50; // Control Rotation
        RangeAutoAggressiveness = 12.5f; // Ball Speed
        ForwardAggressiveness = 2500;
        StrafingAggressiveness = 2500;
        InvertControllers = false;
        MaxLauncherSpeed = -2500;
        MaxIntakeVelocity = 2000;
        MaxBallMoverSpeed = 3000;
        MaxTurretYawSpeed = 3000;
        IsFlowerDown = false;
        LastSeenBall = BallType.POLLEN;

    }
    public void StopRobot()
    {
        FrontLeftDrive.setVelocity(0);
        FrontRightDrive.setVelocity(0);
        BackLeftDrive.setVelocity(0);
        BackRightDrive.setVelocity(0);
        //BallMover.setVelocity(0);
        Intake.setVelocity(0);
        Launcher.setVelocity(0);
        TurretYawMotor.setVelocity(0);
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
    public AprilTagClusterDetection UpdateGoalLocation(RobotControl.AllianceColor Alliance)
    {
        aprilTagWebcam.update();
        AprilTagClusterDetection GoalID = aprilTagWebcam.getClusterDetection(Alliance);
        aprilTagWebcam.displayDetectionTelemetry(GoalID);
        return GoalID;
    }
    /**
     * Describe this function...
     */
    public void RobotMoveDistance(float Forward_in, int Rotate_cw_degrees, float Lateral_right_in, int waitTime_sec) {
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

    private void SetIntakeState(float  SpeedCmd_rpm)
    {
        float RPM2DPS = 360f/60f;
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setVelocity(SpeedCmd_rpm*RPM2DPS, AngleUnit.DEGREES);
    }
    /**
     * Describe this function...
     */
    public void LaunchBalls(int SpinUpTime_sec, int LaunchTime_sec, int LauncherSpeed) {
        // Spin up the Launcher
        ((DcMotorEx) Launcher).setVelocity(LauncherSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < SpinUpTime_sec) {
        }
        // Open the Gates

        //((DcMotorEx) BallMover).setVelocity(MaxBallMoverSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < LaunchTime_sec) {
        }
        ((DcMotorEx) Launcher).setVelocity(0);
        //((DcMotorEx) BallMover).setVelocity(0);
    }
    public void LaunchBallsWithCamera(int TargetTime_s) {

        float ClockwiseVelocityCmd = 0;
        float LauncherSpeedCmd = MaxLauncherSpeed;
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < TargetTime_s) {

            AprilTagClusterDetection GoalLocation = this.UpdateGoalLocation(ActiveAlliance);
            if (GoalLocation != null) {
                ClockwiseVelocityCmd = -(float) (GoalLocation.ftcPose.bearing) * TargetingAggressiveness;
            }
            this.SetDriveMotionCmds(0, 0, ClockwiseVelocityCmd);


            if(GoalLocation != null) {
                LauncherSpeedCmd = 1600 + 3.61f * (TempRange_cm - 147);
            }
            // Spin up the Launcher
            ((DcMotorEx) Launcher).setVelocity(LauncherSpeedCmd);
       }
//        // Spin up the Launcher
//        ((DcMotorEx) LeftLauncher).setVelocity(LauncherSpeed);
//        ((DcMotorEx) Launcher).setVelocity(LauncherSpeed);
//        RunTimer_sec.reset();
//        while (RunTimer_sec.seconds() < SpinUpTime_sec) {
//        }
//        // Open the Gates
//
//        ((DcMotorEx) BallMover).setVelocity(MaxBallMoverSpeed);
//        RunTimer_sec.reset();
//        while (RunTimer_sec.seconds() < LaunchTime_sec) {
//    }
//        ((DcMotorEx) Launcher).setVelocity(0);
//        ((DcMotorEx) LeftLauncher).setVelocity(0);
//        ((DcMotorEx) BallMover).setVelocity(0);
    }
    public void SetNectorPosition()
    {
        LauncherThroat.setPosition(0.65);
    }
    public void SetPollenPosition()
    {
        LauncherThroat.setPosition(0.35);
    }

    public void RaiseRamp()
    {
        float DownPosition = 0.3f;
        FlowerRamp.setPosition(0.75);
        DozerLeftServo.setPosition(1-DownPosition);
        DozerRightServo.setPosition((DownPosition));
    }
    public void LowerRamp()
    {
        float UpPosition = 0.73f;
        FlowerRamp.setPosition(0.25);
        DozerLeftServo.setPosition(1-UpPosition);
        DozerRightServo.setPosition((UpPosition));
    }
    // Manual TeleOp Commands
    public void LowerPosition()
    {
        BallLifter.setPosition(LifterLowerPosition);
    }
    public void UpperPosition()
    {
        BallLifter.setPosition(LifterUpperPosition);
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

        //BallMover.setVelocity(BallMoverSpeedCmd);
    }
    public void IntakeNewBalls()
    {
        float IntakeSpeed_rpm = 900;
        float ForwardSpeed_ips = 1;
        float ShiftRight_in = 12;
        int AttemptRepeatsCount = 1;

        int AttemptNum = 0;
        // Start Intake
        this.SetIntakeState(IntakeSpeed_rpm);
        // Move Forward Slowly
        this.RobotMoveAtSpeed(ForwardSpeed_ips,0,0,5);
        for (AttemptNum = 0; AttemptNum < AttemptRepeatsCount; AttemptNum++) {
            // Backup
            this.RobotMoveDistance(12,0,0,5);
            // Shift Laterally
            this.RobotMoveDistance(0,0,ShiftRight_in,5);
            // Move Forward Slowly
            this.RobotMoveAtSpeed(ForwardSpeed_ips,0,0,5);
        }
        // Stop Intake
        this.SetIntakeState(0);

    }
    public void MoveToStartingSequence(int SequenceNum)
    {
        switch(SequenceNum)
        {
            case 1:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                // Action 2
                this.RobotMoveDistance(0, 45, 0, 5);
                //Action 3
                this.RobotMoveDistance(24, 0, 0, 5);
                //Action 4
                this.RobotMoveDistance(0, 90, 0, 5);
                //Action 5
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 6
                this.RobotMoveDistance(36, 0, 0, 5);
                break;
            case 2:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                // Action 2
                this.RobotMoveDistance(0, 45, 0, 5);
                //Action 3
                this.RobotMoveDistance(24, 0, 0, 5);
                //Action 4
                this.RobotMoveDistance(0, 90, 0, 5);
                //Action 5
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 6
                this.RobotMoveDistance(36, 0, 0, 5);
                break;
            case 3:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                // Action 2
                this.RobotMoveDistance(0, 45, 0, 5);
                //Action 3
                this.RobotMoveDistance(24, 0, 0, 5);
                //Action 4
                this.RobotMoveDistance(0, 90, 0, 5);
                //Action 5
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 6
                this.RobotMoveDistance(36, 0, 0, 5);
                break;
            case 4:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                // Action 2
                this.RobotMoveDistance(0, 45, 0, 5);
                //Action 3
                this.RobotMoveDistance(24, 0, 0, 5);
                //Action 4
                this.RobotMoveDistance(0, 90, 0, 5);
                //Action 5
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 6
                this.RobotMoveDistance(36, 0, 0, 5);
                break;
            case 5:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                //Action 2
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 3
                this.RobotMoveDistance(36, 0, 0, 5);
                break;

        }

    }
    public void DetectColors()
    {

        telemetry.addData("Detected Red: ",BallColor.red());
        telemetry.addData(" | Green: ",BallColor.green());
        telemetry.addData(" | Blue: ",BallColor.blue());
        telemetry.addLine();
        int VisibleThreshold = 300;
        if (BallColor.red() > VisibleThreshold || BallColor.blue() > VisibleThreshold || BallColor.green()> VisibleThreshold)
        { //Ball Detected
            if (BallColor.blue() > 2 * BallColor.green() && BallColor.blue() > 2 * BallColor.red())
            {// Probably a Blue Ball
                LastSeenBall = BallType.NECTAR;
            }
            else if(BallColor.red() > BallColor.green() && BallColor.red() > 2 * BallColor.blue())
            { // Probably Red
                LastSeenBall = BallType.NECTAR;
            }
            else
            { // We think by elimination it's yellow
                LastSeenBall = BallType.POLLEN;
            }
        }
        if (LastSeenBall == BallType.NECTAR) {
            telemetry.addLine("Nectar Detected");
        } else if (LastSeenBall == BallType.POLLEN) {
            telemetry.addLine("Pollen Detected");
        } else {
            telemetry.addLine("Unknown Detected");
        }
    }
    public void SetThroatPosition()
    {// Invert As Needed
        this.DetectColors();
        // Manual Throat Position Setting
        if (gamepad2.left_bumper) {
            this.SetNectorPosition();
            telemetry.addLine("NECTOR Position");
        } else {
            this.SetPollenPosition();
            telemetry.addLine("POLLEN Position");
        }
        if (gamepad2.left_trigger_pressed){
            if( LastSeenBall == BallType.NECTAR) {
                this.SetNectorPosition();
                telemetry.addLine("NECTOR Position");
            }
            else if(LastSeenBall == BallType.POLLEN){
                this.SetPollenPosition();
                telemetry.addLine("POLLEN Position");
            }
        }
    }
    public void SetDriveCommands()
    {
        // Drive Logic -----------------------------------------------------------------------
        // Remote Control Forward Cmds
        float ForwardSpeedCmd = (-gamepad1.left_stick_y)*ForwardAggressiveness;
        // Remote Control Strafing Cmds
        float RightSpeedCmd ;
        if (gamepad1.left_trigger > 0) {
            RightSpeedCmd  = -(gamepad1.left_trigger * StrafingAggressiveness);
        } else if (gamepad1.right_trigger > 0) {
            RightSpeedCmd  = gamepad1.right_trigger * StrafingAggressiveness;
        } else {
            RightSpeedCmd  = 0;
        }
        float ClockwiseVelocityCmd = 0;
        // Remote Control Turning Cmds
        ClockwiseVelocityCmd = gamepad1.right_stick_x*SteeringAggressiveness;

        this.SetDriveMotionCmds(ForwardSpeedCmd, RightSpeedCmd, ClockwiseVelocityCmd);
        telemetry.addData("Velocity Cmds", "Forward (%.2f), Rightward (%.2f), ClockWise (%.2f)",
                ForwardSpeedCmd, RightSpeedCmd,ClockwiseVelocityCmd);
    }
    public void SetRampCommand()
    {
        // Sets the ramp Position
        if (gamepad2.aWasPressed()) {
            if(IsFlowerDown)
            {
                this.RaiseRamp();
                IsFlowerDown = false;
            }
            else {
                this.LowerRamp();
                IsFlowerDown = true;
            }
        }
    }
    public void SetLaunchCommand()
    {
        // Launcher
        float LauncherSpeedCmd;
        if(gamepad1.a)
        {
            LauncherSpeedCmd = -4000; //MaxLauncherSpeed
        }
        else if (gamepad1.b)
        {
            LauncherSpeedCmd = -3500; //MaxLauncherSpeed
        }
        else if (gamepad1.x)
        {
            LauncherSpeedCmd = -3000; //MaxLauncherSpeed
        }
        else if (gamepad1.y)
        {
            LauncherSpeedCmd = -2500; //MaxLauncherSpeed
        }
        else if (gamepad1.dpad_up)
        {
            LauncherSpeedCmd = -2000; //MaxLauncherSpeed
        }
        else if (gamepad1.dpad_right)
        {
            LauncherSpeedCmd = -1500; //MaxLauncherSpeed
        }
        else if (gamepad1.dpad_down)
        {
            LauncherSpeedCmd = -1000; //MaxLauncherSpeed
        }
        else if (gamepad1.dpad_left)
        {
            LauncherSpeedCmd = -800; //MaxLauncherSpeed
        }
        else {
            if (gamepad2.right_bumper && (GoalLocation != null || TempRange_cm != 0)) {
                if (GoalLocation != null) {
                    TempRange_cm = (float) (GoalLocation.ftcPose.range);
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }
                if (TempRange_cm < 147) {
                    TempRange_cm = 147;
                }
                // Need to Fix for '26/27
                LauncherSpeedCmd = 1600 + 3.61f * (TempRange_cm - 147);
            } else {
                gamepad1.stopRumble();
                gamepad2.stopRumble();
                LauncherSpeedCmd = MaxLauncherSpeed * gamepad2.right_trigger;
            }
        }
        Launcher.setVelocity(LauncherSpeedCmd);

        telemetry.addData("Launcher Velocity: (%.2f)",LauncherSpeedCmd);
    }
    public void SetIntakeCommand()
    {
        // Intake Logic -----------------------------------------------------------------------
        float IntakeVelocityCmd;
        if (gamepad2.dpad_down) {
            IntakeVelocityCmd = -MaxIntakeVelocity;
        } else if (gamepad2.dpad_up) {
            IntakeVelocityCmd = MaxIntakeVelocity;
        } else {
            IntakeVelocityCmd = 0;
        }
        Intake.setVelocity(IntakeVelocityCmd);
    }
    public void SetRCTurretYawCommand()
    {
        float TurretYawVelocity = gamepad2.right_stick_x*MaxTurretYawSpeed;
        this.SetTurretYawCommand(TurretYawVelocity);
    }
    public void SetTurretYawCommand(float YawCmd)
    {
        float MaxEncoderCountDisplacement = 2250;
        if(TurretYawMotor.getCurrentPosition() <-MaxEncoderCountDisplacement/2) // OOR Left
        {
            if(YawCmd < 0)
            {
                YawCmd = 0;
            }
        }
        else if(TurretYawMotor.getCurrentPosition() > MaxEncoderCountDisplacement/2) // #3 OOR Right
        {
          if(YawCmd > 0)
          {
              YawCmd = 0;
          }
        }
        telemetry.addData("Yaw Position = ",TurretYawMotor.getCurrentPosition());
        telemetry.addData(" | Yaw Cmd = ",YawCmd);

        TurretYawMotor.setVelocity((YawCmd));
    }

    public void RunTeleOp()
    {
        GoalLocation = this.UpdateGoalLocation(ActiveAlliance);

        // Sets the Turret Throat Position
        this.SetThroatPosition();
        this.SetRampCommand();
        this.SetDriveCommands();
        this.SetIntakeCommand();
        this.SetRCTurretYawCommand();
        this.SetLaunchCommand();
//
//        if(gamepad2.xWasPressed())
//        {
//            DozerPosition += + 0.01f;
//        } else if (gamepad2.yWasPressed()) {
//
//            DozerPosition -= + 0.01f;
//        }
//        if(DozerPosition > 1) {
//            DozerPosition = 1;
//        }
//        else if(DozerPosition < 0) {
//            DozerPosition = 0;
//        }

        //telemetry.addData("Dozer Position: (%.2f)",DozerPosition);


        if( GoalLocation == null){
            telemetry.addLine("Targeting Status: NOT READY");
        }
        else
        {
            telemetry.addLine("Targeting Status: READY");
        }



    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
