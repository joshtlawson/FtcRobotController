package org.firstinspires.ftc.teamcode;

import android.icu.text.CaseMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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
    private static final double BALL_QUEUE_ADVANCE_SECONDS = 1.0;

    private boolean SetLauncherSpeedRoutine;
    private static final float LAUNCHER_SPEED_STEP = 100;
    private static final float MIN_AUTO_LAUNCHER_SPEED = 100;
    private static final float MAX_AUTO_LAUNCHER_SPEED = 4000;
    private static final double DISTANCE_MOVE_VELOCITY = 750; // Encoder ticks per second

    private DcMotorEx FrontRightDrive;
    private DcMotorEx FrontLeftDrive;
    private DcMotorEx BackLeftDrive;
    private DcMotorEx BackRightDrive;
    private DcMotorEx BallMover;
    private DcMotorEx Intake;
    private DcMotorEx Launcher;
    private DcMotorEx TurretYawMotor;
    private Servo LauncherThroat;
    private Servo DozerLeftServo;
    private Servo DozerRightServo;
    private Servo LaunchGate;
    private ColorSensor BallColor;
    private TouchSensor YawLimitSensor;
    private DigitalChannel LED1;
    private DigitalChannel LED2;
    private boolean IsFlowerDown;
    private boolean LastIsFlowerButtonPressed;
    double RotationRatio;
    double GearRatio;
    private float SteeringAggressiveness;
    private float ForwardAggressiveness;
    private float StrafingAggressiveness;
    private boolean InvertControllers;
    private float MaxLauncherSpeed;
    private float autoLauncherSpeed;
    private float MaxIntakeVelocity;
    private float MaxBallMoverSpeed;
    private float MaxTurretYawSpeed;
    private float DozerPosition;

    private float ThroatPosition;
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
    private float TestLaunchGatePosition;
    private boolean SetThroatPositionRoutine;
    private boolean SetDozerPositionRoutine;
    private HardwareMap hardwareMap;
    ElapsedTime RunTimer_sec = new ElapsedTime();
    private final ElapsedTime launchTimer = new ElapsedTime();
    private final ElapsedTime goalLedTimer = new ElapsedTime();
    private final ElapsedTime ballQueueTimer = new ElapsedTime();
    private final BallQueue<BallType> ballQueue = new BallQueue<>(BallType.POLLEN, BALL_QUEUE_ADVANCE_SECONDS);
    private boolean goalDetected = false;
    private LaunchState launchState = LaunchState.Idle;
    private boolean launchGateOpen = false;
    private boolean intakeRunning = false;
    private boolean gateOpen = false;
    private boolean ballVisible = false;

    private enum LaunchState {
        Idle,
        LauncherSpinup,
        OpenGates,
        Launch
    }

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

        DozerLeftServo = hardwareMap.get(Servo.class, "DozerLeft");
        DozerRightServo = hardwareMap.get(Servo.class, "DozerRight");
        LaunchGate = hardwareMap.get(Servo.class, "LaunchGate");

        TurretYawMotor = (DcMotorEx) (hardwareMap.get(DcMotor.class,"TurretYaw"));

        // Initialize the Vision System
        aprilTagWebcam.init(hardwareMap,telemetry);
        // Initialize Sensors
        BallColor = (ColorSensor)(hardwareMap.get(ColorSensor.class,"ColorSensor"));
        YawLimitSensor = (TouchSensor)(hardwareMap.get(TouchSensor.class,"YawLimitSW"));
        LED1 = hardwareMap.get(DigitalChannel.class, "LED1");
        LED2 = hardwareMap.get(DigitalChannel.class, "LED2");
        LED1.setMode(DigitalChannel.Mode.OUTPUT);
        LED2.setMode(DigitalChannel.Mode.OUTPUT);
        StopIndicators();
        // Setup the PID Tuning
        ((DcMotorEx) FrontRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) FrontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 7, MotorControlAlgorithm.PIDF));
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
        TempRange_cm = 0; // Init to zero
        DozerPosition = 0.5f;

        RotationRatio = 11.1;//22.2;
        GearRatio = 44; //45.8;

        // Control Parameters
        SteeringAggressiveness = 1500;
        TargetingAggressiveness = 50; // Control Rotation
        RangeAutoAggressiveness = 12.5f; // Ball Speed
        ForwardAggressiveness = 2500;
        StrafingAggressiveness = 2500;
        InvertControllers = false;
        MaxLauncherSpeed = 2050;
        autoLauncherSpeed = MaxLauncherSpeed;
        MaxIntakeVelocity = 2000;
        MaxBallMoverSpeed = 3000;
        MaxTurretYawSpeed = 3000;
        IsFlowerDown = false;
        LastSeenBall = BallType.POLLEN;
        ballQueue.reset();
        ballQueueTimer.reset();
        intakeRunning = false;
        ballVisible = false;
        launchState = LaunchState.Idle;
        launchGateOpen = false;
        goalDetected = false;
        TestLaunchGatePosition = 0;
        ThroatPosition = 0;
        this.CloseGate();
        this.SetThroatToPollenPosition();

        // Optional Calibration Routines
        SetThroatPositionRoutine = false;
        SetDozerPositionRoutine = true;
        SetLauncherSpeedRoutine = true;
    }
    public void StopRobot()
    {
        FrontLeftDrive.setVelocity(0);
        FrontRightDrive.setVelocity(0);
        BackLeftDrive.setVelocity(0);
        BackRightDrive.setVelocity(0);
        //BallMover.setVelocity(0);
        SetIntakeVelocity(0);
        Launcher.setVelocity(0);
        TurretYawMotor.setVelocity(0);
        CloseGate();
        launchState = LaunchState.Idle;
        launchGateOpen = false;
        StopIndicators();
    }
    public void StopIndicators()
    {
        if (LED1 != null) {
            LED1.setState(true);
        }
        if (LED2 != null) {
            LED2.setState(true);
        }
    }

    private void UpdateGoalIndicator(boolean detected)
    {
        if (detected && !goalDetected) {
            goalLedTimer.reset();
        }
        goalDetected = detected;
        boolean on = launchState == LaunchState.Launch
                || (goalDetected && goalLedTimer.seconds() % (1.0 / 3.0) < (1.0 / 6.0));
        LED2.setState(!on);
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
        UpdateGoalIndicator(GoalID != null);
        return GoalID;
    }
    public boolean IsHiveTipped()
    {
        aprilTagWebcam.update();
        return aprilTagWebcam.IsHiveTipped(ActiveAlliance);
    }
    public void RobotMoveDistance(float Forward_in, int Rotate_cw_degrees, float Lateral_right_in, int waitTime_sec) {
        if (waitTime_sec <= 0) {
            throw new IllegalArgumentException("Distance move timeout must be positive");
        }
        telemetry.addLine("RobotMoveDistance");
        BackLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setTargetPosition((int) (GearRatio * Forward_in + GearRatio * Lateral_right_in + RotationRatio * Rotate_cw_degrees));
        BackLeftDrive.setTargetPosition((int) ((GearRatio * Forward_in - GearRatio * Lateral_right_in) + RotationRatio * Rotate_cw_degrees));
        FrontRightDrive.setTargetPosition((int) ((-GearRatio * Forward_in + GearRatio * Lateral_right_in) + RotationRatio * Rotate_cw_degrees));
        BackRightDrive.setTargetPosition((int) ((GearRatio * Forward_in + GearRatio * Lateral_right_in) - RotationRatio * Rotate_cw_degrees));
        BackRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime moveTimer = new ElapsedTime();
        try {
            FrontLeftDrive.setVelocity(DISTANCE_MOVE_VELOCITY);
            FrontRightDrive.setVelocity(DISTANCE_MOVE_VELOCITY);
            BackLeftDrive.setVelocity(DISTANCE_MOVE_VELOCITY);
            BackRightDrive.setVelocity(DISTANCE_MOVE_VELOCITY);
            telemetry.addLine("waiting for move");
            ElapsedTime debugTimer = new ElapsedTime();
            while (FrontLeftDrive.isBusy() || FrontRightDrive.isBusy() || BackLeftDrive.isBusy() || BackRightDrive.isBusy()) {
                if (Thread.currentThread().isInterrupted()) {
                    throw new IllegalStateException("Distance move interrupted");
                }
                if (moveTimer.seconds() >= waitTime_sec) {
                    ReportDistanceMoveDebug(moveTimer.seconds());
                    telemetry.addLine("Distance move timed out before reaching its target");
                    telemetry.update();
                }
                if (debugTimer.seconds() >= 0.1) {
                    ReportDistanceMoveDebug(moveTimer.seconds());
                    telemetry.update();
                    debugTimer.reset();
                }
            }
            ReportDistanceMoveDebug(moveTimer.seconds());
            telemetry.addLine("done with move");
            telemetry.update();
        } finally {
            FrontLeftDrive.setVelocity(0);
            FrontRightDrive.setVelocity(0);
            BackLeftDrive.setVelocity(0);
            BackRightDrive.setVelocity(0);
            FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void ReportDistanceMoveDebug(double elapsedSeconds)
    {
        telemetry.addData("Move elapsed (sec)", "%.1f", elapsedSeconds);
        ReportDriveMotorDebug("Front right", FrontRightDrive);
        ReportDriveMotorDebug("Front left", FrontLeftDrive);
        ReportDriveMotorDebug("Back right", BackRightDrive);
        ReportDriveMotorDebug("Back left", BackLeftDrive);
    }

    private void ReportDriveMotorDebug(String name, DcMotorEx motor)
    {
        telemetry.addData(name, "target: %d | position: %d | busy: %s",
                motor.getTargetPosition(), motor.getCurrentPosition(), motor.isBusy());
    }

    private void SetIntakeState(float  SpeedCmd_rpm)
    {
        float RPM2DPS = 360f/60f;
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setVelocity(SpeedCmd_rpm*RPM2DPS, AngleUnit.DEGREES);
        intakeRunning = SpeedCmd_rpm != 0;
    }
    private void SetIntakeVelocity(double velocity)
    {
        Intake.setVelocity(velocity);
        intakeRunning = velocity != 0;
    }
    /**
     * Describe this function...
     */
    public void SetThroatToNectarPosition()
    {
        if(SetThroatPositionRoutine) {
            LauncherThroat.setPosition(ThroatPosition);
        }
        else {
            LauncherThroat.setPosition(0.3);
        }
    }
    public void SetThroatToPollenPosition()
    {
        if(SetThroatPositionRoutine) {
            LauncherThroat.setPosition(ThroatPosition);
        }
        else {
            LauncherThroat.setPosition(0.04);
        }
    }

    public void RaiseDozerRamp()
    {
        float DownPosition = 0.71f;
        if(SetDozerPositionRoutine) {
            DozerLeftServo.setPosition(1-DozerPosition);
            DozerRightServo.setPosition((DozerPosition));
        }
        else {
            DozerLeftServo.setPosition(1-DownPosition);
            DozerRightServo.setPosition((DownPosition));
        }
    }
    public void LowerDozerRamp()
    {
        float UpPosition = 0.24f;
        if(SetDozerPositionRoutine) {
            DozerLeftServo.setPosition(1-DozerPosition);
            DozerRightServo.setPosition((DozerPosition));
        }
        else {
            DozerLeftServo.setPosition(1 - UpPosition);
            DozerRightServo.setPosition((UpPosition));
        }
    }
    public void OpenGate()
    {
        LaunchGate.setPosition(0.14);
        gateOpen = true;
    }
    public void CloseGate()
    {
        LaunchGate.setPosition(0.33);
        gateOpen = false;
    }
    public void SetDriveMotionCmds(float ForwardSpeed, float RightSpeed, float ClockwiseSpeed)
    {
        FrontLeftDrive.setVelocity(ForwardSpeed  + RightSpeed + ClockwiseSpeed);
        BackLeftDrive.setVelocity(ForwardSpeed - RightSpeed + ClockwiseSpeed);
        FrontRightDrive.setVelocity(-ForwardSpeed + RightSpeed + ClockwiseSpeed);
        BackRightDrive.setVelocity(ForwardSpeed + RightSpeed - ClockwiseSpeed);
    }
    public void RunAutonomousSequence(int AutoSeqNumber)
    {
        int CurrCollectionIdx = 1;
        int MaxCollections = 3;
        boolean IsHiveTipped = false;
        boolean AreBallsLoaded = true;
        this.MoveToStartingSequence(AutoSeqNumber);
        // TBD - Launch Balls
        this.LaunchAutoBalls(10);
        AreBallsLoaded = false;
        // Check for Hive Tipped
        IsHiveTipped = this.IsHiveTipped();
        while (CurrCollectionIdx < MaxCollections && !IsHiveTipped) {
            // TBD - Move to Garden
            this.MoveToAutoPollenStorage(CurrCollectionIdx);
            // TBD - Pickup New Balls
            this.IntakeNewBalls();
            AreBallsLoaded = true;
            // TBD - Return to Launch Position
            this.ReturnFromPollenStorageToLaunchArea(CurrCollectionIdx);
            // Check for Hive Tipped
            IsHiveTipped = this.IsHiveTipped();
            if(IsHiveTipped)
            {
                break;
            }
            // TBD - Launch Balls
            this.LaunchAutoBalls(10);
            AreBallsLoaded = false;
            CurrCollectionIdx++;
        }
        if(IsHiveTipped && AreBallsLoaded) {
            // Go to Opposite Hive Side
        }
        else if(IsHiveTipped)
        // Load Balls and go to Opposite Hive Side
        {

        }
        else // Hive is not Tipped
        {

        }
    }

    public void IntakeNewBalls()
    {
        float IntakeSpeed_rpm = -900;
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
    public void MoveToAutoPollenStorage(int SequenceNum)
    {
        switch(SequenceNum) {
            case 1:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
            case 2:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
            case 3:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
        }
    }
    public void ReturnFromPollenStorageToLaunchArea(int SequenceNum)
    {
        switch(SequenceNum) {
            case 1:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
            case 2:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
            case 3:
                // Action 1
                this.RobotMoveDistance(24, 0, 0, 5);
                break;
        }
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
                this.LaunchAutoBalls(10);
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
                this.LaunchAutoBalls(10);
                break;
            case 4:
                // Action 1
                this.RobotMoveDistance(12,0,0,5);
                // Action 2
                this.RobotMoveDistance(0, 0, 24, 5);
                this.LaunchAutoBalls(10);
                break;
            case 5:
                // Action 1
                this.RobotMoveDistance(24,0,0,5);
                //Action 2
                this.RobotMoveDistance(0, 0, 80, 5);
                //Action 3
                this.RobotMoveDistance(36, 0, 0, 5);
                this.LaunchAutoBalls(10);
                break;
        }
    }

    public void DetectColors()
    {
        DetectColors(true);
    }
    private void Wait(double Time_s)
    {
        ElapsedTime WaitTimer = new ElapsedTime();
        while (WaitTimer.seconds() < Time_s && !Thread.currentThread().isInterrupted()) {
            telemetry.addData("Waiting | Time Left: ", Time_s- WaitTimer.seconds());
            telemetry.update();
        }
    }
    public void LaunchAutoBalls(double Time_s)
    {
        ElapsedTime autoLaunchTimer = new ElapsedTime();
        try {
            while (autoLaunchTimer.seconds() < Time_s && !Thread.currentThread().isInterrupted()) {
                LaunchControl(true);
                telemetry.update();
            }
        } finally {
            LaunchControl(false);
        }
    }
    private void DetectColors(boolean reportTelemetry)
    {
        int red = BallColor.red();
        int green = BallColor.green();
        int blue = BallColor.blue();
        if (reportTelemetry) {
            telemetry.addData("Detected Red: ",red);
            telemetry.addData(" | Green: ",green);
            telemetry.addData(" | Blue: ",blue);
            telemetry.addLine();
        }
        int VisibleThreshold = 300;
        ballVisible = red > VisibleThreshold || blue > VisibleThreshold || green > VisibleThreshold;
        if (ballVisible)
        { //Ball Detected
            if (blue > 2 * green && blue > 2 * red)
            {// Probably a Blue Ball
                LastSeenBall = BallType.NECTAR;
            }
            else if(red > green && red > 2 * blue)
            { // Probably Red
                LastSeenBall = BallType.NECTAR;
            }
            else
            { // We think by elimination it's yellow
                LastSeenBall = BallType.POLLEN;
            }
        }
        LED1.setState(!(ballVisible && LastSeenBall == BallType.NECTAR));
        if (reportTelemetry) {
            if (LastSeenBall == BallType.NECTAR) {
                telemetry.addLine("Nectar Detected");
            } else if (LastSeenBall == BallType.POLLEN) {
                telemetry.addLine("Pollen Detected");
            } else {
                telemetry.addLine("Unknown Detected");
            }
        }
    }

    private void UpdateBallQueue(boolean reportTelemetry)
    {
        DetectColors(reportTelemetry);
        ballQueue.update(intakeRunning, gateOpen, ballQueueTimer.seconds(),
                ballVisible ? LastSeenBall : null);
        if (reportTelemetry) {
            SetAUTOThroatPosition();
            telemetry.addData("Ball Queue (oldest first)", ballQueue);
        } else if (ballQueue.oldest() == BallType.NECTAR) {
            SetThroatToNectarPosition();
        } else {
            SetThroatToPollenPosition();
        }
    }
    public void SetAUTOThroatPosition()
    {
        if (ballQueue.oldest() == BallType.NECTAR) {
            this.SetThroatToNectarPosition();
            telemetry.addLine("NECTAR Position");
        }
        else {
            this.SetThroatToPollenPosition();
            telemetry.addLine("POLLEN Position");
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
                this.RaiseDozerRamp();
                IsFlowerDown = false;
            }
            else {
                this.LowerDozerRamp();
                IsFlowerDown = true;
            }
        }
    }
    public void SetRCLaunchCommand()
    {
        // Launcher
        float LauncherSpeedCmd = 0;
        if (gamepad2.left_bumper)
        {
            LauncherSpeedCmd = -800; //MaxLauncherSpeed
        }

        Launcher.setVelocity(LauncherSpeedCmd);
        telemetry.addData("Manual Launcher Velocity: (%.2f)",LauncherSpeedCmd);
    }
    public void SetRCIntakeCommand()
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
        SetIntakeVelocity(IntakeVelocityCmd);
    }
    public void LaunchControl(boolean IsActive)
    {
        final double spinupTimeoutSeconds = 2.0;
        final double gateOpenDelaySeconds = 1.0;
        final double launcherSetpointFraction = 1.0;
        final double maxYawErrorDegrees = 5.0;
        // Detect April Tags
        GoalLocation = this.UpdateGoalLocation(ActiveAlliance);
        // Keep the ball Queue Up to date
        this.UpdateBallQueue(true);

        if(SetLauncherSpeedRoutine) {
            if (gamepad2.xWasPressed()) {
                autoLauncherSpeed = Math.min(MAX_AUTO_LAUNCHER_SPEED, autoLauncherSpeed + LAUNCHER_SPEED_STEP);
            }
            if (gamepad2.yWasPressed()) {
                autoLauncherSpeed = Math.max(MIN_AUTO_LAUNCHER_SPEED, autoLauncherSpeed - LAUNCHER_SPEED_STEP);
            }
        }
        final double launcherSpeedCommand = autoLauncherSpeed;
        final double intakeSpeedCommand = MaxIntakeVelocity;

        if (GoalLocation != null) {
            gamepad1.rumbleBlips(100);
            gamepad2.rumbleBlips(100);
        }
        else {
            gamepad1.stopRumble();
            gamepad2.stopRumble();
        }

        if (!IsActive) {
            launchGateOpen = false;
            if (launchState != LaunchState.Idle) {
                launchState = LaunchState.Idle;
                Launcher.setVelocity(0);
                SetIntakeVelocity(0);
            } else {
                SetRCLaunchCommand();
                SetRCIntakeCommand();
                SetRCTurretYawCommand();
                CloseGate();
            }
        } else {
            boolean yawAligned = false;
            if (GoalLocation != null) {
                SetAutoTurretYawCommand();
                yawAligned = Math.abs(GoalLocation.ftcPose.bearing) <= maxYawErrorDegrees;
            } else {
                SetTurretYawCommand(0);

            }

            if (launchState == LaunchState.Idle) {
                launchState = LaunchState.LauncherSpinup;
                launchTimer.reset();
            }

            if (launchState == LaunchState.LauncherSpinup) {
                double launcherVelocity = Launcher.getVelocity();
                double launcherSpeedThreshold = launcherSpeedCommand * launcherSetpointFraction;
                boolean atSetpoint = launcherSpeedCommand < 0
                        ? launcherVelocity <= launcherSpeedThreshold
                        : launcherVelocity >= launcherSpeedThreshold;
                if (yawAligned && (launchTimer.seconds() >= spinupTimeoutSeconds || atSetpoint)) {
                    launchState = LaunchState.OpenGates;
                }
            } else if (launchState != LaunchState.Launch && !yawAligned) {
                launchState = LaunchState.OpenGates;
                launchGateOpen = false;
            }

            if (launchState == LaunchState.OpenGates && yawAligned) {
                if (!launchGateOpen) {
                    launchTimer.reset();
                    launchGateOpen = true;
                } else if (launchTimer.seconds() >= gateOpenDelaySeconds) {
                    launchState = LaunchState.Launch;
                }
            }

            Launcher.setVelocity(launcherSpeedCommand);
            telemetry.addData("Auto Launcher Velocity (ticks/sec)", Launcher.getVelocity());
            if (!launchGateOpen) {
                CloseGate();
            } else {
                OpenGate();
            }
            SetIntakeVelocity(launchState == LaunchState.Launch ? intakeSpeedCommand : 0);
        }
        UpdateGoalIndicator(GoalLocation != null);
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Auto Launcher Setpoint (ticks/sec)", autoLauncherSpeed);
    }
    public void SetRCTurretYawCommand()
    {
        float TurretYawVelocity = gamepad2.right_stick_x*MaxTurretYawSpeed;
        this.SetTurretYawCommand(TurretYawVelocity);
    }
    public void SetAutoTurretYawCommand()
    {
        float Kp = 250f;
        float TurretYawVelocity  =- (float)(GoalLocation.ftcPose.bearing) * Kp;
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
    public void UpdateManualDozerPosition()
    {
        if(gamepad2.xWasPressed())
        {
            DozerPosition += + 0.01f;
        } else if (gamepad2.yWasPressed()) {

            DozerPosition -= + 0.01f;
        }
        if(DozerPosition > 1) {
            DozerPosition = 1;
        }
        else if(DozerPosition < 0) {
            DozerPosition = 0;
        }
        telemetry.addData("Dozer Position = ",DozerPosition);
    }
    public void UpdateManualThroatPosition()
    {
        if(gamepad2.xWasPressed())
        {
            ThroatPosition += + 0.01f;
        } else if (gamepad2.yWasPressed()) {

            ThroatPosition -= + 0.01f;
        }
        if(ThroatPosition > 1) {
            ThroatPosition = 1;
        }
        else if(ThroatPosition < 0) {
            ThroatPosition = 0;
        }
        telemetry.addData("Throat Position = ",ThroatPosition);
    }
    public void RunTeleOp()
    {

        this.SetRampCommand();

        // Manual Calibrations, Normally Off
        if(SetDozerPositionRoutine)
        { this.UpdateManualDozerPosition();}
        if(SetThroatPositionRoutine)
        { this.UpdateManualThroatPosition();}



        this.SetDriveCommands();
        if( GoalLocation == null){
            telemetry.addLine("Targeting Status: NOT READY");
        }
        else
        {
            telemetry.addData("Targeting Status: READY | Bearing: ",GoalLocation.ftcPose.bearing);
        }
        if(SetThroatPositionRoutine)
        {
            LauncherThroat.setPosition(DozerPosition);
        }
        else
        {
            this.LaunchControl(gamepad2.right_bumper);
        }


        telemetry.update();
        //telemetry.addData("Dozer Position: (%.2f)",DozerPosition);






    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
