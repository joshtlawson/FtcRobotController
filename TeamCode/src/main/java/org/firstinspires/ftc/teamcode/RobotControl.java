package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

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
    ElapsedTime RunTimer_sec = new ElapsedTime();
    public void InitRobot()
    {
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

        // Setup the PID Tuning
        ((DcMotorEx) FrontRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) FrontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 7, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 15, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 15, MotorControlAlgorithm.PIDF));

        // Configure Movement Types
        BackRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BallMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
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

        RotationRatio = 11.1;
        GearRatio = 41.25;
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
    public void RobotMoveAtSpeed(int ForwardVelocity, int CWRotationalVelocity, int RightVelocity, double MoveTime_sec) {
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
    public void LaunchBalls(int SpinUpTime_sec, int LaunchTime_sec, int LauncherSpeed,int BallMoverSpeed) {
        // Spin up the Launcher
        ((DcMotorEx) LeftLauncher).setVelocity(LauncherSpeed);
        ((DcMotorEx) RightLauncher).setVelocity(LauncherSpeed);
        RunTimer_sec.reset();
        while (RunTimer_sec.seconds() < SpinUpTime_sec) {
        }
        // Open the Gates

        ((DcMotorEx) BallMover).setVelocity(BallMoverSpeed);
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
        FrontRightDrive.setVelocity(ForwardSpeed + RightSpeed - ClockwiseSpeed);
        BackRightDrive.setVelocity(ForwardSpeed - RightSpeed - ClockwiseSpeed);
    }
    public void SetLaunchControlCmds(float intakeVelocity, float BallMoverSpeedCmd, float LauncherSpeedCmd)
    {
        Intake.setVelocity(intakeVelocity);
        BallMover.setVelocity(BallMoverSpeedCmd);
        RightLauncher.setVelocity(LauncherSpeedCmd);
        LeftLauncher.setVelocity(LauncherSpeedCmd);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
