package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class TeleOp25_26Season extends OpMode {

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
    //private RobotControl Robo;
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
        ((DcMotorEx) FrontLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(10, 0, 0, 18, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackLeftDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 15, MotorControlAlgorithm.PIDF));
        ((DcMotorEx) BackRightDrive).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(15, 5, 0, 15, MotorControlAlgorithm.PIDF));

        // Configure Movement Types
        BallMover.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LeftLauncher.setDirection(DcMotor.Direction.REVERSE);
        RightLauncher.setDirection(DcMotor.Direction.REVERSE);

        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        BackRightDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        BackLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Control Parameters
        SteeringAggressiveness = 1500;
        ForwardAggressiveness = 2500;
        StrafingAggressiveness = 2500;
        InvertControllers = false;
        MaxLauncherSpeed = 3000;
        MaxIntakeVelocity = 2000;
        MaxBallMoverSpeed = 3000;
        LeftStopPosition = 0.75f;
        LeftOpenPosition = 0.5f;
        RightStopPosition = 0.5f;
        RightOpenPosition = 0.75f;
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

        // Remote Control Turning Cmds
        float ClockwiseVelocityCmd = gamepad1.right_stick_x*SteeringAggressiveness;

        
        
        
        // Intake Logic -----------------------------------------------------------------------
        float intakeVelocity;
        if (gamepad2.dpad_down) {
            intakeVelocity = -MaxIntakeVelocity;
        } else if (gamepad2.dpad_up) {
            intakeVelocity = MaxIntakeVelocity;
        } else {
            intakeVelocity = 0;
        }

        // Launcher
        float LauncherSpeedCmd;
        if (gamepad2.left_bumper) {
            LauncherSpeedCmd = MaxLauncherSpeed;
        } else if (gamepad2.right_bumper) {
            LauncherSpeedCmd = MaxLauncherSpeed * 0.54f;
        } else {
            LauncherSpeedCmd = MaxLauncherSpeed * gamepad2.right_trigger;
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
            RightBallStop.setPosition(RightOpenPosition);
            LeftBallStop.setPosition(LeftOpenPosition);
        } else {
            RightBallStop.setPosition(RightStopPosition);
            LeftBallStop.setPosition(LeftStopPosition);
        }
        // Output Commands
        Intake.setVelocity(intakeVelocity);
        BallMover.setVelocity(BallMoverSpeedCmd);
        RightLauncher.setVelocity(-LauncherSpeedCmd);
        LeftLauncher.setVelocity(-LauncherSpeedCmd);
        this.SetDriveMotionCmds(ForwardSpeedCmd, RightSpeedCmd, ClockwiseVelocityCmd);

        // Add some Telemetry
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Velocity Cmds", "Forward (%.2f), Rightward (%.2f), ClockWise (%.2f)",
                ForwardSpeedCmd, RightSpeedCmd,ClockwiseVelocityCmd);
    }
    void SetDriveMotionCmds(float ForwardSpeed, float RightSpeed, float ClockwiseSpeed)
    {
        FrontLeftDrive.setVelocity(ForwardSpeed  - RightSpeed + ClockwiseSpeed);
        BackLeftDrive.setVelocity(ForwardSpeed + RightSpeed + ClockwiseSpeed);
        FrontRightDrive.setVelocity(ForwardSpeed + RightSpeed - ClockwiseSpeed);
        BackRightDrive.setVelocity(ForwardSpeed - RightSpeed - ClockwiseSpeed);
    }
}