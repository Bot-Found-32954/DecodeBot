package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="StarterBotAuto_Simple", group="MecanumBot")
//@Disabled
public class AutoSimple extends OpMode {

    // ===== Shooter constants =====
    final double FEED_TIME = 0.40;
    final double LAUNCHER_TARGET_VELOCITY = 1295;
    final double LAUNCHER_MIN_VELOCITY = 1175;
    final double TIME_BETWEEN_SHOTS = 2.0;

    // ===== Movement constants (tune these) =====
    final double DRIVE_FORWARD_TIME = 2.0;  // seconds to drive straight
    final double DRIVE_FORWARD_POWER = 0.5;
    final double STRAFE_TIME = 0.5;         // seconds to strafe
    final double STRAFE_POWER = 0.4;
    final double ROTATE_TIME = 0.5;         // seconds to rotate
    final double ROTATE_POWER = 0.3;

    // ===== Hardware =====
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // ===== Timers =====
    private final ElapsedTime shotTimer   = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime moveTimer   = new ElapsedTime();

    // ===== Launch state machine =====
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState = LaunchState.IDLE;

    // ===== Auto state machine =====
    private enum AutoState {
        SHOOT_3,
        STRAFE,
        ROTATE,
        COMPLETE
    }
    private AutoState autoState = AutoState.SHOOT_3;

    private int shotsRemaining = 3;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

    private enum StartZone { LAUNCH, NO_LAUNCH }
    private StartZone startZone = StartZone.LAUNCH;

    @Override
    public void init() {
        // Drive motors
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "left_drive_front");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "left_drive_back");
        backRightDrive  = hardwareMap.get(DcMotor.class, "right_drive_back");

        // Shooter hardware
        launcher   = hardwareMap.get(DcMotorEx.class, "launch_motor");
        leftFeeder = hardwareMap.get(CRServo.class, "left_servo");
        rightFeeder= hardwareMap.get(CRServo.class, "right_servo");

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        // Run drive open-loop
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Launcher velocity control
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        // Alliance selection
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.x) alliance = Alliance.BLUE;

        // Start zone selection
        if (gamepad1.dpad_up) startZone = StartZone.LAUNCH;
        if (gamepad1.dpad_down) startZone = StartZone.NO_LAUNCH;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Press B", "RED");
        telemetry.addData("Press X", "BLUE");
        telemetry.addData("Start Zone", startZone);
        telemetry.addData("Press D-Pad Up", "LAUNCH ZONE");
        telemetry.addData("Press D-Pad Down", "NO LAUNCH ZONE");
    }

    @Override
    public void start() {
        if (startZone == StartZone.LAUNCH) {
            autoState = AutoState.SHOOT_3;
            shotsRemaining = 3;
            launchState = LaunchState.IDLE;
        } else {
            autoState = AutoState.STRAFE;
            moveTimer.reset();
        }
    }

    @Override
    public void loop() {
        if (startZone == StartZone.NO_LAUNCH) {
            // No launch zone: just drive straight
            if (autoState != AutoState.COMPLETE) {
                driveForward(DRIVE_FORWARD_POWER);
                if (moveTimer.seconds() >= DRIVE_FORWARD_TIME) {
                    stopDrive();
                    autoState = AutoState.COMPLETE;
                }
            }
        } else {
            // Launch zone: shoot 3, then strafe and rotate
            switch (autoState) {
                case SHOOT_3:
                    if (shootN(3)) {
                        autoState = AutoState.STRAFE;
                        moveTimer.reset();
                    }
                    break;

                case STRAFE:
                    // Strafe direction depends on alliance
                    double strafeDir = (alliance == Alliance.RED) ? -1 : 1;
                    strafe(STRAFE_POWER * strafeDir);
                    if (moveTimer.seconds() >= STRAFE_TIME) {
                        stopDrive();
                        autoState = AutoState.ROTATE;
                        moveTimer.reset();
                    }
                    break;

                case ROTATE:
                    // Rotate direction depends on alliance
                    double rotateDir = (alliance == Alliance.RED) ? -1 : 1;
                    rotate(ROTATE_POWER * rotateDir);
                    if (moveTimer.seconds() >= ROTATE_TIME) {
                        stopDrive();
                        autoState = AutoState.COMPLETE;
                    }
                    break;

                case COMPLETE:
                    stopAll();
                    break;
            }
        }

        telemetry.addData("State", autoState);
        telemetry.addData("Start Zone", startZone);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("LauncherVel", "%.0f", launcher.getVelocity());
        telemetry.update();
    }

    // ===================== Drive commands =====================
    private void driveForward(double power) {
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    private void strafe(double power) {
        // Positive power = strafe right, negative = strafe left
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    private void rotate(double power) {
        // Positive power = rotate clockwise, negative = counter-clockwise
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    private void stopAll() {
        stopDrive();
        launcher.setVelocity(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    // ===================== Shooter (launch state machine) =====================
    private boolean shootN(int n) {
        if (shotsRemaining > n) shotsRemaining = n;
        if (shotsRemaining <= 0) return true;

        boolean request = (launchState == LaunchState.IDLE);
        if (launch(request)) {
            shotsRemaining--;
        }
        return shotsRemaining <= 0;
    }

    private boolean launch(boolean shotRequested){
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;

            case PREPARE:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY){
                    launchState = LaunchState.LAUNCH;
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    feederTimer.reset();
                }
                break;

            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);

                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true; // one shot complete
                    }
                }
                break;
        }
        return false;
    }
}