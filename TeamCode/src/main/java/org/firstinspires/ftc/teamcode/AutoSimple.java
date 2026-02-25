package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="StarterBotAuto_Simple", group="MecanumBot")
//@Disabled
public class AutoSimple extends OpMode {

    // ===== Shooter constants (matching teleop) =====
    final double LAUNCHER_WARMUP_TIME = 0.67;  // Match teleop warmup time
    final double SERVO_MOVE_TIME = 1.7;  // Time to wait for servo to move to position
    final double FEED_TIME = 0.40;
    final double LAUNCHER_TARGET_VELOCITY = 1295;
    final double LAUNCHER_MIN_VELOCITY = 1175;
    final double DELAY_BETWEEN_SHOTS = 3;  // Delay after each shot before next shot starts

    // ===== Servo positions (matching teleop) =====
    final double SERVO_OPEN_POSITION = 0.1567;    // Open for launcher
    final double SERVO_CLOSED_POSITION = 0.4367;  // Closed for intake

    // ===== Movement constants (tune these) =====
    final double INITIAL_BACKUP_TIME = 0.3;    // Time to back up at start
    final double INITIAL_BACKUP_POWER = -0.4;  // Negative = backward
    final double DRIVE_FORWARD_TIME = 1.0;     // seconds to drive straight
    final double DRIVE_FORWARD_POWER = 0.5;
    final double STRAFE_TIME = 0.5;            // seconds to strafe
    final double STRAFE_POWER = 0.4;
    final double ROTATE_TIME = 0.5;            // seconds to rotate
    final double ROTATE_POWER = 0.3;

    // ===== Hardware =====
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx launcher;
    private DcMotor intake;
    private CRServo leftFeeder, rightFeeder;
    private Servo rotationServo;

    // ===== Timers =====
    private final ElapsedTime shotTimer   = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime launcherTimer = new ElapsedTime();
    private final ElapsedTime servoTimer = new ElapsedTime();
    private final ElapsedTime delayTimer = new ElapsedTime();
    private final ElapsedTime moveTimer   = new ElapsedTime();

    // ===== Launch state machine =====
    private enum LaunchState { IDLE, SERVO_MOVE, WARMUP, FEED, DELAY }
    private LaunchState launchState = LaunchState.IDLE;

    // ===== Auto state machine =====
    private enum AutoState {
        INITIAL_BACKUP,
        SHOOT_3,
        STRAFE,
        ROTATE,
        COMPLETE
    }
    private AutoState autoState = AutoState.INITIAL_BACKUP;

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
        intake     = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_servo");
        rightFeeder= hardwareMap.get(CRServo.class, "right_servo");
        rotationServo = hardwareMap.get(Servo.class, "block_servo");

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
        intake.setZeroPowerBehavior(BRAKE);

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

        // Set servo to OPEN position for launcher (matching teleop)
        rotationServo.setPosition(SERVO_OPEN_POSITION);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Servo", "OPEN (Ready for Launch)");
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
            autoState = AutoState.INITIAL_BACKUP;
            shotsRemaining = 3;
            launchState = LaunchState.IDLE;
            moveTimer.reset();
            // Ensure servo is in OPEN position for launch
            rotationServo.setPosition(SERVO_OPEN_POSITION);
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
            // Launch zone: back up, shoot 3, then strafe and rotate
            switch (autoState) {
                case INITIAL_BACKUP:
                    driveForward(INITIAL_BACKUP_POWER);  // Negative power = backward
                    if (moveTimer.seconds() >= INITIAL_BACKUP_TIME) {
                        stopDrive();
                        autoState = AutoState.SHOOT_3;
                        moveTimer.reset();
                    }
                    break;

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
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Shots Remaining", shotsRemaining);
        telemetry.addData("Start Zone", startZone);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("LauncherVel", "%.0f", launcher.getVelocity());
        telemetry.addData("Servo Position", "%.3f", rotationServo.getPosition());
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
        intake.setPower(0);
        leftFeeder.setPower(0);
        rightFeeder.setPower(0);
    }

    // ===================== Shooter (matching teleop procedure) =====================
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
                    // Move servo to OPEN position first
                    rotationServo.setPosition(SERVO_OPEN_POSITION);
                    launchState = LaunchState.SERVO_MOVE;
                    servoTimer.reset();
                }
                break;

            case SERVO_MOVE:
                // Wait for servo to reach position before starting launcher
                if (servoTimer.seconds() >= SERVO_MOVE_TIME) {
                    launchState = LaunchState.WARMUP;
                    shotTimer.reset();
                    launcherTimer.reset();
                    // Start launcher motor after servo is in position
                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                }
                break;

            case WARMUP:
                // Keep launcher running during warmup
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Wait for warmup time AND velocity threshold (matching teleop logic)
                if (launcherTimer.seconds() >= LAUNCHER_WARMUP_TIME &&
                        launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.FEED;
                    // Start feeding (matching teleop: feeders + intake)
                    leftFeeder.setPower(1);
                    rightFeeder.setPower(1);
                    intake.setPower(1);  // Run intake during feeding (matching teleop)
                    feederTimer.reset();
                }
                break;

            case FEED:
                // Keep launcher running during feed
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                // Keep intake running during feed
                intake.setPower(1);

                if (feederTimer.seconds() > FEED_TIME) {
                    // Stop feeders and intake
                    leftFeeder.setPower(0);
                    rightFeeder.setPower(0);
                    intake.setPower(0);

                    // Move to delay state
                    launchState = LaunchState.DELAY;
                    delayTimer.reset();
                }
                break;

            case DELAY:
                // Keep launcher running during delay so it stays warm
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);

                // Wait for delay before allowing next shot
                if (delayTimer.seconds() >= DELAY_BETWEEN_SHOTS) {
                    launchState = LaunchState.IDLE;
                    return true; // one shot complete
                }
                break;
        }
        return false;
    }
}