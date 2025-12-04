package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAuto_OTOS_3Piles", group="MecanumBot")
//@Disabled
public class AutoOtos3Piles extends OpMode {

    // ===== Shooter constants (from your existing auto) =====
    final double FEED_TIME = 0.40;
    final double LAUNCHER_TARGET_VELOCITY = 1295;
    final double LAUNCHER_MIN_VELOCITY = 1175;
    final double TIME_BETWEEN_SHOTS = 2.0;

    // ===== OTOS drive tuning (you MUST tune these) =====
    final double kP_XY = 0.035;        // inches -> power
    final double kP_H  = 0.012;        // degrees -> power
    final double MAX_TRANSLATE = 0.55;
    final double MAX_ROTATE    = 0.40;

    final double POS_TOL_IN    = 1.5;  // inches
    final double HEAD_TOL_DEG  = 5.0;  // degrees
    final double HOLD_SEC      = 0.20; // must stay in tolerance this long

    // ===== Pile coordinates (PLACEHOLDERS) =====
    // Coordinate frame: we force start/shoot pose to (0,0,0) at start().
    // +Y = “forward” from your start, +X = “right” from your start.
    //
    // IMPORTANT: These are just placeholders. You should print OTOS pose in TeleOp
    // and record the real x/y when your intake is centered on each pile.
    //
    // If the piles are on your left for red and right for blue, you can flip X with alliance.
    final double[][] PILES_COMMON = new double[][] {
            // x,   y,    h  (h is where you want to end up; 0 = same heading as start)
            {  0,  24,   0 },  // pile 1 (closest)
            {  0,  48,   0 },  // pile 2
            {  0,  72,   0 },  // pile 3 (furthest)
    };

    // ===== Hardware =====
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
    private DcMotorEx launcher;
    private CRServo leftFeeder, rightFeeder;

    // OTOS
    private SparkFunOTOS otos;

    // ===== Timers =====
    private final ElapsedTime shotTimer   = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime holdTimer   = new ElapsedTime();
    private final ElapsedTime intakeTimer = new ElapsedTime();

    // ===== Existing launch state machine =====
    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState = LaunchState.IDLE;

    // ===== Auto state machine =====
    private enum AutoState {
        SHOOT_PRELOAD_3,
        GO_TO_PILE,
        INTAKE_3,
        RETURN_TO_SHOOT,
        SHOOT_TRIP_3,
        NEXT_PILE,
        COMPLETE
    }
    private AutoState autoState = AutoState.SHOOT_PRELOAD_3;

    private int shotsRemaining = 3;
    private int pileIndex = 0;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;

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

        // Motor directions (match your code)
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

        // Run drive open-loop (OTOS provides feedback)
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

        // OTOS
        otos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        initOTOS();

        telemetry.addData("Status", "Initialized (OTOS + Shooter)");
    }

    @Override
    public void init_loop() {
        // simple alliance select if you want it
        if (gamepad1.b) alliance = Alliance.RED;
        if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Press B", "RED");
        telemetry.addData("Press X", "BLUE");
    }

    @Override
    public void start() {
        // Define start shooting position as (0,0,0)
        otos.resetTracking();
        otos.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));

        autoState = AutoState.SHOOT_PRELOAD_3;
        pileIndex = 0;

        shotsRemaining = 3;
        launchState = LaunchState.IDLE;
        holdTimer.reset();
    }

    @Override
    public void loop() {
        SparkFunOTOS.Pose2D p = otos.getPosition();

        switch (autoState) {
            case SHOOT_PRELOAD_3:
                if (shootN(3)) {
                    autoState = AutoState.GO_TO_PILE;
                    holdTimer.reset();
                }
                break;

            case GO_TO_PILE: {
                double[] tgt = getPileTarget(pileIndex);
                if (goToPose(tgt[0], tgt[1], tgt[2])) {
                    stopDrive();
                    // Start placeholder intake
                    intakeStart();          // <-- placeholder (no code)
                    intakeTimer.reset();
                    autoState = AutoState.INTAKE_3;
                }
                break;
            }

            case INTAKE_3:
                // Placeholder: “collect 3” just waits (replace later with sensor logic)
                if (intakeTimer.seconds() >= 1.0) {
                    intakeStop();           // <-- placeholder (no code)
                    autoState = AutoState.RETURN_TO_SHOOT;
                    holdTimer.reset();
                }
                break;

            case RETURN_TO_SHOOT:
                // Return to start position AND start heading, so shooting is aligned like preload shots
                if (goToPose(0, 0, 0)) {
                    stopDrive();
                    shotsRemaining = 3;
                    launchState = LaunchState.IDLE;
                    autoState = AutoState.SHOOT_TRIP_3;
                }
                break;

            case SHOOT_TRIP_3:
                if (shootN(3)) {
                    autoState = AutoState.NEXT_PILE;
                }
                break;

            case NEXT_PILE:
                pileIndex++;
                if (pileIndex >= 3) {
                    autoState = AutoState.COMPLETE;
                    stopAll();
                } else {
                    autoState = AutoState.GO_TO_PILE;
                    holdTimer.reset();
                }
                break;

            case COMPLETE:
                stopAll();
                break;
        }

        telemetry.addData("State", autoState);
        telemetry.addData("PileIndex", pileIndex);
        telemetry.addData("OTOS", "x=%.1f y=%.1f h=%.1f", p.x, p.y, p.h);
        telemetry.addData("LauncherVel", "%.0f", launcher.getVelocity());
        telemetry.update();
    }

    // ===================== OTOS init =====================
    private void initOTOS() {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(0, 0, 0)); // change if your OTOS isn't at robot center
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);
        // Keep robot still while calibrating
        otos.calibrateImu();
        otos.resetTracking();
    }

    // ===================== Placeholder intake =====================
    private void intakeStart() {
        // TODO: implement hardware here
    }

    private void intakeStop() {
        // TODO: implement intake hardware here
    }

    // ===================== Navigation (OTOS closed-loop) =====================
    private boolean goToPose(double tx, double ty, double thDeg) {
        SparkFunOTOS.Pose2D p = otos.getPosition();

        double ex = tx - p.x;
        double ey = ty - p.y;

        double dist = Math.hypot(ex, ey);
        double eh = angleWrapDeg(thDeg - p.h);

        boolean posOk = dist <= POS_TOL_IN;
        boolean headOk = Math.abs(eh) <= HEAD_TOL_DEG;

        if (!(posOk && headOk)) holdTimer.reset();
        if (posOk && headOk && holdTimer.seconds() >= HOLD_SEC) return true;

        // Convert field error into robot-centric commands using current heading
        double headingRad = Math.toRadians(p.h);
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);

        // robotX = strafe error, robotY = forward error
        double robotX = ex * cos - ey * sin;
        double robotY = ex * sin + ey * cos;

        double forward = clip(robotY * kP_XY, -MAX_TRANSLATE, MAX_TRANSLATE);
        double strafe  = clip(robotX * kP_XY, -MAX_TRANSLATE, MAX_TRANSLATE);
        double turn    = clip(eh * kP_H,      -MAX_ROTATE,    MAX_ROTATE);

        driveRobotCentric(forward, strafe, turn);
        return false;
    }

    private void driveRobotCentric(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) { fl/=max; fr/=max; bl/=max; br/=max; }

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
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
        intakeStop();
    }

    // Shooter (launch state machine)
    private boolean shootN(int n) {
        // Ensure we are shooting exactly n shots for this phase
        if (shotsRemaining > n) shotsRemaining = n;
        if (shotsRemaining <= 0) return true;

        // Request shot when idle, otherwise keep advancing
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

    // Pile targets helper
    private double[] getPileTarget(int idx) {
        // If you want red/blue mirrored, flip X here:
        double x = PILES_COMMON[idx][0];
        double y = PILES_COMMON[idx][1];
        double h = PILES_COMMON[idx][2];

        // Example mirror: red piles on left, blue piles on right
        // If your common list assumes "to the right", invert for red, etc.
        if (alliance == Alliance.RED) x = -x;

        return new double[]{x, y, h};
    }

    // Utils
    private double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double angleWrapDeg(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
