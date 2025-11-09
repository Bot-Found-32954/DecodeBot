package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Modified controls:
 * GAMEPAD 1 (Driver):
 * - Left stick Y: Forward/backward
 * - Right stick X: Turn left/right
 *
 * GAMEPAD 2 (Operator):
 * - Y button: Start launcher motor continuously
 * - B button: Stop launcher motor
 * - X button: Run feeders (only works if launcher is running)
 */

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class Teleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.40; // Time feeders run to launch one artifact
    final double COOLDOWN_TIME_SECONDS = 0.5; // Cooldown time between launches
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target velocity that the launcher should run at.
     */
    final double LAUNCHER_TARGET_VELOCITY = -1275;

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // Track whether launcher is running
    private boolean launcherRunning = false;

    // Track feeder state
    private boolean feedersRunning = false;
    private boolean inCooldown = false;
    private ElapsedTime feederTimer = new ElapsedTime();
    private boolean xButtonPreviouslyPressed = false;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER run-mode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() { }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() { }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // GAMEPAD 1: Drive control
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        /*
         * GAMEPAD 2: Y button - Start launcher motor continuously
         */
        if (gamepad2.y) {
            launcherRunning = true;
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        }

        /*
         * GAMEPAD 2: B button - Stop launcher motor
         */
        if (gamepad2.b) {
            launcherRunning = false;
            launcher.setPower(0);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*
         * GAMEPAD 2: X button - Run feeders for a timed duration (only if launcher is running)
         * Detects button press (not hold) to start feeding cycle
         * Includes cooldown between launches
         */
        boolean xButtonCurrentlyPressed = gamepad2.x;

        // Detect X button press (transition from not pressed to pressed)
        if (xButtonCurrentlyPressed && !xButtonPreviouslyPressed && launcherRunning && !feedersRunning && !inCooldown) {
            // Start feeding cycle
            feedersRunning = true;
            feederTimer.reset();
        }

        xButtonPreviouslyPressed = xButtonCurrentlyPressed;

        // Control feeders based on timer
        if (feedersRunning) {
            if (feederTimer.seconds() < FEED_TIME_SECONDS) {
                // Still feeding
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
            } else {
                // Feeding time complete, stop feeders and start cooldown
                feedersRunning = false;
                inCooldown = true;
                feederTimer.reset();
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }
        } else if (inCooldown) {
            // In cooldown period
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);

            if (feederTimer.seconds() >= COOLDOWN_TIME_SECONDS) {
                // Cooldown complete
                inCooldown = false;
            }
        } else {
            // Not in feeding cycle or cooldown
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
        }

        // Telemetry for debugging
        telemetry.addData("Status", "Driver: GP1 | Operator: GP2");
        telemetry.addData("Launcher Status", launcherRunning ? "RUNNING" : "STOPPED");
        telemetry.addData("Launcher Velocity", launcher.getVelocity());

        String feederStatus;
        if (feedersRunning) {
            feederStatus = "FEEDING";
        } else if (inCooldown) {
            feederStatus = "COOLDOWN";
        } else {
            feederStatus = "READY";
        }
        telemetry.addData("Feeders Status", feederStatus);

        if (feedersRunning || inCooldown) {
            telemetry.addData("Timer", "%.2f sec", feederTimer.seconds());
        }

        telemetry.addData("Feeder Power", "L: %.1f  R: %.1f",
                leftFeeder.getPower(), rightFeeder.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { }

    void arcadeDrive(double forward, double rotate) {
        leftPower = forward - rotate;
        rightPower = forward + rotate;

        /*
         * Send calculated power to wheels
         */
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}