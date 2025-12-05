package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a mecanum drive
 * system for robot mobility (allows strafing), one high-speed motor driving two "launcher wheels",
 * and two servos which feed that launcher.
 *
 * Modified controls:
 * GAMEPAD 1 (Driver):
 * - Left stick Y: Forward/backward (field-centric)
 * - Left stick X: Strafe left/right (field-centric)
 * - Right stick X: Turn left/right
 * - Options button: Reset heading (make current direction "forward")
 *
 * GAMEPAD 2 (Operator):
 * - Right trigger: Hold to run launcher motor (releases when let go)
 * - Left trigger: Hold to run intake motor forward AND feeders in reverse (releases when let go)
 * - Left bumper: Hold to run intake motor in reverse (outtake) (releases when let go)
 * - X button: Run feeders forward (only works if launcher has been running for 1.5+ seconds)
 */

@TeleOp(name = "Teleop", group = "StarterBot")
//@Disabled
public class Teleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.40; // Time feeders run to launch one artifact
    final double COOLDOWN_TIME_SECONDS = 0.5; // Cooldown time between launches
    final double LAUNCHER_WARMUP_TIME = 1.5; // Time for launcher to reach full speed before feeding
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double DRIVE_SPEED_MULTIPLIER = 2; // Speed multiplier for mecanum drive (overclock)

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target velocity that the launcher should run at.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1250;

    // Declare OpMode members for mecanum drive
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor intake = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    // IMU for field-centric drive
    private com.qualcomm.robotcore.hardware.IMU imu = null;

    // Track whether launcher is running
    private boolean launcherRunning = false;
    private ElapsedTime launcherTimer = new ElapsedTime();

    // Track feeder state
    private boolean feedersRunning = false;
    private boolean inCooldown = false;
    private ElapsedTime feederTimer = new ElapsedTime();
    private boolean xButtonPreviouslyPressed = false;

    // Setup variables for drive wheel power levels for telemetry
    double frontLeftPower;
    double frontRightPower;
    double backLeftPower;
    double backRightPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step. Standard mecanum naming convention.
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        backLeftDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_drive_back");
        launcher = hardwareMap.get(DcMotorEx.class, "launch_motor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFeeder = hardwareMap.get(CRServo.class, "left_servo");
        rightFeeder = hardwareMap.get(CRServo.class, "right_servo");

        /*
         * Initialize the IMU with standard parameters for field-centric drive
         */
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

        /*
         * For mecanum drive, motors on the right side typically need to be reversed.
         * Adjust these based on your robot's actual configuration after first test drive.
         */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER run-mode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

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
        telemetry.addData("Status", "Initialized - Mecanum Drive");
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
        // GAMEPAD 1: Reset heading with options button
        if (gamepad1.options) {
            imu.resetYaw();
        }

        // GAMEPAD 1: Field-centric mecanum drive control
        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        /*
         * GAMEPAD 2: Right trigger - Hold to run launcher motor, release to stop
         */
        if (gamepad2.right_trigger > 0.1) { // Trigger threshold to avoid accidental activation
            if (!launcherRunning) {
                // Just started the launcher
                launcherRunning = true;
                launcherTimer.reset();
            }
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        } else {
            // Trigger released, stop launcher
            if (launcherRunning) {
                launcherRunning = false;
            }
            launcher.setPower(0);
            launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        /*
         * GAMEPAD 2: Left trigger - Hold to run intake motor forward AND feeders in reverse
         */
        boolean intakeActive = gamepad2.left_trigger > 0.1;
        boolean outtakeActive = gamepad2.left_bumper;

        if (intakeActive) {
            intake.setPower(FULL_SPEED);
            // Run feeders in REVERSE during intake (opposite of launch direction)
            if (!feedersRunning && !inCooldown) {
                leftFeeder.setPower(-FULL_SPEED);
                rightFeeder.setPower(-FULL_SPEED);
            }
        } else if (outtakeActive) {
            /*
             * GAMEPAD 2: Left bumper - Hold to run intake motor in reverse (outtake)
             */
            intake.setPower(-FULL_SPEED);
            if (!feedersRunning && !inCooldown) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }
        } else {
            intake.setPower(0);
            if (!feedersRunning && !inCooldown) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }
        }

        // Check if launcher has warmed up (been running for at least 1.5 seconds)
        boolean launcherWarmedUp = launcherRunning && (launcherTimer.seconds() >= LAUNCHER_WARMUP_TIME);

        /*
         * GAMEPAD 2: X button - Run feeders for a timed duration
         * Only works if launcher has been warmed up for 1.5+ seconds
         * Detects button press (not hold) to start feeding cycle
         * Includes cooldown between launches
         */
        boolean xButtonCurrentlyPressed = gamepad2.x;

        // Detect X button press (transition from not pressed to pressed)
        if (xButtonCurrentlyPressed && !xButtonPreviouslyPressed && launcherWarmedUp && !feedersRunning && !inCooldown) {
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
                intake.setPower(-FULL_SPEED);
            } else {
                // Feeding time complete, stop feeders and start cooldown
                feedersRunning = false;
                inCooldown = true;
                feederTimer.reset();
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
                intake.setPower(0);
            }
        } else if (inCooldown) {
            // In cooldown period
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            intake.setPower(0);

            if (feederTimer.seconds() >= COOLDOWN_TIME_SECONDS) {
                // Cooldown complete
                inCooldown = false;
            }
        }

        // Telemetry for debugging
        telemetry.addData("Status", "Driver: GP1 | Operator: GP2");
        telemetry.addData("Heading", "%.2f degrees", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("Drive", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                frontLeftPower, frontRightPower, backLeftPower, backRightPower);
        telemetry.addData("Launcher Status", launcherRunning ? "RUNNING" : "STOPPED");

        if (launcherRunning) {
            double warmupTime = launcherTimer.seconds();
            if (warmupTime < LAUNCHER_WARMUP_TIME) {
                telemetry.addData("Launcher Warmup", "%.1f / %.1f sec", warmupTime, LAUNCHER_WARMUP_TIME);
            } else {
                telemetry.addData("Launcher", "READY TO FEED");
            }
        }

        telemetry.addData("Launcher Velocity", launcher.getVelocity());

        String feederStatus;
        if (feedersRunning) {
            feederStatus = "FEEDING";
        } else if (inCooldown) {
            feederStatus = "COOLDOWN";
        } else if (launcherRunning && launcherTimer.seconds() < LAUNCHER_WARMUP_TIME) {
            feederStatus = "WARMING UP";
        } else if (launcherRunning) {
            feederStatus = "READY";
        } else {
            feederStatus = "LAUNCHER OFF";
        }
        telemetry.addData("Feeders Status", feederStatus);

        if (feedersRunning || inCooldown) {
            telemetry.addData("Timer", "%.2f sec", feederTimer.seconds());
        }

        telemetry.addData("Feeder Power", "L: %.1f  R: %.1f",
                leftFeeder.getPower(), rightFeeder.getPower());

        String intakeStatus;
        if (gamepad2.left_trigger > 0.1) {
            intakeStatus = "INTAKE";
        } else if (gamepad2.left_bumper) {
            intakeStatus = "OUTTAKE";
        } else {
            intakeStatus = "STOPPED";
        }
        telemetry.addData("Intake Status", intakeStatus);
        telemetry.addData("Intake Power", "%.1f", intake.getPower());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { }

    /*
     * Field-centric mecanum drive method
     * Transforms driver inputs based on robot's heading so forward is always forward relative to field
     * @param forward - forward/backward movement (left stick Y)
     * @param strafe - left/right strafing (left stick X)
     * @param rotate - rotation (right stick X)
     */
    void mecanumDrive(double forward, double strafe, double rotate) {
        // Get robot heading from IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction based on robot heading for field-centric control
        double rotatedForward = forward * Math.cos(botHeading) - strafe * Math.sin(botHeading);
        double rotatedStrafe = forward * Math.sin(botHeading) + strafe * Math.cos(botHeading);

        // Apply speed multiplier for overdrive
        rotatedForward *= DRIVE_SPEED_MULTIPLIER;
        rotatedStrafe *= DRIVE_SPEED_MULTIPLIER;
        rotate *= DRIVE_SPEED_MULTIPLIER;

        // Calculate power for each wheel using mecanum drive kinematics
        frontLeftPower = rotatedForward + rotatedStrafe + rotate;
        frontRightPower = rotatedForward - rotatedStrafe - rotate;
        backLeftPower = rotatedForward - rotatedStrafe + rotate;
        backRightPower = rotatedForward + rotatedStrafe - rotate;

        // Normalize wheel powers to ensure no value exceeds 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower),
                                Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}