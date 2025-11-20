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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="StarterBotAuto", group="MecanumBot")
//@Disabled
public class Auto extends OpMode
{

    final double FEED_TIME = 0.40; //The feeder servos run this long when a shot is requested.
    final double LAUNCHER_TARGET_VELOCITY = 1295;
    final double LAUNCHER_MIN_VELOCITY = 1175;
    final double TIME_BETWEEN_SHOTS = 2;
    final double DRIVE_SPEED = 0.5;
    final double STRAFE_SPEED = 0.5;
    final double TURN_SPEED = 0.4;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double ROBOT_WIDTH_MM = 450; // Adjust this to your robot's width (track width)

    int shotsToFire = 3; //The number of shots to fire in this auto.

    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();

    // Declare OpMode members for mecanum wheels
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private enum LaunchState {
        IDLE,
        PREPARE,
        LAUNCH,
    }


    private LaunchState launchState;

    private enum AutonomousState {
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        STRAFING,
        NO_LAUNCH_STRAIGHT,
        COMPLETE;
    }

    private AutonomousState autonomousState;

    /*
     * Here we create an enum not to create a state machine, but to capture which alliance we are on.
     */
    private enum Alliance {
        RED,
        BLUE;
    }

    /*
     * Enum to select autonomous mode
     */
    private enum AutoMode {
        LAUNCH_RED,
        LAUNCH_BLUE,
        NO_LAUNCH;
    }

    /*
     * When we create the instance of our enum we can also assign a default state.
     */
    private Alliance alliance = Alliance.RED;
    private AutoMode autoMode = AutoMode.LAUNCH_RED;

    /*
     * This code runs ONCE when the driver hits INIT.
     */
    @Override
    public void init() {
        /*
         * Here we set the first step of our autonomous state machine by setting autoStep = AutoStep.LAUNCH.
         * Later in our code, we will progress through the state machine by moving to other enum members.
         * We do the same for our launcher state machine, setting it to IDLE before we use it later.
         */
        autonomousState = AutonomousState.LAUNCH;
        launchState = LaunchState.IDLE;


        /*
         * Initialize the hardware variables
         */
        frontLeftDrive = hardwareMap.get(DcMotor.class, "left_drive_front");
        frontRightDrive = hardwareMap.get(DcMotor.class, "right_drive_front");
        backLeftDrive = hardwareMap.get(DcMotor.class, "left_drive_back");
        backRightDrive = hardwareMap.get(DcMotor.class, "right_drive_back");
        launcher = hardwareMap.get(DcMotorEx.class, "launch_motor");
        leftFeeder = hardwareMap.get(CRServo.class, "left_servo");
        rightFeeder = hardwareMap.get(CRServo.class, "right_servo");


        /*
         * For mecanum wheels, we need to reverse the right side motors.
         * This assumes standard mecanum wheel orientation.
         */
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        frontLeftDrive.setZeroPowerBehavior(BRAKE);
        frontRightDrive.setZeroPowerBehavior(BRAKE);
        backLeftDrive.setZeroPowerBehavior(BRAKE);
        backRightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Here we set the aforementioned PID coefficients.
         */
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,new PIDFCoefficients(300,0,0,10));

        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // set launcher to spin the right way
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * This code runs REPEATEDLY after the driver hits INIT, but before they hit START.
     */
    @Override
    public void init_loop() {
        /*
         * We also set the servo power to 0 here to make sure that the servo controller is booted
         * up and ready to go.
         */
        rightFeeder.setPower(0);
        leftFeeder.setPower(0);


        /*
         * Here we allow the driver to select which mode using the gamepad.
         */
        if (gamepad1.b) {
            autoMode = AutoMode.LAUNCH_RED;
            alliance = Alliance.RED;
        } else if (gamepad1.x) {
            autoMode = AutoMode.LAUNCH_BLUE;
            alliance = Alliance.BLUE;
        } else if (gamepad1.y) {
            autoMode = AutoMode.NO_LAUNCH;
            alliance = Alliance.RED;
        } else if (gamepad1.a) {
            autoMode = AutoMode.NO_LAUNCH;
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press B", "for LAUNCH RED");
        telemetry.addData("Press X", "for LAUNCH BLUE");
        telemetry.addData("Press Y", "for NO LAUNCH RED");
        telemetry.addData("Press A", "for NO LAUNCH BLUE");
        telemetry.addData("Selected Mode", autoMode);
        telemetry.addData("Alliance", alliance);
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        // Set initial state based on selected mode
        if (autoMode == AutoMode.NO_LAUNCH) {
            autonomousState = AutonomousState.NO_LAUNCH_STRAIGHT;
        } else {
            autonomousState = AutonomousState.LAUNCH;
        }
    }

    /*
     * This code runs REPEATEDLY after the driver hits START but before they hit STOP.
     */
    @Override
    public void loop() {
        switch (autonomousState){
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if(launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                if(drive(DRIVE_SPEED, -20, DistanceUnit.INCH, 1)){
                    frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                // Rotate 20 degrees for RED, -20 degrees for BLUE
                double rotationDegrees = (alliance == Alliance.RED) ? 20 : -20;

                if(rotate(TURN_SPEED, rotationDegrees, 1)){
                    frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.STRAFING;
                }
                break;

            case STRAFING:
                // Strafe right for RED, strafe left for BLUE
                double strafeDistance = (alliance == Alliance.RED) ? 23 : -23;

                if(strafe(STRAFE_SPEED, strafeDistance, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            /*
             * NO LAUNCH mode - just drives straight forward 20 inches
             */
            case NO_LAUNCH_STRAIGHT:
                if(drive(DRIVE_SPEED, 35, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Telemetry updated for mecanum wheels
         */
        telemetry.addData("AutoMode", autoMode);
        telemetry.addData("Alliance", alliance);
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("FL Position", frontLeftDrive.getCurrentPosition());
        telemetry.addData("FR Position", frontRightDrive.getCurrentPosition());
        telemetry.addData("BL Position", backLeftDrive.getCurrentPosition());
        telemetry.addData("BR Position", backRightDrive.getCurrentPosition());
        telemetry.update();
    }

    /*
     * This code runs ONCE after the driver hits STOP.
     */
    @Override
    public void stop() {
    }

    /**
     * Launches one ball, when a shot is requested spins up the motor and once it is above a minimum
     * velocity, runs the feeder servos for the right amount of time to feed the next ball.
     * @param shotRequested "true" if the user would like to fire a new shot, and "false" if a shot
     *                      has already been requested and we need to continue to move through the
     *                      state machine and launch the ball.
     * @return "true" for one cycle after a ball has been successfully launched, "false" otherwise.
     */
    boolean launch(boolean shotRequested){
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

                    if(shotTimer.seconds() > TIME_BETWEEN_SHOTS){
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
        }
        return false;
    }

    /**
     * Drives the robot forward/backward using all four mecanum wheels.
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // Set same target for all wheels for straight driving
        frontLeftDrive.setTargetPosition((int) targetPosition);
        frontRightDrive.setTargetPosition((int) targetPosition);
        backLeftDrive.setTargetPosition((int) targetPosition);
        backRightDrive.setTargetPosition((int) targetPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

        // Check if front left wheel is within tolerance
        if(Math.abs(targetPosition - frontLeftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Strafes the robot left/right using mecanum wheels.
     * @param speed From 0-1
     * @param distance In specified unit (positive = right, negative = left)
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean strafe(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        // For strafing right: FL and BR go forward, FR and BL go backward
        // For strafing left: FL and BR go backward, FR and BL go forward
        frontLeftDrive.setTargetPosition((int) targetPosition);
        frontRightDrive.setTargetPosition((int) -targetPosition);
        backLeftDrive.setTargetPosition((int) -targetPosition);
        backRightDrive.setTargetPosition((int) targetPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

        // Check if front left wheel is within tolerance
        if(Math.abs(targetPosition - frontLeftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * Rotates the robot in place.
     * @param speed From 0-1
     * @param degrees Degrees to rotate (positive = clockwise, negative = counter-clockwise)
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean rotate(double speed, double degrees, double holdSeconds) {
        final double TOLERANCE_MM = 10;

        // Calculate arc length for rotation: arc = (degrees/360) * pi * robotWidth
        double arcLengthMm = (Math.abs(degrees) / 360.0) * Math.PI * ROBOT_WIDTH_MM;
        double targetTicks = arcLengthMm * TICKS_PER_MM;

        // For clockwise rotation (positive degrees): left side forward, right side backward
        // For counter-clockwise rotation (negative degrees): left side backward, right side forward
        if (degrees > 0) {
            // Clockwise
            frontLeftDrive.setTargetPosition((int) targetTicks);
            frontRightDrive.setTargetPosition((int) -targetTicks);
            backLeftDrive.setTargetPosition((int) targetTicks);
            backRightDrive.setTargetPosition((int) -targetTicks);
        } else {
            // Counter-clockwise
            frontLeftDrive.setTargetPosition((int) -targetTicks);
            frontRightDrive.setTargetPosition((int) targetTicks);
            backLeftDrive.setTargetPosition((int) -targetTicks);
            backRightDrive.setTargetPosition((int) targetTicks);
        }

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);

        // Check if front left wheel is within tolerance
        if(Math.abs(Math.abs(frontLeftDrive.getTargetPosition()) - Math.abs(frontLeftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}