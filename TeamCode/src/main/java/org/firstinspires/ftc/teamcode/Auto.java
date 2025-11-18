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

@Autonomous(name="MecanumBotAuto", group="MecanumBot")
//@Disabled
public class Auto extends OpMode
{

    final double FEED_TIME = 0.40; //The feeder servos run this long when a shot is requested.
    final double LAUNCHER_TARGET_VELOCITY = 1295;
    final double LAUNCHER_MIN_VELOCITY = 1175;
    final double TIME_BETWEEN_SHOTS = 2;
    final double DRIVE_SPEED = 0.5;
    final double ROTATE_SPEED = 0.2;
    final double WHEEL_DIAMETER_MM = 96;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3; //The number of shots to fire in this auto.

    double robotRotationAngle = 60;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime feederTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime straightTimer = new ElapsedTime();
    private final ElapsedTime turnTimer = new ElapsedTime();

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
        DRIVING_OFF_LINE,
        NO_LAUNCH_STRAIGHT,
        NO_LAUNCH_TURN,
        NO_LAUNCH_FINAL,
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
        NO_LAUNCH_RED,
        NO_LAUNCH_BLUE;
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
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

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
            autoMode = AutoMode.NO_LAUNCH_RED;
            alliance = Alliance.RED;
        } else if (gamepad1.a) {
            autoMode = AutoMode.NO_LAUNCH_BLUE;
            alliance = Alliance.BLUE;
        }

        telemetry.addData("Press B", "for LAUNCH RED");
        telemetry.addData("Press X", "for LAUNCH BLUE");
        telemetry.addData("Press Y", "for NO LAUNCH RED");
        telemetry.addData("Press A", "for NO LAUNCH BLUE");
        telemetry.addData("Selected Mode", autoMode);
    }

    /*
     * This code runs ONCE when the driver hits START.
     */
    @Override
    public void start() {
        // Set initial state based on selected mode
        if (autoMode == AutoMode.NO_LAUNCH_RED || autoMode == AutoMode.NO_LAUNCH_BLUE) {
            autonomousState = AutonomousState.NO_LAUNCH_STRAIGHT;
            straightTimer.reset();
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
                //RED now rotates -95 degrees, BLUE now rotates 95 degrees
                if(alliance == Alliance.RED){
                    robotRotationAngle = -95;
                } else if (alliance == Alliance.BLUE){
                    robotRotationAngle = 95;
                }

                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                    frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                if(drive(DRIVE_SPEED, -59, DistanceUnit.INCH, 1)){
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            /*
             * NO LAUNCH mode states - drives straight for 4 seconds, then turns and drives for 6 seconds
             */
            case NO_LAUNCH_STRAIGHT:
                // Drive straight for 4 seconds
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                frontLeftDrive.setPower(DRIVE_SPEED);
                frontRightDrive.setPower(DRIVE_SPEED);
                backLeftDrive.setPower(DRIVE_SPEED);
                backRightDrive.setPower(DRIVE_SPEED);

                if (straightTimer.seconds() > 4.0) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    turnTimer.reset();
                    autonomousState = AutonomousState.NO_LAUNCH_TURN;
                }
                break;

            case NO_LAUNCH_TURN:
                // Turn right for RED, turn left for BLUE, then move to final drive
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (alliance == Alliance.RED) {
                    // Turn right - left side forward, right side backward
                    frontLeftDrive.setPower(ROTATE_SPEED);
                    backLeftDrive.setPower(ROTATE_SPEED);
                    frontRightDrive.setPower(-ROTATE_SPEED);
                    backRightDrive.setPower(-ROTATE_SPEED);
                } else {
                    // Turn left - left side backward, right side forward
                    frontLeftDrive.setPower(-ROTATE_SPEED);
                    backLeftDrive.setPower(-ROTATE_SPEED);
                    frontRightDrive.setPower(ROTATE_SPEED);
                    backRightDrive.setPower(ROTATE_SPEED);
                }

                if (turnTimer.seconds() > 1.0) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    driveTimer.reset();
                    autonomousState = AutonomousState.NO_LAUNCH_FINAL;
                }
                break;

            case NO_LAUNCH_FINAL:
                // Drive forward for 6 seconds
                frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                frontLeftDrive.setPower(DRIVE_SPEED);
                frontRightDrive.setPower(DRIVE_SPEED);
                backLeftDrive.setPower(DRIVE_SPEED);
                backRightDrive.setPower(DRIVE_SPEED);

                if (driveTimer.seconds() > 6.0) {
                    frontLeftDrive.setPower(0);
                    frontRightDrive.setPower(0);
                    backLeftDrive.setPower(0);
                    backRightDrive.setPower(0);
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Telemetry updated for mecanum wheels
         */
        telemetry.addData("AutoMode", autoMode);
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
     * Rotates the robot using mecanum wheels.
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     *         holdSeconds. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        final double TOLERANCE_MM = 10;

        double targetMm = angleUnit.toRadians(angle)*(TRACK_WIDTH_MM/2);

        // For rotation: left side goes opposite direction of right side
        double leftTargetPosition = -(targetMm*TICKS_PER_MM);
        double rightTargetPosition = targetMm*TICKS_PER_MM;

        frontLeftDrive.setTargetPosition((int) leftTargetPosition);
        backLeftDrive.setTargetPosition((int) leftTargetPosition);
        frontRightDrive.setTargetPosition((int) rightTargetPosition);
        backRightDrive.setTargetPosition((int) rightTargetPosition);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeftDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backRightDrive.setPower(speed);

        if((Math.abs(leftTargetPosition - frontLeftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}