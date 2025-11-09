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

@Autonomous(name="StarterBotAuto", group="StarterBot")
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

    // Declare OpMode members.
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
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
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the driver's station).
         */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make the robot go forward. So, adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90° drives may require direction flips
         */
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Here we reset the encoders on our drive motors before we start moving.
         */
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode." This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, and it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Here we set the aforementioned PID coefficients. You shouldn't have to do this for any
         * other motors on this robot.
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
        /*
         * TECH TIP: Switch Statements
         * switch statements are an excellent way to take advantage of an enum. They work very
         * similarly to a series of "if" statements, but allow for cleaner and more readable code.
         * We switch between each enum member and write the code that should run when our enum
         * reflects that state. We end each case with "break" to skip out of checking the rest
         * of the members of the enum for a match, since if we find the "break" line in one case,
         * we know our enum isn't reflecting a different state.
         */
        switch (autonomousState){
            /*
             * Since the first state of our auto is LAUNCH, this is the first "case" we encounter.
             * This case is very simple. We call our .launch() function with "true" in the parameter.
             * This "true" value informs our launch function that we'd like to start the process of
             * firing a shot. We will call this function with a "false" in the next case. This
             * "false" condition means that we are continuing to call the function every loop,
             * allowing it to cycle through and continue the process of launching the first ball.
             */
            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                /*
                 * A technique we leverage frequently in this code are functions which return a
                 * boolean. We are using this function in two ways. This function actually moves the
                 * motors and servos in a way that launches the ball, but it also "talks back" to
                 * our main loop by returning either "true" or "false". We've written it so that
                 * after the shot we requested has been fired, the function will return "true" for
                 * one cycle. Once the launch function returns "true", we proceed in the code, removing
                 * one from the shotsToFire variable. If shots remain, we move back to the LAUNCH
                 * state on our state machine. Otherwise, we reset the encoders on our drive motors
                 * and move onto the next state.
                 */
                if(launch(false)) {
                    shotsToFire -= 1;
                    if(shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        launcher.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                /*
                 * This is another function that returns a boolean. This time we return "true" if
                 * the robot has been within a tolerance of the target position for "holdSeconds."
                 * Once the function returns "true" we reset the encoders again and move on.
                 */
                if(drive(DRIVE_SPEED, -20, DistanceUnit.INCH, 1)){
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                //RED now rotates -45 degrees, BLUE now rotates 45 degrees
                if(alliance == Alliance.RED){
                    robotRotationAngle = -95;
                } else if (alliance == Alliance.BLUE){
                    robotRotationAngle = 95;
                }

                if(rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES,1)){
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftDrive.setPower(DRIVE_SPEED);
                rightDrive.setPower(DRIVE_SPEED);

                if (straightTimer.seconds() > 4.0) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    turnTimer.reset();
                    autonomousState = AutonomousState.NO_LAUNCH_TURN;
                }
                break;

            case NO_LAUNCH_TURN:
                // Turn right for RED, turn left for BLUE, then move to final drive
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                if (alliance == Alliance.RED) {
                    // Turn right
                    leftDrive.setPower(ROTATE_SPEED);
                    rightDrive.setPower(-ROTATE_SPEED);
                } else {
                    // Turn left
                    leftDrive.setPower(-ROTATE_SPEED);
                    rightDrive.setPower(ROTATE_SPEED);
                }

                if (turnTimer.seconds() > 1.0) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    driveTimer.reset();
                    autonomousState = AutonomousState.NO_LAUNCH_FINAL;
                }
                break;

            case NO_LAUNCH_FINAL:
                // Drive forward for 6 seconds
                leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                leftDrive.setPower(DRIVE_SPEED);
                rightDrive.setPower(DRIVE_SPEED);

                if (driveTimer.seconds() > 6.0) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;
        }

        /*
         * Here is our telemetry that keeps us informed of what is going on in the robot. Since this
         * part of the code exists outside of our switch statement, it will run once every loop.
         * No matter what state our robot is in. This is the huge advantage of using state machines.
         * We can have code inside of our state machine that runs only when necessary, and code
         * after the last "case" that runs every loop. This means we can avoid a lot of
         * "copy-and-paste" that non-state machine autonomous routines fall into.
         */
        telemetry.addData("AutoMode", autoMode);
        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.addData("Motor Current Positions", "left (%d), right (%d)",
                leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
        telemetry.addData("Motor Target Positions", "left (%d), right (%d)",
                leftDrive.getTargetPosition(), rightDrive.getTargetPosition());
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
     * @param speed From 0-1
     * @param distance In specified unit
     * @param distanceUnit the unit of measurement for distance
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return "true" if the motors are within tolerance of the target position for more than
     * holdSeconds. "false" otherwise.
     */
    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        /*
         * In this function we use a DistanceUnits. This is a class that the FTC SDK implements
         * which allows us to accept different input units depending on the user's preference.
         * To use these, put both a double and a DistanceUnit as parameters in a function and then
         * call distanceUnit.toMm(distance). This will return the number of mm that are equivalent
         * to whatever distance in the unit specified. We are working in mm for this, so that's the
         * unit we request from distanceUnit. But if we want to use inches in our function, we could
         * use distanceUnit.toInches() instead!
         */
        double targetPosition = (distanceUnit.toMm(distance) * TICKS_PER_MM);

        leftDrive.setTargetPosition((int) targetPosition);
        rightDrive.setTargetPosition((int) targetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        /*
         * Here we check if we are within tolerance of our target position or not. We calculate the
         * absolute error (distance from our setpoint regardless of if it is positive or negative)
         * and compare that to our tolerance. If we have not reached our target yet, then we reset
         * the driveTimer. Only after we reach the target can the timer count higher than our
         * holdSeconds variable.
         */
        if(Math.abs(targetPosition - leftDrive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }

    /**
     * @param speed From 0-1
     * @param angle the amount that the robot should rotate
     * @param angleUnit the unit that angle is in
     * @param holdSeconds the number of seconds to wait at position before returning true.
     * @return True if the motors are within tolerance of the target position for more than
     *         holdSeconds. False otherwise.
     */
    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds){
        final double TOLERANCE_MM = 10;

        /*
         * Here we establish the number of mm that our drive wheels need to cover to create the
         * requested angle. We use radians here because it makes the math much easier.
         * Our robot will have rotated one radian when the wheels of the robot have driven
         * 1/2 of the track width of our robot in a circle. This is also the radius of the circle
         * that the robot tracks when it is rotating. So, to find the number of mm that our wheels
         * need to travel, we just need to multiply the requested angle in radians by the radius
         * of our turning circle.
         */
        double targetMm = angleUnit.toRadians(angle)*(TRACK_WIDTH_MM/2);

        /*
         * We need to set the left motor to the inverse of the target so that we rotate instead
         * of driving straight.
         */
        double leftTargetPosition = -(targetMm*TICKS_PER_MM);
        double rightTargetPosition = targetMm*TICKS_PER_MM;

        leftDrive.setTargetPosition((int) leftTargetPosition);
        rightDrive.setTargetPosition((int) rightTargetPosition);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        if((Math.abs(leftTargetPosition - leftDrive.getCurrentPosition())) > (TOLERANCE_MM * TICKS_PER_MM)){
            driveTimer.reset();
        }

        return (driveTimer.seconds() > holdSeconds);
    }
}