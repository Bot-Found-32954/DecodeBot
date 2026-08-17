package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/*
 * Teleop (driver-controlled) OpMode for a 4-motor rear-wheel drive robot
 * (2 motors per side, geared/paired together).
 * Uses simple arcade drive (no strafing, no field-centric heading).
 *
 * GAMEPAD 1 (Driver):
 * - Left stick Y: Forward/backward
 * - Right stick X: Turn left/right
 */
@TeleOp(name = "Teleop", group = "RearDrive")
@Config
//@Disabled
public class Teleop extends OpMode {

    final double DRIVE_SPEED_MULTIPLIER = 2; // Speed multiplier for arcade drive (overclock)

    // Declare OpMode members for rear-wheel drive (4 motors, 2 per side)
    private DcMotor leftDriveFront = null;
    private DcMotor leftDriveBack = null;
    private DcMotor rightDriveFront = null;
    private DcMotor rightDriveBack = null;

    // Setup variables for drive wheel power levels for telemetry
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
        leftDriveFront = hardwareMap.get(DcMotor.class, "left_drive_front");
        leftDriveBack = hardwareMap.get(DcMotor.class, "left_drive_back");
        rightDriveFront = hardwareMap.get(DcMotor.class, "right_drive_front");
        rightDriveBack = hardwareMap.get(DcMotor.class, "right_drive_back");

        /*
         * Motors on the right side typically need to be reversed so that positive power
         * on both sides drives the robot forward. Adjust based on your robot's actual
         * configuration after a first test drive.
         */
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotorSimple.Direction.FORWARD);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain, as the robot stops much quicker.
         */
        leftDriveFront.setZeroPowerBehavior(BRAKE);
        leftDriveBack.setZeroPowerBehavior(BRAKE);
        rightDriveFront.setZeroPowerBehavior(BRAKE);
        rightDriveBack.setZeroPowerBehavior(BRAKE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized - Rear Wheel Drive");
        telemetry.addData("", "Press START to begin");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Ready to Start!");
        telemetry.update();
    }

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
        // GAMEPAD 1: Arcade drive control
        arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        // Telemetry for debugging
        telemetry.addData("Status", "Driver: GP1");
        telemetry.addData("Drive", "L:%.2f R:%.2f", leftPower, rightPower);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() { }

    /*
     * Simple arcade drive method for a 4-motor rear-wheel drive robot (2 per side).
     * @param forward - forward/backward movement (left stick Y)
     * @param rotate - rotation (right stick X)
     */
    void arcadeDrive(double forward, double rotate) {
        // Apply speed multiplier for overdrive
        forward *= DRIVE_SPEED_MULTIPLIER;
        rotate *= DRIVE_SPEED_MULTIPLIER;

        // Calculate power for each side using arcade drive kinematics
        leftPower = forward + rotate;
        rightPower = forward - rotate;

        // Normalize wheel powers to ensure no value exceeds 1.0
        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (maxPower > 1.0) {
            leftPower /= maxPower;
            rightPower /= maxPower;
        }

        // Send calculated power to wheels
        leftDriveFront.setPower(leftPower);
        leftDriveBack.setPower(leftPower);
        rightDriveFront.setPower(rightPower);
        rightDriveBack.setPower(rightPower);
    }
}