package com.robotechnix.tripernicus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp
@SuppressWarnings({"unused"})
public class TeleOpHolonomic extends OpMode {
    // The number of omni drive wheels (and motors). In theory, this can work
    // with any number of motors/wheels, but in practice I've only tried it
    // with three.
    final private static int wheelCount = 3;

    // The robot configuration's prefix for motor names (we assume they are
    // named with a common prefix, e.g. as "m0", "m1", ...).
    final private static String motorNamePrefix = "m";

    // The orientation of the robot, which can be set between 0 and 360 to set
    // what the "front" of the robot is (the rest of the code doesn't care).
    final private static double robotOrientationAngle = Math.toRadians(180);

    // An array of DcMotor variables, one for each drive motor.
    private DcMotor driveMotor[] = new DcMotor[wheelCount];

    // Convert the joystick's x/y values to a deflected angle, in radians.
    private static double getJoystickAngle(final double x, final double y) {
        return Math.atan2(x, y);
    }

    // Convert the joystick's x/y values to a deflected magnitude (0.0 to 1.0).
    private static double getJoyStickMagnitude(final double x, final double y) {
        return Math.sqrt(Math.pow(Math.abs(x), 2) + Math.pow(Math.abs(y), 2));
    }

    // The wheel spacing angle is fixed based on the number of motors (e.g.,
    // it is 120° with three motors). Calculate it here for use below.
    final private static double wheelSpacingAngle = Math.toRadians(360 / wheelCount);

    // Calculate the power level for a particular motor to translate the robot
    // at a given angle (in radians) and magnitude (0.0 to 1.0, where 1.0 is
    // as fast as possible).
    private static double calculateTranslationMotorPower(final int motor,
                                                         final double angle,
                                                         final double magnitude) {
        // Calculate the motor's nominal power level for the direction of
        // desired translation, taking into account the robot's orientation
        // angle (which end is the front), and the relative angle of the
        // particular wheel in question.
        double motorPower = Math.sin(robotOrientationAngle +
                                     (wheelSpacingAngle * motor) -
                                     angle);

        // Rescale the motor power based on the desired overall power level (the
        // magnitude of stick deflection).
        return magnitude * motorPower;
    }

    // Calculate a factor to rescale the input double array so that any value
    // has a maximum of |1.0|.
    private static double calculateScalingFactor(double[] ary) {
        double max = 0.0;
        for (double val : ary)
            max = Math.max(max, Math.abs(val));
        return 1.0 / Math.max(1.0, max);
    }

    @Override
    public void init() {
        // Reduce the dead zone to 5% so the joysticks are more sensitive.
        gamepad1.setJoystickDeadzone((float) 0.05);

        // Initialize all drive motors (with the same direction and mode).
        for (int i=0; i < wheelCount; i++) {
            driveMotor[i] = hardwareMap.dcMotor.get(motorNamePrefix + i);
            driveMotor[i].setDirection(DcMotorSimple.Direction.REVERSE);
            driveMotor[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void loop() {
        // Store the gamepad stick values into shorter-named variables.
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double r = gamepad1.right_stick_x;

        // Convert the left stick x/y values into an angle (what direction the
        // stick is being deflected) and magnitude (how far it's being deflected).
        double a = getJoystickAngle(x, y);
        double m = getJoyStickMagnitude(x, y);

        // Calculate the motor power desired for each drive motor for the desired
        // translation angle, mixed with the desired rotation.
        double p[] = new double[3];
        for (int i=0; i < wheelCount; i++) {
            p[i] = calculateTranslationMotorPower(i, a, m) + r;
        }

        // Calculate a scaling factor so that the translation + rotation values
        // do not exceed full power. This allows mixing translation and rotation
        // in a way that drives quite smoothly.
        double pScalingFactor = calculateScalingFactor(p);

        // Tell the motors what to do, applying the scaling factor!
        for (int i=0; i < wheelCount; i++) {
            driveMotor[i].setPower(p[i] * pScalingFactor);
        }

        // Send the control parameters for debugging purposes.
        telemetry.addData("a", a);
        telemetry.addData("m", m);
        telemetry.addData("r", r);
        for (int i=0; i < wheelCount; i++) {
            telemetry.addData("driveMotor[" + i + "]",
                    String.format(Locale.US, "%.0f%%", 100.0 * driveMotor[i].getPower()));
        }
        telemetry.update();
    }
}
