package com.robotechnix.tripernicus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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

        // Send the angle and magnitude just for debugging purposes.
        telemetry.addData("a", a);
        telemetry.addData("m", m);
        telemetry.update();

        // Calculate the motor power desired for each drive motor and set its
        // power level appropriately. Combine the translation power directly
        // with the rotation value from r so that the robot can translate and
        // rotate simultaneously. The rotation value could likely be combined
        // in a more intelligent way so that it doesn't overwhelm the overall
        // power level unfairly to make combined translation and rotation more
        // smooth.
        for (int i=0; i < wheelCount; i++) {
            double p = calculateTranslationMotorPower(i, a, m) + r;
            driveMotor[i].setPower(Range.clip(p, -1.0, +1.0));
        }
    }
}
