package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Brake", group = "TeleOp")
public class brake extends OpMode {

    private DcMotorEx leftFrontDrive  = null;
    private DcMotorEx leftBackDrive   = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive  = null;

    private GoBildaPinpointDriver pinpoint = null;

    private static final double TICKS_PER_MOTOR_REV  = 28.0;
    private static final double GEAR_RATIO           = 10.0 / 3.0;
    private static final double TICKS_PER_WHEEL_REV  = TICKS_PER_MOTOR_REV * GEAR_RATIO;

    private static final double Kp_rpm              = 0.001;
    private static final double Ki_rpm              = 0.0;
    private static final double Kd_rpm              = 0.0;
    private static final double RPM_INTEGRAL_CAP    = 0.3;
    private static final double RPM_ACTIVE_THRESHOLD = 20.0;
    private static final double MAX_RPM_CORRECTION  = 0.15;

    private double[] rpmIntegral   = new double[4];
    private double[] rpmLastError  = new double[4];
    private int[]    lastTickCount = new int[4];

    private static final double Kp = 0.3;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;

    private double targetHeading = 0.0;
    private double integralSum   = 0.0;
    private double lastError     = 0.0;
    private static final double INTEGRAL_CAP           = 1.0;
    private static final double YAW_OVERRIDE_THRESHOLD = 0.05;

    private static final double BRAKE_STRENGTH = 0.4;
    private static final double BRAKE_DECAY    = 0.7;

    private double brake_axial   = 0.0;
    private double brake_lateral = 0.0;
    private double brake_yaw     = 0.0;

    private double last_axial   = 0.0;
    private double last_lateral = 0.0;
    private double last_yaw     = 0.0;
    private static final double MAX_CHANGE = 0.3;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime pidTimer  = new ElapsedTime();

    private double applyDeadzone(double joystick) {
        return (Math.abs(joystick) < 0.1) ? 0.0 : joystick;
    }

    private double skew(double input, double last_input) {
        if (Math.signum(input) == 1 && input > last_input + MAX_CHANGE)
            return last_input + MAX_CHANGE;
        else if (Math.signum(input) == -1 && input < last_input - MAX_CHANGE)
            return last_input - MAX_CHANGE;
        else
            return input;
    }

    private double computeBrake(double input, double lastInput, double lastBrake) {
        if (input == 0.0 && lastInput != 0.0) {
            return -lastInput * BRAKE_STRENGTH;
        } else if (input == 0.0 && lastBrake != 0.0) {
            double decayed = lastBrake * BRAKE_DECAY;
            return (Math.abs(decayed) < 0.01) ? 0.0 : decayed;
        } else {
            return 0.0;
        }
    }

    private double computeHeadingCorrection() {
        double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        double error = targetHeading - currentHeading;

        while (error >  Math.PI) error -= 2.0 * Math.PI;
        while (error < -Math.PI) error += 2.0 * Math.PI;

        double dt = Math.min(pidTimer.seconds(), 0.1);
        pidTimer.reset();

        if (Math.abs(error) < Math.toRadians(10)) {
            integralSum += error * dt;
            integralSum  = Math.max(-INTEGRAL_CAP, Math.min(INTEGRAL_CAP, integralSum));
        } else {
            integralSum = 0.0;
        }

        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
        lastError = error;

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }

    private boolean[] getActiveMask(double lf, double rf, double lb, double rb) {
        double threshold = 0.05;
        return new boolean[]{
                Math.abs(lf) > threshold,
                Math.abs(rf) > threshold,
                Math.abs(lb) > threshold,
                Math.abs(rb) > threshold
        };
    }

    private double getMotorRPM(DcMotorEx motor, int motorIndex, double dt) {
        int currentTicks = motor.getCurrentPosition();
        int deltaTicks   = currentTicks - lastTickCount[motorIndex];
        lastTickCount[motorIndex] = currentTicks;

        if (dt <= 0) return 0.0;

        double ticksPerSec = deltaTicks / dt;
        return Math.abs((ticksPerSec / TICKS_PER_WHEEL_REV) * 60.0);
    }

    private double[] computeRpmCorrections(
            double[] rpms,
            boolean[] active,
            double[] powers,
            double dt) {

        double[] corrections = new double[4];

        double lowestRPM = Double.MAX_VALUE;
        for (int i = 0; i < 4; i++) {
            if (active[i] && rpms[i] > RPM_ACTIVE_THRESHOLD) {
                lowestRPM = Math.min(lowestRPM, rpms[i]);
            }
        }

        if (lowestRPM == Double.MAX_VALUE) {
            for (int i = 0; i < 4; i++) rpmIntegral[i] = 0.0;
            return corrections;
        }

        for (int i = 0; i < 4; i++) {
            if (!active[i]) {
                rpmIntegral[i]  = 0.0;
                rpmLastError[i] = 0.0;
                corrections[i]  = 0.0;
                continue;
            }

            double error = rpms[i] - lowestRPM;

            if (error <= 0) {
                rpmIntegral[i]  = 0.0;
                rpmLastError[i] = 0.0;
                corrections[i]  = 0.0;
                continue;
            }

            if (dt > 0) {
                rpmIntegral[i] += error * dt;
                rpmIntegral[i]  = Math.max(-RPM_INTEGRAL_CAP,
                        Math.min( RPM_INTEGRAL_CAP, rpmIntegral[i]));
            }

            double derivative = (dt > 0) ? (error - rpmLastError[i]) / dt : 0.0;
            rpmLastError[i]   = error;

            double rawCorrection = (Kp_rpm * error)
                    + (Ki_rpm * rpmIntegral[i])
                    + (Kd_rpm * derivative);

            corrections[i] = -Math.signum(powers[i])
                    * Math.min(rawCorrection, MAX_RPM_CORRECTION);
        }

        return corrections;
    }

    @Override
    public void init() {
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "leftfrontdrive");
        leftBackDrive   = hardwareMap.get(DcMotorEx.class, "leftbackdrive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightfrontdrive");
        rightBackDrive  = hardwareMap.get(DcMotorEx.class, "rightbackdrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(120.0, -130.0);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

        while (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            pinpoint.update();
            telemetry.addData("Pinpoint", "Calibrating... do not move robot");
            telemetry.update();
        }
        telemetry.addData("Pinpoint", "Ready!");
        telemetry.update();

        lastTickCount[0] = leftFrontDrive.getCurrentPosition();
        lastTickCount[1] = rightFrontDrive.getCurrentPosition();
        lastTickCount[2] = leftBackDrive.getCurrentPosition();
        lastTickCount[3] = rightBackDrive.getCurrentPosition();

        targetHeading = 0.0;
        pidTimer.reset();
        loopTimer.reset();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        pinpoint.update();

        double axial   = applyDeadzone(-gamepad1.left_stick_y);
        double lateral = applyDeadzone(gamepad1.left_stick_x);
        double yaw     = applyDeadzone(gamepad1.right_stick_x);

        axial   = skew(axial,   last_axial);
        lateral = skew(lateral, last_lateral);
        yaw     = skew(yaw,     last_yaw);

        brake_axial   = computeBrake(axial,   last_axial,   brake_axial);
        brake_lateral = computeBrake(lateral, last_lateral, brake_lateral);
        brake_yaw     = computeBrake(yaw,     last_yaw,     brake_yaw);

        last_axial   = axial;
        last_lateral = lateral;
        last_yaw     = yaw;

        double headingCorrection;
        if (Math.abs(yaw) > YAW_OVERRIDE_THRESHOLD) {
            targetHeading     = pinpoint.getHeading(AngleUnit.RADIANS);
            integralSum       = 0.0;
            lastError         = 0.0;
            headingCorrection = 0.0;
        } else {
            headingCorrection = computeHeadingCorrection();
        }

        double a = axial   + brake_axial;
        double l = lateral + brake_lateral;
        double y = yaw     + brake_yaw + headingCorrection;

        double lfPower = a + l + y;
        double rfPower = a - l - y;
        double lbPower = a - l + y;
        double rbPower = a + l - y;

        double[] rpms = new double[]{
                Math.abs(getMotorRPM(leftFrontDrive,  0, dt)),
                Math.abs(getMotorRPM(rightFrontDrive, 1, dt)),
                Math.abs(getMotorRPM(leftBackDrive,   2, dt)),
                Math.abs(getMotorRPM(rightBackDrive,  3, dt))
        };

        boolean[] active  = getActiveMask(lfPower, rfPower, lbPower, rbPower);
        double[]  powers  = new double[]{lfPower, rfPower, lbPower, rbPower};
        double[]  rpmCorrections = computeRpmCorrections(rpms, active, powers, dt);

        lfPower += rpmCorrections[0];
        rfPower += rpmCorrections[1];
        lbPower += rpmCorrections[2];
        rbPower += rpmCorrections[3];

        leftFrontDrive.setPower(lfPower);
        rightFrontDrive.setPower(rfPower);
        leftBackDrive.setPower(lbPower);
        rightBackDrive.setPower(rbPower);

        telemetry.addData("--- Heading ---",  "");
        telemetry.addData("Heading (deg)",    Math.toDegrees(pinpoint.getHeading(AngleUnit.RADIANS)));
        telemetry.addData("Target  (deg)",    Math.toDegrees(targetHeading));
        telemetry.addData("Heading PID",      headingCorrection);
        telemetry.addData("--- RPM ---",      "");
        telemetry.addData("LF rpm | corr",   String.format("%.1f | %.4f", rpms[0], rpmCorrections[0]));
        telemetry.addData("RF rpm | corr",   String.format("%.1f | %.4f", rpms[1], rpmCorrections[1]));
        telemetry.addData("LB rpm | corr",   String.format("%.1f | %.4f", rpms[2], rpmCorrections[2]));
        telemetry.addData("RB rpm | corr",   String.format("%.1f | %.4f", rpms[3], rpmCorrections[3]));
        telemetry.addData("Active mask",     String.format("LF:%b RF:%b LB:%b RB:%b",
                active[0], active[1], active[2], active[3]));
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.update();
    }
}