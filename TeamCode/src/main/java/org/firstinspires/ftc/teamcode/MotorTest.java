package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "MotorTest", group = "TeleOp")
public class MotorTest extends OpMode {

    private DcMotorEx leftFrontDrive  = null;
    private DcMotorEx leftBackDrive   = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive  = null;

    // Match your gear ratio from Brake.java
    private static final double TICKS_PER_MOTOR_REV = 28.0;
    private static final double GEAR_RATIO          = 10.0 / 3.0;
    private static final double TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * GEAR_RATIO;

    // Test power level — raise with right bumper, lower with left bumper
    private double testPower = 0.0;
    private static final double POWER_STEP = 0.05;

    private int[] lastTicks = new int[4];
    private final ElapsedTime loopTimer = new ElapsedTime();

    private double getMotorRPM(DcMotorEx motor, int index, double dt) {
        int current = motor.getCurrentPosition();
        int delta   = current - lastTicks[index];
        lastTicks[index] = current;
        if (dt <= 0) return 0.0;
        return Math.abs((delta / dt / TICKS_PER_WHEEL_REV) * 60.0);
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

        lastTicks[0] = leftFrontDrive.getCurrentPosition();
        lastTicks[1] = rightFrontDrive.getCurrentPosition();
        lastTicks[2] = leftBackDrive.getCurrentPosition();
        lastTicks[3] = rightBackDrive.getCurrentPosition();

        telemetry.addData("Status", "Initialized — RB to increase power, LB to decrease");
        telemetry.update();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        // Right bumper increases power, left bumper decreases
        // Uses rising edge detection so holding the button doesn't spam
        if (gamepad1.right_bumper) {
            testPower = Math.min(1.0, testPower + POWER_STEP);
        } else if (gamepad1.left_bumper) {
            testPower = Math.max(0.0, testPower - POWER_STEP);
        }

        // B button stops all motors immediately
        if (gamepad1.b) {
            testPower = 0.0;
        }

        leftFrontDrive.setPower(testPower);
        leftBackDrive.setPower(testPower);
        rightFrontDrive.setPower(testPower);
        rightBackDrive.setPower(testPower);

        // RPM
        double lfRPM = getMotorRPM(leftFrontDrive,  0, dt);
        double rfRPM = getMotorRPM(rightFrontDrive, 1, dt);
        double lbRPM = getMotorRPM(leftBackDrive,   2, dt);
        double rbRPM = getMotorRPM(rightBackDrive,  3, dt);

        // Current draw in amps
        double lfAmps = leftFrontDrive.getCurrent(CurrentUnit.AMPS);
        double rfAmps = rightFrontDrive.getCurrent(CurrentUnit.AMPS);
        double lbAmps = leftBackDrive.getCurrent(CurrentUnit.AMPS);
        double rbAmps = rightBackDrive.getCurrent(CurrentUnit.AMPS);

        // RPM spread — difference between fastest and slowest
        double maxRPM = Math.max(Math.max(lfRPM, rfRPM), Math.max(lbRPM, rbRPM));
        double minRPM = Math.min(Math.min(lfRPM, rfRPM), Math.min(lbRPM, rbRPM));
        double rpmSpread = maxRPM - minRPM;

        // Total current
        double totalAmps = lfAmps + rfAmps + lbAmps + rbAmps;

        telemetry.addData("Test power",       String.format("%.2f  (RB=up  LB=down  B=stop)", testPower));
        telemetry.addData("", "");
        telemetry.addData("Motor",            "  RPM      Current");
        telemetry.addData("LF",               String.format("  %.1f rpm   %.2f A", lfRPM, lfAmps));
        telemetry.addData("RF",               String.format("  %.1f rpm   %.2f A", rfRPM, rfAmps));
        telemetry.addData("LB",               String.format("  %.1f rpm   %.2f A", lbRPM, lbAmps));
        telemetry.addData("RB",               String.format("  %.1f rpm   %.2f A", rbRPM, rbAmps));
        telemetry.addData("", "");
        telemetry.addData("RPM spread",       String.format("%.1f  (ideal = 0)", rpmSpread));
        telemetry.addData("Total current",    String.format("%.2f A", totalAmps));
        telemetry.addData("LF ticks", leftFrontDrive.getCurrentPosition());
        telemetry.addData("RF ticks", rightFrontDrive.getCurrentPosition());
        telemetry.addData("LB ticks", leftBackDrive.getCurrentPosition());
        telemetry.addData("RB ticks", rightBackDrive.getCurrentPosition());
        telemetry.update();
    }
}