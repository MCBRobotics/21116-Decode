package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "NewDriveSystem", group = "TeleOp")
public class NewDriveSystem extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU imu = null;

    double last_rotX = 0.0;
    double last_rotY = 0.0;
    double last_yaw = 0.0;
    double MAX_CHANGE = 0.3;

    // RPM tracking
    private long lastTime = 0;
    private int lastLF = 0, lastLB = 0, lastRF = 0, lastRB = 0;
    private static final double CPR = 28.0 * (10.0 / 3.0); // 93.33 counts per output revolution

    public double applyDeadzone(double joystick) {
        double deadzone_boundary = 0.2;
        if (joystick < deadzone_boundary && joystick > -deadzone_boundary) {
            return 0.0;
        } else {
            return joystick;
        }
    }

    public double skew(double input, double last_input) {
        if (Math.signum(input) == 1 && input > last_input + MAX_CHANGE) {
            return last_input + MAX_CHANGE;
        } else if (Math.signum(input) == -1 && input < last_input - MAX_CHANGE) {
            return last_input - MAX_CHANGE;
        } else {
            return input;
        }
    }

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Reset encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );
        imu.initialize(parameters);

        lastTime = System.nanoTime();
    }

    @Override
    public void init_loop() {
        imu.resetYaw();
    }

    @Override
    public void loop() {
        double axial = applyDeadzone(-gamepad1.left_stick_y);
        double lateral = applyDeadzone(gamepad1.left_stick_x);
        double yaw = applyDeadzone(gamepad1.right_stick_x);

        axial = Math.signum(axial) * Math.pow(axial, 2);
        lateral = Math.signum(lateral) * Math.pow(lateral, 2);
        yaw = Math.signum(yaw) * Math.pow(yaw, -1);

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double heading = angles.getYaw(AngleUnit.RADIANS);

        double rotX = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotY = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        rotX = skew(rotX, last_rotX);
        rotY = skew(rotY, last_rotY);
        yaw = skew(yaw, last_yaw);

        last_rotX = rotX;
        last_rotY = rotY;
        last_yaw = yaw;

        double leftfrontPower = rotY + rotX + yaw;
        double rightfrontPower = rotY - rotX - yaw;
        double leftbackPower = rotY - rotX + yaw;
        double rightbackPower = rotY + rotX - yaw;

        leftFrontDrive.setPower(leftfrontPower);
        rightFrontDrive.setPower(rightfrontPower);
        leftBackDrive.setPower(leftbackPower);
        rightBackDrive.setPower(rightbackPower);

        if (gamepad1.b) {
            imu.resetYaw();
        }

        // ---------- RPM TELEMETRY ----------
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9; // convert to seconds

        int lf = leftFrontDrive.getCurrentPosition();
        int lb = leftBackDrive.getCurrentPosition();
        int rf = rightFrontDrive.getCurrentPosition();
        int rb = rightBackDrive.getCurrentPosition();

        if (dt > 0) {
            double lfRPM = ((lf - lastLF) / CPR) / dt * 60.0;
            double lbRPM = ((lb - lastLB) / CPR) / dt * 60.0;
            double rfRPM = ((rf - lastRF) / CPR) / dt * 60.0;
            double rbRPM = ((rb - lastRB) / CPR) / dt * 60.0;

            telemetry.addData("LF RPM", "%.1f", lfRPM);
            telemetry.addData("LB RPM", "%.1f", lbRPM);
            telemetry.addData("RF RPM", "%.1f", rfRPM);
            telemetry.addData("RB RPM", "%.1f", rbRPM);
        }

        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(heading));
        telemetry.update();

        lastLF = lf;
        lastLB = lb;
        lastRF = rf;
        lastRB = rb;
        lastTime = now;
    }
}