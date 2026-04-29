package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp (name = "NewDriveSystem", group = "TeleOp")
public class NewDriveSystem extends LinearOpMode {

    public double applyDeadzone(double joystick) {
        double deadzone_boundary = 0.07;
        if (joystick < deadzone_boundary && joystick > -deadzone_boundary ) {
            return 0.0;
        } else {
            return joystick;
        }
    }

    @Override
    public void runOpMode() {

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
        waitForStart();

        while (opModeIsActive()) {

            double axial = applyDeadzone(-gamepad1.left_stick_y);
            double lateral = applyDeadzone(gamepad1.left_stick_x);
            double yaw = applyDeadzone(gamepad1.right_stick_x);

            axial = Math.signum(axial) * Math.pow(axial, 2);
            lateral = Math.signum(lateral) * Math.pow(lateral, 2);
            yaw = Math.signum(yaw) * Math.pow(yaw, 2);

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double heading = angles.getYaw(AngleUnit.RADIANS);

            double rotX = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
            double rotY = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

            double leftfrontPower = rotY + rotX + yaw;
            double rightfrontPower = rotY - rotX - yaw;
            double leftbackPower = rotY - rotX + yaw;
            double rightbackPower = rotY + rotX - yaw;

            leftFrontDrive.setPower(leftfrontPower);
            rightFrontDrive.setPower(rightfrontPower);
            leftBackDrive.setPower(leftbackPower);
            rightBackDrive.setPower(rightbackPower);

            if (gamepad2.b) {
                imu.resetYaw();
            }

        }
    }
}
