package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Test Car2", group="Linear OpMode")
//@Disabled
public class TestCar2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    //    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor leftBackDrive = null;
    private DcMotor intake = null;
    private DcMotor kicker = null;
    private Limelight3A limelight;
    private Servo servo;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        kicker = hardwareMap.get(DcMotor.class, "kicker");
        ////Arm2 = hardwareMap.get(DcMotor.class, "ArmMotor2");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "servo1");
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Optional: set direction
        intake.setDirection(DcMotor.Direction.REVERSE);
        kicker.setDirection(DcMotor.Direction.FORWARD);

        // Highly recommended for REV Core Hex motors
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kicker.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ////Arm2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            /*
            to find the power:
            find difference in distance between starting point and where it wants to go
            divide the current distance away over the total distance
            set this as the power
            */

            // testing the intake and kicker
            if (gamepad1.dpad_up) {
                intake.setPower(0.9);     // forward hi
//            } else if (gamepad1.dpad_down) {
//                intake.setPower(1);    // reverse
            } else if (gamepad1.dpad_down){
                intake.setPower(0);
            } else if (gamepad1.dpad_right){
                kicker.setPower(-1);
            } else if (gamepad1.dpad_left){
                kicker.setPower(0);
            }

            // for competition; using a (X on our gamepad) inst4
            if (gamepad1.aWasPressed()) {
                kicker.setPower(-1);
                intake.setPower(0.9);
            } else if (gamepad1.aWasReleased()) {
                kicker.setPower(0);
                intake.setPower(0);

             if (gamepad1.y) {
                 servo.setPosition(1.0);
             }
             if (gamepad1.x) {
                 servo.setPosition(0.0);
             }
             telemetry.addData("Triangle(Y)", gamepad1.y);
             telemetry.addData("Square (X)", gamepad1.x);
             telemetry.addData("Servo Position", servo.getPosition());
             telemetry.update();
            }
                LLResult result = limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        Pose3D botpose = result.getBotpose();
                        telemetry.addData("tx", result.getTx());
                        telemetry.addData("ty", result.getTy());
                        telemetry.addData("Botpose", botpose.toString());
}
}           // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            //telemetry.addData("difference Test", differenceInArms);
            //telemetry.addData("targetPosition Test", targetPosition);
           // telemetry.addData("targetPositionArm2 Test", targetPositionArm2);
           // telemetry.addData("zeroPositionArm1 Test", zeroPositionArm1);
          //  telemetry.addData("zeroPositionArm2 Test", zeroPositionArm2);

            // Show the elapsed game time and wheel power.
            telemetry.addData("leftFrontDrive", leftFrontPower);
            telemetry.addData("rightFrontDrive", rightFrontPower);
            telemetry.addData("leftBackDrive", leftBackPower);
            telemetry.addData("rightBackDrive", rightBackPower);
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
           // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            //telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
           // telemetry.update();

            if (isStopRequested()) {
                return;
            }
        }
    }
}




