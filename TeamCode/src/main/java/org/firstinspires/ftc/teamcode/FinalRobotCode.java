package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "Robot: Final", group = "Main")
public class FinalRobotCode extends LinearOpMode {

    /* Hardware Members */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor rotate, shooter1, shooter2;
    private Servo kicker, pusher;
    private CRServo intake;

    private Limelight3A limelight;

    /* Limelight Tuning Constants */
    // Proportional gain: scales how much rotate power is applied per degree of tx offset
    private static final double kP = 0.03;
    // Maximum power allowed to the rotate motor when vision-tracking (prevents violent spinning)
    private static final double MAX_ROTATE_POWER = 0.4;
    // Minimum tx (horizontal) offset (degrees) before rotation kicks in — prevents jitter when nearly centered
    private static final double TX_DEADBAND = 0.5;
    // Low-pass filter weight: higher = more responsive but noisier; lower = smoother but slower to react
    private static final double GAIN = 0.2;
    // Exponential moving average of tx — smooths out noisy Limelight readings frame-to-frame
    private double smoothedTx = 0;
    // AprilTag ID currently being tracked; bumpers cycle this between 20–25
    private int targetTagID = 20;


    private boolean kickerOn = false;
    private boolean lastX = false;

    /* State tracking for toggle/button debouncing */
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rb");

        // Left-back is reversed so all wheels push the robot forward when given positive power
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // 2. Initialize Mechanisms
        rotate  = hardwareMap.get(DcMotor.class, "rotate");
        shooter1 = hardwareMap.get(DcMotor.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter2");

        intake  = hardwareMap.get(CRServo.class, "intake");

        kicker  = hardwareMap.get(Servo.class, "kicker");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        rotate.setDirection(DcMotorSimple.Direction.REVERSE);

        //intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // BRAKE holds the turret in place when no input is given, preventing drift
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Initialize Vision
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized - Ready to Fly");
        telemetry.update();

        waitForStart();

        /*
        * all button mappings:
        * Left stick: forward and backward, strafe left and right
        * right stick, rotate (change heading) left and right
        * a: intake
        * b: shooter
        * x: kicker on intake
        * y: pusher on shooter
        * bumpers:  limelight targettagID
        * dpad up/down: override limelight rotation
        * right trigger then bumpers: change hood angle */

        while (opModeIsActive()) {

            // --- SECTION 1: MECANUM DRIVE ---
            // Negate left_stick_y because pushing the stick forward gives a negative value on FTC gamepads
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Mecanum wheel mixing: each wheel gets a combination of axial, lateral, and yaw
            // The signs come from the physical roller orientation of mecanum wheels
            double lf = axial + lateral + yaw;
            double rf = axial - lateral - yaw;
            double lb = axial - lateral + yaw;
            double rb = axial + lateral - yaw;

            // Normalize powers
            double max = Math.max(Math.max(Math.abs(lf), Math.abs(rf)), Math.max(Math.abs(lb), Math.abs(rb)));
            if (max > 1.0) {
                lf /= max; rf /= max; lb /= max; rb /= max;
            }

            leftFrontDrive.setPower(lf);
            rightFrontDrive.setPower(rf);
            leftBackDrive.setPower(lb);
            rightBackDrive.setPower(rb);


            // --- SECTION 2: LIMELIGHT TARGETING (ROTATE MOTOR) ---
            LLResult result = limelight.getLatestResult();

            // Cycle target ID with bumpers
            if (gamepad1.left_bumper && !lastLeftBumper) targetTagID = Math.max(20, targetTagID - 1);
            if (gamepad1.right_bumper && !lastRightBumper) targetTagID = Math.min(25, targetTagID + 1);
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;

            double rotatePower = 0.0;
            boolean tagFound = false;

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducials) {
                    if (fr.getFiducialId() == targetTagID) {
                        tagFound = true;
                        // Exponential moving average: blends new reading with previous smooth value
                        // to reduce jitter from noisy camera detections
                        smoothedTx = (result.getTx() * GAIN) + (smoothedTx * (1.0 - GAIN));

                        // Only rotate if the tag is outside the deadband — avoids oscillating when centered
                        if (Math.abs(smoothedTx) > TX_DEADBAND) {
                            rotatePower = smoothedTx * kP;
                        }
                        break;
                    }
                }
            } else {
                smoothedTx = 0; // Reset filter if vision lost
            }

            // Constraints and Manual Overrides for Rotation
            rotatePower = Math.max(-MAX_ROTATE_POWER, Math.min(MAX_ROTATE_POWER, rotatePower));
            if (gamepad1.dpad_up) rotatePower = 0.3;      // Manual override up
            else if (gamepad1.dpad_down) rotatePower = -0.3; // Manual override down

            rotate.setPower(rotatePower);



            // --- SECTION 3: MECHANISMS (INTAKE, SHOOTER, SERVOS) ---

            // Intake (Motor) - Spin while A is pressed
//            if (gamepad1.a) intake.setPower(1);
//            else intake.setPower(0);

            if (gamepad1.a) {
                intake.setPower(-1);   // spin forward
            } else {
                intake.setPower(0);   // stop
            }


            // Shooter (Motor) - Spin while B is pressed
            if (gamepad1.b) shooter1.setPower(1.0);
            else shooter1.setPower(0);

            // Shooter (Motor) - Spin while B is pressed
            if (gamepad1.y) shooter2.setPower(1.0);
            else shooter2.setPower(0);

            // Kicker (Continuous Rotation Servo): 0.5 = stop, <0.5 = reverse, >0.5 = forward
            // 0.25 spins the kicker inward; 1.0 is full speed the other way (used as idle/stop here)
//            if (gamepad1.x) kicker.setPosition(0.25);
//            else kicker.setPosition(0.5);

            if (gamepad1.x && !lastX) {
                kickerOn = !kickerOn; // flip state
            }

            if (kickerOn) {
                kicker.setPosition(0.25);
            } else {
                kicker.setPosition(0.5);
            }

            lastX = gamepad1.x;


            // Hood angle (Servo) - triggers control tilt speed proportionally to press depth
            // right_trigger tilts up, left_trigger tilts down; harder press = faster movement
            // Max step per loop (0.02) keeps motion smooth — lower this if it moves too fast
            double hoodStep = (gamepad1.right_trigger - gamepad1.left_trigger) * 0.02;
            pusher.setPosition(Math.max(0.0, Math.min(1.0, pusher.getPosition() + hoodStep)));

            // Quick-set overrides: dpad_left = full up (1.0), dpad_right = full down (0.0)
            if (gamepad1.dpad_left) pusher.setPosition(1);
            else if (gamepad1.dpad_right) pusher.setPosition(0);


            // --- SECTION 4: TELEMETRY ---
            telemetry.addData("Target ID", targetTagID);
            telemetry.addData("Vision Locked", tagFound);
            telemetry.addData("Rotate Power", "%.2f", rotatePower);
            telemetry.addData("Drive", "LF:%.2f RF:%.2f", lf, rf);
            telemetry.addData("smoothedtx: ",smoothedTx);
            telemetry.addData("rotate power: ",rotatePower);
            telemetry.update();
        }
    }
}