package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@TeleOp(name = "TeleOp Decode", group = "Main")
public class FinalRobotCode extends LinearOpMode {

    /* Hardware Members */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor rotate, intake, shooter;
    private Servo blocker, pusher, hood;
    private Limelight3A limelight;

    /* Limelight Tuning Constants */
    private static final double kP = 0.02;
    private static final double MAX_ROTATE_POWER = 0.25;
    private static final double TX_DEADBAND = 1.2;
    private static final double GAIN = 0.2; // Low-pass filter gain
    private double smoothedTx = 0;
    private int targetTagID = 20;

    /* State tracking for toggle/button debouncing */
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    @Override
    public void runOpMode() {
        // 1. Initialize Drivetrain
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // 2. Initialize Mechanisms
        rotate  = hardwareMap.get(DcMotor.class, "rotate");
        intake  = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");
        pusher  = hardwareMap.get(Servo.class, "pusher");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 3. Initialize Vision
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData("Status", "Initialized - Ready to Fly");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- SECTION 1: MECANUM DRIVE ---
            double axial   = -gamepad1.left_stick_y;
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

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
                        // Apply Smoothing Filter
                        smoothedTx = (result.getTx() * GAIN) + (smoothedTx * (1.0 - GAIN));

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
            // assign face buttons to back buttons on Matthew's controller
            if (gamepad1.a) intake.setPower(1.0);
            else intake.setPower(0.0);

            if (gamepad1.b) {

            }

            // --- SECTION 4: TELEMETRY ---
            telemetry.addData("Target ID", targetTagID);
            telemetry.addData("Vision Locked", tagFound);
            telemetry.addData("Rotate Power", "%.2f", rotatePower);
            telemetry.addData("Drive", "LF:%.2f RF:%.2f", lf, rf);
            telemetry.update();
        }
    }
}