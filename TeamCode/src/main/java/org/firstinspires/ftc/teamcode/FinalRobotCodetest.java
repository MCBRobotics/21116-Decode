package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

@TeleOp(name = "Robot: Predictive Turret Tracking", group = "Main")
public class FinalRobotCodetest extends LinearOpMode {

    /* --- HARDWARE --- */
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor rotate, intake, shooter;
    private Servo kicker, pusher;
    private Limelight3A limelight;

    /* --- CONSTANTS --- */
    private static final double TICKS_PER_REV = 288.0;
    private static final double GEAR_RATIO = 1.6; // (64/40)
    private static final double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    private static final double TICKS_PER_INCH = 1800.0;
    private static final double TRACK_WIDTH_TICKS = 15000.0;

    // Tuning: kP handles how fast it snaps to target.
    // If it overshoots, decrease this. If too slow, increase.
    private double kP = 0.015;
    private int targetTagID = 22;

    /* --- COORDINATE MATH --- */
    private double robotX = 0, robotY = 0, robotHeading = 0;
    private double tagFieldX = 0, tagFieldY = 0;
    private double lastTagDistCM = 0;
    private boolean tagInitialized = false;

    @Override
    public void runOpMode() {
        // Init Hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rotate = hardwareMap.get(DcMotor.class, "rotate");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        kicker = hardwareMap.get(Servo.class, "kicker");
        pusher = hardwareMap.get(Servo.class, "pusher");

        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            // --- 1. ODOMETRY UPDATE ---
            double lPos = leftFrontDrive.getCurrentPosition();
            double rPos = rightFrontDrive.getCurrentPosition();

            robotHeading = (rPos - lPos) / TRACK_WIDTH_TICKS * 360.0;
            double avgDist = ((lPos + rPos) / 2.0) / TICKS_PER_INCH;

            // Robot position relative to start (Inches)
            robotX = avgDist * Math.cos(Math.toRadians(robotHeading));
            robotY = avgDist * Math.sin(Math.toRadians(robotHeading));

            // --- 2. DRIVE CONTROLS ---
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            leftFrontDrive.setPower(axial + lateral + yaw);
            rightFrontDrive.setPower(axial - lateral - yaw);
            leftBackDrive.setPower(axial - lateral + yaw);
            rightBackDrive.setPower(axial + lateral - yaw);

            // --- 3. VISION & TURRET LOGIC ---
            LLResult result = limelight.getLatestResult();
            double currentTurretDeg = rotate.getCurrentPosition() / TICKS_PER_DEG;
            double rotatePower = 0;
            boolean seeTagRightNow = false;

            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                    if (fr.getFiducialId() == targetTagID) {
                        seeTagRightNow = true;

                        // Get 3D Distance (Meters -> CM)
                        double xCam = fr.getCameraPoseTargetSpace().getPosition().x;
                        double zCam = fr.getCameraPoseTargetSpace().getPosition().z;
                        lastTagDistCM = Math.sqrt(Math.pow(xCam, 2) + Math.pow(zCam, 2)) * 100.0;

                        // Update the Global "Stored" position of the tag
                        double tx = result.getTx();
                        double absoluteAngleRad = Math.toRadians(robotHeading + currentTurretDeg + tx);
                        double distInches = lastTagDistCM / 2.54;

                        tagFieldX = robotX + (distInches * Math.cos(absoluteAngleRad));
                        tagFieldY = robotY + (distInches * Math.sin(absoluteAngleRad));
                        tagInitialized = true;

                        // Immediate correction based on Vision
                        rotatePower = tx * kP;
                        break;
                    }
                }
            }

            // PREDICTIVE TRACKING: If tag is lost, use Odometry to stay pointed at tagFieldX/Y
            if (tagInitialized && !seeTagRightNow) {
                // Calculate angle from robot to the stored tag location
                double angleToTag = Math.toDegrees(Math.atan2(tagFieldY - robotY, tagFieldX - robotX));

                // Target turret angle is the difference between global tag angle and robot heading
                double targetTurretAngle = angleToTag - robotHeading;

                // Normalize error to avoid 360-degree spins
                double error = targetTurretAngle - currentTurretDeg;
                while (error > 180)  error -= 360;
                while (error < -180) error += 360;

                // Only move if error is significant (prevents jitter)
                if (Math.abs(error) > 1.0) {
                    rotatePower = error * kP;
                }
            }

            // Cap power for safety
            rotate.setPower(Range.clip(rotatePower, -0.45, 0.45));

            // --- 4. TELEMETRY ---
            telemetry.addLine(seeTagRightNow ? "--- LIVE TRACKING ---" : "--- PREDICTIVE TRACKING ---");
            telemetry.addData("Status", tagInitialized ? "Locked on Target" : "Searching...");
            telemetry.addData("Distance", "%.1f cm", lastTagDistCM);
            telemetry.addData("Turret Angle", "%.1f°", currentTurretDeg);
            telemetry.addData("Robot Pose", "X:%.1f, Y:%.1f, H:%.1f", robotX, robotY, robotHeading);
            telemetry.update();
        }
    }
}