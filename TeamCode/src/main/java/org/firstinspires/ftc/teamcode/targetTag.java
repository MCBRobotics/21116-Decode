package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Edited for stable aiming:
 *  - Uses getFiducialId() and per-tag getTargetXDegrees()/getTargetYDegrees()
 *  - Adds deadband, hysteresis, low-pass filtering, and derivative damping
 *  - Stops motor when no tag or small error
 */
@TeleOp(name = "Sensor: Limelight3A (Tag 22 Stable)", group = "Testing")
public class targetTag extends LinearOpMode {

    // ---------- Tuning constants (feel free to adjust) ----------
    private static final int    TARGET_ID        = 22;    // tag to track
    private static final double kP               = 0.010; // proportional gain
    private static final double kD               = 0.0025;// derivative (damping)
    private static final double DEAD_BAND_DEG    = 1.0;   // no motion inside ±1.0°
    private static final double HYSTERESIS_EXTRA = 0.5;   // extra to re-engage after stop
    private static final double MAX_POWER        = 0.20;  // absolute safety cap
    private static final double MIN_EFF_POWER    = 0.06;  // min to overcome stiction (only outside deadband)
    private static final double LPF_ALPHA        = 0.35;  // 0..1 (lower = more smoothing)

    private Limelight3A limelight;
    private DcMotor rotate;

    // ---------- Controller state ----------
    private double prevErrorDeg = 0.0;
    private double filtErrorDeg = 0.0;
    private boolean wasDriving  = false;

    @Override
    public void runOpMode() throws InterruptedException {
        rotate = hardwareMap.get(DcMotor.class, "rotate");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // One-time motor setup to prevent drift
        rotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotate.setDirection(DcMotor.Direction.FORWARD); // flip if the sign feels backward

        telemetry.setMsTransmissionInterval(11);

        // Select your AprilTag/fiducial pipeline index
        limelight.pipelineSwitch(0); // make sure pipeline 0 is an AprilTag pipeline in the LL UI

        // Start polling the Limelight (required or getLatestResult() returns null)
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Status info (temp, CPU, FPS, active pipeline)
            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();
            boolean drivingThisCycle = false;

            if (result != null && result.isValid()) {
                // Find your specific tag
                List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                LLResultTypes.FiducialResult target = null;
                for (LLResultTypes.FiducialResult t : tags) {
                    if (t.getFiducialId() == TARGET_ID) { // correct accessor
                        target = t;
                        break;
                    }
                }

                if (target != null) {
                    // Per-tag horizontal error (degrees). Do NOT call tag.getTx().
                    double errorDeg = target.getTargetXDegrees();

                    // Low-pass filter to reduce jitter
                    filtErrorDeg = LPF_ALPHA * errorDeg + (1.0 - LPF_ALPHA) * filtErrorDeg;

                    // Deadband + hysteresis: require a bigger error to re-engage after stopping
                    double engageBand = DEAD_BAND_DEG + (wasDriving ? 0.0 : HYSTERESIS_EXTRA);

                    if (Math.abs(filtErrorDeg) >= engageBand) {
                        // Derivative on measurement for damping
                        double dErr = (filtErrorDeg - prevErrorDeg);

                        double power = kP * filtErrorDeg + kD * dErr;

                        // Clip and enforce minimum effective power
                        double sign = Math.signum(power);
                        power = Math.min(Math.abs(power), MAX_POWER);
                        power = Math.max(power, MIN_EFF_POWER) * sign;

                        rotate.setPower(power);
                        drivingThisCycle = true;

                        telemetry.addData("Aim(Tag " + TARGET_ID + ")",
                                "err=%.2f°, filt=%.2f°, d=%.3f, pwr=%.2f",
                                errorDeg, filtErrorDeg, dErr, power);
                    } else {
                        rotate.setPower(0.0);
                    }

                    prevErrorDeg = filtErrorDeg;

                    // Optional: tag details
                    telemetry.addData("Tag", "id=%d, family=%s", target.getFiducialId(), target.getFamily());
                    telemetry.addData("Angles", "x=%.2f°, y=%.2f°", target.getTargetXDegrees(), target.getTargetYDegrees());
                    telemetry.addData("Area", "%.3f", target.getTargetArea());

                } else {
                    telemetry.addLine("Target tag not visible");
                    rotate.setPower(0.0);
                    // Decay filter slowly so we don't kick when the tag reappears
                    filtErrorDeg *= 0.9;
                    prevErrorDeg = filtErrorDeg;
                }

                // Optional: frame-level selected target (may be a different object than your TARGET_ID)
                telemetry.addData("Primary tx", "%.2f°", result.getTx());

                // Optional: pose and latency readouts
                Pose3D botpose = result.getBotpose();
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("Botpose", botpose.toString());
                telemetry.addData("Latency", "proc=%.1f ms (parse=%.1f ms)",
                        (captureLatency + targetingLatency), parseLatency);

            } else {
                telemetry.addLine("Limelight: No data available");
                rotate.setPower(0.0);
                // Decay filter slowly so we don't kick when data returns
                filtErrorDeg *= 0.9;
                prevErrorDeg = filtErrorDeg;
            }

            telemetry.update();
            wasDriving = drivingThisCycle;
        }

        limelight.stop();
        rotate.setPower(0.0);
    }
}
