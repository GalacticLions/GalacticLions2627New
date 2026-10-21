package org.firstinspires.ftc.team5898.LimelightUtils;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.team5898.Hardware.Constants.Limelight;

import java.util.List;

@Configurable
public class VisualServoing {
    // Horizontal (tx) alignment - turning left/right
    public static final double Kp_HEADING = 0.02; // Proportional gain for horizontal alignment
    public static final double MIN_COMMAND_HEADING = 0.10;
    public static final double HEADING_THRESHOLD = 0.05; // Horizontal offset threshold (degrees)
    public static final double MAX_TURN_POWER = 0.5;

    // Vertical (ty) alignment - moving forward/backward
    public static final double Kp_DISTANCE = 0.03; // Proportional gain for vertical alignment
    public static final double MIN_COMMAND_DISTANCE = 0.12;
    public static final double DISTANCE_THRESHOLD = 1.0; // Vertical offset threshold (degrees)
    public static final double MAX_DRIVE_POWER = 0.5;

    // Target values
    public static double TARGET_TY = 0.0; // Target ty value (adjust based on desired distance)
    public static double TARGET_TX = 0.0; // Target tx value (default 0.0 = center)
    public static int TagID = -1;
    private final Limelight3A limelight;
    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;
    private final Telemetry telemetry;

    // Constructor

    public VisualServoing(Limelight3A limelight, DcMotor frontLeft, DcMotor frontRight, DcMotor backRight,
                          DcMotor backLeft, Telemetry telemetry) {
        this.limelight = limelight;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.telemetry = telemetry;
    }

    // getLatestResult() keeps returning the last frame if the camera stops updating, so reject old ones
    private LLResultTypes.FiducialResult findTarget() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > Limelight.MAX_STALENESS_MS) return null;

        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
        if (fiducialResults == null) return null;

        for (LLResultTypes.FiducialResult fr : fiducialResults) {
            int id = fr.getFiducialId();
            if (id == 20 || id == 24) return fr;
        }
        return null;
    }

    // Far away (small area) we aim off-center at the goal; up close we aim at the tag itself
    private static double targetTxFor(LLResultTypes.FiducialResult target) {
        if (target.getTargetArea() > Limelight.TARGET_AREA_THRESHOLD) return 0.0;
        return target.getFiducialId() == 20 ? Limelight.BLUE_ALLIANCE_TX : Limelight.RED_ALLIANCE_TX;
    }

    /**
     * Returns true when the robot is aligned to the target (both tx and ty errors within thresholds).
     * Returns false if there is no valid target or alignment is not yet complete.
     */
    public boolean isAligned() {
        LLResultTypes.FiducialResult target = findTarget();
        if (target == null) return false;

        double txError = target.getTargetXDegrees() - targetTxFor(target);
        double tyError = target.getTargetYDegrees() - TARGET_TY;

        return Math.abs(txError) <= HEADING_THRESHOLD && Math.abs(tyError) <= DISTANCE_THRESHOLD;
    }

    public void visualServo() {
        LLResultTypes.FiducialResult target = findTarget();
        boolean targetFound = target != null;
        TagID = targetFound ? target.getFiducialId() : -1;

        if (targetFound) {
            double tx = target.getTargetXDegrees(); // Horizontal offset from crosshair
            double ty = target.getTargetYDegrees(); // Vertical offset from crosshair
            double ta = target.getTargetArea(); // Target Area to determine distance
            TARGET_TX = targetTxFor(target);

            // --- TX (Heading) Control ---
            // Calculate error relative to TARGET_TX
            double txError = tx - TARGET_TX;
            double steeringAdjust = 0.0;

            if (Math.abs(txError) > HEADING_THRESHOLD) {
                // Proportional control based on error
                steeringAdjust = Kp_HEADING * txError;

                // Add minimum command to overcome friction
                if (txError > 0) {
                    steeringAdjust += MIN_COMMAND_HEADING;
                } else {
                    steeringAdjust -= MIN_COMMAND_HEADING;
                }
                steeringAdjust = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, steeringAdjust));
            }

            // --- TY (Distance) Control ---
            double tyError = ty - TARGET_TY;
            double driveAdjust = 0.0;

            // DISABLED by default: Set to true if you want distance control
            boolean enableDistanceControl = false;

            if (enableDistanceControl && Math.abs(tyError) > DISTANCE_THRESHOLD) {
                driveAdjust = Kp_DISTANCE * tyError;

                if (tyError > 0) {
                    driveAdjust += MIN_COMMAND_DISTANCE;
                } else {
                    driveAdjust -= MIN_COMMAND_DISTANCE;
                }
                driveAdjust = Math.max(-MAX_DRIVE_POWER, Math.min(MAX_DRIVE_POWER, driveAdjust));
            }

            // Combine turning and driving
            double frontLeftPower = driveAdjust + steeringAdjust;
            double backLeftPower = driveAdjust + steeringAdjust;
            double frontRightPower = driveAdjust - steeringAdjust;
            double backRightPower = driveAdjust - steeringAdjust;

            // Normalize powers if they exceed 1.0
            double maxPower = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(backLeftPower),
                            Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                backLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Apply motor powers
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("txError", txError);
            telemetry.addData("steeringAdjust", steeringAdjust);
            telemetry.addData("driveAdjust", driveAdjust);
            telemetry.addData("Heading Aligned", Math.abs(txError) <= HEADING_THRESHOLD);
            telemetry.addData("Distance Aligned", Math.abs(tyError) <= DISTANCE_THRESHOLD);
            telemetry.update();
        } else {
            // Stop all motors when no valid target
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);

            telemetry.addData("No fresh target", "");
            telemetry.update();
        }
    }
}