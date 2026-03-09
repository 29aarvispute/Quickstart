package TeleOp_Nico;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class FlywheelController {
    // --- HARDWARE ---2
    private DcMotorEx shooterM1; // Left motor on your schematic
    private DcMotorEx shooterM2; // Right motor on your schematic

    // --- NEW: VOLTAGE SENSOR ---
    private VoltageSensor batteryVoltageSensor;
    private final double NOMINAL_VOLTAGE = 13.0; // Tuned for fresh batteries, adjust if your "fresh" is different

    // --- HARDWARE CONSTANTS ---
    // !!! IMPORTANT: Verify these directions match your physical robot's wiring.
    // If they are on the same shaft, they usually need to be set to the same direction.
    private final DcMotorSimple.Direction MOTOR1_DIR = DcMotorSimple.Direction.FORWARD;
    private final DcMotorSimple.Direction MOTOR2_DIR = DcMotorSimple.Direction.FORWARD; // Assuming both motors turn the shaft the same way
    private final double TICKS_PER_REVOLUTION = 28.0; // GoBilda 5203 1:1

    // --- PIDF COEFFICIENTS ---
    public static double kF = 0.000330;
    public static double kP = 0.000090;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // --- PID State Variables (Unified for the single shaft) ---
    private double lastError = 0;
    private double integralSum = 0;
    private ElapsedTime pidTimer = new ElapsedTime();

    // --- Flywheel State ---
    private double targetRPM = 0;
    private boolean flywheelOn = false;

    // --- Reverse Mode Flag ---
    private boolean reversing = false;

    // --- Constructor ---
    public FlywheelController(HardwareMap hardwareMap) {
        shooterM1 = hardwareMap.get(DcMotorEx.class, "shooterM1");
        shooterM2 = hardwareMap.get(DcMotorEx.class, "shooterM2");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterM1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterM2.setDirection(DcMotorSimple.Direction.FORWARD); // Both motors should ideally run in the same direction for a common shaft

        shooterM1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterM2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterM1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterM2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pidTimer.reset();
    }

    // --- Methods to Control the Flywheel ---
    public void setFlywheelTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, rpm);
    }

    public void turnFlywheelOn() {
        if (this.targetRPM > 100) {
            flywheelOn = true;
            reversing = false;
        }
    }

    public void turnFlywheelOff() {
        flywheelOn = false;
        targetRPM = 0;
        reversing = false;
    }

    public boolean isFlywheelOn() {
        return flywheelOn;
    }

    public void reverseFlywheel() {
        reversing = true;
        flywheelOn = false;
        setFlywheelPower(-0.2, -0.2); // Apply fixed negative power
        resetPIDState();
    }

    public void stopReverse() {
        reversing = false;
        if (this.targetRPM > 100) {
            flywheelOn = true;
        } else {
            setFlywheelPower(0,0);
        }
        resetPIDState();
    }

    public boolean isReversing() {
        return reversing;
    }

    // --- Main Update Method: Acts as a State Machine ---
    public void update() {
        if (reversing) {
            return;
        }

        if (flywheelOn && targetRPM > 0) {
            double targetTPS = (targetRPM / 60.0) * TICKS_PER_REVOLUTION;

            // Calculate current velocity as the average of the two motors
            double currentTPS_M1 = -shooterM1.getVelocity();
            double currentTPS_M2 = -shooterM2.getVelocity();
            double averageCurrentTPS = (currentTPS_M1 + currentTPS_M2) / 2.0;

            // Calculate unified PID power
            double power = calculateUnifiedPIDF(averageCurrentTPS, targetTPS);
            setFlywheelPower(power, power); // Apply the same power to both motors
            return;
        }

        setFlywheelPower(0, 0);
        resetPIDState();
    }

    private void setFlywheelPower(double powerM1, double powerM2) {
        shooterM1.setPower(powerM1);
        shooterM2.setPower(powerM2);
    }

    // --- Unified PIDF calculation for the single shaft ---
    private double calculateUnifiedPIDF(double currentTPS, double targetTPS) {
        double dt = pidTimer.seconds();
        if (dt <= 0) dt = 1e-3;
        pidTimer.reset();

        double currentBatteryVoltage = batteryVoltageSensor.getVoltage();
        double voltageMultiplier = NOMINAL_VOLTAGE / Math.max(currentBatteryVoltage, 1.0);

        double error = targetTPS - currentTPS;

        double feedforward = (targetTPS * kF) * voltageMultiplier;
        double proportional = error * kP;

        // --- IMPROVED INTEGRAL ANTI-WINDUP (Unified) ---
        if (Math.abs(error) > 50 && Math.abs(currentTPS) < targetTPS * 1.1) {
            integralSum += error * dt;
            double maxIntegralSum = 0.5 / kI; // Limit integral sum to prevent windup
            integralSum = Range.clip(integralSum, -maxIntegralSum, maxIntegralSum);
        } else {
            integralSum *= 0.95; // Decay integral if conditions not met
        }
        double integralTerm = integralSum * kI;

        double derivative = (error - lastError) / dt;
        double derivativeTerm = derivative * kD;

        lastError = error; // Update lastError for next iteration

        double totalPower = feedforward + proportional + integralTerm + derivativeTerm;

        return Range.clip(totalPower, 0.0, 1.0);
    }

    private void resetPIDState() {
        lastError = 0;
        integralSum = 0;
        pidTimer.reset();
    }

    // --- Telemetry Helpers ---
    // This will return the average RPM used for PID control
    public double getCurrentRPM_Average() {
        double currentTPS_M1 = -shooterM1.getVelocity();
        double currentTPS_M2 = -shooterM2.getVelocity();
        return ((currentTPS_M1 + currentTPS_M2) / 2.0 / TICKS_PER_REVOLUTION) * 60.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getErrorRPM_Average() {
        return targetRPM - getCurrentRPM_Average();
    }

    // --- Individual Motor RPM for Diagnostic Purposes (Re-added) ---
    public double getCurrentRPM_M1() {
        return (-shooterM1.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }
    public double getCurrentRPM_M2() {
        return (-shooterM2.getVelocity() / TICKS_PER_REVOLUTION) * 60.0;
    }
}
