package frc.robot.PIDMotors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDSparkMax extends CANSparkMax implements Sendable {
    private SparkMaxPIDController pidCtrl;
    private RelativeEncoder encoder;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double maxRPM = 5700;
    private double targetRPM = 0;

    public PIDSparkMax(int portNum, MotorType type) {
        super(portNum, type);

        super.restoreFactoryDefaults();

        pidCtrl = super.getPIDController();
        encoder = super.getEncoder();

        // PID coefficents
        kP = 6e-5;
        kI = 0;
        kD = 0;
        kIz = 0;
        kFF = 0.000015;
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;

        pidCtrl.setP(kP);
        pidCtrl.setI(kI);
        pidCtrl.setD(kD);
        pidCtrl.setIZone(kIz);
        pidCtrl.setFF(kFF);
        pidCtrl.setOutputRange(kMinOutput, kMaxOutput);
    }

    @Override
    public void set(double speed) {
        targetRPM = speed * maxRPM;
        pidCtrl.setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Max RPM", this::getMaxRPM, this::setMaxRPM);
        builder.addDoubleProperty("Target RPM", this::getTargetRPM, null);
        builder.addDoubleProperty("Actual RPM", encoder::getVelocity, null);
    }

    public double getMaxRPM() {
        return maxRPM;
    }

    public void setMaxRPM(double rpm) {
        maxRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }
}
