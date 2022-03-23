package frc.robot.PIDMotors;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class PIDSparkMax extends CANSparkMax {
    private SparkMaxPIDController pidCtrl;
    // private RelativeEncoder encoder;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    public PIDSparkMax(int portNum, MotorType type) {
        super(portNum, type);
        pidCtrl = super.getPIDController();
        // encoder = super.getEncoder();

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
        double setPoint = speed * maxRPM;
        pidCtrl.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
    }
}
