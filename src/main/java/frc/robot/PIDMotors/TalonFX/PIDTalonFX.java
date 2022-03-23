package frc.robot.PIDMotors.TalonFX;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class PIDTalonFX extends WPI_TalonFX {

    public PIDTalonFX(int deviceNumber) {
        super(deviceNumber);

        super.configFactoryDefault();

        /* Config neutral deadband to be the smallest possible */
        super.configNeutralDeadband(0.001);

        /* Config sensor used for Primary PID [Velocity] */
        super.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        /* Config the peak and nominal outputs */
        super.configNominalOutputForward(0, Constants.kTimeoutMs);
        super.configNominalOutputReverse(0, Constants.kTimeoutMs);
        super.configPeakOutputForward(1, Constants.kTimeoutMs);
        super.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        super.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
        super.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
        super.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
        super.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
    }

    @Override
    public void set(double speed) {
        final float MAX_RPM = 3000.f;

        /* 2000 RPM in either direction */
        double targetVelocity_UnitsPer100ms = speed * MAX_RPM * 2048.0 / 600.0;
        super.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }

}
