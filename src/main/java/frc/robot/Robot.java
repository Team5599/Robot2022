// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import for TalonFX (Drivetrain motors)
import frc.robot.PIDMotors.TalonFX.PIDTalonFX;

// Imports for SparkMax (Neo motors)
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.PIDMotors.PIDSparkMax;

// Import for pneumatics (PCM)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Import for xbox controller
import frc.robot.Controllers.XBoxController;
import frc.robot.Controllers.LogitechExtreme3DProController;

enum AUTO_STATE {
    TAXIING,
    SEEKING,
    ALIGNING,
    SHOOTING
}

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {

    // Declaration of Objects
    // Falcon FX (Falcon 500) Motors
    PIDTalonFX l0, l1, r0, r1;
    CANSparkMax intake, shooterPivot;
    PIDSparkMax sLeft, sRight;
    Spark cargoPush;
    DoubleSolenoid dSole;

    MotorControllerGroup lDrive, rDrive;
    // Drivetrain
    DifferentialDrive drivetrain;

    // Controller
    XBoxController driveCtrl;
    // Op controller
    LogitechExtreme3DProController opCtrl;

    // Limelight
    NetworkTable limelight;
    NetworkTableEntry tX, tY, tA;

    // encoder
    RelativeEncoder sLeftEncoder, sRightEncoder;

    AUTO_STATE autoState;
    final double DRIVE_WHEEL_RADIUS = 0.0762; // meters 
    final double TARMAC_DISTANCE = 2.15; // meters

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Initialize objects
        driveCtrl = new XBoxController(0);
        opCtrl = new LogitechExtreme3DProController(1);

        // Left Falcon motor(s)
        l0 = new PIDTalonFX(0);
        l1 = new PIDTalonFX(1);

        // right Falcon motor(s)
        r0 = new PIDTalonFX(2);
        r1 = new PIDTalonFX(3);

        // shooter motor(s)
        sLeft = new PIDSparkMax(4, MotorType.kBrushless);
        sRight = new PIDSparkMax(5, MotorType.kBrushless);

        intake = new CANSparkMax(6, MotorType.kBrushless);
        shooterPivot = new CANSparkMax(7, MotorType.kBrushless);

        cargoPush = new Spark(0); // TODO: Get from electronics

        // left drivetrain
        lDrive = new MotorControllerGroup(l0, l1);

        // right drivetrain
        rDrive = new MotorControllerGroup(r0, r1);

        // drivetrain
        drivetrain = new DifferentialDrive(lDrive, rDrive);

        // pneumatics
        // need to figure out the type of pneumatic
        dSole = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        tX = limelight.getEntry("tx");
        tY = limelight.getEntry("ty");
        tA = limelight.getEntry("ta");

        // shooter motor encoder
        sLeftEncoder = sLeft.getEncoder();
        sRightEncoder = sRight.getEncoder();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        SmartDashboard.putData("Left Shooter", sLeft);
        SmartDashboard.putData("Right Shooter", sLeft);
        SmartDashboard.putNumber("Pivot Angle (degrees)", shooterPivot.getEncoder().getPosition() * 360);
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different
     * autonomous modes using the dashboard. The sendable chooser code works with
     * the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
     * chooser code and
     * uncomment the getString line to get the auto name from the text box below the
     * Gyro
     *
     * <p>
     * You can add additional auto modes by adding additional comparisons to the
     * switch structure
     * below with additional strings. If using the SendableChooser make sure to add
     * them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        autoState = AUTO_STATE.TAXIING;

        // Zero sensor
        l0.getSensorCollection().setIntegratedSensorPosition(0, 1000);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        double dX = tX.getDouble(0.0f);
        double dY = tY.getDouble(0.0f);
        double dA = tA.getDouble(0.0f);

        /**
         * - TalonFX encoder measures in 2048 units per revolutions.
         * - Get the distance in raw sensor units and divide it by 2048 to the number of revolutions
         * - In theory a wheel travels the distance of its circumference in a revolution. 
         * - Multiply the number of revolutions by the circumference to get the distance travelled
         * - We know how far we need to travel to taxi off the tarmax and score points
         * - Taxi forward until we achieve that distance
         * - Move on to the next phase once we taxi the required distance
         */
        double distanceTaxied = (l0.getSensorCollection().getIntegratedSensorPosition() / 2048) * (Math.PI * 2 * DRIVE_WHEEL_RADIUS);

        switch (autoState) {

            case TAXIING:

                if (distanceTaxied < TARMAC_DISTANCE) {
                    drivetrain.tankDrive(0.5, 0.5);
                } else {
                    drivetrain.stopMotor();
                    sLeft.stopMotor();
                    sRight.stopMotor();
                    shooterPivot.stopMotor();

                    autoState = AUTO_STATE.SEEKING;
                }

            case SEEKING:

                // Spin until limelight is within range
                
                break;

            case ALIGNING:

                // Final adjustments
                
                break;


            case SHOOTING:

                // FIRE!
                
            break;
        
            default:
                drivetrain.stopMotor();
                sLeft.stopMotor();
                sRight.stopMotor();
                shooterPivot.stopMotor();
                break;
        }
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        drivetrain.tankDrive(driveCtrl.getLeftThumbstickY(), driveCtrl.getRightThumbstickY());

        // intake
        if (opCtrl.getButtonTwo()) {
            intake.set(-1);
        }

        // shooter and cargo push
        double getThreshold = 0.05f;

        if (opCtrl.getButtonOne()) {
            sLeft.set(opCtrl.getSlider());
            sRight.set(-opCtrl.getSlider());

            // % error = (actual - expected) / expected
            double sLeftError = Math.abs((sLeftEncoder.getVelocity() - (opCtrl.getSlider() * sLeft.getMaxRPM())) / (opCtrl.getSlider() * sLeft.getMaxRPM()));
            double sRightError = Math.abs((sRightEncoder.getVelocity() - (opCtrl.getSlider() * sRight.getMaxRPM())) / (opCtrl.getSlider() * sRight.getMaxRPM()));

            if ((sLeftError < getThreshold) && (sRightError < getThreshold)) {
                cargoPush.set(1);
            }
        } else {
            sLeft.stopMotor();
            sRight.stopMotor();
            cargoPush.stopMotor();
        }

        // pistons
        if (opCtrl.getButtonEleven()) {
            dSole.set(Value.kForward);
        } else if (opCtrl.getButtonTwelve()) {
            dSole.set(Value.kReverse);
        }

        // pivot controls
        shooterPivot.set(-opCtrl.getJoystickY());
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        drivetrain.stopMotor();
        sLeft.stopMotor();
        sRight.stopMotor();
        shooterPivot.stopMotor();
        dSole.set(Value.kOff);
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
