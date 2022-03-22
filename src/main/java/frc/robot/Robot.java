// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import for TalonFX (Drivetrain motors)
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

// Imports for SparkMax (Neo motors)
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

// Import for pneumatics (PCM)
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

// Import for xbox controller
import frc.robot.Controllers.XBoxController;

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
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();

    // Declaration of Objects
    // Falcon FX (Falcon 500) Motors
    WPI_TalonFX l0, l1, r0, r1;
    CANSparkMax intake, shooterPivot;
    CtrlSpark s0, s1;
    DoubleSolenoid dSole;

    MotorControllerGroup lDrive, rDrive, shooter;

    // Drivetrain
    DifferentialDrive drivetrain;

    // Controller
    XBoxController ctrl;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);

        // Initialize objects
        ctrl = new XBoxController(0);

        // Left Falcon motor(s)
        l0 = new WPI_TalonFX(0);
        l1 = new WPI_TalonFX(1);

        // right Falcon motor(s)
        r0 = new WPI_TalonFX(3);
        r1 = new WPI_TalonFX(4);    
        
        // shooter motor(s)
        s0 = new CtrlSpark(8, MotorType.kBrushless);
        s1 = new CtrlSpark(9, MotorType.kBrushless);

        intake = new CANSparkMax(6, MotorType.kBrushless);
        shooterPivot = new CANSparkMax(7, MotorType.kBrushless);

        l0.configFactoryDefault();
        l1.configFactoryDefault();

        r0.configFactoryDefault();
        r1.configFactoryDefault();

        shooter = new MotorControllerGroup(s0, s1);

        // left drivetrain
        lDrive = new MotorControllerGroup(l0, l1);

        // right drivetrain
        rDrive = new MotorControllerGroup(r0, r1);

        // drivetrain
        drivetrain = new DifferentialDrive(lDrive, rDrive);

        // pneumatics
        // need to figure out the type of pneumatic
        dSole = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

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
        m_autoSelected = m_chooser.getSelected();
        // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        System.out.println("Auto selected: " + m_autoSelected);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        switch (m_autoSelected) {
            case kCustomAuto:
                // Put custom auto code here
                break;
            case kDefaultAuto:
            default:
                // Put default auto code here
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
        drivetrain.tankDrive(ctrl.getLeftThumbstickY(), ctrl.getRightThumbstickY());
        intake.set(ctrl.getLeftTriggerAbsolute());
        shooter.set(ctrl.getRightTriggerAbsolute());
        dSole.set(ctrl.getRightBumper() ? Value.kForward : Value.kOff);
        dSole.set(ctrl.getLeftBumper() ? Value.kReverse : Value.kOff);
        
        // pivot controls
        if (ctrl.getDPadUp()) {
            shooterPivot.set(-1.0f);
        } else if (ctrl.getDPadDown()) {
            shooterPivot.set(1.0f);
        } else {
            shooterPivot.stopMotor();
        }
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
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
