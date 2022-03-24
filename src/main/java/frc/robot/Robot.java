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

// Import for controllers
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
    XBoxController driverController;
    // Op controller
    LogitechExtreme3DProController operatorController;

    // Limelight
    NetworkTable limelight;
    NetworkTableEntry tXEntry, tYEntry, targetAreaEntry, targetValidEntry;

    // encoder
    RelativeEncoder sLeftEncoder, sRightEncoder;

    AUTO_STATE autoState;
    final double DRIVE_WHEEL_RADIUS = 0.0762; // meters 
    final double TARMAC_DISTANCE = 2.15; // meters

    final double KpAim = -0.1;
    final double KpDistance = -0.1;
    final double min_aim_command = 0.05;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Initialize objects
        driverController = new XBoxController(0);
        operatorController = new LogitechExtreme3DProController(1);

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
        tXEntry = limelight.getEntry("tx");
        tYEntry = limelight.getEntry("ty");
        targetAreaEntry = limelight.getEntry("ta");
        targetValidEntry = limelight.getEntry("tv");

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
        
        SmartDashboard.putString("LimelightTarget", (targetValidEntry.getDouble(0) == 1) ? "FOUND" : "NO TARGET");
        SmartDashboard.putNumber("TargetX", tXEntry.getDouble(0));
        SmartDashboard.putNumber("TargetY", tYEntry.getDouble(0));
        SmartDashboard.putNumber("TargetAreaPercent", targetAreaEntry.getDouble(0));

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

        double targetXOffset = tXEntry.getDouble(0.0);
        double targetYOffset = tYEntry.getDouble(0.0);
        double targetArea = targetAreaEntry.getDouble(0.0);
        double targetValid = targetValidEntry.getDouble(0.0);

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
        double steering_adjust = 0.0; 

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

                // Spin until limelight detects a target

                if (targetValid == 0.0) {
                    // We don't see the target, seek for the target by spinning in place at a safe speed.

                    // TODO
                    // Determine if it's better to spin left rather than right depending on our starting position, or a dashboard option
                    boolean preferSpinLeft = true;

                    steering_adjust = preferSpinLeft ? 0.3 : -0.3;

                    // This is the wrong way to steer the robot with the given steering adjust but ...
                    drivetrain.tankDrive(steering_adjust, -steering_adjust);

                } else {
                    autoState = AUTO_STATE.ALIGNING;
                }
         
                break;

            case ALIGNING:

                // Final adjustments

                double heading_error = -targetXOffset;
                double distance_error = -targetYOffset;

                if (targetXOffset > 1.0){
                    steering_adjust = KpAim*heading_error - min_aim_command;
                } else if (targetXOffset < -1.0){
                    steering_adjust = KpAim*heading_error + min_aim_command;
                } else {
                    if (targetValid == 0.0){
                        // we lost the target
                        autoState = AUTO_STATE.SEEKING;
                        break;
                    } else {
                        // we have the target, and we are as aligned as we'll ever be
                        autoState = AUTO_STATE.SHOOTING;
                    }
                }

                double distance_adjust = KpDistance * distance_error;

                // This is the wrong way to drive the robot with the given steering adjust and distance, but ...
                drivetrain.tankDrive(steering_adjust + distance_adjust, steering_adjust + distance_adjust);
                
                break;


            case SHOOTING:

                // TODO
                // maybe save our target coordinates so we don't lose them while we adjust/angle
                // calculate pivot angle based on height
                // calculate shooting motor speeds based on distance
                // spin shooting motors up
                // once rpm is reached according to the sensor, push the ball to the shooter
                // watch as we clock someone in the stands
                
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
        drivetrain.tankDrive(driverController.getLeftThumbstickY(), driverController.getRightThumbstickY());

        // intake
        if (operatorController.getButtonTwo()) {
            intake.set(-1);
        }

        // shooter and cargo push
        double getThreshold = 0.05f;

        if (operatorController.getButtonOne()) {
            sLeft.set(operatorController.getSlider());
            sRight.set(-operatorController.getSlider());

            // % error = (actual - expected) / expected
            double sLeftError = Math.abs(1- ((sLeftEncoder.getVelocity() - (operatorController.getSlider() * sLeft.getMaxRPM())) / (operatorController.getSlider() * sLeft.getMaxRPM())));
            double sRightError = Math.abs(1 - ((sRightEncoder.getVelocity() - (operatorController.getSlider() * sRight.getMaxRPM())) / (operatorController.getSlider() * sRight.getMaxRPM())));

            if ((sLeftError < getThreshold) && (sRightError < getThreshold)) {
                cargoPush.set(1);
            }
        } else {
            sLeft.stopMotor();
            sRight.stopMotor();
            cargoPush.stopMotor();
        }

        // pistons
        if (operatorController.getButtonEleven()) {
            dSole.set(Value.kForward);
        } else if (operatorController.getButtonTwelve()) {
            dSole.set(Value.kReverse);
        }

        // pivot controls
        shooterPivot.set(-operatorController.getJoystickY());
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
