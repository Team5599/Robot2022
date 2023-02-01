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
import edu.wpi.first.cameraserver.CameraServer;

// Import for TalonFX (Drivetrain motors)
import frc.robot.PIDMotors.TalonFX.PIDTalonFX;

// Imports for SparkMax (Neo motors)
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

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
    CANSparkMax intake;
    PIDSparkMax sLeft, sRight;
    Spark cargoPush;
    DoubleSolenoid dSole;

    Spark shooterPivot;

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
    HashMap<AUTO_STATE, String> autoStateToString = new HashMap<AUTO_STATE, String>();

    final double DRIVE_WHEEL_RADIUS = 0.0762; // meters 
    final double TARMAC_DISTANCE = 6.5; // meters
    final double SHOOTER_THRESHOLD = 0.05;


    final double KpAim = -0.1;
    final double KpDistance = -0.1;
    final double min_aim_command = 0.05;

    // we use 100 as some arbitrary number and tune from here
    final double RANGE_MULTIPLIER = 100;
    double rangeEstimate = 0;

    boolean isLoadedWithBall = false;
    double distanceTaxied;

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
        l0 = new PIDTalonFX(2); //14
        l1 = new PIDTalonFX(3); //15

        // right Falcon motor(s)
        r0 = new PIDTalonFX(1); //0
        r1 = new PIDTalonFX(0); //1

        // shooter motor(s)
        sLeft = new PIDSparkMax(12, MotorType.kBrushless);
        sRight = new PIDSparkMax(3, MotorType.kBrushless);
        // sRight.setInverted(true);

        l0.setNeutralMode(NeutralMode.Coast);
        r0.setNeutralMode(NeutralMode.Coast);
        l1.setNeutralMode(NeutralMode.Coast);
        r1.setNeutralMode(NeutralMode.Coast);

        intake = new CANSparkMax(2, MotorType.kBrushless);
        shooterPivot = new Spark(2);

        cargoPush = new Spark(1);

        // left drivetrain
        lDrive = new MotorControllerGroup(l0, l1);

        // right drivetrain
        rDrive = new MotorControllerGroup(r0, r1);
        rDrive.setInverted(true);
        
        // drivetrain
        drivetrain = new DifferentialDrive(lDrive, rDrive);

        CameraServer.startAutomaticCapture();

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

        sLeft.setMaxRPM(1600);
        sRight.setMaxRPM(1600);

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
        SmartDashboard.putData("Right Shooter", sRight);
        // SmartDashboard.putNumber("Pivot Angle (degrees)", shooterPivot.getEncoder().getPosition() * 360);
        
        SmartDashboard.putString("LimelightTarget", (targetValidEntry.getDouble(0) == 1) ? "FOUND" : "NO TARGET");
        SmartDashboard.putNumber("TargetX", tXEntry.getDouble(0));
        SmartDashboard.putNumber("TargetY", tYEntry.getDouble(0));
        SmartDashboard.putNumber("TargetAreaPercent", targetAreaEntry.getDouble(0));

        // SmartDashboard.putString("AutonomousState", autoState.toString());

        SmartDashboard.putNumber("Left Shooter Encoder", sLeftEncoder.getVelocity());
        SmartDashboard.putNumber("Right Shooter Encoder", sRightEncoder.getVelocity());
        SmartDashboard.putNumber("shooterThreshold", SHOOTER_THRESHOLD);

        SmartDashboard.putNumber("Distance Taxied", distanceTaxied);
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

        autoStateToString.put(AUTO_STATE.TAXIING, "TAXIING");
        autoStateToString.put(AUTO_STATE.SEEKING, "SEEKING");
        autoStateToString.put(AUTO_STATE.ALIGNING, "ALIGNING");
        autoStateToString.put(AUTO_STATE.SHOOTING, "SHOOTING");

        setAutonomousState(AUTO_STATE.TAXIING);

        // Zero sensor
        l0.getSensorCollection().setIntegratedSensorPosition(0, 1000);
    }

    public void setAutonomousState(AUTO_STATE state) {
        System.out.println("Set Autonomous State " + autoStateToString.get(state));
        autoState = state;
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

        // drivetrain.tankDrive(0.4, 0.4);
        SmartDashboard.putString("Autonomous State", autoStateToString.get(autoState));

        double targetXOffset = tXEntry.getDouble(0.0);
        double targetYOffset = tYEntry.getDouble(0.0);
        double targetAreaPercent = targetAreaEntry.getDouble(0.0);
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
        distanceTaxied = (l0.getSensorCollection().getIntegratedSensorPosition() / 2048) * (Math.PI * 2 * DRIVE_WHEEL_RADIUS); // meters
        double steering_adjust = 0.0; 

        switch (autoState) {
            case TAXIING:
                if (distanceTaxied < TARMAC_DISTANCE) {
                    drivetrain.tankDrive(0.3, 0.3);
                } else {
                    drivetrain.stopMotor();
                    sLeft.stopMotor();
                    sRight.stopMotor();
                    shooterPivot.stopMotor();

                    setAutonomousState(AUTO_STATE.SEEKING);
                }

                break;

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
                    setAutonomousState(AUTO_STATE.ALIGNING);
                }
         
                break;

            case ALIGNING:

                // Final adjustments

                // TODO
                // Maybe use targetAreaPercent to also determine if we need to go closer/farther
                // For now, it's using the target's height to guage distance, which may not be the most accurate?

                double heading_error = -targetXOffset;
                double distance_error = -targetYOffset;

                if (targetXOffset > 1.0){
                    steering_adjust = KpAim*heading_error - min_aim_command;
                } else if (targetXOffset < -1.0){
                    steering_adjust = KpAim*heading_error + min_aim_command;
                } else {
                    if (targetValid == 0.0){
                        // we lost the target
                        setAutonomousState(AUTO_STATE.SEEKING);
                        break;
                    } else {
                        // we have the target, and we are as aligned as we'll ever be

                        // calculate range to target
                        
                        rangeEstimate = targetAreaPercent * RANGE_MULTIPLIER;
                        SmartDashboard.putNumber("EstimatedDistanceToTarget", rangeEstimate);
                        
                        setAutonomousState(AUTO_STATE.SHOOTING);
                    }
                }

                double distance_adjust = KpDistance * distance_error;

                // This is the wrong way to drive the robot with the given steering adjust and distance, but ...
                drivetrain.tankDrive(steering_adjust + distance_adjust, steering_adjust + distance_adjust);
                
                break;


            case SHOOTING:

                // TODO
                // maybe save our target coordinates so we don't lose them while we adjust/angle

                if (targetValid == 0.0){
                    // we lost the target somehow :|
                    setAutonomousState(AUTO_STATE.SEEKING);
                    break;
                }

                // calculate pivot angle based on height
                // calculate shooting motor speeds based on distance

                
                // spin shooting motors up

                double predictedSpeedForShot = 0.8;

                sLeft.set(-predictedSpeedForShot);
                sRight.set(predictedSpeedForShot);

                // shooter and cargo push

                // % error = (actual - expected) / expected
                double sLeftError = Math.abs((sLeftEncoder.getVelocity() - (-predictedSpeedForShot * sLeft.getMaxRPM())) / (-predictedSpeedForShot * sLeft.getMaxRPM()));
                double sRightError = Math.abs((sRightEncoder.getVelocity() - (predictedSpeedForShot * sRight.getMaxRPM())) / (predictedSpeedForShot * sRight.getMaxRPM()));

                // once rpm is reached according to the sensor, push the ball to the shooter
                // watch as we clock someone in the stands
                if ((sLeftError < SHOOTER_THRESHOLD) && (sRightError < SHOOTER_THRESHOLD)) {
                    cargoPush.set(1.0);
                }

                // TODO
                // Wait a certain number of rpms/seconds
                // change state to AUTO_STATE.DONE?
                
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
        drivetrain.tankDrive(driverController.getLeftThumbstickY() * 0.7 , driverController.getRightThumbstickY() * 0.7);

        // intake
        if (operatorController.getButtonThree()) {
            sLeft.set(-0.9);
            sRight.set(0.9);
            intake.set(0.4);
            shooterPivot.set(0.95);
        } else if (operatorController.getButtonFive()){
            sLeft.set(0.9);
            sRight.set(0.9);
            intake.set(-0.4);
            shooterPivot.set(0.95);
            cargoPush.set(1.0);
        }

        // shooter and cargo push
        if (operatorController.getButtonOne()) {

            double rpmDifference = sLeftEncoder.getVelocity() - Math.abs(sRightEncoder.getVelocity());
            double decreaseLeftBy = (rpmDifference / 1600);
            System.out.println("decrease by " + ( 1 - decreaseLeftBy));

            sRight.set(operatorController.getSlider());
            sLeft.set(-operatorController.getSlider());

            // sRight.set(operatorController.getSlider());
            // sLeft.set(operatorController.getSlider());
            

            // % error = (actual - expected) / expected

        
            // double sLeftError =     Math.abs((sLeftEncoder.getVelocity()  - (operatorController.getSlider() * sLeft.getMaxRPM() )) / (operatorController.getSlider() * sLeft.getMaxRPM() ));
            // double sRightError =   -Math.abs((sRightEncoder.getVelocity() - (operatorController.getSlider() * sRight.getMaxRPM())) / (operatorController.getSlider() * sRight.getMaxRPM()));

            // SmartDashboard.putNumber("sLeftError", sLeftError);
            // SmartDashboard.putNumber("sRightError", sRightError);

            // if ((sLeftError < SHOOTER_THRESHOLD) && (sRightError < SHOOTER_THRESHOLD)) {
            //      cargoPush.set(0.99);
            // }


            // shoot based on operator
            double expectedRPMLeft = operatorController.getSlider() * -0.9 * 400;
            double expectedRPMRight = operatorController.getSlider() * 0.9 * 400;

            Boolean isLeftAtRPM = (sLeftEncoder.getVelocity() > expectedRPMLeft);
            Boolean isRightAtRPM = (sRightEncoder.getVelocity() < expectedRPMRight);

            SmartDashboard.putNumber("ExpectedRPMLeft", expectedRPMLeft);
            SmartDashboard.putNumber("ExpectedRPMRight", expectedRPMRight);

            if (isLeftAtRPM && isRightAtRPM){
                System.out.println("FIRE");
                cargoPush.set(-1.0);
                // intake.set(0.9);
                shooterPivot.set(-0.95);
            }

        } else if (!operatorController.getButtonFive()) {
            cargoPush.stopMotor();
        }

        if (!operatorController.getButtonThree() && !operatorController.getButtonOne() && !operatorController.getButtonFive()){
            shooterPivot.stopMotor();
            sLeft.stopMotor();
            sRight.stopMotor();
            intake.stopMotor();
        }

        // pistons
        if (operatorController.getButtonEleven()) {
            dSole.set(Value.kForward);
        } else if (operatorController.getButtonTwelve()) {
            dSole.set(Value.kReverse);
        }

        // pivot controls
        // shooterPivot.set(-operatorController.getJoystickY());
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
