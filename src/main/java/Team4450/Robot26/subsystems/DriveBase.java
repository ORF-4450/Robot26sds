package Team4450.Robot26.subsystems;

import static Team4450.Robot26.Constants.*;
import static Team4450.Robot26.Constants.DriveConstants.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import Team4450.Lib.Util;
import Team4450.Robot26.Constants.DriveConstants;
import Team4450.Robot26.subsystems.SDS.CommandSwerveDrivetrain;
import Team4450.Robot26.subsystems.SDS.TunerConstants;
import Team4450.Robot26.subsystems.SDS.Telemetry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

/**
 * This class wraps the SDS drive base subsystem allowing us to add/modify drive base
 * functions without modifyinig the SDS code generated from Tuner. Also allows for
 * convenience wrappers for more complex functions in SDS code.
 */
public class DriveBase extends SubsystemBase
{
    private CommandSwerveDrivetrain     sdsDriveBase = TunerConstants.createDrivetrain();

    public PigeonWrapper                gyro = new PigeonWrapper(sdsDriveBase.getPigeon2());
    
    private final Telemetry     		logger = new Telemetry(kMaxSpeed);
    
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private boolean                     fieldRelativeDriving = true, slowMode = false;
    private double                      driveSlowfactor = 1.0, rotateSlowfactor = 1.0;
    private double                      startingGyroRotation;

    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public DriveBase()
    {
        Util.consoleLog();

        // Add gyro as a Sendable. Updates the dashboard heading indicator automatically.

		SmartDashboard.putData("Gyro2", gyro); 

		// Check Gyro.
	  
		if (gyro.getPigeon().isConnected())
			Util.consoleLog("Pigeon connected version=%s", gyro.getPigeon().getVersion());
		else
		{
			Exception e = new Exception("Pigeon is NOT connected!");
			Util.logException(e);
		}

		// Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        
		final var idle = new SwerveRequest.Idle();

        RobotModeTriggers.disabled().whileTrue(
            sdsDriveBase.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Set tracking of robot field position at starting point. Blue perspective.
        // note that this doesn't really do much because PathPlanner redoes this anyway.
        // More for a starting pose in sim testing.

        resetOdometry(DriveConstants.DEFAULT_STARTING_POSE);

        // Under sim, we pose the robot before you can change the alliance in the sim UI.
        // We can't really do it anywhere else or it would interfere with transition from
        // auto to teleop. So we pose robot at lower left (blue) corner and force the blue
        // driving perspective.

        if (RobotBase.isSimulation()) driveField.ForwardPerspective = ForwardPerspectiveValue.BlueAlliance;
		         
        // Register SDS telemetry.

        sdsDriveBase.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void periodic() 
    {
        super.periodic();

        sdsDriveBase.periodic();
    }

    public void drive(double throttle, double strafe, double rotation)
    {
        if (fieldRelativeDriving)
            sdsDriveBase.setControl(
                driveField.withVelocityX(throttle * driveSlowfactor * kMaxSpeed) 
                        .withVelocityY(strafe * driveSlowfactor * kMaxSpeed) 
                        .withRotationalRate(rotation * rotateSlowfactor * kMaxAngularRate));
        else
            sdsDriveBase.setControl(
                driveRobot.withVelocityX(throttle * driveSlowfactor * kMaxSpeed) 
                        .withVelocityY(strafe * driveSlowfactor * kMaxSpeed) 
                        .withRotationalRate(rotation * rotateSlowfactor * kMaxAngularRate));
    }

    /**
     * Set drive wheels to X configuration to lock robot from moving.
     */
    public void setX()
    {
        Util.consoleLog();

        sdsDriveBase.applyRequest(() -> brake);
    }
        
	public void toggleFieldRelativeDriving()
	{
		fieldRelativeDriving = !fieldRelativeDriving;
        
        SmartDashboard.putBoolean("Field Relative", fieldRelativeDriving);

        Util.consoleLog("%b", fieldRelativeDriving);
	}

    public void toggleSlowMode() 
    {
        slowMode = !slowMode;

        Util.consoleLog("%b", slowMode);

        if (slowMode)
        {
            driveSlowfactor = kSlowModeFactor;
            rotateSlowfactor = kRotSlowModeFactor;
        } else driveSlowfactor = rotateSlowfactor = 1.0;
    }

    public void resetFieldOrientation()
    {
        Util.consoleLog();
        
        sdsDriveBase.seedFieldCentric();
    }

    /**
     * Sets the gyroscope yaw angle to zero. This can be used to set the direction
     * the robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyro()
    {
        Util.consoleLog();

        //navx.reset();
        
        //m_gyro.reset();
    }
    
    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        Util.consoleLog(pose.toString());

        sdsDriveBase.resetPose(pose);

        setStartingGyroRotation(pose.getRotation().getDegrees());
    }
    
    /**
     * Set a starting pose rotation for the case where robot is not starting
     * with bumper parallel to the wall. 
     * @param degrees - is clockwise (cw or right).
     */
    public void setStartingGyroRotation(double degrees)
    {
        startingGyroRotation = degrees;
    }

    /**
     * Returns current pose of the robot.
     * @return Robot pose.
     */
    public Pose2d getPose()
    {
        return sdsDriveBase.getState().Pose;
    }

    public double getYaw()
    {
        return gyro.getYaw();
    }
}
