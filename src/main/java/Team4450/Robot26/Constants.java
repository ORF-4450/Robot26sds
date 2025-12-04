
package Team4450.Robot26;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Properties;

import Team4450.Robot26.subsystems.SDS.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "RAC26sds-12.03.25";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;
	    
    public static String                     functionMarker = "-".repeat(30);

    public static final double  ROBOT_PERIOD_SEC = .02;
    public static final int     ROBOT_PERIOD_MS = 20;

    // See DriveBase SimulationPeriodic for an explanation of this constant.
    //public static final double  DEG_INCR_MULTIPLIER = ROBOT_PERIOD_SEC * 57.2957795;

	// Non-drive base motor controller port assignments
    
    // Pneumatic valve controller port assignments.
    
    // CAMERAS 

    public static double maxVisionDistance = 3.0; // meters
    
    public static double xCameraOffset = 0;
    public static double yCameraOffset = 0;

    // the names of the cameras in the PhotonVision software

    //Limelight Constants:
	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

    public static final int     REV_PDB = 20;
    public static final int     CTRE_CANDLE = 21;
    public static final int     PIGEON_ID = 1;
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;
    public static final double  DRIVE_DEADBAND = 0.1, ROTATION_DEADBAND = 0.1;

	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // Swerve Drive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // Autonomous commands.
	public static final int		LCD_6 = 6;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

    public static final class DriveConstants {
        // Driving Parameters - These are the maximum capable speeds of the robot.

        public static double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double kSlowModeFactor = .20; // 50% of normal.
        public static final double kRotSlowModeFactor = .20; // 20% of normal.

        // Starting pose for sim. Is lower left corner (blue) or where we want sim robot to start.
        //public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(7.473, .559, Rotation2d.kZero);
        public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(45));
    }

    // public static final class AutoConstants {
    //     public static final double kMaxSpeedMetersPerSecond = 4.0;
    //     public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    //     public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    //     public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    //     public static final double kPXController = 1;
    //     public static final double kPYController = 1;
    //     public static final double kPThetaController = 1;

    //     public static final double kHolonomicPathFollowerP = 5.0;
        
    //     // Constraint for the motion profiled robot angle controller
    //     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    //         kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    // }

    // public static final class NeoMotorConstants {
    //     public static final double kNeoFreeSpeedRpm = 5676;
    //     public static final double kVortexFreeSpeedRpm = 6784;
    // }

  //-------------------- No student code above this line ------------------------------------------------------

}
;