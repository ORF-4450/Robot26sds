package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.subsystems.SDS.CommandSwerveDrivetrain;
import static Team4450.Robot26.Constants.*;
import static Team4450.Robot26.Constants.DriveConstants.*;


public class DriveCommand extends Command 
{
    private final CommandSwerveDrivetrain driveBase;

    private final DoubleSupplier    throttleSupplier;
    private final DoubleSupplier    strafeSupplier;
    private final DoubleSupplier    rotationSupplier;
    private final XboxController    controller;

    private boolean                 fieldRelativeDriving = true, slowMode = false;
    private double                  driveSlowfactor = 1.0, rotateSlowfactor = 1.0;

    private final SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
            .withDeadband(kMaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(kMaxAngularRate * ROTATION_DEADBAND) // Add deadbands
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public DriveCommand(CommandSwerveDrivetrain driveBase,
                        DoubleSupplier throttleSupplier,
                        DoubleSupplier strafeSupplier,
                        DoubleSupplier rotationSupplier,
                        XboxController controller) 
    {
        Util.consoleLog();

        this.driveBase = driveBase;
        this.throttleSupplier = throttleSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.controller = controller;

        addRequirements(driveBase);
    }

    @Override
    public void initialize()
    {
        Util.consoleLog();
    }

    @Override
    public void execute() 
    {
        LCD.printLine(2, "rx=%.3f  ry=%.3f  throttle=%.3f  strafe=%.3f  rot=%.3f",
            controller.getRightX(),
            controller.getRightY(),
            throttleSupplier.getAsDouble(),
            strafeSupplier.getAsDouble(),
            rotationSupplier.getAsDouble()
        );

        LCD.printLine(3, "lx=%.3f  ly=%.3f", // yaw=%.3f",
            controller.getLeftX(),
            controller.getLeftY()
            //driveBase.getGyroRotation2d().getDegrees(),
            //driveBase.getGyroYaw() rich
        );

        // This is the default command for the DriveBase. When running in autonmous, the auto commands
        // require DriveBase, which preempts the default DriveBase command. However, if our auto code ends 
        // before end of auto period, then this drive command resumes and is feeding drivebase during remainder
        // of auto period. This was not an issue until the joystick drift problems arose, so the resumption of a 
        // driving command during auto had the robot driving randomly after our auto program completed. The if 
        // statment below prevents this.
        
        if (robot.isAutonomous()) return;

        double throttle = throttleSupplier.getAsDouble() * driveSlowfactor;
        double strafe = strafeSupplier.getAsDouble() * driveSlowfactor;
        double rotation = rotationSupplier.getAsDouble() * rotateSlowfactor;

        // throttle = Util.squareInput(throttle);
        // strafe = Util.squareInput(strafe);
        // rotation = Util.squareInput(rotation);
        // rotation = Math.pow(rotation, 5);

        if (fieldRelativeDriving)
            driveBase.setControl(
                driveField.withVelocityX(throttle * kMaxSpeed) 
                          .withVelocityY(strafe * kMaxSpeed) 
                          .withRotationalRate(rotation * kMaxAngularRate));
        else
            driveBase.setControl(
                driveRobot.withVelocityX(throttle * kMaxSpeed) 
                          .withVelocityY(strafe * kMaxSpeed) 
                          .withRotationalRate(rotation * kMaxAngularRate));
    }

    @Override 
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);
    }
    
	public void toggleFieldRelativeDriving() // rich
	{
		fieldRelativeDriving = !fieldRelativeDriving;
	}

    public void toggleSlowMode() // rich
    {
        slowMode = !slowMode;

        if (slowMode)
        {
            driveSlowfactor = kSlowModeFactor;
            rotateSlowfactor = kRotSlowModeFactor;
        } else driveSlowfactor = rotateSlowfactor = 1.0;
    }
}
