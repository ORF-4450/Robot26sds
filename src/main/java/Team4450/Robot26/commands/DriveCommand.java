package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import static Team4450.Robot26.Constants.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Robot26.subsystems.SDS.CommandSwerveDrivetrain;
import Team4450.Robot26.subsystems.SDS.TunerConstants;

public class DriveCommand extends Command 
{
    private final CommandSwerveDrivetrain driveBase;

    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier strafeSupplier;
    private final DoubleSupplier rotationSupplier;
    private final XboxController controller;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * DRIVE_DEADBAND)
            .withRotationalDeadband(MaxAngularRate * ROTATION_DEADBAND) // Add a 10% deadband
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

        //double throttle = deadband(throttleSupplier.getAsDouble(), DRIVE_DEADBAND);
        //double strafe = deadband(strafeSupplier.getAsDouble(), DRIVE_DEADBAND);
        //double rotation = deadband(rotationSupplier.getAsDouble(), ROTATION_DEADBAND);

        double throttle = throttleSupplier.getAsDouble();
        double strafe = strafeSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();

        // throttle = Util.squareInput(throttle);
        // strafe = Util.squareInput(strafe);
        // rotation = Util.squareInput(rotation);
        // rotation = Math.pow(rotation, 5);

        // Have to invert for sim...not sure why.
        //if (RobotBase.isSimulation()) rotation *= -1; rich
        
        //driveBase.drive(throttle, strafe, rotation, true); rich

        driveBase.applyRequest(() ->
            drive.withVelocityX(throttle * MaxSpeed) // Drive forward with negative Y (forward)
                 .withVelocityY(strafe * MaxSpeed) // Drive left with negative X (left)
                 .withRotationalRate(rotation * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }

    @Override 
    public void end(boolean interrupted) 
    {
        Util.consoleLog("interrupted=%b", interrupted);

        //driveBase.drive(0.0, 0.0, 0.0);
    }
 
    private static double deadband(double value, double deadband) 
    {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    // commented out because Util.squareInput does this already and it was giving an error
    // private static double squareTheInput(double value) 
    // {
    //     return Math.copySign(value * value, value);
    // }
}
