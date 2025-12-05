package Team4450.Robot26.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import Team4450.Lib.Util;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Wrapper class for Pigeon2 gyro.
 */
public class PigeonWrapper extends SubsystemBase
{
    public Pigeon2      pigeon;
    private double      startingYaw;

    public PigeonWrapper(Pigeon2 pigeon)
    {
        Util.consoleLog();

        this.pigeon = pigeon;
    }

    public Pigeon2 getPigeon()
    {
        return pigeon;
    }

    /**
     * Returns yaw of robot. Continues beyond 360.
     * @return Yaw in degrees ccw(left)- cw(right)+.
     */
    public double getYaw()
    {
        return -pigeon.getYaw().getValueAsDouble() + startingYaw;
    }

	/**
	 * Return current robot heading (0-359.n) relative to direction robot was
	 * pointed at last reset. Will return fractional angle.
	 * 1 degree is right of zero (clockwise) and 359 is left (counter clockwise).
	 * @return Robot heading in degrees.
	 */
	public double getHeading()
	{
		double heading;
		
		heading = getYaw();

		heading = heading - ((int) (heading / 360) * 360);
		
		if (heading < 0) heading += 360;
		
		return heading;
	}
    	
    /**
     * Return total yaw angle accumulated since last call to reset() constrained
     * to +-180 degrees no matter how many degrees we have rotated.
	 * @return Total yaw angle in degrees.
     */
	public double getTotalYaw180() 
    {
        return Math.IEEEremainder(getYaw(), 360);
    }

    public void reset()
    {
        Util.consoleLog();
        
        pigeon.reset();
    }
  
    /**
     * Set a starting yaw for the case where robot is not starting
     * with back bumper parallel to the wall. 
     * @param degrees - is clockwise (cw or right).
     */
    public void setStartingGyroYaw(double degrees)
    {
        Util.consoleLog("%.1f", degrees);

        startingYaw = degrees;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> getYaw(), null);
        builder.addDoubleProperty("Yaw 180", () -> getTotalYaw180(), null);
        builder.addDoubleProperty("Heading", () -> getHeading(), null);
    }
}
