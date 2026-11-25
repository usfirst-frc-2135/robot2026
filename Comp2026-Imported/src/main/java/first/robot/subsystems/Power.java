//
// Power subystem - handles Power Distribution Hub readings
//
package first.robot.subsystems;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.hal.PowerDistributionFaults;
import org.wpilib.hardware.hal.PowerDistributionStickyFaults;
import org.wpilib.system.DataLogManager;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.hardware.power.PowerDistribution.ModuleType;
import org.wpilib.command2.SubsystemBase;

/****************************************************************************
 * 
 * Power subsystem class
 */
public class Power extends SubsystemBase
{
    // Member objects
    private static final PowerDistribution m_powerDistribution = new PowerDistribution(CANPort.CAN_S0, 1, ModuleType.REV);

    /****************************************************************************
     * 
     * Constructor
     */
    public Power( )
    {
        setName("Power");

        initialize( );
    }

    /****************************************************************************
     * 
     * Periodic actions that run every scheduler loop time (20 msec)
     */
    @Override
    public void periodic( )
    {
        // This method will be called once per scheduler run
    }

    /****************************************************************************
     * 
     * Periodic actions that run every scheduler loop time (20 msec) during simulation
     */
    @Override
    public void simulationPeriodic( )
    {
        // This method will be called once per scheduler run during simulation
    }

    // Put methods for controlling this subsystem here. Call these from Commands.

    /****************************************************************************
     * 
     * Initialize subsystem during mode changes
     */
    public void initialize( )
    {
        DataLogManager
                .log(String.format("%s: Subsystem initialized!  %.1f volts", getName( ), m_powerDistribution.getVoltage( )));
    }

    /****************************************************************************
     * 
     * Write out hardware faults and reset sticky faults
     */
    public void printFaults( )
    {
        DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", getName( )));
        DataLogManager.log(String.format("  Temperature ...... %.1f C", m_powerDistribution.getTemperature( )));
        DataLogManager.log(String.format("  Input Voltage .... %.1f volts", m_powerDistribution.getVoltage( )));
        for (int i = 0; i < m_powerDistribution.getNumChannels( ); i++)
        {
            DataLogManager.log(String.format("  Channel %2d ....... %.1f amps", i, m_powerDistribution.getCurrent(i)));
        }
        DataLogManager.log(String.format("  Total Current .... %.1f Amps", m_powerDistribution.getTotalCurrent( )));
        DataLogManager.log(String.format("  Total Power ...... %.1f watts", m_powerDistribution.getTotalPower( )));
        DataLogManager.log(String.format("  Total Energy ..... %.1f joules", m_powerDistribution.getTotalEnergy( )));

        DataLogManager.log(String.format("%s: ------------------ DUMP FAULTS ------------------", getName( )));
        PowerDistributionFaults faults = m_powerDistribution.getFaults( );
        PowerDistributionStickyFaults stickyFaults = m_powerDistribution.getStickyFaults( );
        DataLogManager.log(String.format("  Brownout ......... %5s %5s", faults.brownout, stickyFaults.brownout));
        DataLogManager.log(String.format("  CanBusOff ........ %5s %5s", "", stickyFaults.canBusOff));
        DataLogManager.log(String.format("  CanWarning ....... %5s %5s", faults.canWarning, stickyFaults.canWarning));
        DataLogManager.log(String.format("  HardwareFault .... %5s %5s", faults.hardwareFault, ""));
        DataLogManager.log(String.format("  HasReset ......... %5s %5s", "", stickyFaults.hasReset));
        DataLogManager.log(String.format("  channel0BreakerFault ... %5s %5s", faults.channel0BreakerFault,
                stickyFaults.channel0BreakerFault));
        DataLogManager.log(String.format("  channel1BreakerFault ... %5s %5s", faults.channel1BreakerFault,
                stickyFaults.channel1BreakerFault));
        DataLogManager.log(String.format("  channel2BreakerFault ... %5s %5s", faults.channel2BreakerFault,
                stickyFaults.channel2BreakerFault));
        DataLogManager.log(String.format("  channel3BreakerFault ... %5s %5s", faults.channel3BreakerFault,
                stickyFaults.channel3BreakerFault));
        DataLogManager.log(String.format("  channel4BreakerFault ... %5s %5s", faults.channel4BreakerFault,
                stickyFaults.channel4BreakerFault));
        DataLogManager.log(String.format("  channel5BreakerFault ... %5s %5s", faults.channel5BreakerFault,
                stickyFaults.channel5BreakerFault));
        DataLogManager.log(String.format("  channel6BreakerFault ... %5s %5s", faults.channel6BreakerFault,
                stickyFaults.channel6BreakerFault));
        DataLogManager.log(String.format("  channel7BreakerFault ... %5s %5s", faults.channel7BreakerFault,
                stickyFaults.channel7BreakerFault));
        DataLogManager.log(String.format("  channel8BreakerFault ... %5s %5s", faults.channel8BreakerFault,
                stickyFaults.channel8BreakerFault));
        DataLogManager.log(String.format("  channel9BreakerFault ... %5s %5s", faults.channel9BreakerFault,
                stickyFaults.channel9BreakerFault));
        DataLogManager.log(String.format("  channel10BreakerFault .. %5s %5s", faults.channel10BreakerFault,
                stickyFaults.channel10BreakerFault));
        DataLogManager.log(String.format("  channel11BreakerFault .. %5s %5s", faults.channel11BreakerFault,
                stickyFaults.channel11BreakerFault));
        DataLogManager.log(String.format("  channel12BreakerFault .. %5s %5s", faults.channel12BreakerFault,
                stickyFaults.channel12BreakerFault));
        DataLogManager.log(String.format("  channel13BreakerFault .. %5s %5s", faults.channel13BreakerFault,
                stickyFaults.channel13BreakerFault));
        DataLogManager.log(String.format("  channel14BreakerFault .. %5s %5s", faults.channel14BreakerFault,
                stickyFaults.channel14BreakerFault));
        DataLogManager.log(String.format("  channel15BreakerFault .. %5s %5s", faults.channel15BreakerFault,
                stickyFaults.channel15BreakerFault));
        DataLogManager.log(String.format("  channel16BreakerFault .. %5s %5s", faults.channel16BreakerFault,
                stickyFaults.channel16BreakerFault));
        DataLogManager.log(String.format("  channel17BreakerFault .. %5s %5s", faults.channel17BreakerFault,
                stickyFaults.channel17BreakerFault));
        DataLogManager.log(String.format("  channel18BreakerFault .. %5s %5s", faults.channel18BreakerFault,
                stickyFaults.channel18BreakerFault));
        DataLogManager.log(String.format("  channel19BreakerFault .. %5s %5s", faults.channel19BreakerFault,
                stickyFaults.channel19BreakerFault));
        DataLogManager.log(String.format("  channel20BreakerFault .. %5s %5s", faults.channel20BreakerFault,
                stickyFaults.channel20BreakerFault));
        DataLogManager.log(String.format("  channel21BreakerFault .. %5s %5s", faults.channel21BreakerFault,
                stickyFaults.channel21BreakerFault));
        DataLogManager.log(String.format("  channel22BreakerFault .. %5s %5s", faults.channel22BreakerFault,
                stickyFaults.channel22BreakerFault));
        DataLogManager.log(String.format("  channel23BreakerFault .. %5s %5s", faults.channel23BreakerFault,
                stickyFaults.channel23BreakerFault));

        // m_powerDistribution.getFaults().
        m_powerDistribution.resetTotalEnergy( );
        m_powerDistribution.clearStickyFaults( );
    }
}
