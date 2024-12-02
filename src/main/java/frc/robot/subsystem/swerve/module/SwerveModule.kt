package frc.robot.subsystem.swerve.module

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import frc.robot.RobotType
import frc.robot.subsystem.swerve.Drivebase.Companion.ModuleConfig
import frc.robot.subsystem.swerve.Drivebase.Companion.ModuleGearings
import lib.controllers.ControllerGains
import lib.math.vector

class SwerveModule(private val moduleConfig: ModuleConfig, private val translation: Translation2d) {
    /**
     * The IO implementation for the module.
     * Picks the correct implementation based on the robot type.
     */
    val io: ModuleIO =
        when (RobotType.robot) {
            RobotType.Robot.BOUNTY ->
                ModuleIOHybrid(
                    moduleConfig = moduleConfig,
                    gearings = ModuleGearings.MK4I_L2,
                    driveGains =
                        ControllerGains(
                            kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                            kS = 0.0,
                            kV = 0.0,
                            kA = 0.0,
                        ),
                    turnGains =
                        ControllerGains(
                            kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                            kS = 0.0,
                            kV = 0.0,
                            kA = 0.0,
                        ),
                )
            RobotType.Robot.SIM ->
                ModuleIOSim(
                    driveMotor = DCMotor.getKrakenX60(1),
                    turnMotor = DCMotor.getNEO(1),
                    driveGains =
                        ControllerGains(
                            kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                            kS = 0.0,
                            kV = 0.0,
                            kA = 0.0,
                        ),
                    turnGains =
                        ControllerGains(
                            kP = 0.0,
                            kI = 0.0,
                            kD = 0.0,
                        ),
                    gearings = ModuleGearings.MK4I_L2,
                )
            else -> object : ModuleIO {}
        }

    /**
     * Inputs object for storing data pulled from the module.
     */
    val inputs: ModuleIO.ModuleInputs = ModuleIO.ModuleInputs()

    /**
     * The current angle of the module.
     */
    val angle: Rotation2d
        get() = inputs.absoluteTurnPosition

    /**
     * The current speed of the module.
     */
    val speed: LinearVelocity
        get() = inputs.driveVelocity

    val state: SwerveModuleState
        get() = SwerveModuleState(speed, angle)

    val position
        get() = SwerveModulePosition(inputs.drivePosition, angle)

    /**
     * The vector pointing in the direction that the module would be facing if it was contributing 100% of its velocity
     * to the robot rotating counter-clockwise.
     */
    val positiveRotVec = (translation.angle + Rotation2d.fromDegrees(90.0)).vector

    /**
     * Set the target translation and velocity of the module.
     *
     * @param state The target state of the module.
     * @see SwerveModuleState
     */
    fun apply(state: SwerveModuleState) {
        state.optimize(inputs.absoluteTurnPosition)
        state.cosineScale(inputs.absoluteTurnPosition)

        io.runDriveVelocitySetpoint(state.speedMetersPerSecond)
        io.runTurnPositionSetpoint(state.angle)
    }

    /**
     * Send an arbitrary voltage to the module.
     * Used for characterizing the module.
     *
     * @param voltage The voltage to send to the module.
     */
    fun applyVoltage(voltage: Voltage) {
        io.runDriveVolts(voltage)
    }

    /**
     * Set the target angle of the module.
     *
     * @param angle The target angle of the module.
     */
    fun pointAt(angle: Rotation2d) {
        io.runTurnPositionSetpoint(angle)
    }

    /** @suppress */
    fun updateInputs() {
        io.updateInputs(inputs)
    }

    companion object {
        val wheelDiameter: Distance = Inches.of(4.0)
        val slipCurrent: Current = Amps.of(150.0)
    }
}
