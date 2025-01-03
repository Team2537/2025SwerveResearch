package frc.robot.subsystem.swerve

import choreo.trajectory.SwerveSample
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.math.util.Units.inchesToMeters
import edu.wpi.first.units.Units.FeetPerSecond
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.RobotType
import frc.robot.subsystem.swerve.module.SwerveModule
import frc.robot.subsystems.swerve.gyro.GyroIO
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2
import frc.robot.subsystems.swerve.gyro.GyroIOSim
import lib.controllers.ControllerGains
import lib.controllers.pathfollowing.SimplePathFollower
import lib.controllers.pathfollowing.repulsor.RepulsorFieldPlanner
import lib.math.rotationFromVector
import lib.math.swerve.minus
import lib.math.units.into
import lib.math.units.measuredIn
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.sqrt

class Drivebase : SubsystemBase("drivebase") {
    val moduleTranslations =
        arrayOf(
            Translation2d(moduleOffset, moduleOffset),
            Translation2d(moduleOffset, -moduleOffset),
            Translation2d(-moduleOffset, moduleOffset),
            Translation2d(-moduleOffset, -moduleOffset),
        )

    val modules: List<SwerveModule> =
        ModuleConfig.entries.mapIndexed { index, config ->
            SwerveModule(config, moduleTranslations[index])
        }

    val gyroInputs = GyroIO.GyroInputs()

    val kinematics: SwerveDriveKinematics =
        SwerveDriveKinematics(*moduleTranslations)

    val poseEstimator: SwerveDrivePoseEstimator =
        SwerveDrivePoseEstimator(kinematics, gyroInputs.yaw, modulePositions, Pose2d())

    val modulePositions: Array<SwerveModulePosition>
        get() = modules.map { it.position }.toTypedArray()

    val measuredModuleStates: Array<SwerveModuleState>
        get() = modules.map { it.state }.toTypedArray()

    val rotationalStateComponents: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }
    val translationalStateComponents: Array<SwerveModuleState> = Array(4) { SwerveModuleState() }

    val targetModuleStates = Array(4) { SwerveModuleState() }

    val robotRelativeSpeeds: ChassisSpeeds
        get() = kinematics.toChassisSpeeds(*measuredModuleStates)

    val pose: Pose2d
        get() = Pose2d(poseEstimator.estimatedPosition.translation, gyroInputs.yaw)

    val fieldPlanner = RepulsorFieldPlanner()

    val autoController =
        SimplePathFollower(
            ControllerGains(),
            ControllerGains(),
            ControllerGains(),
            ::applyChassisSpeeds,
            ::pose,
        )

    val gyro: GyroIO =
        when (RobotType.robot) {
            RobotType.Robot.BOUNTY -> GyroIOPigeon2("rio", 13)
            RobotType.Robot.SIM -> GyroIOSim(::robotRelativeSpeeds)
            else -> object : GyroIO {}
        }

    fun applyChassisSpeeds(speeds: ChassisSpeeds) {
        speeds.discretize(0.02)

        val moduleStates = kinematics.toSwerveModuleStates(speeds)

        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            speeds,
            maxModuleVelocity,
            maxModuleVelocity,
            maxAngularVelocity,
        )

        modules.forEachIndexed { index, module ->
            module.apply(moduleStates[index])
            targetModuleStates[index] = moduleStates[index]
        }
    }

    fun getDriveCommand(
        desiredForwards: DoubleSupplier,
        desiredStrafe: DoubleSupplier,
        desiredRotation: DoubleSupplier,
        shouldFieldOrient: BooleanSupplier,
    ): Command {
        return this.run {
            val speeds =
                ChassisSpeeds(
                    desiredForwards.asDouble * (maxModuleVelocity into MetersPerSecond),
                    desiredStrafe.asDouble * (maxModuleVelocity into MetersPerSecond),
                    desiredRotation.asDouble * (maxAngularVelocity into RadiansPerSecond),
                )

            if (shouldFieldOrient.asBoolean) {
                speeds.toFieldRelativeSpeeds(gyroInputs.yaw)
                println("yaw: ${gyroInputs.yaw}")
            }

            Logger.recordOutput("$name/desiredSpeeds", speeds)

            applyChassisSpeeds(speeds)
        }
    }

    fun getSendToPoseCmd(pose: Supplier<Pose2d>): Command {
        return run {
            fieldPlanner.goal = pose.get().translation
            autoController.accept(fieldPlanner.getNextSample(::pose, 0.1))
        }
    }

    fun addVisionMeasurement(
        pose: Pose2d,
        timestamp: Double,
        stddevX: Double,
        stddevY: Double,
    ) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stddevX, stddevY, 99999999.0))
        poseEstimator.addVisionMeasurement(pose, timestamp)
    }

    override fun periodic() {
        modules.forEachIndexed { index, module ->
            module.updateInputs()
            Logger.processInputs("Module [$index]", module.inputs)
        }

        gyro.updateInputs(gyroInputs)
        Logger.processInputs("Gyro", gyroInputs)

        modules.forEachIndexed { index, swerveModule ->
            val rotationalState =
                SwerveModuleState(
                    (gyroInputs.yawVelocity into RadiansPerSecond) * moduleTranslations[index].norm,
                    rotationFromVector(swerveModule.positiveRotVec),
                )

            rotationalStateComponents[index] = rotationalState

            val translationalState = swerveModule.state - rotationalState
            translationalStateComponents[index] = translationalState
        }

        poseEstimator.update(
            gyroInputs.yaw,
            modulePositions,
        )

        if (RobotBase.isSimulation()) {
            createLoggedRepulsorPath()
        }

        Logger.recordOutput("$name/targetModuleStates", *targetModuleStates)
        Logger.recordOutput("$name/measuredModuleStates", *measuredModuleStates)
        Logger.recordOutput("$name/pose", Pose2d.struct, poseEstimator.estimatedPosition)
        Logger.recordOutput("$name/robotRelativeSpeeds", robotRelativeSpeeds)
        Logger.recordOutput("$name/fieldPlanner", *fieldPlanner.arrows.toTypedArray())
        Logger.recordOutput("$name/rotationalStateComponents", *rotationalStateComponents)
        Logger.recordOutput("$name/translationalStateComponents", *translationalStateComponents)
    }

    private fun createLoggedRepulsorPath() {
        val trajectory: MutableList<SwerveSample> = mutableListOf()
        val sample = fieldPlanner.getNextSample(::pose, 0.1)
        trajectory.add(sample)
        for (i in 0..400) {
            trajectory.add(fieldPlanner.getNextSample({ trajectory.last().pose }, 0.1))
        }
        Logger.recordOutput("$name/repulsorTrajectory", *trajectory.map { it.pose }.toTypedArray())
    }

    companion object {
        val moduleOffset: Double =
            when (RobotType.robot) {
                else -> inchesToMeters(9.7859)
            }

        val maxModuleVelocity = FeetPerSecond.of(15.5)
        val chassisRadius: Distance = ((moduleOffset * sqrt(2.0)) measuredIn Meters) as Distance
        val maxAngularVelocity: AngularVelocity =
            RadiansPerSecond.of(((maxModuleVelocity into MetersPerSecond) / (chassisRadius into Meters)))

        /**
         * Enum for storing the configuration of each swerve module.
         *
         * @param driveID The ID of the drive motor.
         * @param turnID The ID of the turn motor.
         * @param encoderID The ID of the encoder.
         * @param encoderOffset The offset of the encoder.
         * @param driveInverted Whether the drive motor is inverted.
         * @param turnInverted Whether the turn motor is inverted.
         */
        enum class ModuleConfig(
            val driveID: Int,
            val turnID: Int,
            val encoderID: Int,
            val encoderOffset: Double,
            val driveInverted: Boolean,
            val turnInverted: Boolean,
        ) {
            FRONT_LEFT(1, 2, 3, 0.0, false, false),
            FRONT_RIGHT(4, 5, 6, 0.0, false, false),
            BACK_LEFT(7, 8, 9, 0.0, false, false),
            BACK_RIGHT(10, 11, 12, 0.0, false, false),
        }

        /**
         * Enum for storing the gearings of different kinds of swerve modules.
         *
         * @param drive The drive gearing.
         * @param steer The steer gearing.
         */
        enum class ModuleGearings(val drive: Double, val steer: Double) {
            MK4I_L2(6.75, 150.0 / 7.0),
        }
    }
}
