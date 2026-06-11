package org.firstinspires.ftc.teamcode.core.sim

import com.qualcomm.robotcore.hardware.HardwareMap
import java.io.File
import org.firstinspires.ftc.teamcode.core.control.PIDFGains
import org.firstinspires.ftc.teamcode.core.control.ProfileConstraints
import org.firstinspires.ftc.teamcode.core.control.ProfiledController
import org.firstinspires.ftc.teamcode.core.hw.MotorIO
import org.firstinspires.ftc.teamcode.core.hw.SimMotorIO
import org.firstinspires.ftc.teamcode.core.logging.WpiLog
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.subsystems.ProfiledMotorSubsystem
import org.firstinspires.ftc.teamcode.core.util.FakeClock
import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

/**
 * Deterministic replay through the IO seam: run a mechanism against the sim
 * model while the flight recorder logs its channels, then re-run the *same
 * subsystem code* against a [ReplayMotorIO] fed from the recorded log and
 * assert it produces bit-identical output powers. This is the regression
 * harness shape for "did this refactor change control behavior?".
 */
class MechanismReplayTest {

    private companion object {
        const val TICKS = 200
        const val TICK_MS = 20.0
        const val GOAL = 24.0
    }

    private fun controller() = ProfiledController(
        ProfileConstraints(maxVelocity = 50.0, maxAcceleration = 150.0),
        PIDFGains(kP = 0.4, kV = 0.01),
    )

    /** Feeds recorded encoder readings back; collects commanded powers. */
    private class ReplayMotorIO(
        private val positionsTicks: List<Double>,
        private val velocitiesTicksPerSec: List<Double>,
    ) : MotorIO {
        var tick = 0
        val powers = mutableListOf<Double>()

        override val positionTicks: Double get() = positionsTicks[tick.coerceAtMost(positionsTicks.size - 1)]
        override val velocityTicksPerSec: Double get() = velocitiesTicksPerSec[tick.coerceAtMost(velocitiesTicksPerSec.size - 1)]
        override var lastPower: Double = 0.0
            private set

        override fun setPower(power: Double) {
            lastPower = power
            powers += power
            tick++
        }

        override fun resetEncoder() {}
    }

    @Test
    fun replayReproducesRecordedOutputsExactly() {
        val logDir = File.createTempFile("replay-logs", "").let {
            it.delete()
            it.mkdirs().let { _ -> it }
        }
        try {
            // ---- Run 1: sim physics, flight recorder logging channels.
            val clock = FakeClock()
            val robot = Robot(HardwareMap(null, null), clock)
            val lift = robot.register(
                ProfiledMotorSubsystem(
                    name = "Lift",
                    motorName = "unused-in-sim",
                    controller = controller(),
                    ticksPerUnit = 1.0,
                    io = SimMotorIO(clock, freeSpeedTicksPerSec = 100.0, timeConstantSec = 0.05),
                    clock = clock,
                ),
            )
            robot.enableFlightRecorder(
                "ReplayTest",
                driver = { null },
                operator = { null },
                batteryVoltage = { null },
                directory = logDir,
            )
            robot.init()
            robot.start()
            robot.scheduler.schedule(lift.goToCommand(GOAL, toleranceUnits = 0.5))
            repeat(TICKS) {
                clock.advanceMs(TICK_MS)
                robot.loop()
            }
            robot.stop()

            // ---- Read the recorded channels back.
            val logFile = logDir.listFiles { f -> f.extension == "wpilog" }!!.single()
            val log = WpiLog.read(logFile)
            val positions = log.doubles("Lift/positionUnits").map { it.second }
            val velocities = log.doubles("Lift/velocityUnitsPerSec").map { it.second }
            val recordedPowers = log.doubles("Lift/outputPower").map { it.second }
            assertEquals(TICKS, positions.size)
            assertEquals(TICKS, recordedPowers.size)

            // ---- Run 2: same code, inputs replayed from the log.
            val replayClock = FakeClock()
            val replayRobot = Robot(HardwareMap(null, null), replayClock)
            val replayIo = ReplayMotorIO(positions, velocities) // ticksPerUnit = 1.0
            val replayLift = replayRobot.register(
                ProfiledMotorSubsystem(
                    name = "Lift",
                    motorName = "unused-in-sim",
                    controller = controller(),
                    ticksPerUnit = 1.0,
                    io = replayIo,
                    clock = replayClock,
                ),
            )
            replayRobot.init()
            replayRobot.start()
            replayRobot.scheduler.schedule(replayLift.goToCommand(GOAL, toleranceUnits = 0.5))
            repeat(TICKS) {
                replayClock.advanceMs(TICK_MS)
                replayRobot.loop()
            }
            replayRobot.stop()

            // robot.stop() writes a final 0.0; compare the in-loop powers.
            val replayedPowers = replayIo.powers.take(TICKS)
            assertEquals(TICKS, replayedPowers.size)
            for (i in 0 until TICKS) {
                assertEquals("tick $i", recordedPowers[i], replayedPowers[i], 1e-12)
            }
            // Sanity: the run actually did something.
            assertTrue(recordedPowers.any { it > 0.05 })
        } finally {
            logDir.deleteRecursively()
        }
    }
}
