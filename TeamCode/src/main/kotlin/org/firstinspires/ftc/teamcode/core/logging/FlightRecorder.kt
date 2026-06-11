package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.util.RobotLog
import java.io.BufferedOutputStream
import java.io.File
import java.io.FileOutputStream
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import org.firstinspires.ftc.teamcode.core.runtime.DriveTelemetrySource
import org.firstinspires.ftc.teamcode.core.runtime.LoopPhase
import org.firstinspires.ftc.teamcode.core.runtime.Robot
import org.firstinspires.ftc.teamcode.core.util.GamepadEx

/**
 * Per-op-mode binary flight recorder.
 *
 * I/O failures permanently disable the recorder for this op-mode. The loop
 * should keep running even if storage is full or unavailable.
 */
class FlightRecorder private constructor(
    private val writer: WpiLogWriter,
    private val gamepad1: () -> GamepadEx?,
    private val gamepad2: () -> GamepadEx?,
    private val batteryVoltage: () -> Double?,
    private val runningCommandNames: () -> List<String>,
) : AutoCloseable {
    private val startNs = System.nanoTime()
    private var enabled = true
    private var lastRunningCommands = ""
    private var lastFlushNs = startNs

    // Resolved once and reused: record() runs every tick, so no per-tick
    // subsystem filtering or array allocation.
    private var driveSource: DriveTelemetrySource? = null
    private var driveResolved = false
    private val poseValues = DoubleArray(3)
    private val velocityValues = DoubleArray(3)
    private val axesValues = DoubleArray(6)

    private val pose = writer.startEntry("pose", "double[]")
    private val velocity = writer.startEntry("velocity", "double[]")
    private val driveMode = writer.startEntry("driveMode", "string")
    private val gamepad1Axes = writer.startEntry("gamepad1/axes", "double[]")
    private val gamepad1Buttons = writer.startEntry("gamepad1/buttons", "int64")
    private val gamepad2Axes = writer.startEntry("gamepad2/axes", "double[]")
    private val gamepad2Buttons = writer.startEntry("gamepad2/buttons", "int64")
    private val loopTotal = writer.startEntry("loop/totalNanos", "int64")
    private val loopPhaseEntries = IntArray(LoopPhase.entries.size) { i ->
        writer.startEntry("loop/${'$'}{LoopPhase.entries[i].label}Nanos", "int64")
    }
    private val battery = writer.startEntry("battery", "double")
    private val runningCommands = writer.startEntry("commands/running", "string")
    private val events = writer.startEntry("events", "string")
    private val followTranslationalError = writer.startEntry("follow/translationalErrorIn", "double")
    private val followHeadingError = writer.startEntry("follow/headingErrorRad", "double")

    // Subsystem channels (SubsystemBase.logState) are created lazily on first
    // put and cached by full name. One reusable sink instance — no per-tick
    // allocation beyond a first-time channel registration.
    private val channelIds = HashMap<String, Int>()
    private val lastStrings = HashMap<String, String>()
    private val subsystemSink = SubsystemSink()

    private inner class SubsystemSink : StateLog {
        var prefix: String = ""
        var timestampUs: Long = 0L

        override fun put(channel: String, value: Double) {
            writer.appendDouble(entry(channel, "double"), value, timestampUs)
        }

        override fun put(channel: String, value: Long) {
            writer.appendInt64(entry(channel, "int64"), value, timestampUs)
        }

        override fun put(channel: String, value: Boolean) {
            writer.appendBoolean(entry(channel, "boolean"), value, timestampUs)
        }

        override fun put(channel: String, value: String) {
            val name = prefix + channel
            if (lastStrings[name] == value) return
            lastStrings[name] = value
            writer.appendString(entry(channel, "string"), value, timestampUs)
        }

        private fun entry(channel: String, type: String): Int {
            val name = prefix + channel
            return channelIds.getOrPut(name) { writer.startEntry(name, type) }
        }
    }

    fun record(robot: Robot) {
        if (!enabled) return
        guard {
            val ts = timestampUs()
            if (!driveResolved) {
                driveResolved = true
                for (subsystem in robot.subsystems()) {
                    if (subsystem is DriveTelemetrySource) {
                        driveSource = subsystem
                        break
                    }
                }
            }
            val drive = driveSource
            if (drive != null) {
                val p = drive.pose
                poseValues[0] = p.x
                poseValues[1] = p.y
                poseValues[2] = p.heading
                writer.appendDoubleArray(pose, poseValues, ts)
                val v = drive.velocity
                velocityValues[0] = v.x
                velocityValues[1] = v.y
                val angular = drive.angularVelocityRadPerSec
                velocityValues[2] = if (angular.isFinite()) angular else 0.0
                writer.appendDoubleArray(velocity, velocityValues, ts)
                writer.appendString(driveMode, drive.driveModeName, ts)
                if (drive.isPathing) {
                    // The follower's error terms answer "was the path bad or
                    // did the PID oscillate?" — NaN means the source couldn't
                    // read them; the channel just goes quiet.
                    val translational = drive.followTranslationalErrorInches
                    if (translational.isFinite()) {
                        writer.appendDouble(followTranslationalError, translational, ts)
                    }
                    val headingErr = drive.followHeadingErrorRad
                    if (headingErr.isFinite()) {
                        writer.appendDouble(followHeadingError, headingErr, ts)
                    }
                }
            }

            writeGamepad(gamepad1(), gamepad1Axes, gamepad1Buttons, ts)
            writeGamepad(gamepad2(), gamepad2Axes, gamepad2Buttons, ts)
            val p = robot.profile
            writer.appendInt64(loopTotal, p.totalNanos, ts)
            for (phase in LoopPhase.entries) {
                // RECORD is written by recordRecorderNanos after this call —
                // the recorder can't time its own final write.
                if (phase == LoopPhase.RECORD) continue
                writer.appendInt64(loopPhaseEntries[phase.ordinal], p[phase], ts)
            }
            batteryVoltage()?.takeIf { it.isFinite() }?.let {
                writer.appendDouble(battery, it, ts)
            }

            val running = runningCommandNames().joinToString("\n")
            if (running != lastRunningCommands) {
                writer.appendString(runningCommands, running, ts)
                lastRunningCommands = running
            }

            subsystemSink.timestampUs = ts
            for (subsystem in robot.subsystems()) {
                subsystemSink.prefix = subsystem.name + "/"
                subsystem.logState(subsystemSink)
            }

            // Periodic flush so a brownout or battery pull — exactly the runs
            // worth diagnosing — doesn't lose the buffered tail of the log.
            val now = System.nanoTime()
            if (now - lastFlushNs >= FLUSH_INTERVAL_NS) {
                writer.flush()
                lastFlushNs = now
            }
        }
    }

    fun recordRecorderNanos(recordNanos: Long) {
        if (!enabled) return
        guard {
            writer.appendInt64(loopPhaseEntries[LoopPhase.RECORD.ordinal], recordNanos, timestampUs())
        }
    }

    fun event(message: String) {
        if (!enabled) return
        guard {
            writer.appendString(events, message, timestampUs())
        }
    }

    override fun close() {
        if (!enabled) return
        guard {
            writer.flush()
            writer.close()
        }
        enabled = false
    }

    private fun writeGamepad(pad: GamepadEx?, axesEntry: Int, buttonsEntry: Int, timestampUs: Long) {
        if (pad == null) return
        axesValues[0] = pad.leftStickX
        axesValues[1] = pad.leftStickY
        axesValues[2] = pad.rightStickX
        axesValues[3] = pad.rightStickY
        axesValues[4] = pad.leftTrigger
        axesValues[5] = pad.rightTrigger
        writer.appendDoubleArray(axesEntry, axesValues, timestampUs)
        writer.appendInt64(buttonsEntry, buttonMask(pad), timestampUs)
    }

    private fun buttonMask(pad: GamepadEx): Long {
        var mask = 0L
        fun bit(index: Int, value: Boolean) {
            if (value) mask = mask or (1L shl index)
        }
        bit(0, pad.a)
        bit(1, pad.b)
        bit(2, pad.x)
        bit(3, pad.y)
        bit(4, pad.leftBumper)
        bit(5, pad.rightBumper)
        bit(6, pad.dpadUp)
        bit(7, pad.dpadDown)
        bit(8, pad.dpadLeft)
        bit(9, pad.dpadRight)
        bit(10, pad.start)
        bit(11, pad.back)
        bit(12, pad.leftStickButton)
        bit(13, pad.rightStickButton)
        return mask
    }

    private fun timestampUs(): Long = (System.nanoTime() - startNs) / 1_000L

    private inline fun guard(block: () -> Unit) {
        try {
            block()
        } catch (e: IOException) {
            disable(e)
        } catch (e: RuntimeException) {
            if (e.cause is IOException) disable(e) else throw e
        }
    }

    private fun disable(t: Throwable) {
        enabled = false
        try {
            RobotLog.ee("FlightRecorder", t, "Flight recorder disabled")
        } catch (_: Throwable) {
            // Logging must stay best-effort on host and robot.
        }
        try {
            writer.close()
        } catch (_: Throwable) {
        }
    }

    companion object {
        private const val MAX_LOG_FILES = 30
        private const val FLUSH_INTERVAL_NS = 1_000_000_000L

        fun open(
            opModeClassName: String,
            gamepad1: () -> GamepadEx?,
            gamepad2: () -> GamepadEx?,
            batteryVoltage: () -> Double?,
            runningCommandNames: () -> List<String>,
            directory: File = File("/sdcard/FIRST/logs"),
        ): FlightRecorder? = try {
            directory.mkdirs()
            prune(directory)
            val stamp = SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(Date())
            val file = File(directory, "$opModeClassName-$stamp.wpilog")
            FlightRecorder(
                WpiLogWriter(BufferedOutputStream(FileOutputStream(file)), "ftc-starter"),
                gamepad1,
                gamepad2,
                batteryVoltage,
                runningCommandNames,
            ).also { it.event("init $opModeClassName") }
        } catch (t: Throwable) {
            try {
                RobotLog.ee("FlightRecorder", t, "Failed to open flight recorder")
            } catch (_: Throwable) {
            }
            null
        }

        private fun prune(directory: File) {
            val logs = directory.listFiles { file -> file.extension == "wpilog" }
                ?.sortedBy { it.lastModified() }
                ?: return
            val excess = logs.size - MAX_LOG_FILES + 1
            if (excess <= 0) return
            logs.take(excess).forEach { it.delete() }
        }
    }
}
