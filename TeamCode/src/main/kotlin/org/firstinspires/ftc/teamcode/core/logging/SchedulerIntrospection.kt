package org.firstinspires.ftc.teamcode.core.logging

import com.pedropathing.ivy.Scheduler
import com.qualcomm.robotcore.util.RobotLog
import java.lang.reflect.Field

/**
 * Reflective read-only view of Ivy scheduler state for logging.
 *
 * If Ivy internals change, this disables itself after the first failure and
 * returns empty values forever after. Logging must never affect scheduling.
 */
class SchedulerIntrospection(
    private val schedulerClass: Class<*> = Scheduler::class.java,
) {
    private var disabled = false
    private var loggedFailure = false
    private var runningCommandsField: Field? = null
    private var activeRequirementsField: Field? = null

    fun runningCommandNames(): List<String> {
        if (disabled) return emptyList()
        return try {
            val running = runningCommands() ?: return emptyList()
            running.map { it.toString() }
        } catch (t: Throwable) {
            disable(t)
            emptyList()
        }
    }

    fun activeRequirementCount(): Int {
        if (disabled) return 0
        return try {
            val field = activeRequirementsField ?: field("activeRequirements").also {
                activeRequirementsField = it
            }
            (field.get(null) as? Map<*, *>)?.size ?: 0
        } catch (t: Throwable) {
            disable(t)
            0
        }
    }

    private fun runningCommands(): Collection<*>? {
        val field = runningCommandsField ?: field("runningCommands").also {
            runningCommandsField = it
        }
        return field.get(null) as? Collection<*>
    }

    private fun field(name: String): Field =
        schedulerClass.getDeclaredField(name).also { it.isAccessible = true }

    private fun disable(t: Throwable) {
        disabled = true
        if (!loggedFailure) {
            loggedFailure = true
            try {
                RobotLog.ee("SchedulerIntrospection", t, "Failed to inspect Ivy scheduler")
            } catch (_: Throwable) {
                // Host-side tests stub Android logging; ignore secondary failures.
            }
        }
    }

    companion object {
        val DEFAULT = SchedulerIntrospection()
    }
}
