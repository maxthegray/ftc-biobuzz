package org.firstinspires.ftc.teamcode.core.logging

import com.qualcomm.robotcore.util.RobotLog
import java.lang.reflect.InvocationTargetException
import java.lang.reflect.Method
import java.lang.reflect.Modifier
import java.util.WeakHashMap
import java.util.concurrent.ConcurrentHashMap
import org.firstinspires.ftc.teamcode.core.SubsystemBase

/**
 * Cached scalar state discovery for the default SubsystemBase logger. A getter
 * that throws an Exception is dropped for that subsystem and recorded once; an
 * Error escapes to the flight recorder, which disables the subsystem's channels.
 */
internal object AutomaticStateLog {
    fun record(subsystem: SubsystemBase, log: StateLog) {
        val dropped = droppedGetters[subsystem]
        for ((getter, channel) in stateGetters.getOrPut(subsystem.javaClass) { discoverStateGetters(subsystem.javaClass) }) {
            if (dropped != null && getter in dropped) continue
            val value = try {
                getter.invoke(subsystem)
            } catch (e: Exception) {
                val cause = (e as? InvocationTargetException)?.cause ?: e
                if (cause is Error) throw cause
                drop(subsystem, getter, channel, cause)
                continue
            }
            when (value) {
                is Double -> log.put(channel, value)
                is Float -> log.put(channel, value.toDouble())
                is Int -> log.put(channel, value.toLong())
                is Long -> log.put(channel, value)
                is Boolean -> log.put(channel, value)
                is String -> log.put(channel, value)
                is Enum<*> -> log.put(channel, value.name)
            }
        }
    }

    private fun drop(subsystem: SubsystemBase, getter: Method, channel: String, cause: Throwable) {
        droppedGetters.getOrPut(subsystem) { HashSet() } += getter
        val message = "SUBSYSTEM LOGGING DISABLED: ${subsystem.name}/$channel: ${cause.javaClass.simpleName}: ${cause.message}"
        RobotLog.ee("AutomaticStateLog", cause, message)
        Events.record(message)
    }

    private val stateGetters = ConcurrentHashMap<Class<*>, List<Pair<Method, String>>>()

    // Per instance, so the next op-mode's subsystem tries the getter again.
    private val droppedGetters = WeakHashMap<SubsystemBase, MutableSet<Method>>()

    private val stateTypes = setOf(
        java.lang.Double.TYPE, java.lang.Double::class.java,
        java.lang.Float.TYPE, java.lang.Float::class.java,
        java.lang.Integer.TYPE, java.lang.Integer::class.java,
        java.lang.Long.TYPE, java.lang.Long::class.java,
        java.lang.Boolean.TYPE, java.lang.Boolean::class.java,
        String::class.java,
    )

    private fun discoverStateGetters(type: Class<*>): List<Pair<Method, String>> =
        type.declaredMethods.mapNotNull { getter ->
            // '$': Kotlin `internal` members compile to public methods with a
            // build-variant suffix (getFoo$TeamCode_debug).
            if (!Modifier.isPublic(getter.modifiers) || Modifier.isStatic(getter.modifiers) ||
                getter.isSynthetic || '$' in getter.name || getter.parameterCount != 0 ||
                (getter.returnType !in stateTypes && !getter.returnType.isEnum)
            ) return@mapNotNull null
            val suffix = when {
                getter.name.startsWith("get") -> getter.name.drop(3)
                getter.name.startsWith("is") -> getter.name.drop(2)
                else -> return@mapNotNull null
            }
            if (suffix.isEmpty() || !suffix.first().isUpperCase()) return@mapNotNull null
            if (type.methods.any { it.name == "set$suffix" && it.parameterCount == 1 }) return@mapNotNull null
            val channel = suffix.replaceFirstChar { it.lowercaseChar() }
            getter to channel
        }.sortedBy { it.second }
}
