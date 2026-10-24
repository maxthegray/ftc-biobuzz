package org.firstinspires.ftc.teamcode.core.logging

import java.lang.reflect.InvocationTargetException
import java.lang.reflect.Method
import java.lang.reflect.Modifier
import java.util.concurrent.ConcurrentHashMap

/** Cached scalar state discovery for the default SubsystemBase logger. */
internal object AutomaticStateLog {
    fun record(state: Any, log: StateLog) {
        for ((getter, channel) in stateGetters.getOrPut(state.javaClass) { discoverStateGetters(state.javaClass) }) {
            val value = try {
                getter.invoke(state)
            } catch (e: InvocationTargetException) {
                val cause = e.cause
                if (cause is Error) throw cause
                continue
            } catch (_: Exception) {
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

    private val stateGetters = ConcurrentHashMap<Class<*>, List<Pair<Method, String>>>()
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
            if (!Modifier.isPublic(getter.modifiers) || Modifier.isStatic(getter.modifiers) ||
                getter.isSynthetic || getter.parameterCount != 0 ||
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
