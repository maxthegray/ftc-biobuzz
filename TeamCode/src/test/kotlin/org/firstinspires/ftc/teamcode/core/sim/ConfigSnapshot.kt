package org.firstinspires.ftc.teamcode.core.sim

import java.lang.reflect.Modifier

/** Saves a config object's tunable fields so a test can put them back after changing them. */
class ConfigSnapshot(private val config: Any) {
    private val saved = config.javaClass.declaredFields
        .filter { Modifier.isPublic(it.modifiers) && !Modifier.isFinal(it.modifiers) }
        .associateWith { it.get(config) }

    fun restore() {
        for ((field, value) in saved) field.set(config, value)
    }
}
