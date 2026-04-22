package me.anno.traffic

import me.anno.Time
import org.joml.Vector3d

class Crossing(val center: Vector3d, val radius: Double) {

    val sections = ArrayList<CrossingSection>()

    fun mayDrive(from: Int, to: Int): Boolean {
        // todo how can we implement right-before left?
        val sectionCanDrive = 1 + (Time.gameTime / 5.0).toInt() % (sections.size - 1)
        return from == sectionCanDrive
    }
}