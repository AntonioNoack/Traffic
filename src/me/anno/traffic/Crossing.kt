package me.anno.traffic

import me.anno.Time
import org.joml.Vector3d

open class Crossing(val center: Vector3d, val radius: Double) {

    val sections = ArrayList<CrossingSection>()

    open fun mayDrive(from: Int, to: Int): Boolean {
        // todo how can we implement right-before left?
        // section0 = center, then take turns allowing driving on a lane, and stopping
        // todo we don't really need 'to'
        val sectionCanDrive = (Time.gameTime / 5.0).toInt() % ((sections.size - 1) * 2)
        return (from - 1) * 2 == sectionCanDrive
    }
}