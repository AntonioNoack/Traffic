package me.anno.traffic

import me.anno.Time

class Crossing {

    val sections = ArrayList<CrossingSection>()

    fun mayDrive(from: Int, to: Int): Boolean {
        // todo how can we implement right-before left?
        val sectionCanDrive = (Time.gameTime / 5.0).toInt() % sections.size
        return from == sectionCanDrive
    }
}