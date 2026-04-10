package me.anno.traffic

class CrossingSection(val crossing: Crossing, val sectionId: Int) {
    val lanes = ArrayList<Lane>()
}