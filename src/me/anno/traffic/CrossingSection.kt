package me.anno.traffic

data class CrossingSection(val crossing: Crossing, val sectionId: Int) {
    fun mayStopOnSection() = crossing.isRealSection(sectionId)
}