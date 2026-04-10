package me.anno.traffic

data class Lane(val from: LanePoint, val control: LanePoint, val to: LanePoint) {
    var currSection: CrossingSection? = null
    val vehicles = ArrayList<Vehicle>()

    fun mayEnterNextLane(nextLane: Lane): Boolean {
        val curr = currSection
        val next = nextLane.currSection
        if (curr == null || next == null) return true
        if (curr.crossing != next.crossing) return true
        return curr.crossing.mayDrive(curr.sectionId, next.sectionId)
    }
}