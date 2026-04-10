package me.anno.traffic

import org.joml.Vector3d

data class LanePoint(val pos: Vector3d, val radius: Double) {
    val lanes = ArrayList<Lane>()
}