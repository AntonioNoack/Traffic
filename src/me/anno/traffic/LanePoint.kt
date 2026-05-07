package me.anno.traffic

import org.joml.Quaternionf
import org.joml.Vector3d

data class LanePoint(
    val position: Vector3d,
    val rotation: Quaternionf,
    val angle: Double,
    val radius: Double
) {
    lateinit var streetPoint: StreetPoint
    val lanes = ArrayList<Lane>()
}