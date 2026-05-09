package me.anno.traffic

import org.joml.AABBf
import org.joml.Quaternionf
import org.joml.Vector3d

/**
 * find route from one parking spot to another...
 * */
class ParkingSpot {
    val position = Vector3d()
    val rotation = Quaternionf()
    val localBounds = AABBf()
}