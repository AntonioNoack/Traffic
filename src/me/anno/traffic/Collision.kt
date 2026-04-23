package me.anno.traffic

import org.joml.Vector3d
import org.joml.Vector3f

data class Collision(val mtvAxis: Vector3d, val minOverlap: Double, val relCenter: Vector3f)
    