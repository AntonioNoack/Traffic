package me.anno.traffic

import org.joml.Vector3f

data class Collision(val mtvAxis: Vector3f, val minOverlap: Float, val relCenter: Vector3f)
    