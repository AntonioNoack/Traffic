package me.anno.traffic.utils

fun <V> ArrayList<V>.addUnique(v: V): Boolean {
    if (v in this) return false
    return add(v)
}
