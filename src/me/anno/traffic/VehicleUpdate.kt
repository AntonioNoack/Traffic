package me.anno.traffic

fun List<Vehicle>.update(dt: Float) {
    if (!(dt > 0f)) return

    for (i in indices) this[i].update0(dt)
    for (i in indices) this[i].update1(dt)

    // todo if(size>32) build a tree to resolve the collisions faster (~10x speedup?)
    for (i in indices) this[i].update2()
}


fun Vehicle.updateS(dt: Float) {
    if (!(dt > 0f)) return

    update0(dt)
    update1(dt)
    update2()
}