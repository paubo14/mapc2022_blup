package massim.common

import kotlinx.serialization.Serializable
import kotlin.math.abs
import kotlin.math.min
import kotlin.math.roundToInt

@Serializable
data class Bounds(val width: Int?, val height: Int?)

@Serializable
data class Position(val x: Int, val y: Int) {
  operator fun plus(other: Position) = Position(x + other.x, y + other.y)
  operator fun minus(other: Position) = Position(x - other.x, y - other.y)
  operator fun times(factor: Double) = Position((x * factor).roundToInt(), (y * factor).roundToInt())

  val isZero
    get() = x == 0 && y == 0
  val norm
    get() = abs(x) + abs(y)

  fun neighboursAtMost(maxDistance: Int) = (-maxDistance..maxDistance).asSequence().flatMap { yOff ->
    val xAmount = maxDistance - abs(yOff)
    (-xAmount..xAmount).asSequence().map { xOff -> this + Position(xOff, yOff) }
  }

  fun neighboursLess(distance: Int) = neighboursAtMost(distance - 1)

  fun neighboursExactly(distance: Int) = (-distance..distance).asSequence().flatMap { yOff ->
    val xAmount = distance - abs(yOff)
    val seq = if (xAmount == 0) sequenceOf(0) else sequenceOf(-xAmount, xAmount)
    seq.map { xOff -> this + Position(xOff, yOff) }
  }

  // The closest variant of other to the current instance (with respect to the bounds)
  fun closestVariantOf(other: Position, bounds: Bounds): Position {
    val impl = { val1: Int, val2: Int, dim: Int? ->
      if (dim != null) {
        sequenceOf(val2 - dim, val2, val2 + dim).minBy { abs(val1 - it) }
      } else {
        val2
      }
    }
    return Position(impl(this.x, other.x, bounds.width), impl(this.y, other.y, bounds.height))
  }

  fun interpolate(otherPos: Position, arg: Int, max: Int, bounds: Bounds): Position {
    val interpolated = this + (closestVariantOf(otherPos, bounds) - this) * ((arg + 0.5) / max)
    return interpolated.intoBounds(bounds)
  }

  // Transform the instance into the bounds
  fun intoBounds(bounds: Bounds): Position {
    val axisOp = { value: Int, bound: Int? -> if (bound != null) Math.floorMod(value, bound) else value }
    return Position(axisOp(x, bounds.width), axisOp(y, bounds.height))
  }

  override fun toString() = "($x, $y)"
  operator fun unaryMinus() = Position(-x, -y)
}

// The smallest distance between the two positions with respect to the bounds
fun distanceBounded(p1: Position, p2: Position, bounds: Bounds): Int {
  val axisOp = { val1: Int, val2: Int, dim: Int? ->
    if (dim != null) {
      min(abs(val1 - val2), min(abs(val1 + dim - val2), abs(val1 - dim - val2)))
    } else {
      abs(val1 - val2)
    }
  }
  return axisOp(p1.x, p2.x, bounds.width) + axisOp(p1.y, p2.y, bounds.height)
}

// The smallest difference between p1 and p2
fun subClosest(p1: Position, p2:Position, bounds: Bounds) = p1 - p1.closestVariantOf(p2, bounds)
