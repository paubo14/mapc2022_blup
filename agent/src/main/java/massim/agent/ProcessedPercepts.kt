package massim.agent

import massim.agent.agents.Agent
import massim.common.Bounds
import massim.common.Position
import kotlin.math.abs

// This file contains some helper classes to store information, e.g. about a role or a goal zone

data class Role(
  val vision: Int, val action: List<String>, val speed: List<Int>, val clearChance: Double, val clearMaxDistance: Int
)

data class GoalZone(
  val id: Int,
  val positions: MutableMap<Position, Int>,
  val agents: MutableSet<String>,
  val groupOfAgents: Int,
  var explored: Boolean,
  val missingPositions: MutableSet<Position>,
  val nearestDispenser: MutableMap<String, Pair<Position, Int>>,
  var task: Task?,
  var taskName: String?,
  var constructorAgent: Agent?,
  var constructorPos: Position?,
  val minMaxOfDispenserType: MutableMap<String, MutableMap<String, Pair<Int, Int>>>,
  val agentsOfDispenserType: MutableMap<String, MutableMap<String, MutableSet<Agent>>>,
  var overallMinAgents: Int?,
  // Store the number of the last consecutive steps in which an opponent has blocked a constructor cell
  var opponents: MutableMap<Position, Int>
) {

  // Called when the agents are removed from the goal zone
  fun clearBecauseOfLostAgents() {
    agents.clear()
    agentsOfDispenserType.clear()
    constructorAgent = null
    constructorPos = null
    opponents.clear()
  }

  // Get the tube between the constructor position and the dispenser of
  // the given type (that is closest to the goal zone)
  fun getTube(blockType: String, clearMaxDistance: Int, bounds: Bounds): MutableSet<Position> {
    val p0 = nearestDispenser.getValue(blockType).first
    val p1 = constructorPos!!
    val widthOfTube = clearMaxDistance
    val tubeSet = mutableSetOf<Position>()
    // Bresenham line algorithm, based on
    // https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    var (x0, y0) = p0
    val (x1, y1) = p0.closestVariantOf(p1, bounds)
    val dx = abs(x1 - x0)
    val sx = if (x0 < x1) 1 else -1
    val dy = -abs(y1 - y0)
    val sy = if (y0 < y1) 1 else -1
    var error = dx + dy
    while (true) {
      tubeSet.addAll(Position(x0, y0).neighboursAtMost(widthOfTube / 2).map { it.intoBounds(bounds) })
      if (x0 == x1 && y0 == y1) break
      val e2 = 2 * error
      if (e2 >= dy) {
        if (x0 == x1) break
        error += dy
        x0 += sx
      }
      if (e2 <= dx) {
        if (y0 == y1) break
        error += dx
        y0 += sy
      }
    }
    return tubeSet
  }
}

sealed class PositionInfo {
  companion object {
    fun make(type: String, details: String) = when (type) {
      "obstacle" -> {
        assert(details.isEmpty())
        Obstacle
      }

      "block" -> Block(details)
      "dispenser" -> Dispenser(details)
      "entity" -> Entity(details)
      else -> Other(type, details)
    }
  }

  object Empty : PositionInfo() {
    override fun toString() = "Empty"
  }

  object Obstacle : PositionInfo() {
    override fun toString() = "Obstacle"
  }

  data class Block(val block_type: String) : PositionInfo()
  data class Dispenser(val block_type: String) : PositionInfo()
  data class Agent(val name: String) : PositionInfo()
  data class Entity(val team: String) : PositionInfo()
  data class Other(val type: String, val details: String) : PositionInfo()
}

data class PositionInfoWithStep(val info: PositionInfo, val lastStep: Int)

data class Task(val deadline: Int, val reward: Int, val req: Map<Position, String>)

data class Requirement(val type: String, val name: String, val quantity: Int, val details: String)

data class Norm(val start: Int, val end: Int, val requirements: List<Requirement>, val fine: Int)

data class SurveyedAgent(val name: String, val role: String, val energy: Int)

data class SurveyedThing(val thingName: String, val distance: Int)

data class ProcessedPercepts(
  val step: Int,
  val actionID: Int,
  val timestamp: Int,
  val deadline: Int,
  val lastAction: String,
  val lastActionResult: String,
  val lastActionParams: List<String>,
  val score: Int,
  val things: Map<Position, PositionInfo>,
  val attachedThings: List<Position>,
  val energy: Int,
  val deactivated: Boolean,
  val role: String,
  val roleZones: Set<Position>,
  val goalZones: Set<Position>,
  val violations: List<String>,
  val norms: Map<String, Norm>,
  val surveyedAgent: SurveyedAgent?,
  val surveyedThing: SurveyedThing?,
  val hit: Position?
)
