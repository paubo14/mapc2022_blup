package massim.agent.agents

import cpsat.tasking.WorkerStatus
import massim.agent.*
import massim.common.*
import java.io.PrintWriter

// Possible phases of an agent
enum class Phase {
  SEARCH_ROLE_ZONE, GO_TO_ROLE_ZONE, GET_DESIRED_ROLE, SEARCH_MAP, GO_TO_GOAL_ZONE, WORK_ON_GOAL_ZONE, DETACH_THINGS,
  RESTART_DETACH_NORTH, RESTART_DETACH_EAST, RESTART_DETACH_SOUTH, RESTART_DETACH_WEST
}

class Agent(val name: String) {
  var failedRandomly = false
  var lastAction: String? = null
  var currRole: String? = null
  var phase = Phase.SEARCH_ROLE_ZONE
  var team: Int? = null
  var group: Int? = null

  // The position of the agent in the coordinate system of the agent's group
  var offset = Position(0, 0)

  // The attached things of the agent in the local coordinate system of the agent
  var attachedThings = mutableSetOf<Position>()

  // The agents are only allowed to carry blocks of a single type
  var attachedThingsType: String? = null

  // The position of the block which will be attached to the constructor if the current connect action
  // will be successful
  var newAttachedThingAfterConnect: Position? = null

  // Blocks adjacent to the agent → the blocks attached to the adjacent block
  var attachedThingsToAttached = mutableMapOf<Position, MutableSet<Position>>()

  // The role the agent wants to have
  var desiredRole: String? = null

  var goalZone: GoalZone? = null
  var isConstructor = false

  // The index of the digger, which is used to interpolate between the dispenser and constructor position
  // to determine the digger's flock
  var diggerNum: Int? = null

  // The dispenser the agent has to use
  var dispenserPos: Position? = null

  // The type of the blocks the agent has to gather/deliver
  var blockType: String? = null

  // The status (deliverer or gatherer or null)
  var workerStatus: WorkerStatus? = null

  // Last successful action
  // This is stored so that a deliverer detaches the block it has connected to the constructor (if not done already)
  // if the constructor position changes
  var lastSuccessfulAction: String? = null

  // Called before the next round starts
  fun clear() {
    phase = Phase.SEARCH_ROLE_ZONE
    team = null
    offset = Position(0, 0)
    group = null
    attachedThings.clear()
    attachedThingsType = null
    desiredRole = null
    isConstructor = false
    newAttachedThingAfterConnect = null
    attachedThingsToAttached.clear()
    dispenserPos = null
    goalZone = null
    diggerNum = null
    blockType = null
    workerStatus = null
    lastSuccessfulAction = null
  }

  // Called from clearRemovedGoalZone
  fun clear(newPhase: Phase) {
    team = null
    phase = newPhase
    isConstructor = false
    desiredRole = null
    dispenserPos = null
    goalZone = null
    diggerNum = null
    blockType = null
    workerStatus = null
  }

  // Called if the agent is removed from a unit
  fun clearRemovedGoalZone() {
    clear(if (attachedThings.isEmpty()) Phase.SEARCH_MAP else Phase.DETACH_THINGS)
  }

  override fun toString() =
    "Agent(name=$name, team=$team, offset=$offset, group=$group, phase=$phase, currRole=$currRole, desiredRole=$desiredRole)"

  // Returns the flock of a digger and should only be called if the agent is a digger
  fun getFlock(clearMaxDistance: Int, bounds: Bounds): Set<Position> {
    val roleThatMatters = desiredRole ?: currRole!!
    val goalZone = goalZone!!
    val constructorPos = goalZone.constructorPos!!
    val dispenserPos = dispenserPos!!
    val blockType = blockType!!

    // Get the tube between the dispenser and the goal zone
    val tubeOfDigger = goalZone.getTube(blockType, clearMaxDistance, bounds)
    // Get the centre of the flock by interpolating
    val diggerCenter = constructorPos.interpolate(
      dispenserPos,
      diggerNum!!,
      goalZone.agentsOfDispenserType.getValue(blockType).getValue(roleThatMatters).size,
      bounds
    )
    val diameter = distanceBounded(
      constructorPos, dispenserPos, bounds
    ) divCeil goalZone.agentsOfDispenserType.getValue(blockType).getValue("digger").size
    // The flock contains all cells in the tube which have at most the specified distance from the centre
    return diggerCenter.neighboursAtMost((diameter + clearMaxDistance) / 2).map { it.intoBounds(bounds) }
      .filter { it in tubeOfDigger }.toSet()
  }

  // Returns the target position of the agent with respect to its job at the goal zone.
  // Digger should use getFlock instead!
  // Be careful: The returned position can be the same as the position of the agent!
  fun getTargetPos(
    currRole: String, map: MutableMap<Position, PositionInfoWithStep>, agentsOfGroup: List<Agent>, bounds: Bounds
  ): Position? {
    // Returns true iff another agent of the team, a block attached to an agent
    // of the group or an entity is on the position
    val occupiedPos = { pos: Position ->
      when (val info = map[pos]?.info) {
        is PositionInfo.Agent -> info.name != name
        is PositionInfo.Block -> {
          agentsOfGroup.any { agent -> agent.attachedThings.any { (it + agent.offset).intoBounds(bounds) == pos } }
        }

        is PositionInfo.Entity -> true
        else -> false
      }
    }
    val role = if (desiredRole != null) desiredRole else currRole
    val goalZone = goalZone
    if (goalZone != null && isConstructor) {
      // The agent is the constructor → go to the constructor position
      return goalZone.constructorPos
    } else if (goalZone != null && role == "worker") {
      val dispenserPos = dispenserPos!!
      // Do not go onto the dispenser!
      val firstTargetPos = dispenserPos.neighboursExactly(1).map { it.intoBounds(bounds) }.minBy {
        distanceBounded(it, offset, bounds)
      }
      // This is a hack :(
      if (occupiedPos(firstTargetPos)) {
        val possibleNewTargetPos = dispenserPos.neighboursExactly(2).map {
          it.intoBounds(bounds)
        }.filter { !occupiedPos(it) }
        if (possibleNewTargetPos.isNotEmpty()) {
          return possibleNewTargetPos.minBy { distanceBounded(it, firstTargetPos, bounds) }
        }
      }
      return firstTargetPos
    }
    return null
  }

  fun updatePhase(
    map: MutableMap<Position, PositionInfoWithStep>,
    roleZones: MutableSet<Position>,
    dispensers: MutableMap<String, MutableSet<Position>>,
    agentPerceptsProcessed: ProcessedPercepts,
    agentsOfGroup: List<Agent>,
    roles: MutableMap<String, Role>,
    bounds: Bounds,
    log: PrintWriter,
  ) {
    val currRole = agentPerceptsProcessed.role
    val timer = Timer()
    log.printFlush { "Phase: $phase, currRole: $currRole, Agent: $this" }
    when (phase) {
      // TODO This does not work correctly.
      // This was supposed to be a bug fix when the program is restarted during a simulation.
      // The goal was that the agent detaches in all directions one after the other to make sure that it does
      // not carry anything
      Phase.RESTART_DETACH_NORTH -> {
        if (!failedRandomly && lastAction != null && lastAction == "detach") phase = Phase.RESTART_DETACH_EAST
      }

      Phase.RESTART_DETACH_EAST -> {
        if (!failedRandomly) phase = Phase.RESTART_DETACH_SOUTH
      }

      Phase.RESTART_DETACH_SOUTH -> {
        if (!failedRandomly) phase = Phase.RESTART_DETACH_WEST
      }

      Phase.RESTART_DETACH_WEST -> {
        if (!failedRandomly) phase = Phase.SEARCH_ROLE_ZONE
        updatePhase(map, roleZones, dispensers, agentPerceptsProcessed, agentsOfGroup, roles, bounds, log)
      }

      Phase.SEARCH_ROLE_ZONE -> {
        if (roleZones.isNotEmpty()) {
          phase = Phase.GO_TO_ROLE_ZONE
          if (desiredRole == null) {
            desiredRole = "explorer"
          }
          updatePhase(map, roleZones, dispensers, agentPerceptsProcessed, agentsOfGroup, roles, bounds, log)
        }
      }

      Phase.GO_TO_ROLE_ZONE -> {
        if (agentPerceptsProcessed.role == desiredRole || offset in roleZones) {
          // The agent reached a role zone
          phase = Phase.GET_DESIRED_ROLE
          updatePhase(map, roleZones, dispensers, agentPerceptsProcessed, agentsOfGroup, roles, bounds, log)
        }
      }

      Phase.GET_DESIRED_ROLE -> {
        log.printFlush { "goalZone: $goalZone, desiredRole: $desiredRole, currRole: $currRole" }
        if (agentPerceptsProcessed.role == desiredRole) {
          // The agent already has the desired role
          desiredRole = null
          phase = if (team == null) Phase.SEARCH_MAP else Phase.GO_TO_GOAL_ZONE
          if (phase == Phase.GO_TO_GOAL_ZONE) {
            updatePhase(map, roleZones, dispensers, agentPerceptsProcessed, agentsOfGroup, roles, bounds, log)
          }
        }
      }

      Phase.GO_TO_GOAL_ZONE -> {
        log.printFlush { "GO_TO_GOAL_ZONE: currRole=$currRole, offset=$offset, goalZone: $goalZone" }
        if (!isConstructor && currRole == "digger") {
          // A digger wants to go to its flock
          if (offset in getFlock(roles.getValue(currRole).clearMaxDistance, bounds)) {
            phase = Phase.WORK_ON_GOAL_ZONE
          }
        } else if (isConstructor) {
          val targetPos = getTargetPos(currRole, map, agentsOfGroup, bounds)!!
          if (offset == targetPos) {
            phase = Phase.WORK_ON_GOAL_ZONE
          }
        } else if (currRole == "worker") {
          val targetPos = getTargetPos(currRole, map, agentsOfGroup, bounds)!!
          // If a worker has a distance of at most 4 from its target position, its phase is changed,
          // so that the worker becomes part of the optimization problem
          if (distanceBounded(targetPos, offset, bounds) < 5) {
            phase = Phase.WORK_ON_GOAL_ZONE
          }
        }
      }

      Phase.DETACH_THINGS -> {
        if (attachedThings.isEmpty()) {
          attachedThingsType = null
          phase = if (team != null) {
            if (desiredRole != null) Phase.GO_TO_ROLE_ZONE else Phase.GO_TO_GOAL_ZONE
          } else {
            if (roleZones.isEmpty()) Phase.SEARCH_ROLE_ZONE else Phase.SEARCH_MAP
          }
          updatePhase(map, roleZones, dispensers, agentPerceptsProcessed, agentsOfGroup, roles, bounds, log)
        }
      }

      Phase.SEARCH_MAP -> {
        // An agent is only allowed to search the map when it carries nothing
        if (attachedThings.isNotEmpty()) {
          phase = Phase.DETACH_THINGS
        }
      }

      else -> {}
    }
    timer.printReset(
      log, "Time in updatePhase for: goalZone: $goalZone, desiredRole: $desiredRole, currRole: $currRole, Agent: $this"
    )
  }
}
