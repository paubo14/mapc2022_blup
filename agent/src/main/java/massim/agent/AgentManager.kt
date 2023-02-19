package massim.agent

import cpsat.tasking.Rotation
import eis.exceptions.ActException
import eis.iilang.*
import eis.iilang.Function
import massim.agent.agents.Agent
import massim.agent.agents.Phase
import massim.agent.agents.optimizedExplore
import massim.agent.agents.optimizedTasking
import massim.common.*
import massim.common.Timer
import massim.eismassim.EnvironmentInterface
import massim.eismassim.Log
import org.jgrapht.alg.shortestpath.DijkstraShortestPath
import org.jgrapht.graph.DefaultEdge
import org.jgrapht.graph.DefaultWeightedEdge
import org.jgrapht.graph.SimpleDirectedGraph
import org.jgrapht.graph.SimpleDirectedWeightedGraph
import java.io.File
import java.util.*
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import kotlin.system.measureTimeMillis

fun <T> getList(pList: Parameter, getVal: (Parameter) -> T) = (pList as ParameterList).map(getVal)

// Get the parameter for moving onto an adjacent cell
fun Position.toParam(): Parameter? = when {
  x == 0 && y == 0 -> null
  x == 0 && y == -1 -> Identifier("n")
  x == 0 && y == 1 -> Identifier("s")
  x == -1 && y == 0 -> Identifier("w")
  x == 1 && y == 0 -> Identifier("e")
  else -> throw Exception("Invalid position $this")
}

// Rotate a position
fun Position.rotate(rot: Rotation) = when (rot) {
  Rotation.ANTICLOCKWISE -> Position(y, -x)
  Rotation.CLOCKWISE -> Position(-y, x)
}

// Get the offset of a given direction
fun posFromDir(dir: String): Position {
  var xDiff = 0
  var yDiff = 0
  when (dir) {
    "n" -> --yDiff
    "s" -> ++yDiff
    "e" -> ++xDiff
    "w" -> --xDiff
  }
  return Position(xDiff, yDiff)
}

class AgentManager(private val eis: EnvironmentInterface, val team: String) {
  var teamSize: Int = 0
    private set
  private var steps: Int = 0
  var roles = mutableMapOf<String, Role>()
    private set
  private var nextIndexForGroup = 0
  private var gotStep = false
  private var ranking: Int? = null

  // Careful, more agents than necessary are created at the beginning!
  val agents = mutableMapOf<String, Agent>()
  private val agentPercepts = mutableMapOf<String, MutableSet<Percept>>()
  val agentPerceptsProcessed = mutableMapOf<String, ProcessedPercepts>()
  private val log = File("log/log_$team.txt").printWriter()

  // Counts how many agents received percepts during the current step
  private var agentsWithPercepts = 0

  // These are reset for the next round
  val agentsOfGroup = mutableMapOf<Int, MutableList<Agent>>()
  val mapOfGroup = mutableMapOf<Int, MutableMap<Position, PositionInfoWithStep>>()
  private val rolePositionsOfGroup = mutableMapOf<Int, MutableSet<Position>>()
  val markerPosOfGroup = mutableMapOf<Int, MutableMap<Position, Pair<String, Int>>>()

  // The goal zones (consisting of "goal zone positions") of each group
  val goalZonesOfGroup = mutableMapOf<Int, MutableMap<Int, GoalZone>>()

  // Stores the ID of the next goal zone for each group
  private val goalZoneCounter = mutableMapOf<Int, Int>()

  // group → goal zone positions which are not grouped to a goal zone yet → the last time step in which it was seen
  private val goalZonePositionsOfGroup = mutableMapOf<Int, MutableMap<Position, Int>>()

  private var score: Int? = null

  // group → all goal zone positions of the group
  val allGoalZonePositions = mutableMapOf<Int, MutableMap<Position, Pair<Int?, Int>>>()

  // group → block type → set of positions of dispensers of the block type
  val dispenserOfGroup = mutableMapOf<Int, MutableMap<String, MutableSet<Position>>>()

  // agent name → agents of the same team the agents currently sees
  private val mapAgentVisibleAgents = mutableMapOf<String, MutableList<Position>>()

  // The currently available tasks
  private val tasks = mutableMapOf<String, Task>()

  // The types of blocks that are needed for the current tasks
  private val possibleDispensers = mutableSetOf<String>()

  // The best known width of the world
  private var bestWidth: Int? = null

  // The best known height of the world
  private var bestHeight: Int? = null
  val bounds
    get() = Bounds(bestWidth, bestHeight)

  // Contains the number of blocks that can be carried with respect to the current norms
  var normBlock = mutableSetOf<Int>()

  fun setPercepts(agent: Agent, addList: List<Percept>, delList: List<Percept>) {
    // Measure the time needed for the setPerceptsImpl function
    val time = measureTimeMillis { setPerceptsImpl(agent, addList, delList) }
    val duration = time.toDouble() / 1000
    if (duration > 0.1) Log.log("Time: $duration s")
    log.printFlush { "setPerceptsImpl time: $duration s" }
  }

  private fun setPerceptsImpl(agent: Agent, addList: List<Percept>, delList: List<Percept>) {
    val agentName = agent.name
    val givenAgentPercepts = agentPercepts.getOrPut(agentName) { mutableSetOf() }
    givenAgentPercepts.removeAll(delList.toSet())
    givenAgentPercepts.addAll(addList)
    normBlock.clear()

    var actionID: Int? = null
    var timestamp: Int? = null
    var deadline: Int? = null
    var step: Int? = null
    var lastAction: String? = null
    var lastActionResult: String? = null
    var lastActionParams: List<String>? = null
    val things = mutableMapOf<Position, PositionInfo>()
    val foundDispenser = mutableMapOf<Position, PositionInfo.Dispenser>()
    val attachedThings = mutableListOf<Position>()
    var energy: Int? = null
    var deactivated: Boolean? = null
    var role: String? = null
    val roleZones = mutableSetOf<Position>()
    val goalZones = mutableSetOf<Position>()
    val violations = mutableListOf<String>()
    val norms = mutableMapOf<String, Norm>()
    var surveyedAgent: SurveyedAgent? = null
    var surveyedThing: SurveyedThing? = null
    var hit: Position? = null
    val localRoles = mutableMapOf<String, Role>()
    val currTasks = mutableMapOf<String, Task>()
    val currPosDispenser = mutableSetOf<String>()
    val currMarkers = mutableMapOf<Position, String>()

    log.printFlush { addList.map { it.name }.toString() }
    val firstPercept = addList.any { it.name == "simStart" }
    val lastPercept = addList.any { it.name == "simEnd" }

    val getString = { p: Percept, i: Int ->
      val param = p.parameters[i]
      (param as Identifier).value
    }
    val getInt = { p: Percept, i: Int ->
      val param = p.parameters[i]
      (param as Numeral).value.toInt()
    }
    val getDouble = { p: Percept, i: Int ->
      val param = p.parameters[i]
      (param as Numeral).value.toDouble()
    }
    // Resolve the given percepts for the current agent
    for (percept in givenAgentPercepts) {
      when (percept.name) {
        "teamSize" -> {
          teamSize = getInt(percept, 0)
        }

        "ranking" -> {
          ranking = getInt(percept, 0)
        }

        "steps" -> {
          steps = getInt(percept, 0)
        }

        "role" -> {
          if (percept.parameters.size == 1) {
            // The current role of the agent
            role = getString(percept, 0)
          } else {
            // One of the possible roles
            val rName = getString(percept, 0)
            val rVision = getInt(percept, 1)
            val rActions = getList(percept.parameters[2]) { (it as Identifier).value }
            val rSpeed = getList(percept.parameters[3]) { (it as Numeral).value.toInt() }
            val rClearChance = getDouble(percept, 4)
            val rClearMaxDistance = getInt(percept, 5)
            localRoles[rName] = Role(rVision, rActions, rSpeed, rClearChance, rClearMaxDistance)
          }
        }

        "actionID" -> {
          actionID = getInt(percept, 0)
        }

        "timestamp" -> {
          timestamp = getInt(percept, 0)
        }

        "deadline" -> {
          deadline = getInt(percept, 0)
        }

        "step" -> {
          step = getInt(percept, 0)
        }

        "lastAction" -> {
          lastAction = getString(percept, 0)
        }

        "lastActionResult" -> {
          lastActionResult = getString(percept, 0)
        }

        "lastActionParams" -> {
          lastActionParams = getList(percept.parameters[0]) { (it as Identifier).value }
        }

        "score" -> {
          score = getInt(percept, 0)
        }

        "thing" -> {
          val thingX = getInt(percept, 0)
          val thingY = getInt(percept, 1)
          val thingType = getString(percept, 2)
          val thingDetails = getString(percept, 3)
          val currPos = Position(thingX, thingY)
          if (thingType == "marker") {
            currMarkers[currPos] = thingDetails
          } else if (thingType == "dispenser") {
            // Store the found dispenser
            foundDispenser[currPos] = PositionInfo.Dispenser(thingDetails)
          } else {
            val currPosInfo = things[currPos]
            if (currPosInfo == null || ((currPosInfo is PositionInfo.Entity) && currPosInfo.team != team)) {
              // There can be an entity of my team and of the other team on the same cell at the beginning
              // → store that of my team (second condition)
              things[currPos] = PositionInfo.make(thingType, thingDetails)
            }
            // If an agent of my team (but not myself) can be seen, store it
            if (thingType == "entity" && thingDetails == team && (thingX != 0 || thingY != 0)) {
              mapAgentVisibleAgents.getOrPut(agentName) { mutableListOf() }.add(currPos)
            }
          }
        }

        "task" -> {
          val tName = getString(percept, 0)
          val tDeadline = getInt(percept, 1)
          val tReward = getInt(percept, 2)
          val tReqs = (percept.parameters[3] as ParameterList).associate {
            val f = it as Function
            val x = (f.parameters[0] as Numeral).value.toInt()
            val y = (f.parameters[1] as Numeral).value.toInt()
            val type = (f.parameters[2] as Identifier).value
            currPosDispenser.add(type)
            Position(x, y) to type
          }
          currTasks[tName] = Task(tDeadline, tReward, tReqs)
        }

        "attached" -> {
          val x = getInt(percept, 0)
          val y = getInt(percept, 1)
          attachedThings.add(Position(x, y))
        }

        "energy" -> {
          energy = getInt(percept, 0)
        }

        "deactivated" -> {
          deactivated = getString(percept, 0) == "true"
        }

        "roleZone" -> {
          val x = getInt(percept, 0)
          val y = getInt(percept, 1)
          roleZones.add(Position(x, y))
        }

        "goalZone" -> {
          val x = getInt(percept, 0)
          val y = getInt(percept, 1)
          goalZones.add(Position(x, y))
        }

        "violation" -> {
          violations.add(getString(percept, 0))
        }

        "norm" -> {
          val nId = getString(percept, 0)
          val nStart = getInt(percept, 1)
          val nEnd = getInt(percept, 2)
          val nReqs = (percept.parameters[3] as ParameterList).map {
            val f = it as Function
            val type = (f.parameters[0] as Identifier).value
            val rName = (f.parameters[1] as Identifier).value
            val quantity = (f.parameters[2] as Numeral).value.toInt()
            val details = (f.parameters[3] as Identifier).value
            if (type == "block" && rName == "any") {
              normBlock.add(quantity)
            }
            Requirement(type, rName, quantity, details)
          }
          val nFine = getInt(percept, 4)
          norms[nId] = Norm(nStart, nEnd, nReqs, nFine)
        }

        "surveyed" -> {
          val type = getString(percept, 0)
          if (type == "agent") {
            val sName = getString(percept, 1)
            val sRole = getString(percept, 2)
            val sEnergy = getInt(percept, 3)
            surveyedAgent = SurveyedAgent(sName, sRole, sEnergy)
          } else {
            val distance = getInt(percept, 1)
            surveyedThing = SurveyedThing(type, distance)
          }
        }

        "hit" -> {
          val hX = getInt(percept, 0)
          val hY = getInt(percept, 1)
          hit = Position(hX, hY)
        }
      }
    }
    log.printFlush { "teamSize: $teamSize, step: $step" }
    if (step == null) {
      log.printFlush { "$givenAgentPercepts" }
    }

    // Update tasks and possibleDispensers if corresponding information has been received
    if (currTasks.isNotEmpty()) {
      tasks.clear()
      tasks.putAll(currTasks)
      if (agent.name == "A1") {
        log.printFlush { "***** tasks: $tasks, currPosDispenser: $currPosDispenser" }
      }
      if (currPosDispenser.isNotEmpty()) {
        possibleDispensers.clear()
        possibleDispensers.addAll(currPosDispenser)
      }
    }

    if (role != null) {
      agent.currRole = role
    }

    if (lastPercept) {
      Log.log("ranking: $ranking")
      // Clear attributes
      teamSize = 0
      steps = 0
      roles = mutableMapOf()
      nextIndexForGroup = 0
      agentPercepts.clear()
      agentPerceptsProcessed.clear()
      agentsWithPercepts = 0
      agentsOfGroup.clear()
      mapOfGroup.clear()
      rolePositionsOfGroup.clear()
      markerPosOfGroup.clear()
      goalZonesOfGroup.clear()
      goalZoneCounter.clear()
      goalZonePositionsOfGroup.clear()
      allGoalZonePositions.clear()
      dispenserOfGroup.clear()
      mapAgentVisibleAgents.clear()
      tasks.clear()
      possibleDispensers.clear()
      bestWidth = null
      bestHeight = null
      // Clear the agents
      for (ag in agents.values) {
        ag.clear()
      }
      return
    }

    if (firstPercept || agent.group == null) {
      // Initialize attributes
      agent.group = nextIndexForGroup
      agentsOfGroup[nextIndexForGroup] = mutableListOf(agent)
      mapOfGroup[nextIndexForGroup] = mutableMapOf()
      rolePositionsOfGroup[nextIndexForGroup] = mutableSetOf()
      markerPosOfGroup[nextIndexForGroup] = mutableMapOf()
      goalZonesOfGroup[nextIndexForGroup] = mutableMapOf()
      goalZoneCounter[nextIndexForGroup] = 0
      goalZonePositionsOfGroup[nextIndexForGroup] = mutableMapOf()
      allGoalZonePositions[nextIndexForGroup] = mutableMapOf()
      dispenserOfGroup[nextIndexForGroup] = mutableMapOf()
      ++nextIndexForGroup
      roles = localRoles
    }

    // The first percept can be received without information about step 0
    if (step == null) {
      return
    }
    if (!gotStep) {
      // The program has been restarted
      gotStep = true
      agent.phase = Phase.RESTART_DETACH_NORTH
    }
    agent.failedRandomly = lastActionResult == "failed_random"
    agent.lastAction = lastAction
    log.printFlush { "ag: $agent.name, step: $step, norm: $norms, storedNorm: $normBlock" }
    if (lastActionResult == "success") {
      agent.lastSuccessfulAction = lastAction
    }

    val posFromDir = {
      var diff = Position(0, 0)
      for (dir in lastActionParams!!) {
        diff += posFromDir(dir)
      }
      diff
    }
    // Change the offset of the agent
    val group = agent.group
    if (lastActionResult == "success" && lastAction == "move") {
      agent.offset = (agent.offset + posFromDir()).intoBounds(bounds)
    } else if (lastActionResult == "partial_success" && lastAction == "move") {
      val offsetDiff = getOffsetDiff(
        agent,
        roles.getValue(role!!).vision,
        lastActionParams!!,
        group!!,
        things,
        goalZones,
        roleZones,
        foundDispenser
      )
      log.printFlush { "${agent.name} lastActionResult: $lastActionResult, lastAction: $lastAction, lastActionParams: $lastActionParams" }
      agent.offset = (agent.offset + offsetDiff).intoBounds(bounds)
    } else if (lastActionResult == "success" && lastAction == "attach") {
      log.printFlush { "${agent.name} lastActionResult: $lastActionResult, lastAction: $lastAction, lastActionParams: $lastActionParams" }
      var diff = Position(0, 0)
      for (dir in lastActionParams!!) {
        diff += posFromDir(dir)
      }
      // Store the attached thing
      agent.attachedThings.add(diff)
      agent.attachedThingsType = agent.blockType
    } else if (lastActionResult == "success" && lastAction == "detach") {
      var firstPosToRemove = Position(0, 0)
      for (dir in lastActionParams!!) {
        firstPosToRemove += posFromDir(dir)
      }
      // Remove the thing that was detached 
      agent.attachedThings.remove(firstPosToRemove)
      if (firstPosToRemove in agent.attachedThingsToAttached) {
        // Remove the things that were attached to the thing the agent detached from
        val keysToRemove = mutableSetOf(firstPosToRemove)
        var currPosSet = agent.attachedThingsToAttached.getValue(firstPosToRemove)
        while (currPosSet.isNotEmpty()) {
          val newPosSet = mutableSetOf<Position>()
          for (posR in currPosSet) {
            agent.attachedThings.remove(posR)
            agent.attachedThingsToAttached[posR]?.let {
              keysToRemove.add(posR)
              newPosSet.addAll(it)
            }
          }
          currPosSet = newPosSet
        }
        for (keyToRem in keysToRemove) {
          agent.attachedThingsToAttached.remove(keyToRem)
        }
      }
      if (agent.attachedThings.isEmpty()) {
        agent.attachedThingsType = null
      }
    } else if (lastActionResult == "success" && lastAction == "connect") {
      log.printFlush { "${agent.name} lastActionResult: $lastActionResult, lastAction: $lastAction, lastActionParams: $lastActionParams" }
      if (agent.isConstructor) {
        log.printFlush { "${agent.newAttachedThingAfterConnect}" }
        agent.attachedThings.add(agent.newAttachedThingAfterConnect!!)
        val oldAttachedThing = Position(lastActionParams!![1].toInt(), lastActionParams[2].toInt())
        // Store that the thing is now attached to a previously attached thing
        val attachedThingsForThisPos = agent.attachedThingsToAttached.getOrPut(oldAttachedThing) { mutableSetOf() }
        attachedThingsForThisPos.add(agent.newAttachedThingAfterConnect!!)
        agent.newAttachedThingAfterConnect = null
      } else {
        // A worker detaches from the block it delivered to the constructor in the next step
        // so that the blocks which are newly attached to the agent after connecting are ignored
        agent.attachedThings.add(Position(lastActionParams!![1].toInt(), lastActionParams[2].toInt()))
      }
      agent.attachedThingsType = agent.blockType
    } else if (lastActionResult == "success" && lastAction == "rotate") {
      log.printFlush { "${agent.name} lastActionResult: $lastActionResult, lastAction: $lastAction, lastActionParams: $lastActionParams" }
      // Update the positions in attachedThings and attachedThingsToAttached
      val newPosAttachedThings = if (lastActionParams!![0] == "cw") {
        agent.attachedThings.map { pos -> pos.rotate(Rotation.CLOCKWISE) }
      } else {
        agent.attachedThings.map { pos -> pos.rotate(Rotation.ANTICLOCKWISE) }
      }
      agent.attachedThings.clear()
      agent.attachedThings.addAll(newPosAttachedThings)

      val newAttachedBlockToAttached = if (lastActionParams[0] == "cw") {
        agent.attachedThingsToAttached.asSequence().associate { (pos, posSet) ->
          pos.rotate(Rotation.CLOCKWISE) to posSet.map { it.rotate(Rotation.CLOCKWISE) }.toMutableSet()
        }
      } else {
        agent.attachedThingsToAttached.asSequence().associate { (pos, posSet) ->
          pos.rotate(Rotation.ANTICLOCKWISE) to posSet.map { it.rotate(Rotation.ANTICLOCKWISE) }.toMutableSet()
        }
      }
      agent.attachedThingsToAttached.clear()
      agent.attachedThingsToAttached.putAll(newAttachedBlockToAttached)
    } else {
      log.printFlush { "${agent.name} lastActionResult: $lastActionResult, lastAction: $lastAction, lastActionParams: $lastActionParams" }
    }

    val attachedPosToRemove = mutableSetOf<Position>()
    for (attachedPos in agent.attachedThings) {
      if (attachedPos !in attachedThings) {
        // The block is stored as an attached thing but the percept tells the agent that there is no attached block
        // → remove the stored information
        attachedPosToRemove.add(attachedPos)
      }
    }
    agent.attachedThings.removeAll(attachedPosToRemove)

    // Add information to the map of the current agent's group
    val visionOfCurrentAgent = roles.getValue(role!!).vision
    val mapOfCurrentGroup = mapOfGroup.getValue(group!!)
    // Compute the positions the agent currently sees where a goal zone should be
    val allGoalZonePos = allGoalZonePositions.getValue(group)
    val goalZonesToBeReceived = mutableSetOf<Position>()

    for (nPos in Position(0, 0).neighboursAtMost(visionOfCurrentAgent)) {
      updateMapAtPosition(things, nPos, agent.offset, step, mapOfCurrentGroup)
      val posForAllGoalZones = (agent.offset + nPos).intoBounds(bounds)
      val pair = allGoalZonePos[posForAllGoalZones]
      if (pair != null && pair.second != step) {
        goalZonesToBeReceived.add(nPos)
      }
      updateMissingPositionsAtPosition(agent, posForAllGoalZones, step)
    }

    mapOfCurrentGroup[agent.offset] = PositionInfoWithStep(PositionInfo.Agent(agentName), step)
    // Add role and goal zones and the found dispensers and markers
    val roleZonesOfCurrGroup = rolePositionsOfGroup.getValue(group)
    for (pos in roleZones) {
      roleZonesOfCurrGroup.add((pos + agent.offset).intoBounds(bounds))
    }
    val markerPosOfCurrGroup = markerPosOfGroup.getValue(group)
    for ((pos, info) in currMarkers) {
      markerPosOfCurrGroup[(pos + agent.offset).intoBounds(bounds)] = info to step
    }
    val goalZonesPosOfCurrGroup = goalZonePositionsOfGroup.getValue(group)
    val goalZonesOfCurrGroup = goalZonesOfGroup.getValue(group)
    for (pos in goalZones) {
      goalZonesToBeReceived.remove(pos)
      val globalPos = (pos + agent.offset).intoBounds(bounds)
      val id = allGoalZonePos[globalPos]?.first
      if (id != null) {
        // Update the step
        allGoalZonePos[globalPos] = id to step
        continue
      }
      log.printFlush { "pos of goalZones: $pos, globalPos: $globalPos" }
      // Look whether the free goal zone positions belong to a goal zone
      var belongingToGoalZone = false
      var goalZoneId: Int? = null
      for (nPos in globalPos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
        val info = allGoalZonePos[nPos]
        if (info != null) {
          val teamToAdd = info.first
          if (teamToAdd != null) {
            belongingToGoalZone = true
            if (goalZoneId == null) {
              // The free goal zone position is added to the goal zone with ID teamToAdd
              goalZoneId = teamToAdd
              log.printFlush { "${agent.name}, set goalZoneId to $teamToAdd" }
              goalZonesOfCurrGroup.getValue(goalZoneId).positions[globalPos] = step
              allGoalZonePos[globalPos] = goalZoneId to step
              goalZonesPosOfCurrGroup.remove(globalPos)
            } else if (goalZoneId != teamToAdd) {
              // Merge the goal zones
              mergeGoalZones(group, goalZoneId, teamToAdd)
            }
          }
        }
      }
      if (belongingToGoalZone) {
        // Add more free goal zone positions (not assigned to a goal zone) from allGoalZonePos
        // if these positions are neighbours now
        addFreeGoalZonePosToGoalZone(group, globalPos, goalZoneId!!)
      } else {
        goalZonesPosOfCurrGroup[globalPos] = step
        allGoalZonePos[globalPos] = null to step
      }
    }

    for (pos in goalZonesToBeReceived) {
      val globalPos = (pos + agent.offset).intoBounds(bounds)
      val teamNum = allGoalZonePos.getValue(globalPos).first
      if (teamNum != null) {
        // There is a team working on the goal zone, but the goal zone moved
        // → Destroy the team (it is possible that the team has been destroyed in a previous loop iteration)
        val goalZoneToRemove = goalZonesOfGroup.getValue(group)[teamNum]
        if (goalZoneToRemove != null) {
          for (ag in goalZoneToRemove.agents.asSequence().map { agName -> agents.getValue(agName) }) {
            ag.clearRemovedGoalZone()
          }
          goalZonesOfGroup.getValue(group).remove(teamNum)
        }
      } else {
        goalZonesPosOfCurrGroup.remove(globalPos)
      }
      allGoalZonePos.remove(globalPos)
    }

    val dispenserOfCurrGroup = dispenserOfGroup.getValue(group)
    for ((pos, posInfo) in foundDispenser) {
      dispenserOfCurrGroup.getOrPut(posInfo.block_type) { mutableSetOf() }.add((pos + agent.offset).intoBounds(bounds))
    }

    agentPerceptsProcessed[agentName] = ProcessedPercepts(
      actionID!!,
      timestamp!!,
      deadline!!,
      step,
      lastAction!!,
      lastActionResult!!,
      lastActionParams!!,
      score!!,
      things,
      attachedThings,
      energy!!,
      deactivated!!,
      role,
      roleZones,
      goalZones,
      violations,
      norms,
      surveyedAgent,
      surveyedThing,
      hit
    )
    ++agentsWithPercepts
    log.printFlush { "step: $step, agent: ${agent.name}: ${agentPerceptsProcessed.keys}" }
    log.printFlush { "agents of group: $agentsOfGroup" }
    if (agentsWithPercepts == teamSize) {
      Log.log("Step $step, bestWidth = $bestWidth, bestHeight = $bestHeight")
      Log.log("Money: $score")
      Log.log("Tasks: $tasks")
      log.printFlush { "!!!!!!!step: $step, bestWidth=$bestWidth, bestHeight=$bestHeight" }
      log.printFlush { "agents of group: $agentsOfGroup" }
      val timer = Timer()
      updateMarkers()
      timer.printReset(log, "Time for update Markers")
      knowledgeExchange(step)
      timer.printReset(log, "Time for knowledge exchange")
      findGoalZones(step)
      timer.printReset(log, "Time for findGoalZones")
      // Update the phase of each agent
      for (agInstance in agents.values) {
        val currGroup = agInstance.group ?: continue
        agInstance.updatePhase(
          mapOfGroup.getValue(currGroup),
          rolePositionsOfGroup.getValue(currGroup),
          dispenserOfGroup.getValue(currGroup),
          agentPerceptsProcessed.getValue(agInstance.name),
          agentsOfGroup.getValue(currGroup),
          roles,
          bounds,
          log
        )
      }
      timer.printReset(log, "Time for update Phase")

      for ((groupId, groupMap) in goalZonesOfGroup) {
        log.printFlush { "groupId: $groupId" }
        for (ag in agentsOfGroup.getValue(groupId)) {
          log.printFlush {
            "agent of group: ${ag.name}, ${ag.phase}, ${ag.goalZone}, ${ag.team}, ${ag.blockType}, ${ag.diggerNum}, " +
                    "${ag.dispenserPos}, ${ag.isConstructor}"
          }
        }
        for ((goalZoneId, currGoalZone) in groupMap) {
          if (currGoalZone.taskName !in tasks.keys && currGoalZone.task != null) {
            // The task of the goal zone vanished
            log.printFlush { "Task vanished: $step, $currGoalZone" }
            currGoalZone.task = null
            currGoalZone.opponents.clear()
            // Try to assign a new task to the goal zone
            determineTask(currGoalZone, step)
            if (currGoalZone.task == null) {
              // Remove the agents of the goal zone
              for (curAgent in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
                curAgent.clearRemovedGoalZone()
              }
              currGoalZone.clearBecauseOfLostAgents()
            } else {
              // Determine the number of agents that are necessary to have at least the minimum required number
              // for the new goal zone
              var missingAgentsNum = 0
              for ((dispType, pairForRoles) in currGoalZone.minMaxOfDispenserType) {
                for (minMaxPair in pairForRoles.values) {
                  if (dispType !in currGoalZone.agentsOfDispenserType) {
                    missingAgentsNum += minMaxPair.first
                  }
                }
              }
              if (missingAgentsNum > 0) {
                // Remove the agents of the goal zone
                for (curAgent in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
                  curAgent.clearRemovedGoalZone()
                }
                currGoalZone.clearBecauseOfLostAgents()
              } else {
                // Change the constructor position
                val conAgent = currGoalZone.constructorAgent!!
                val constructorPos = getConstructorPos(currGoalZone, conAgent).first
                currGoalZone.constructorPos = constructorPos
                if (conAgent.attachedThings.size != 0) {
                  // The constructor detaches its attached things
                  conAgent.phase = Phase.DETACH_THINGS
                  for (ag in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
                    if (!ag.isConstructor && ag.lastSuccessfulAction == "connect") {
                      ag.phase = Phase.DETACH_THINGS
                    }
                  }
                } else if (conAgent.offset != constructorPos && conAgent.phase == Phase.WORK_ON_GOAL_ZONE) {
                  conAgent.phase = Phase.GO_TO_GOAL_ZONE
                }
                // Remove the agents which are no longer necessary, i.e. which have a block type assigned to them
                // which is not needed for the new task
                val keysToRemove = mutableSetOf<String>()
                for ((dispType, dispBlockTypeAgents) in currGoalZone.agentsOfDispenserType) {
                  if (dispType !in currGoalZone.minMaxOfDispenserType) {
                    keysToRemove.add(dispType)
                    for (agSet in dispBlockTypeAgents.values) {
                      for (curAgent in agSet) {
                        curAgent.clearRemovedGoalZone()
                        currGoalZone.agents.remove(curAgent.name)
                      }
                    }
                  }
                }
                for (dispsToRemove in keysToRemove) {
                  currGoalZone.agentsOfDispenserType.remove(dispsToRemove)
                }
              }
            }
          }
          if (currGoalZone.task == null || currGoalZone.agents.isEmpty()) {
            // Determine or update the attributes of the goal zone
            determineAttributes(currGoalZone, step)
          }
          val task = currGoalZone.task
          val conPos = currGoalZone.constructorPos
          val conAgent = currGoalZone.constructorAgent
          if (task != null && conPos != null && conAgent != null) {
            val mapOfGroup = mapOfGroup.getValue(groupId)
            for (pos in task.req.keys) {
              val globalPos = (pos + conPos).intoBounds(bounds)
              val posInfoWithStep = mapOfGroup[globalPos]
              if (posInfoWithStep != null && posInfoWithStep.info is PositionInfo.Entity) {
                var storedNum = currGoalZone.opponents.getOrPut(pos) { 0 }
                storedNum += 1
                if (storedNum >= 8) {
                  // Determine a constructor position because the same constructor cell has been
                  // blocked in each of the last 8 steps
                  val (newConstructorPos, reachablePos) = getConstructorPos(currGoalZone, conAgent)
                  if (reachablePos && newConstructorPos != conPos) {
                    // The constructor position is changed because the new position is reachable
                    log.printFlush { "New goal zone constructor pos chosen!!!!!" }
                    currGoalZone.opponents.clear()
                    currGoalZone.constructorPos = newConstructorPos
                    if (conAgent.attachedThings.isNotEmpty()) {
                      // The constructor detaches its attached things and the agent which connected in the last step
                      conAgent.phase = Phase.DETACH_THINGS
                      for (ag in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
                        if (!ag.isConstructor && ag.lastSuccessfulAction == "connect") {
                          ag.phase = Phase.DETACH_THINGS
                        }
                      }
                    } else {
                      conAgent.phase = Phase.GO_TO_GOAL_ZONE
                    }
                  } else {
                    currGoalZone.opponents[pos] = storedNum
                  }
                } else {
                  currGoalZone.opponents[pos] = storedNum
                }
              } else {
                currGoalZone.opponents.remove(pos)
              }
            }
          }
          if (currGoalZone.explored && currGoalZone.task != null) {
            // Compute the agents of the group which do not belong to a unit
            log.printFlush { "*******************line 447, $currGoalZone." }
            val numSearchingAgent =
              agentsOfGroup.getValue(currGoalZone.groupOfAgents).asSequence().filter { currAgent ->
                val phase = currAgent.phase
                currAgent.team == null && phase != Phase.SEARCH_ROLE_ZONE && phase != Phase.DETACH_THINGS
              }.count()
            val missingMinNum = max(0, currGoalZone.overallMinAgents!! - currGoalZone.agents.size)
            log.printFlush {
              "missingMinNum: ${missingMinNum}, numSearchingAgent: $numSearchingAgent"
            }
            if (missingMinNum > numSearchingAgent) {
              log.printFlush {
                "Not enough agents in the group to build a team for goalZone $currGoalZone " + "(overallMinAgents: ${currGoalZone.overallMinAgents}, numSearchingAgent: $numSearchingAgent)"
              }
              // Look at the next goalZone, as there are not enough agents in the group to build a team for the goalZone 
              continue
            }
            val constructorPos = getConstructorPos(currGoalZone, null).first
            // Store the distance of the agents without a unit from the current goal zone
            val distanceOfAgents = PriorityQueue(compareBy<Pair<Agent, Int>> { it.second })
            for (agentOfGroup in agentsOfGroup.getValue(currGoalZone.groupOfAgents)) {
              val phase = agentOfGroup.phase
              if (agentOfGroup.team == null && phase != Phase.SEARCH_ROLE_ZONE && phase != Phase.DETACH_THINGS) {
                distanceOfAgents.add(agentOfGroup to distanceBounded(agentOfGroup.offset, constructorPos, bounds))
              }
            }
            log.printFlush { "Distances: $distanceOfAgents" }
            if (missingMinNum != 0) {
              // Add the closest agents so that the minimal number for each job is reached
              // Add the constructor first
              val constructorAgent = distanceOfAgents.remove().first
              currGoalZone.constructorAgent = constructorAgent
              currGoalZone.agents.add(constructorAgent.name)
              currGoalZone.constructorPos = getConstructorPos(currGoalZone, currGoalZone.constructorAgent).first
              constructorAgent.isConstructor = true
              constructorAgent.team = goalZoneId
              constructorAgent.desiredRole = "worker"
              constructorAgent.phase = Phase.GO_TO_ROLE_ZONE
              constructorAgent.goalZone = currGoalZone

              for ((dispenserType, dispenserMap) in currGoalZone.minMaxOfDispenserType) {
                for ((roleType, minMaxPair) in dispenserMap) {
                  val agentMapOfThis = currGoalZone.agentsOfDispenserType.getOrPut(dispenserType) { mutableMapOf() }
                  val agentsOfThis = agentMapOfThis.getOrPut(roleType) { mutableSetOf() }
                  val numMissingAgents = minMaxPair.first - agentsOfThis.size
                  log.printFlush { "numMissingAgents: $numMissingAgents" }
                  for (i in 0 until numMissingAgents) {
                    if (distanceOfAgents.isEmpty()) {
                      log.printFlush { "distance empty" }
                      break
                    }
                    val agentToAdd = distanceOfAgents.remove().first
                    log.printFlush { "agentToAdd 1: ${agentToAdd.name}" }
                    agentToAdd.team = goalZoneId
                    agentToAdd.desiredRole = roleType
                    agentToAdd.dispenserPos = currGoalZone.nearestDispenser.getValue(dispenserType).first
                    agentToAdd.phase = Phase.GO_TO_ROLE_ZONE
                    agentToAdd.goalZone = currGoalZone
                    agentToAdd.blockType = dispenserType
                    if (roleType == "digger") {
                      agentToAdd.diggerNum = agentsOfThis.size
                    }
                    agentsOfThis.add(agentToAdd)
                    currGoalZone.agents.add(agentToAdd.name)
                  }
                  if (distanceOfAgents.isEmpty()) {
                    break
                  }
                }
              }
            }
            // Add more agents (until the maximal number of each job is reached)
            // if they are close enough (i.e. distance of at most 20)
            for ((dispenserType, dispenserMap) in currGoalZone.minMaxOfDispenserType) {
              for ((roleType, minMaxPair) in dispenserMap) {
                val agentMapOfThis = currGoalZone.agentsOfDispenserType.getOrPut(dispenserType) { mutableMapOf() }
                val agentsOfThis = agentMapOfThis.getOrPut(roleType) { mutableSetOf() }
                val numMaximalMoreAgents = minMaxPair.second - agentsOfThis.size
                log.printFlush { "numMaximalMoreAgents: $numMaximalMoreAgents" }
                for (i in 0 until numMaximalMoreAgents) {
                  if (distanceOfAgents.isEmpty()) {
                    break
                  }
                  val agentInfoPair = distanceOfAgents.remove()
                  if (agentInfoPair.second < 20) {
                    log.printFlush { "agentToAdd 2: ${agentInfoPair.first.name}" }
                    agentInfoPair.first.team = goalZoneId
                    agentInfoPair.first.desiredRole = roleType
                    agentInfoPair.first.dispenserPos = currGoalZone.nearestDispenser.getValue(dispenserType).first
                    agentInfoPair.first.phase = Phase.GO_TO_ROLE_ZONE
                    agentInfoPair.first.goalZone = currGoalZone
                    agentInfoPair.first.blockType = dispenserType
                    if (roleType == "digger") {
                      agentInfoPair.first.diggerNum = agentsOfThis.size
                    }
                    agentsOfThis.add(agentInfoPair.first)
                    currGoalZone.agents.add(agentInfoPair.first.name)
                  } else {
                    break
                  }
                }
                if (distanceOfAgents.isEmpty()) {
                  break
                }
              }
            }
          }
          log.printFlush { "*******************line 494, $currGoalZone" }
        }
      }
      timer.printReset(log, "Time for goal Zone stuff")
      // Update the phase of each agent again
      for (agInstance in agents.values) {
        val currGroup = agInstance.group ?: continue
        agInstance.updatePhase(
          mapOfGroup.getValue(currGroup),
          rolePositionsOfGroup.getValue(currGroup),
          dispenserOfGroup.getValue(currGroup),
          agentPerceptsProcessed.getValue(agInstance.name),
          agentsOfGroup.getValue(currGroup),
          roles,
          bounds,
          log
        )
      }
      timer.printReset(log, "Time for second update Phase")
      timer.printReset(log, "Time for visualization")

      // Determine the agents which are part of any optimization problem
      val optimizationAgents = agents.values.filter {
        it.group != null && (it.phase == Phase.WORK_ON_GOAL_ZONE ||
                it.phase == Phase.SEARCH_ROLE_ZONE || it.phase == Phase.SEARCH_MAP)
      }
      optimizedExplore(step, log, optimizationAgents.size)
      timer.printReset(log, "Time for explore")
      val agentsWithStepsDone = optimizedTasking(step, log, optimizationAgents.size)
      timer.printReset(log, "Time for tasking")
      log.printFlush { "agentsWithStepsDone: $agentsWithStepsDone" }
      // Choose actions for agents which are assigned to a goal zone
      // but whose actions were not determined by the optimization problem
      for ((groupId, goalZoneMap) in goalZonesOfGroup) {
        for (currGoalZone in goalZoneMap.values) {
          if (currGoalZone.task == null) {
            continue
          }
          log.printFlush { "A goalZone with a task: $currGoalZone" }
          for (ag in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
            if (ag in agentsWithStepsDone) {
              continue
            }
            if (ag.phase == Phase.WORK_ON_GOAL_ZONE || ag.phase == Phase.GO_TO_GOAL_ZONE) {
              val currRole = agentPerceptsProcessed.getValue(ag.name).role
              if (ag.isConstructor && ag.team != null) {
                val goalZone = ag.goalZone!!
                if (getReachableTargetCells(ag, setOf(goalZone.constructorPos!!)).isEmpty()) {
                  // Update the constructor position as the current one is not reachable
                  goalZone.constructorPos = getConstructorPos(goalZone, ag).first
                }
              }
              val targetPosSet = if (ag.diggerNum != null) {
                ag.getFlock(roles.getValue("digger").clearMaxDistance, bounds)
              } else {
                val targetPos = ag.getTargetPos(
                  currRole, mapOfGroup.getValue(groupId), agentsOfGroup.getValue(groupId), bounds
                )!!
                setOf(targetPos.intoBounds(bounds))
              }
              if (ag.offset !in targetPosSet && ag.attachedThings.size == 0) {
                // Follow a shortest path to any target position
                val action = try {
                  getShortestPath(ag, targetPosSet)
                } catch (_: Exception) {
                  // If there is an exception: skip
                  Action("skip")
                }
                log.printFlush { "${ag.name}: Work on goal zone or go to goal zone: $action" }
                performAction(ag) { action }
              } else {
                log.printFlush { "${ag.name}: Work on goal zone or go to goal zone: skip" }
                performAction(ag) { Action("skip") }
              }
            }
          }
        }
      }
      timer.printReset(log, "Time for things are optimization before step")

      for (agInstance in agents.values) {
        agInstance.group ?: continue
        val phase = agInstance.phase
        if (phase == Phase.SEARCH_ROLE_ZONE || phase == Phase.SEARCH_MAP || phase == Phase.GO_TO_GOAL_ZONE ||
          phase == Phase.WORK_ON_GOAL_ZONE
        ) {
          continue
        }
        // For all agents with a different phase: Call the step function
        log.printFlush { "Call step in AgentManager" }
        step(agInstance)
      }
      timer.printReset(log, "Time for step")
      agentsWithPercepts = 0
    }
  }

  // Merge goalZones with goalZoneId and teamToAdd (goalZoneId remains)
  private fun mergeGoalZones(
    currGroup: Int,
    goalZoneId: Int,
    teamToRemId: Int,
    newAllGoalZone: MutableMap<Position, Pair<Int?, Int>>? = null
  ) {
    val goalZonesOfCurrGroup = goalZonesOfGroup.getValue(currGroup)
    log.printFlush { "mergeGoalZones: $currGroup, $goalZoneId, $goalZonesOfGroup, $goalZoneId, $teamToRemId" }
    val goalZoneToRemove = goalZonesOfCurrGroup.getValue(teamToRemId)
    for ((posToAdd, posInfo) in goalZoneToRemove.positions) {
      val positions = goalZonesOfCurrGroup.getValue(goalZoneId).positions
      val oldInfo = positions[posToAdd]
      if (oldInfo == null || oldInfo < posInfo) {
        positions[posToAdd] = posInfo
        if (newAllGoalZone == null) {
          allGoalZonePositions.getValue(currGroup)[posToAdd] = goalZoneId to posInfo
        } else {
          newAllGoalZone[posToAdd] = goalZoneId to posInfo
        }
      }
    }
    // Remove the agents from goalZoneToRemove
    for (ag in goalZoneToRemove.agents.asSequence().map { agName -> agents.getValue(agName) }) {
      ag.clearRemovedGoalZone()
    }
    goalZonesOfCurrGroup.remove(teamToRemId)
  }

  // Add more free goal zone positions (not assigned to a goal zone) from allGoalZonePos if these positions
  // are neighbours now.
  // currGroup is the group of the goalZone with goalZoneId as the ID, while startPos is the last position
  // already added to the goalZone.
  private fun addFreeGoalZonePosToGoalZone(
    currGroup: Int,
    startPos: Position,
    goalZoneId: Int,
    newAllGoalZone: MutableMap<Position, Pair<Int?, Int>>? = null,
  ) {
    val allGoalZonePos = allGoalZonePositions.getValue(currGroup)
    val goalZonesOfCurrGroup = goalZonesOfGroup.getValue(currGroup)
    val goalZonesPosOfCurrGroup = goalZonePositionsOfGroup.getValue(currGroup)

    for (nPos in startPos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
      val nPosInf = if (newAllGoalZone?.containsKey(nPos) == true) {
        newAllGoalZone.getValue(nPos)
      } else if (nPos in allGoalZonePos) {
        allGoalZonePos.getValue(nPos)
      } else {
        continue
      }
      if (nPosInf.first == null) {
        goalZonesOfCurrGroup.getValue(goalZoneId).positions[nPos] = nPosInf.second
        if (newAllGoalZone == null) {
          allGoalZonePos[nPos] = goalZoneId to nPosInf.second
        } else {
          newAllGoalZone[nPos] = goalZoneId to nPosInf.second
        }
        goalZonesPosOfCurrGroup.remove(nPos)
        // Call the function recursively
        addFreeGoalZonePosToGoalZone(currGroup, nPos, goalZoneId, newAllGoalZone)
      } else if (nPosInf.first != goalZoneId) {
        // Merge the two goal zones because they are adjacent
        mergeGoalZones(currGroup, goalZoneId, nPosInf.first!!, newAllGoalZone)
      }
    }
  }

  // Get the difference in the offset of the agent if the last action was only partially successful
  private fun getOffsetDiff(
    agent: Agent,
    vision: Int,
    lastActionParams: List<String>,
    group: Int,
    localThings: MutableMap<Position, PositionInfo>,
    localGoalZones: MutableSet<Position>,
    localRoleZones: MutableSet<Position>,
    localDispensers: MutableMap<Position, PositionInfo.Dispenser>
  ): Position {
    val mapOfCurrentGroup = mapOfGroup.getValue(group)
    val globalAllGoalZonePos = allGoalZonePositions.getValue(group)
    val globalRoleZonePos = rolePositionsOfGroup.getValue(group)
    val globalDispensers = dispenserOfGroup.getValue(group)
    if (lastActionParams.size == 2) {
      // The first action was successful, the second was not
      return posFromDir(lastActionParams[0])
    }
    var diff = Position(0, 0)
    // The best percentage of the things the agent currently sees which agree with the information stored in the map
    var bestPercent = 0.0
    // Best found difference in the offset
    var bestDiff = diff
    // Exclude the last one because the action was partially successful
    for (i in 0 until lastActionParams.size - 1) {
      diff += posFromDir(lastActionParams[i])
      var numAllPos = 0
      var numCorrectPos = 0
      var numAllRole = 0
      var numAllGoal = 0
      var numCorrectGoal = 0
      var numAllDisp = 0
      var wrongPos = false
      for (localCurrPos in Position(0, 0).neighboursAtMost(vision)) {
        val globalCurrPos = (localCurrPos + agent.offset + diff).intoBounds(bounds)
        // Look at the map
        val mapInfo = mapOfCurrentGroup[globalCurrPos]?.info
        if (mapInfo != null) {
          ++numAllPos
          val localPosInfo = localThings[localCurrPos]
          if (localPosInfo != null) {
            if (localPosInfo is PositionInfo.Obstacle && mapInfo is PositionInfo.Obstacle ||
              localPosInfo is PositionInfo.Block && mapInfo is PositionInfo.Block ||
              localPosInfo is PositionInfo.Entity && (mapInfo is PositionInfo.Entity || mapInfo is PositionInfo.Agent)
            ) {
              ++numCorrectPos
            }
          } else if (mapInfo is PositionInfo.Empty) {
            ++numCorrectPos
          }
        }

        // Look at the dispensers
        val disp = localDispensers[localCurrPos]
        if (disp != null) {
          val globalDisp = globalDispensers[disp.block_type]
          val posInfo = mapOfCurrentGroup[globalCurrPos]
          if (
            (globalDisp == null || globalCurrPos !in globalDisp) &&
            posInfo != null && posInfo.info !is PositionInfo.Entity
          ) {
            wrongPos = true
            break
          }
        }

        if (globalDispensers.values.any { globalCurrPos in it }) {
          if (localThings[localCurrPos] is PositionInfo.Entity) {
            continue
          }
          ++numAllDisp
          if (localCurrPos !in localDispensers) {
            wrongPos = true
            break
          }
        }

        // Look at the role zones
        if (globalCurrPos in globalRoleZonePos) {
          ++numAllRole
          if (localCurrPos !in localRoleZones) {
            wrongPos = true
            break
          }
        }
        if (localCurrPos in localRoleZones) {
          if (globalCurrPos !in globalRoleZonePos && globalCurrPos in mapOfCurrentGroup) {
            wrongPos = true
            break
          }
        }

        // Look at the goal zones
        if (globalCurrPos in globalAllGoalZonePos) {
          ++numAllGoal
          if (localCurrPos in localGoalZones) {
            ++numCorrectGoal
          }
        }
      }
      if (wrongPos) {
        continue
      }
      if (!wrongPos && (numAllRole != 0 || numAllDisp != 0)) {
        // With this diff, all role zones the agent should see were seen (analogously for the dispensers)
        // Return the diff, as role zones and dispensers cannot move
        return diff
      }
      val currPercent = numCorrectPos.toDouble() / numAllPos + if (numAllGoal != 0) {
        numCorrectGoal.toDouble() / numAllGoal
      } else {
        0.0
      }
      if (currPercent > bestPercent) {
        bestPercent = currPercent
        bestDiff = diff
      }
    }
    return bestDiff
  }

  // Updates the map of a group with the things one of the agents of the group sees
  // (things are in the local coordinate system of the agent which has the offset posOff)
  private fun updateMapAtPosition(
    things: MutableMap<Position, PositionInfo>,
    pos: Position,
    posOff: Position,
    step: Int,
    mapOfCurrentGroup: MutableMap<Position, PositionInfoWithStep>
  ) {
    val newPos = (pos + posOff).intoBounds(bounds)
    val currThing = things[pos]
    mapOfCurrentGroup[newPos] = if (currThing != null) {
      val oldVal = mapOfCurrentGroup[newPos]
      if (oldVal?.info is PositionInfo.Agent && (currThing as? PositionInfo.Entity)?.team == team) {
        return
      }
      PositionInfoWithStep(currThing, step)
    } else {
      PositionInfoWithStep(PositionInfo.Empty, step)
    }
  }

  // Determine the constructor position for the given goal zone.
  // If an agent is given, goal cells the agent cannot reach are discarded
  private fun getConstructorPos(goalZone: GoalZone, agent: Agent?): Pair<Position, Boolean> {
    val mapOfCourGroup = mapOfGroup.getValue(goalZone.groupOfAgents)
    // Store how many obstacles must be cleared for each goalZonePos on which the task could be submitted
    val obstaclesOfPos = PriorityQueue(compareBy<Pair<Position, Int>> { it.second })
    val dispensersOfCurrGroup = dispenserOfGroup.getValue(goalZone.groupOfAgents)
    val reachableGoalZonePos = if (agent != null) {
      getReachableTargetCells(agent, goalZone.positions.keys)
    } else {
      goalZone.positions.keys
    }
    val constructorPos = goalZone.constructorPos
    for (pos in reachableGoalZonePos) {
      var blocksToClear = 0
      var notPossible = false
      val posToLookAt = goalZone.task!!.req.keys.toMutableSet().apply { add(Position(0, 0)) }
      for (offset in posToLookAt) {
        val currPos = (pos + offset).intoBounds(bounds)
        if (dispensersOfCurrGroup.values.any { it.contains(currPos) }) {
          notPossible = true
          break
        }
        val currPosInfoWithStep = mapOfCourGroup[currPos]
        if (currPosInfoWithStep != null) {
          val info = currPosInfoWithStep.info
          if (info is PositionInfo.Obstacle) {
            ++blocksToClear
          } else if (info is PositionInfo.Block) {
            if (isBlockUnattached(currPos, goalZone.groupOfAgents)) {
              ++blocksToClear
            } else {
              notPossible = true
              break
            }
          } else if (info is PositionInfo.Empty) {
            continue
          } else if (info is PositionInfo.Agent) {
            val constructorAgent = goalZone.constructorAgent
            if (constructorAgent != null && constructorAgent.name == info.name) {
              continue
            } else {
              notPossible = true
              break
            }
          } else {
            notPossible = true
            break
          }
        }
        if (notPossible) {
          break
        }
      }
      if (!notPossible) {
        if (constructorPos == pos) {
          return constructorPos to true
        }
        obstaclesOfPos.add(pos to blocksToClear)
      }
    }
    return if (obstaclesOfPos.isEmpty()) {
      // There is no goal cell for the constructor position
      // so that both the constructor position and the constructor cells can be cleared
      if (constructorPos != null) {
        constructorPos to false
      } else {
        goalZone.positions.keys.iterator().next() to false
      }
    } else {
      // Take the goal cell for which the least number of blocks and obstacles has to be cleared
      obstaclesOfPos.remove().first to true
    }
  }

  // Choose a task for the goal zone
  private fun determineTask(goalZone: GoalZone, step: Int) {
    var bestTaskName: String? = null
    var bestPointsPerSteps = 0.0
    log.printFlush { "possibleTasks: $tasks" }
    for ((taskName, task) in tasks) {
      // The number of steps which will be necessary to complete the task (assuming there are no obstacles or enemies)
      val approxNumSteps = mutableMapOf<String, Int>()
      var notPossible = false
      for (taskReqType in task.req.values) {
        val nearestDispenser = goalZone.nearestDispenser[taskReqType]
        if (nearestDispenser != null) {
          val oldVal = approxNumSteps.getOrPut(taskReqType) { 0 }
          approxNumSteps[taskReqType] = oldVal + nearestDispenser.second * 2
        } else {
          // The group has not found such a dispenser yet
          notPossible = true
          break
        }
      }
      if (notPossible) {
        continue
      }
      // Is it possible to complete a task until the deadline?
      val neededNumSteps = approxNumSteps.values.max()
      if (neededNumSteps < task.deadline - step) {
        val pointsPerStep = task.reward.toDouble() / neededNumSteps.toDouble()
        if (pointsPerStep > bestPointsPerSteps) {
          bestPointsPerSteps = pointsPerStep
          bestTaskName = taskName
        }
      }
    }
    if (bestTaskName != null) {
      goalZone.task = tasks[bestTaskName]
      goalZone.taskName = bestTaskName
    }

    // Get the minimal and maximal number of agents for each job if a task has been chosen
    if (goalZone.task != null) {
      goalZone.constructorPos = getConstructorPos(goalZone, null).first
      goalZone.minMaxOfDispenserType.clear()
      val numTypeBlock = mutableMapOf<String, Int>()
      for (type in goalZone.task!!.req.values) {
        val oldNum = numTypeBlock.getOrPut(type) { 0 }
        numTypeBlock[type] = oldNum + 1
      }
      // One constructor
      var overallMinNumAgents = 1
      val diggerClearDistance = roles.getValue("digger").clearMaxDistance
      val maxNumTypeBlock = numTypeBlock.maxOf { it.value }
      for ((type, num) in numTypeBlock) {
        val currDis = goalZone.nearestDispenser.getValue(type).second
        val mapMinMax = mutableMapOf<String, Pair<Int, Int>>()
        val workerMin = 1
        val diggerMin = 1
        mapMinMax["worker"] = workerMin to (currDis * num).divCeil(6 * maxNumTypeBlock)
        mapMinMax["digger"] = diggerMin to (currDis divCeil diggerClearDistance)
        overallMinNumAgents += workerMin + diggerMin
        goalZone.minMaxOfDispenserType[type] = mapMinMax
      }
      goalZone.overallMinAgents = overallMinNumAgents
    }
  }

  // Determine or update the attributes of the goal zone
  private fun determineAttributes(goalZone: GoalZone, step: Int) {
    val group = goalZone.groupOfAgents
    val onePosOfZone = goalZone.positions.keys.iterator().next()
    // Get the nearest dispenser of each type and the best distance
    for ((dispenserType, dispenserPos) in dispenserOfGroup.getValue(group)) {
      var bestDistance = Int.MAX_VALUE
      var bestPos: Position? = null
      for (pos in dispenserPos) {
        val distance = distanceBounded(pos, onePosOfZone, bounds)
        if (distance < bestDistance) {
          bestDistance = distance
          bestPos = pos
        }
      }
      if (bestPos == null) {
        bestPos = dispenserPos.iterator().next()
      }
      goalZone.nearestDispenser[dispenserType] = bestPos to bestDistance
    }
    if (!goalZone.explored) {
      // A goal zone is marked as explored (only for explored goal zones it is tried to assign a task to it)
      // Check for each dispenser type whether at least one has been found in the near distance and, if this is the case,
      // mark the goal zone as explored
      var foundDispensersClose = true
      for (possibleDisp in possibleDispensers) {
        val nearestDispenser = goalZone.nearestDispenser[possibleDisp]
        if (nearestDispenser == null || nearestDispenser.second > 10) {
          foundDispensersClose = false
          break
        }
      }
      if (foundDispensersClose) {
        goalZone.explored = true
        goalZone.missingPositions.clear()
      }
    }
    if (!goalZone.explored) {
      // Do not choose a task if the goal zone is not explored yet
      return
    }
    determineTask(goalZone, step)
  }

  // Update the missing positions of the goal zones of the group of the agent (the position pos is in
  // the coordinate system of the group)
  private fun updateMissingPositionsAtPosition(agent: Agent, pos: Position, step: Int) {
    for (goalZone in goalZonesOfGroup.getValue(agent.group!!).values) {
      if (goalZone.explored) {
        // There are no missing positions for explored goal zones
        continue
      }
      if (goalZone.missingPositions.contains(pos)) {
        goalZone.missingPositions.remove(pos)
        if (goalZone.missingPositions.isEmpty()) {
          goalZone.explored = true
          determineAttributes(goalZone, step)
        }
      }
    }
  }

  // Agent 1 sees an entity of the same team at position pos. Could agent 2 be the agent on this pos?
  // visible is what each agent sees at the moment
  // vision is the distance each agent sees
  private fun whoAmI(
    pos: Position,
    visible1: Map<Position, PositionInfo>,
    visible2: Map<Position, PositionInfo>,
    vision1: Int,
    vision2: Int
  ): Boolean {
    for ((thingPos, thingInfo) in visible1) {
      val posFor2 = subClosest(thingPos, pos, bounds)
      val infoOf2 = visible2[posFor2] ?: if (posFor2.norm > vision2) {
        // The agent 2 cannot see posFor2
        continue
      } else {
        return false
      }
      if (infoOf2 != thingInfo) {
        return false
      }
    }
    for ((thingPos, thingInfo) in visible2) {
      val posFor1 = (thingPos + pos).intoBounds(bounds)
      val infoOf1 = visible1[posFor1] ?: if (posFor1.norm > vision1) {
        continue
      } else {
        return false
      }
      if (infoOf1 != thingInfo) {
        return false
      }
    }
    return true
  }

  // Update all stored information about the world because new best boundaries of the world have been found
  private fun updateAllPositionsIntoBounds() {
    log.printFlush { "update All Positions Because of Boundaries" }
    for ((groupId, map) in dispenserOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val newMap = buildMap {
        for ((dispType, posSet) in map) {
          val newPosSet = mutableSetOf<Position>()
          for (currPos in posSet) {
            newPosSet.add(currPos.intoBounds(bounds))
          }
          put(dispType, newPosSet)
        }
      }
      map.putAll(newMap)
    }

    for ((groupId, map) in markerPosOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val newMap = mutableMapOf<Position, Pair<String, Int>>()
      for ((currPos, currInfo) in map) {
        val newPos = currPos.intoBounds(bounds)
        val newMapInfo = newMap[newPos]
        if (newMapInfo != null && newMapInfo.second > currInfo.second) {
          continue
        }
        newMap[newPos] = currInfo
      }
      markerPosOfGroup[groupId] = newMap
    }

    for ((groupId, posSet) in rolePositionsOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val newPosSet = mutableSetOf<Position>()
      for (currPos in posSet) {
        newPosSet.add(currPos.intoBounds(bounds))
      }
      rolePositionsOfGroup[groupId] = newPosSet
    }

    for ((groupId, map) in mapOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val newMap = mutableMapOf<Position, PositionInfoWithStep>()
      for ((currPos, currInfo) in map) {
        val newPos = currPos.intoBounds(bounds)
        val newPosInfo = newMap[newPos]
        if (newPosInfo != null && (newPosInfo.info is PositionInfo.Agent ||
                  newPosInfo.lastStep >= currInfo.lastStep && currInfo.info !is PositionInfo.Agent)
        ) {
          continue
        }
        newMap[newPos] = currInfo
      }
      mapOfGroup[groupId] = newMap
    }

    for (currAgent in agents.values) {
      if (currAgent.group == null) {
        continue
      }
      currAgent.offset = currAgent.offset.intoBounds(bounds)
      val dispenserPos = currAgent.dispenserPos ?: continue
      currAgent.dispenserPos = dispenserPos.intoBounds(bounds)
    }

    for ((groupId, map) in allGoalZonePositions) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      log.printFlush { "$groupId, $map" }
      val newMap = mutableMapOf<Position, Pair<Int?, Int>>()
      for ((currPos, currPairInfo) in map) {
        val newPos = currPos.intoBounds(bounds)
        val savedInfo = newMap[newPos]
        if (savedInfo != null) {
          if (savedInfo.first == null && savedInfo.second > currPairInfo.second && currPairInfo.first == null
            || savedInfo.first != null && currPairInfo.first == null
          ) {
            continue
          }
          if (savedInfo.first != null) {
            // Only one team remains
            val goalZoneOfGroupId = goalZonesOfGroup.getValue(groupId)
            val currGoalZoneId = currPairInfo.first!!
            log.printFlush { "Only one team remains: $groupId, $goalZoneOfGroupId, $currGoalZoneId" }
            val goalZoneToRemove = goalZoneOfGroupId[currGoalZoneId]
            if (goalZoneToRemove == null) {
              // There is no such team anymore
              log.printFlush { "There is no such team anymore" }
              continue
            }
            log.printFlush { "Remove it" }
            // Remove it
            for (ag in goalZoneToRemove.agents.asSequence().map { agents.getValue(it) }) {
              ag.clearRemovedGoalZone()
            }
            goalZoneOfGroupId.remove(currGoalZoneId)
            continue
          }
        }
        newMap[newPos] = currPairInfo
      }
      allGoalZonePositions[groupId] = newMap
      log.printFlush { "$groupId, $allGoalZonePositions[groupId]" }
    }

    for ((groupId, groupMap) in goalZonePositionsOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val allGoalZonePos = allGoalZonePositions.getValue(groupId)
      val newMap = mutableMapOf<Position, Int>()
      for ((currPos, currInfo) in groupMap) {
        val newPos = currPos.intoBounds(bounds)
        val newPosInfo = newMap[newPos]
        if (newPosInfo != null && newPosInfo > currInfo) {
          continue
        }
        if (allGoalZonePos.getValue(newPos).first != null) {
          continue
        }
        newMap[newPos] = currInfo
      }
      goalZonePositionsOfGroup[groupId] = newMap
    }

    for ((groupId, groupGoalZoneMap) in goalZonesOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      for ((goalZoneId, currGoalZone) in groupGoalZoneMap) {
        val newPositions = mutableMapOf<Position, Int>()
        for ((currPos, currInfo) in currGoalZone.positions) {
          val newPos = currPos.intoBounds(bounds)
          val newPosInfo = newPositions[newPos]
          if (newPosInfo != null && newPosInfo > currInfo) {
            continue
          }
          newPositions[newPos] = currInfo
        }
        val newMissingPositions = mutableSetOf<Position>()
        for (currPos in currGoalZone.missingPositions) {
          newMissingPositions.add(currPos.intoBounds(bounds))
        }
        val newNearestDispenser = mutableMapOf<String, Pair<Position, Int>>()
        for ((dispType, dispInfo) in currGoalZone.nearestDispenser) {
          val newPos = dispInfo.first.intoBounds(bounds)
          newNearestDispenser[dispType] = newPos to dispInfo.second
        }
        val newConstructorPos = currGoalZone.constructorPos?.intoBounds(bounds)
        val newGoalZone = GoalZone(
          goalZoneId,
          newPositions,
          currGoalZone.agents,
          currGoalZone.groupOfAgents,
          currGoalZone.explored,
          newMissingPositions,
          newNearestDispenser,
          currGoalZone.task,
          currGoalZone.taskName,
          currGoalZone.constructorAgent,
          newConstructorPos,
          currGoalZone.minMaxOfDispenserType,
          currGoalZone.agentsOfDispenserType,
          currGoalZone.overallMinAgents,
          currGoalZone.opponents
        )
        groupGoalZoneMap[goalZoneId] = newGoalZone
        for (ag in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
          ag.goalZone = newGoalZone
        }
      }
    }
  }

  // Add the group of agent 2 to that of agent 1,
  // where agent 2 is at position posAgent2 in the local coordinate system of agent 1
  private fun mergeGroups(agent1: Agent, agent2: Agent, posAgent2: Position, step: Int) {
    val group1 = agent1.group!!
    val group2 = agent2.group!!
    if (group1 == group2) {
      log.printFlush {
        "mergeGroups: agents of the same group (offset ${agent1.name}: ${agent1.offset}, " + "offset ${agent2.name}: ${agent2.offset}, posAgent2: $posAgent2)"
      }
      val offsetAgent2From1 = (agent1.offset + posAgent2).intoBounds(bounds)
      if (offsetAgent2From1 == agent2.offset) {
        // The offsets are matching (there was no wrap around)
        return
      } else {
        // Determine the width and/or the height of the grid (or a multiple of it) and update the map
        log.printFlush { "agent1: $agent1, agent2: $agent2, posAgent2: $agent2" }
        var width: Int? = null
        var height: Int? = null
        if (offsetAgent2From1.x != agent2.offset.x) {
          width = abs(offsetAgent2From1.x - agent2.offset.x)
        }
        if (offsetAgent2From1.y != agent2.offset.y) {
          height = abs(offsetAgent2From1.y - agent2.offset.y)
        }
        var foundBetterBoundaries = false
        if (width != null && (bestWidth == null || bestWidth!! > width)) {
          foundBetterBoundaries = true
          bestWidth = width
        }
        if (height != null && (bestHeight == null || bestHeight!! > height)) {
          foundBetterBoundaries = true
          bestHeight = height
        }
        if (foundBetterBoundaries) {
          updateAllPositionsIntoBounds()
        }
        return
      }
    }
    log.printFlush { "mergeGroups $group1, $group2" }
    Log.log("mergeGroups $group1, $group2")
    val map1 = mapOfGroup.getValue(group1)
    val map2 = mapOfGroup.getValue(group2)
    val offPos = agent1.offset + posAgent2 - agent2.offset
    for ((ePos, eInfo) in map2) {
      // Take what is newer
      val posForMap1 = (ePos + offPos).intoBounds(bounds)
      val posInfoForMap1 = map1[posForMap1]
      if (posInfoForMap1 == null || (eInfo.info is PositionInfo.Agent && posInfoForMap1.info is PositionInfo.Entity) ||
        (eInfo.lastStep > posInfoForMap1.lastStep && posInfoForMap1.info !is PositionInfo.Agent)
      ) {
        map1[posForMap1] = eInfo
      }
    }
    val roleZones1 = rolePositionsOfGroup.getValue(group1)
    val roleZones2 = rolePositionsOfGroup.getValue(group2)
    for (pos in roleZones2) {
      roleZones1.add((pos + offPos).intoBounds(bounds))
    }
    val markerPos1 = markerPosOfGroup.getValue(group1)
    val markerPos2 = markerPosOfGroup.getValue(group2)
    for ((currMarkPos2, currMarkInfo2) in markerPos2) {
      val newPos = (currMarkPos2 + offPos).intoBounds(bounds)
      val currMarkInfo1 = markerPos1[newPos]
      if (currMarkInfo1 != null && currMarkInfo1.second > currMarkInfo2.second) {
        continue
      }
      markerPos1[newPos] = currMarkInfo2
    }

    val dispenser1 = dispenserOfGroup.getValue(group1)
    val dispenser2 = dispenserOfGroup.getValue(group2)
    for ((disName, disPosSet) in dispenser2) {
      for (pos in disPosSet) {
        dispenser1.getOrPut(disName) { mutableSetOf() }.add((pos + offPos).intoBounds(bounds))
      }
    }

    // Merge the found goal zones
    val allGoalZonePos1 = allGoalZonePositions.getValue(group1)
    val allGoalZonePos2 = allGoalZonePositions.getValue(group2)
    val goalZones1 = goalZonesOfGroup.getValue(group1)
    val goalZones2 = goalZonesOfGroup.getValue(group2)
    val goalZonePos1 = goalZonePositionsOfGroup.getValue(group1)
    val goalZonePos2 = goalZonePositionsOfGroup.getValue(group2)

    // Remove the goal zones from goalZones1 for which we know now that they moved
    val goalZonesToBeRemoved = mutableSetOf<Int>()
    for ((currGoalZoneId, currGoalZone) in goalZones1) {
      var goalZoneVanished = false
      for (firstPos1 in currGoalZone.positions.keys) {
        val firstPos2 = (firstPos1 - offPos).intoBounds(bounds)
        val posInfo2 = map2[firstPos2]
        if (firstPos2 !in allGoalZonePos2 && posInfo2 != null &&
          posInfo2.lastStep > map1.getValue(firstPos1).lastStep
        ) {
          goalZoneVanished = true
          break
        }
      }
      if (!goalZoneVanished) {
        continue
      }
      // The goal zone has moved → remove the team
      for (curAgent in currGoalZone.agents.asSequence().map { agents.getValue(it) }) {
        curAgent.clearRemovedGoalZone()
      }
      goalZonesToBeRemoved.add(currGoalZoneId)
    }
    for (goalZone in goalZonesToBeRemoved) {
      for (posToDel in goalZones1.getValue(goalZone).positions.keys) {
        allGoalZonePos1.remove(posToDel)
      }
      goalZones1.remove(goalZone)
    }

    for (currGoalZone in goalZones2.values) {
      var twoTeams = false
      var goalZoneVanished = false
      for (firstPos2 in currGoalZone.positions.keys) {
        val firstPos1 = (firstPos2 + offPos).intoBounds(bounds)
        val map1AtFirstPos1 = map1.getValue(firstPos1)
        val allGoalZoneInfo = allGoalZonePos1[firstPos1]
        if (allGoalZoneInfo?.first != null) {
          // In this case both groups found the goal zone
          twoTeams = true
          break
        } else if (allGoalZoneInfo == null && map1AtFirstPos1.lastStep > map2.getValue(firstPos2).lastStep) {
          goalZoneVanished = true
          break
        }
      }

      if (twoTeams || goalZoneVanished) {
        // Only one team remains or the goal zone has moved → remove the unit
        for (ag in currGoalZone.agents.asSequence().map { agents.getValue(it) }) {
          ag.clearRemovedGoalZone()
        }
        continue
      }

      // Add the goal zone to the goal zones of group 1
      val newId = goalZoneCounter.getValue(group1)
      goalZoneCounter[group1] = newId + 1
      var newConstructorPos: Position? = null
      val currConstrPos = currGoalZone.constructorPos
      if (currConstrPos != null) {
        newConstructorPos = (currConstrPos + offPos).intoBounds(bounds)
      }
      val newGoalZone = GoalZone(
        newId,
        mutableMapOf(),
        currGoalZone.agents,
        group1,
        currGoalZone.explored,
        mutableSetOf(),
        mutableMapOf(),
        currGoalZone.task,
        currGoalZone.taskName,
        currGoalZone.constructorAgent,
        newConstructorPos,
        currGoalZone.minMaxOfDispenserType,
        currGoalZone.agentsOfDispenserType,
        currGoalZone.overallMinAgents,
        currGoalZone.opponents
      )
      goalZones1[newId] = newGoalZone
      for ((pos, posStep) in currGoalZone.positions) {
        val globalPosition = (pos + offPos).intoBounds(bounds)
        newGoalZone.positions[globalPosition] = posStep
        goalZonePos1.remove(globalPosition)
        allGoalZonePos1[globalPosition] = newId to posStep
      }
      for (pos in currGoalZone.missingPositions) {
        newGoalZone.missingPositions.add((pos + offPos).intoBounds(bounds))
      }
      for ((nearestDispType, nearestDispInfo) in currGoalZone.nearestDispenser) {
        newGoalZone.nearestDispenser[nearestDispType] =
          (nearestDispInfo.first + offPos).intoBounds(bounds) to nearestDispInfo.second
      }
      for (ag in currGoalZone.agents.map { agName -> agents.getValue(agName) }) {
        ag.team = newId
        ag.goalZone = goalZones1[newId]
        val oldDispPos = ag.dispenserPos
        if (oldDispPos != null) {
          ag.dispenserPos = (oldDispPos + offPos).intoBounds(bounds)
        }
      }
    }

    // Merge the goal zone positions
    for ((goalPos2, step2) in goalZonePos2) {
      val pos1 = (goalPos2 + offPos).intoBounds(bounds)
      if (allGoalZonePos1[pos1]?.first != null) {
        // In this case we know what the team is
        continue
      } else {
        val goalZonePos1Info = goalZonePos1[pos1]
        if (goalZonePos1Info == null || goalZonePos1Info < step2) {
          goalZonePos1[pos1] = step2
          allGoalZonePos1[pos1] = null to step2
        }
      }
    }

    val newMap = mutableMapOf<Position, Pair<Int?, Int>>()
    for ((lonelyPos, lonelyPair) in allGoalZonePos1) {
      if (newMap[lonelyPos]?.first != null) {
        continue
      }
      if (lonelyPair.first != null) {
        // It is not lonely
        newMap[lonelyPos] = lonelyPair
        continue
      }
      var belongingToGoalZone = false
      var goalZoneId: Int? = null
      for (nPos in lonelyPos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
        val teamToAdd = newMap[nPos]?.first ?: allGoalZonePos1[nPos]?.first ?: continue
        belongingToGoalZone = true
        if (goalZoneId == null) {
          goalZoneId = teamToAdd
          goalZones1.getValue(goalZoneId).positions[lonelyPos] = lonelyPair.second
          newMap[lonelyPos] = goalZoneId to lonelyPair.second
          goalZonePos1.remove(lonelyPos)
        } else if (goalZoneId != teamToAdd) {
          mergeGoalZones(group1, goalZoneId, teamToAdd, newMap)
        }
      }
      if (belongingToGoalZone) {
        // Add more free goal zone positions (not assigned to a goal zone) from allGoalZonePos
        // if these positions are neighbours now
        addFreeGoalZonePosToGoalZone(group1, lonelyPos, goalZoneId!!, newMap)
      } else {
        newMap[lonelyPos] = null to lonelyPair.second
      }
    }
    allGoalZonePos1.putAll(newMap)

    val agentsOfGroup2 = agentsOfGroup.getValue(group2)
    for (ag in agentsOfGroup2) {
      ag.group = group1
      ag.offset = (ag.offset + offPos).intoBounds(bounds)
      // We know who it is :)
      map1[ag.offset] = PositionInfoWithStep(PositionInfo.Agent(ag.name), step)
    }
    agentsOfGroup.getValue(group1).addAll(agentsOfGroup2)
    agentsOfGroup.remove(group2)
    mapOfGroup.remove(group2)
    rolePositionsOfGroup.remove(group2)
    markerPosOfGroup.remove(group2)
    goalZonesOfGroup.remove(group2)
    goalZoneCounter.remove(group2)
    goalZonePositionsOfGroup.remove(group2)
    allGoalZonePositions.remove(group2)
    dispenserOfGroup.remove(group2)
  }

  private fun updateMarkers() {
    // Try to remove stored markers based on the information in the map
    for ((groupId, markerMap) in markerPosOfGroup) {
      if (agentsOfGroup[groupId].isNullOrEmpty()) {
        continue
      }
      val mapOfCurrGroup = mapOfGroup.getValue(groupId)
      val posToRemove = mutableSetOf<Position>()
      for ((markerPos, markerInfo) in markerMap) {
        val markerPosInfo = mapOfCurrGroup[markerPos]
        if (markerPosInfo != null && markerPosInfo.lastStep > markerInfo.second) {
          posToRemove.add(markerPos)
        }
      }
      for (pos in posToRemove) {
        markerMap.remove(pos)
      }
    }
  }

  // Try to merge groups or determine a smaller size of the world
  private fun knowledgeExchange(step: Int) {
    // If agent 1 found out that the entity at pos is agent 2, agent 2 knows that agent 1 is at -pos
    // → store it for both
    val metAgents = mutableSetOf<Pair<String, Position>>()
    for (ag in agents.values) {
      val groupOfCurrentAgent = ag.group ?: continue
      val agPerceptsProcessed = agentPerceptsProcessed.getValue(ag.name)
      mapAgentVisibleAgents[ag.name]?.let {
        for (pos in it) {
          val mapOfCurrentAgent = mapOfGroup.getValue(groupOfCurrentAgent)
          val posInfo = mapOfCurrentAgent.getValue((pos + ag.offset).intoBounds(bounds))
          // If my group knows who it is OR the agent on this position already found out that I am at -pos, do nothing
          if (posInfo.info !is PositionInfo.Agent && (ag.name to pos) !in metAgents) {
            // The number of agents whose things match mine
            var numPosAgents = 0
            var varOtherAgent: Agent? = null
            for (otherAg in agents.values) {
              if (otherAg.name != ag.name) {
                agentPerceptsProcessed[otherAg.name]?.let { otherAgPercepts ->
                  val foundAgent = whoAmI(
                    pos,
                    agPerceptsProcessed.things,
                    otherAgPercepts.things,
                    roles.getValue(agPerceptsProcessed.role).vision,
                    roles.getValue(otherAgPercepts.role).vision
                  )
                  if (foundAgent) {
                    ++numPosAgents
                    varOtherAgent = otherAg
                  }
                }
              }
            }
            // If more than one agent could be the visible one → do not merge the groups
            if (numPosAgents == 1) {
              val otherAgent = varOtherAgent!!
              metAgents.add(ag.name to pos)
              metAgents.add(otherAgent.name to Position(-pos.x, -pos.y))
              // Merge the groups
              mergeGroups(ag, otherAgent, pos, step)
            }
          }
        }
      }
      // Clear it for the next step
      mapAgentVisibleAgents[ag.name] = mutableListOf()
    }
  }

  // Look for the given position whether it is part of a goal zone
  // Returns whether a goal zone has been found: The result, i.e. of which goal cells the goal zone consists,
  // is in alreadyConsidered, which is empty at the beginning
  private fun searchGoalZone(
    currPos: Position,
    alreadyConsidered: MutableMap<Position, Int>,
    groupId: Int,
    goalPositionsOfGroup: MutableMap<Position, Int>
  ): Boolean {
    if (currPos in alreadyConsidered.keys) {
      return true
    }
    val currMap = mapOfGroup.getValue(groupId)
    val goalPosStep = goalPositionsOfGroup[currPos] ?: return currPos in currMap
    alreadyConsidered[currPos] = goalPosStep
    var answer = true
    for (nPos in currPos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
      if (nPos !in currMap) {
        // This part has not been explored by an agent of the group yet
        return false
      }
      answer = answer && searchGoalZone(nPos, alreadyConsidered, groupId, goalPositionsOfGroup)
    }
    return answer
  }

  // Try to find goal zones consisting of currently free goal cells (stored in goalZonePositionsOfGroup)
  private fun findGoalZones(step: Int) {
    for ((groupId, positions) in goalZonePositionsOfGroup) {
      val consideredPos = mutableSetOf<Position>()
      val toBeRemoved = mutableSetOf<Position>()
      for (pos in positions.keys) {
        if (pos in consideredPos) {
          // Already looked at it
          continue
        }
        val alreadyConsideredForPos = mutableMapOf<Position, Int>()
        if (searchGoalZone(pos, alreadyConsideredForPos, groupId, positions)) {
          val goalZonesCurrGroup = goalZonesOfGroup.getValue(groupId)
          val newId = goalZoneCounter.getValue(groupId)
          goalZoneCounter[groupId] = newId + 1
          goalZonesCurrGroup[newId] = GoalZone(
            newId,
            alreadyConsideredForPos,
            mutableSetOf(),
            groupId,
            false,
            mutableSetOf(),
            mutableMapOf(),
            null,
            null,
            null,
            null,
            mutableMapOf(),
            mutableMapOf(),
            null,
            mutableMapOf()
          )
          // Update the "missing position" and "explored" attribute of the goal zone
          val missingPos = mutableSetOf<Position>()
          val onePos = alreadyConsideredForPos.keys.iterator().next()
          for (currPos in onePos.neighboursAtMost(1).map { it.intoBounds(bounds) }) {
            // Look whether it is in the map
            if (currPos in mapOfGroup.getValue(groupId)) {
              continue
            }
            missingPos.add(currPos)
          }
          val newGoalZone = goalZonesCurrGroup.getValue(newId)
          if (missingPos.isNotEmpty()) {
            newGoalZone.missingPositions.addAll(missingPos)
            newGoalZone.explored = false
          } else {
            newGoalZone.missingPositions.clear()
            newGoalZone.explored = true
          }
          determineAttributes(newGoalZone, step)
          for (posOfNewZone in alreadyConsideredForPos.keys) {
            val currAllGoalZonePos = allGoalZonePositions.getValue(groupId)
            val oldStep = currAllGoalZonePos.getValue(posOfNewZone).second
            currAllGoalZonePos[posOfNewZone] = newId to oldStep
            toBeRemoved.add(posOfNewZone)
          }
        }
        consideredPos.addAll(alreadyConsideredForPos.keys)
      }
      for (posToRem in toBeRemoved) {
        positions.remove(posToRem)
      }
    }
  }

  // Determine whether an agent has a block attached which is on position pos
  private fun isBlockUnattached(pos: Position, group: Int): Boolean {
    for (groupAg in agentsOfGroup.getValue(group)) {
      for (attachedThing in groupAg.attachedThings) {
        if ((groupAg.offset + attachedThing).intoBounds(bounds) == pos) {
          return false
        }
      }
    }
    return true
  }

  // Determine a shortest path for the agent to go to any target cell
  fun getShortestPath(ag: Agent, targetCells: Set<Position>): Action {
    val timerShortestPath = Timer()
    val clearProb = roles.getValue(ag.currRole!!).clearChance
    val currGroup = ag.group!!
    val map = mapOfGroup.getValue(currGroup)
    val dispensersMap = dispenserOfGroup.getValue(currGroup)
    val markerPos = markerPosOfGroup.getValue(currGroup)
    if (markerPos.isNotEmpty()) {
      log.printFlush { "$markerPos" }
    }
    val processedPer = agentPerceptsProcessed.getValue(ag.name)
    // Create a graph for the grid the agent knows
    val wGraph = SimpleDirectedWeightedGraph<Position, DefaultWeightedEdge>(DefaultWeightedEdge::class.java)
    // Add the vertices
    for (pos in map.keys) {
      wGraph.addVertex(pos)
    }
    timerShortestPath.printReset(log, "Time after adding Vertices")
    // Add the edges
    for (pos in map.keys) {
      for (neighbourPos in pos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
        val neighbourInfo = map[neighbourPos] ?: continue
        if (dispensersMap.values.any { neighbourPos in it } && neighbourPos !in targetCells) {
          continue
        }
        var weight = 1
        val markerPair = markerPos[neighbourPos]
        if (markerPair != null && markerPair.first != "clear_perimeter") {
          weight = 10
        }
        val obstacle = neighbourInfo.info is PositionInfo.Obstacle
        val unattachedBlock = neighbourInfo.info is PositionInfo.Block && isBlockUnattached(neighbourPos, currGroup)
        if (obstacle || unattachedBlock) {
          if (pos == neighbourPos) {
            log.printFlush {
              "pos=$pos, neighbourPos = $neighbourPos, bestWidth = $bestWidth, bestHeight=$bestHeight, agent=$ag, targetCells = $targetCells"
            }
          }
          wGraph.setEdgeWeight(wGraph.addEdge(pos, neighbourPos), ceil(1.0 / clearProb) + 1)
        } else if (neighbourInfo.info is PositionInfo.Empty) {
          wGraph.setEdgeWeight(wGraph.addEdge(pos, neighbourPos), weight.toDouble())
        }
        // Do not add an edge otherwise (dispenser, agent or entity)
      }
    }
    for (targetPos in targetCells) {
      if (!wGraph.containsVertex(targetPos)) {
        wGraph.addVertex(targetPos)
        for (pos in map.keys.filter { currPos ->
          currPos.neighboursExactly(1).any { it.intoBounds(bounds) !in map }
        }) {
          wGraph.setEdgeWeight(wGraph.addEdge(pos, targetPos), 2 * distanceBounded(pos, targetPos, bounds).toDouble())
        }
      }
    }
    timerShortestPath.printReset(log, "Time for adding the edges")
    // Look at the positions which the agents can go to and choose the "closest" one
    var foundPath = false
    var bestPath = mutableListOf<Position>()
    var bestWeight = Double.POSITIVE_INFINITY
    // The offset is the start vertex
    val dijk = DijkstraShortestPath(wGraph).getPaths(ag.offset)
    timerShortestPath.printReset(log, "Time after DijkstraShortestPath.getPaths, num: ${targetCells.size}")
    for (targetPos in targetCells) {
      val currPath = dijk.getPath(targetPos) ?: continue
      val currWeight = currPath.weight
      foundPath = true
      if (currWeight < bestWeight) {
        bestPath = currPath.vertexList
        bestWeight = currWeight
      }
    }
    timerShortestPath.printReset(log, "Time for loop")
    if (foundPath) {
      val info = map.getValue(bestPath[1]).info
      if (info is PositionInfo.Obstacle || info is PositionInfo.Block) {
        val posToClear = bestPath[1] - bestPath[0]
        return Action("clear", Numeral(posToClear.x), Numeral(posToClear.y))
      }
      // Get the number of steps the agent can do in one step
      val roleSpeed = roles.getValue(processedPer.role).speed
      val numAttachedThings = ag.attachedThings.size
      if (numAttachedThings >= roleSpeed.size) {
        // The agent carries too much to move → it will not work
        return Action("skip")
      }
      val steps = mutableListOf<Parameter>()
      var numSteps = 0
      val maxStep = roleSpeed[numAttachedThings]
      for (i in 1 until bestPath.size) {
        if (numSteps == maxStep || map.getValue(bestPath[i]).info is PositionInfo.Obstacle) {
          break
        }
        val direction = subClosest(bestPath[i], bestPath[i - 1], bounds)
        val updateDir = { c: Int, dim: Int? ->
          var newVal = c
          if (dim != null) {
            if (c < -1) {
              newVal += dim
            } else if (c > 1) {
              newVal -= dim
            }
          }
          newVal
        }
        val newDir = Position(updateDir(direction.x, bestWidth), updateDir(direction.y, bestHeight))
        newDir.toParam()?.let { steps.add(it) }
        ++numSteps
      }
      timerShortestPath.printReset(log, "Time for getting the action")
      return Action("move", steps)
    } else {
      // TODO change default behaviour
      log.printFlush { "Default behaviour!!!!!!!!" }
      timerShortestPath.printReset(log, "Time for getting the action")
      return Action("skip")
    }
  }

  // Determine the cells from targetCells the agent can reach (by looking whether there is a shortest path)
  private fun getReachableTargetCells(ag: Agent, targetCells: Set<Position>): MutableSet<Position> {
    val timerGetReachableTargetCells = Timer()
    val currGroup = ag.group!!
    val map = mapOfGroup.getValue(currGroup)
    val dispensersMap = dispenserOfGroup.getValue(currGroup)
    // Create a graph for the grid the agent knows
    val wGraph = SimpleDirectedGraph<Position, DefaultEdge>(DefaultEdge::class.java)
    // Add the vertices
    for (pos in map.keys) {
      wGraph.addVertex(pos)
    }
    timerGetReachableTargetCells.printReset(log, "Time after adding Vertices in getReachableTargetCells")
    val reachableCells = mutableSetOf<Position>()
    // Add the edges
    for (pos in map.keys) {
      for (neighbourPos in pos.neighboursExactly(1).map { it.intoBounds(bounds) }) {
        val neighbourInfo = map[neighbourPos] ?: continue
        if (dispensersMap.values.any { it.contains(neighbourPos) }) {
          continue
        }
        val obstacle = neighbourInfo.info is PositionInfo.Obstacle
        val unattachedBlock = neighbourInfo.info is PositionInfo.Block && isBlockUnattached(neighbourPos, currGroup)
        val empty = neighbourInfo.info is PositionInfo.Empty
        if (obstacle || unattachedBlock || empty) {
          wGraph.addEdge(pos, neighbourPos)
        }
      }
    }
    timerGetReachableTargetCells.printReset(log, "Time for adding the edges")
    val dijk = DijkstraShortestPath(wGraph).getPaths(ag.offset)
    timerGetReachableTargetCells.printReset(log, "Time after DijkstraShortestPath.getPaths, num: ${targetCells.size}")
    for (targetPos in targetCells) {
      dijk.getPath(targetPos) ?: continue
      reachableCells.add(targetPos)
    }
    timerGetReachableTargetCells.printReset(log, "Time for loop")
    return reachableCells
  }

  // Determine the next action for the given agent
  fun step(ag: Agent) {
    val timerStep = Timer()
    log.printFlush { "step: $ag" }
    val actionToPerform = run {
      when (ag.phase) {
        Phase.GO_TO_ROLE_ZONE -> {
          log.printFlush { "getShortestPath in step()" }
          try {
            getShortestPath(ag, rolePositionsOfGroup.getValue(ag.group!!))
          } catch (_: Exception) {
            Action("skip")
          }
        }

        Phase.GET_DESIRED_ROLE -> {
          Action("adopt", Identifier(ag.desiredRole))
        }

        Phase.DETACH_THINGS -> {
          var detachThing: Position = ag.attachedThings.iterator().next()
          for (pos in ag.attachedThings) {
            if (pos.norm == 1) {
              detachThing = pos
              break
            }
          }
          Action("detach", detachThing.toParam())
        }

        Phase.RESTART_DETACH_NORTH -> {
          Action("detach", Position(0, -1).toParam())
        }

        Phase.RESTART_DETACH_EAST -> {
          Action("detach", Position(1, 0).toParam())
        }

        Phase.RESTART_DETACH_SOUTH -> {
          Action("detach", Position(0, 1).toParam())
        }

        Phase.RESTART_DETACH_WEST -> {
          Action("detach", Position(-1, 0).toParam())
        }

        else -> {
          if (ag.isConstructor) {
            log.printFlush { "Skip" }
            return@run Action("skip")
          }
          val group = ag.group!!
          val agMap = mapOfGroup.getValue(group)
          val roleOfAg = agentPerceptsProcessed.getValue(ag.name).role
          val stepDist = roles.getValue(roleOfAg).speed
          val numAttachedThings = ag.attachedThings.size
          log.printFlush { "new: ${ag.name}, $stepDist, $numAttachedThings" }
          if (numAttachedThings >= stepDist.size) {
            log.printFlush { "Skip" }
            return@run Action("skip")
          }
          val currPosNorth = (Position(0, -1) + ag.offset).intoBounds(bounds)
          val infoNorth = agMap.getValue(currPosNorth)
          val obstacle = infoNorth.info is PositionInfo.Obstacle
          val unattachedBlock = infoNorth.info is PositionInfo.Block && isBlockUnattached(currPosNorth, group)
          if (obstacle || unattachedBlock) {
            return@run Action("clear", Numeral(0), Numeral(-1))
          }
          var xDiff = -1
          val numSteps = stepDist[numAttachedThings]
          val steps = buildList {
            for (i in 0 until numSteps) {
              val currPos = (Position(0, xDiff) + ag.offset).intoBounds(bounds)
              if (currPos !in agMap) {
                break
              }
              if (dispenserOfGroup.getValue(group).values.any { it.contains(currPos) }) {
                break
              }
              val info = agMap.getValue(currPos).info
              if (info is PositionInfo.Empty) {
                add(Identifier("n"))
              } else {
                break
              }
              --xDiff
            }
          }
          if (steps.isEmpty()) {
            return@run Action("skip")
          }
          log.printFlush { "move: $steps" }
          return@run Action("move", steps)
        }
      }
    }
    timerStep.printReset(log, "Time in step for $ag")
    performAction(ag) { actionToPerform }
  }

  // Perform the action of the given agent
  fun performAction(ag: Agent, generateAction: () -> Action?) {
    agentPerceptsProcessed[ag.name]?.let {
      generateAction()?.let {
        try {
          log.printFlush { "performAction agent ${ag.name}, action $it" }
          eis.performAction(ag.name, it)
        } catch (e: ActException) {
          println("Could not perform action ${it.name} for ${ag.name}")
        }
      }
    }
    // To clear it for the next step
    agentPerceptsProcessed.remove(ag.name)
  }
}
