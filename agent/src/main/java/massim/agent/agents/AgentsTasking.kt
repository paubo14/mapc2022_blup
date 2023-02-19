package massim.agent.agents

import com.google.ortools.sat.CpSolverStatus
import cpsat.tasking.*
import eis.iilang.Action
import eis.iilang.Identifier
import eis.iilang.Numeral
import eis.iilang.Parameter
import massim.agent.AgentManager
import massim.agent.OUTPUT_JSON
import massim.agent.PositionInfo
import massim.agent.printFlush
import massim.agent.toParam
import massim.common.Position
import java.io.File
import java.io.PrintWriter
import kotlin.math.min
import kotlin.system.measureTimeMillis

// TODO Change to "" if playing a "real" game
const val prefix = "agent"

fun Rotation.getIdentifier(): Parameter = when (this) {
  Rotation.CLOCKWISE -> Identifier("cw")
  Rotation.ANTICLOCKWISE -> Identifier("ccw")
}

fun AgentManager.optimizedTasking(step: Int, log: PrintWriter, numAllOptimizationsAgents: Int): MutableSet<Agent> {
  // In the end it contains the agents for which performAction was called in
  // this function (i.e. agents for which an action has been chosen)
  val returnOptimizationAgents = mutableSetOf<Agent>()
  // group → the agents of the group which are working on a goal zone and are therefore part of the optimization problem
  val onGoalZoneWorkingAgent = buildMap<Int, MutableList<Agent>> {
    for ((groupId, goalZones) in goalZonesOfGroup) {
      log.printFlush { "goalZones of group $groupId" }
      for (goalZone in goalZones.values) {
        if (goalZone.task == null || goalZone.constructorAgent == null) {
          continue
        }
        log.printFlush { "GoalZone: ${goalZone.agents}, $goalZone" }
        val workingAgents =
          goalZone.agents.map { agName -> agents.getValue(agName) }.filter { it.phase == Phase.WORK_ON_GOAL_ZONE }
        if (workingAgents.isNotEmpty()) {
          val agentsUntilNow = getOrPut(groupId) { mutableListOf() }
          agentsUntilNow.addAll(workingAgents)
        }
      }
    }
  }

  for ((groupId, groupAgents) in onGoalZoneWorkingAgent) {
    if (groupAgents.isEmpty()) {
      continue
    }
    returnOptimizationAgents.addAll(groupAgents)
    log.printFlush { "groupAgents: $groupAgents" }
    val map = mapOfGroup.getValue(groupId)
    val dispOfGroup = dispenserOfGroup.getValue(groupId)
    // Ignore a block in cellInfos if it is on a dispenser or if it is attached
    // (CellInfo.Empty has to be stored in this case)
    val positionsToIgnore = mutableSetOf<Position>()
    for (posSet in dispOfGroup.values) {
      positionsToIgnore.addAll(posSet)
    }
    val attachedBlockPositions = mutableSetOf<Position>()
    for (ag in groupAgents) {
      for (pos in ag.attachedThings) {
        attachedBlockPositions.add((ag.offset + pos).intoBounds(bounds))
      }
    }
    positionsToIgnore.addAll(attachedBlockPositions)
    // Create the information about the cells, which is part of the input of the optimization problem
    val cellInfos = map.asSequence().filter { it.key !in positionsToIgnore }
      .associateTo(mutableMapOf()) { (pos, info) ->
        pos to when (info.info) {
          is PositionInfo.Empty, is PositionInfo.Dispenser -> CellType.EMPTY
          is PositionInfo.Obstacle, is PositionInfo.Block -> CellType.MUTABLE_OBSTACLE
          is PositionInfo.Entity, is PositionInfo.Other -> CellType.FIXED_OBSTACLE
          is PositionInfo.Agent -> {
            if (agents.getValue(info.info.name) in groupAgents) {
              CellType.EMPTY
            } else {
              CellType.FIXED_OBSTACLE
            }
          }
        }
      }
    for (pos in positionsToIgnore) {
      cellInfos[pos] = CellType.EMPTY
    }

    // Create the information about the constructors and their constructor cells,
    // which is part of the input of the optimization problem
    val constructorAgents = groupAgents.filter { ag -> ag.isConstructor }
    val constructorsInfoList = constructorAgents.map { ag ->
      // Information about the constructor cells
      val positionsInfos = ag.goalZone!!.task!!.req.map { (pos, type) ->
        PositionInfo((pos + ag.offset).intoBounds(bounds), ConstructorCellInfo(type, pos in ag.attachedThings))
      }
      // Information about the constructor
      val agRole = roles.getValue(agentPerceptsProcessed.getValue(ag.name).role)
      PositionInfo(
        ag.offset, ConstructorInfo(agRole.clearMaxDistance, agRole.clearChance, positionsInfos)
      )
    }
    // All agents apart from the constructor are allowed to move in principle
    val moveAgents = groupAgents.filter { !it.isConstructor }

    // Create the information about the dispenser
    val dispenserInfoList = dispenserOfGroup.getValue(groupId).flatMap { (type, posSet) ->
      posSet.map { pos ->
        val freeBlockOn = map.getValue(pos).info is PositionInfo.Block && pos !in attachedBlockPositions
        PositionInfo(pos, DispenserInfo(type, freeBlockOn))
      }
    }

    // Create the information about the move agents (workers and diggers)
    val moveAgentsInfoList = moveAgents.map { ag ->
      val agPercepts = agentPerceptsProcessed.getValue(ag.name)
      val agRole = roles.getValue(agPercepts.role)
      val numAttached = ag.attachedThings.size
      val stepDist = if (numAttached < agRole.speed.size) agRole.speed[numAttached] else 0
      if (agPercepts.role == "digger") {
        PositionInfo(
          ag.offset, MoveAgentInfo.Digger(
            agRole.vision,
            stepDist,
            agRole.clearMaxDistance,
            agRole.clearChance,
            ag.getFlock(agRole.clearMaxDistance, bounds)
          ) as MoveAgentInfo
        )
      } else {
        val minNorm = normBlock.minOrNull()
        // If a norm allows to carry only one block, carry one block, otherwise two blocks
        val maxBlockAttached = if (minNorm != null && minNorm > 0) min(minNorm, 2) else 2
        // Determine the status of a worker
        val status = if (ag.workerStatus == null) {
          if (ag.attachedThings.size == maxBlockAttached) WorkerStatus.DELIVERER else WorkerStatus.GATHERER
        } else {
          if (ag.workerStatus == WorkerStatus.DELIVERER) {
            if (ag.attachedThings.size == 0) WorkerStatus.GATHERER else WorkerStatus.DELIVERER
          } else {
            if (ag.attachedThings.size < maxBlockAttached) WorkerStatus.GATHERER else WorkerStatus.DELIVERER
          }
        }
        ag.workerStatus = status
        log.printFlush { "Attached things agent ${ag.name}: ${ag.attachedThings}" }
        val attachedSides = ag.attachedThings.asSequence().mapNotNull { it.direction }.toSet()
        PositionInfo(
          ag.offset, MoveAgentInfo.Worker(status,
            agRole.vision,
            stepDist,
            agRole.clearMaxDistance,
            agRole.clearChance,
            maxBlockAttached,
            ag.blockType!!,
            constructorAgents.indexOfFirst { constrAg -> ag.team == constrAg.team },
            attachedSides,
            dispenserInfoList.indexOfFirst { posInfo -> posInfo.position == ag.dispenserPos!! }) as MoveAgentInfo
        )
      }
    }
    val clearPos = markerPosOfGroup.getValue(groupId).keys
    val problemInfo = ProblemInfo(
      cellInfos.map { PositionInfo(it.key, it.value) },
      moveAgentsInfoList,
      constructorsInfoList,
      dispenserInfoList,
      clearPos,
      bounds,
    )
    if (OUTPUT_JSON) {
      // Store the information about the problem instance in a JSON file for debugging
      File("log/tasking_problem_${step}_${team}_${groupId}.json").outputStream().use {
        problemInfo.encode(it)
      }
    }

    val problem: Problem
    // Simulate only the next 3 actions when the team consists of 40 agents,
    // otherwise the next 2 actions to respect the timeout
    val numSimulatedSteps = if (teamSize > 20) 4 else 5
    val timeProblem = measureTimeMillis {
      problem = Problem(
        cellInfos,
        moveAgentsInfoList,
        constructorsInfoList,
        dispenserInfoList,
        clearPos,
        bounds,
        numSimulatedSteps,
        10,
      )
    }
    log.printFlush { "AgentsTasking problem time: ${timeProblem.toDouble() / 1000} s" }
    val seconds = 3 * groupAgents.size.toDouble() / numAllOptimizationsAgents.toDouble()
    log.printFlush { "3 * ${groupAgents.size} / $numAllOptimizationsAgents = $seconds" }
    val solution: Problem.Solution
    val time = measureTimeMillis { solution = problem.solve(log = false, maxSeconds = seconds) }
    log.printFlush {
      "status: ${solution.status}, AgentsTasking solve time: ${time.toDouble() / 1000} s, groupId: $groupId"
    }
    when (solution.status) {
      CpSolverStatus.FEASIBLE, CpSolverStatus.OPTIMAL -> {
        // The action of the agent is based on the variables which refer to the agent
        // and which are true in the first simulated step
        val (moveAgentActions, constructorActions) = solution.agentActions
        log.printFlush { "actions: $moveAgentActions, $constructorActions" }
        // Determine the actions of all workers and diggers
        for ((agIndex, ag) in moveAgents.withIndex()) {
          val action = when (val action = moveAgentActions[agIndex]) {
            is AgentAction.Clear -> {
              Action("clear", Numeral(action.offset.x), Numeral(action.offset.y))
            }

            is AgentAction.Move -> {
              Action("move", action.offsets.mapNotNull(Position::toParam))
            }

            is AgentAction.Request -> {
              Action("request", action.offset.toParam())
            }

            is AgentAction.Attach -> {
              Action("attach", action.offset.toParam())
            }

            is AgentAction.Detach -> {
              Action("detach", action.offset.toParam())
            }

            is AgentAction.Connect -> {
              val name = constructorAgents[action.agentIdx].name
              Action(
                "connect", listOf(
                  Identifier("$prefix$name"), Numeral(action.offset.x), Numeral(action.offset.y)
                )
              )
            }

            is AgentAction.Rotate -> {
              Action("rotate", action.rotate.getIdentifier())
            }

            is AgentAction.Submit -> {
              throw Exception("A worker cannot submit")
            }

            AgentAction.Nothing -> {
              Action("skip")
            }
          }
          log.printFlush { "Tasking problem ${ag.name}: Action is not null, perform action $action" }
          performAction(ag) { action }
        }
        
        // Determine the next action of each constructor
        for ((agIndex, ag) in constructorAgents.withIndex()) {
          val action = when (val action = constructorActions[agIndex]) {
            is AgentAction.Attach -> {
              Action("attach", action.offset.toParam())
            }

            is AgentAction.Connect -> {
              val name = moveAgents[action.agentIdx].name
              ag.newAttachedThingAfterConnect = action.offsetNewAttached!!
              Action(
                "connect", listOf(
                  Identifier("$prefix$name"), Numeral(action.offset.x), Numeral(action.offset.y)
                )
              )
            }

            is AgentAction.Submit -> {
              Action("submit", Identifier(ag.goalZone!!.taskName!!))
            }

            is AgentAction.Nothing -> Action("skip")

            is AgentAction.Clear -> Action("clear", Numeral(action.offset.x), Numeral(action.offset.y))

            else -> throw Exception("A constructor can not do $action, name: ${ag.name}")
          }
          log.printFlush { "Tasking problem ${ag.name}: Action is not null, perform action $action" }
          performAction(ag) { action }
        }
      }
      // The solution is neither optimal nor feasible
      else -> {
        log.printFlush { "Tasking problem ${step}, ${groupAgents}, the solution is neither optimal nor feasible" }
        for (ag in groupAgents) {
          step(ag)
        }
      }
    }
  }
  return returnOptimizationAgents
}
