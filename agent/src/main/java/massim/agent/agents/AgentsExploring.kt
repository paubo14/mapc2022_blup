package massim.agent.agents

import com.google.ortools.sat.CpSolverStatus
import cpsat.exploring.*
import eis.iilang.Action
import eis.iilang.Numeral
import massim.agent.*
import massim.common.Position
import java.io.File
import java.io.PrintWriter
import kotlin.math.max
import kotlin.system.measureTimeMillis

fun AgentManager.pathAction(ag: Agent, cells: Set<Position>) {
  // If there is an exception when trying to follow a shortest path to the given cells,
  // the step function is called
  try {
    performAction(ag) { getShortestPath(ag, cells) }
  } catch (_: Exception) {
    step(ag)
  }
}


fun AgentManager.optimizedExplore(step: Int, log: PrintWriter, numAllOptimizationsAgents: Int) {
  // group → the agents of the group which are still searching for role or goal zones and which therefore
  // should be part of the optimization problem
  val searchingAgents = agentsOfGroup.asSequence().associate { (groupId, groupAgents) ->
    groupId to groupAgents.filter {
      it.phase == Phase.SEARCH_ROLE_ZONE || it.phase == Phase.SEARCH_MAP
    }
  }

  for ((groupId, groupAgents) in searchingAgents) {
    if (groupAgents.isEmpty()) {
      continue
    }
    val map = mapOfGroup.getValue(groupId)
    // Create the information about the cells which are part of the input to the optimization problem
    val cellInfos = map.asSequence().associateTo(mutableMapOf()) { (pos, info) ->
      pos to CellInfo(
        when (info.info) {
          is PositionInfo.Empty, is PositionInfo.Agent -> CellType.EMPTY
          is PositionInfo.Obstacle, is PositionInfo.Block -> CellType.MUTABLE_OBSTACLE
          is PositionInfo.Entity, is PositionInfo.Dispenser, is PositionInfo.Other -> CellType.FIXED_OBSTACLE
        }
      )
    }
    // Dispensers are fixed obstacles that the agents cannot move onto
    // (so that they do not block agents trying to request a block)
    for (pos in dispenserOfGroup.getValue(groupId).values.asSequence().flatten()) {
      cellInfos[pos] = CellInfo(CellType.FIXED_OBSTACLE)
    }
    val clearPos = markerPosOfGroup.getValue(groupId).keys
    // Store information about the cells and the agents taking part in the optimization problem
    val problemInfo = ProblemInfo(
      cellInfos, groupAgents.map { ag: Agent ->
        val agRole = roles.getValue(agentPerceptsProcessed.getValue(ag.name).role)
        AgentPosInfo(
          ag.offset, AgentInfo(agRole.vision, agRole.speed.first(), agRole.clearMaxDistance, agRole.clearChance)
        )
      }, clearPos, bounds
    )
    if (OUTPUT_JSON) {
      // Store the information about the problem instance in a JSON file for debugging
      File("log/exploring_problem_${step}_${team}_${groupId}.json").outputStream().use {
        problemInfo.jsonInfo.encode(it)
      }
    }

    val goalZonesNeighbourPos = allGoalZonePositions.getValue(groupId).keys.flatMap { goalPos ->
      goalPos.neighboursAtMost(10).map { it.intoBounds(bounds) }.filter { it !in map }
    }.toSet()
    var argGoalPos: Set<Position>? = null
    if (goalZonesNeighbourPos.isNotEmpty()) {
      argGoalPos = goalZonesNeighbourPos
    }
    val problem: Problem
    // Simulate only the next action when the team consists of 40 agents, otherwise the two next actions
    // to respect the timeout
    val timeStepsToSimulate = if (teamSize > 20) 2 else 3
    val timeProblem = measureTimeMillis {
      problem = Problem(
        problemInfo.cells, problemInfo.agents, clearPos, bounds, timeStepsToSimulate, 10, argGoalPos
      )
    }
    log.printFlush { "AgentsExploring problem time: ${timeProblem.toDouble() / 1000} s" }
    // The timeout for the CP-SAT solver to respect the timout of 4s
    val seconds = 1.75 * groupAgents.size.toDouble() / numAllOptimizationsAgents.toDouble()
    log.printFlush { "1.75 * ${groupAgents.size} / $numAllOptimizationsAgents = $seconds" }
    val solution: Problem.Solution
    val time = measureTimeMillis { solution = problem.solve(log = false, maxSeconds = seconds) }
    log.printFlush {
      "status: ${solution.status}, AgentsExploring solve time: ${time.toDouble() / 1000} s, step: $step, groupId: $groupId"
    }
    val unseenTargetCells by lazy {
      map.keys.asSequence().filter { pos ->
        pos.neighboursExactly(1).any { it.intoBounds(bounds) !in map }
      }.toSet()
    }

    val smallestStep by lazy { map.values.minOf { it.lastStep } }

    val oldTargetCells by lazy {
      val oldStep = max(smallestStep, step - 30)
      map.keys.asSequence().filter { map.getValue(it).lastStep <= oldStep }.toSet()
    }

    when (solution.status) {
      CpSolverStatus.FEASIBLE, CpSolverStatus.OPTIMAL -> {
        // The action of the agent is based on the variables which refer to the agent and which are true
        // in the first simulated step
        val actions = solution.agentActions

        for ((agIndex, ag) in groupAgents.withIndex()) {
          val action = when (val action = actions[agIndex]) {
            is AgentAction.Clear -> {
              Action("clear", Numeral(action.offset.x), Numeral(action.offset.y))
            }

            is AgentAction.Move -> {
              Action("move", action.offsets.mapNotNull(Position::toParam))
            }

            AgentAction.Nothing -> null
          }
          if (action != null) {
            log.printFlush { "${ag.name} AgentsExploring: Action is not null, perform action $action" }
            performAction(ag) { action }
          } else {
            // The agent performs no action in the solution of the optimization problem
            log.printFlush { "${ag.name} AgentsExploring: Action is null, getShortestPath" }
            // Follow a shortest path to a cell that has an unseen adjacent cell, if there are any,
            // otherwise to a cell which has been seen a "long" time ago
            pathAction(ag, unseenTargetCells.ifEmpty { oldTargetCells })
          }
        }
      }
      // The solution is neither optimal nor feasible
      else -> {
        // Follow a shortest path to a cell that has an unseen adjacent cell, if there are any,
        // otherwise to a cell which has been seen a "long" time ago
        for (ag in groupAgents) {
          log.printFlush { "getShortestPath in AgentsExploring because solution neither optimal nor feasible" }
          pathAction(ag, unseenTargetCells.ifEmpty { oldTargetCells })
        }
      }
    }
  }
}