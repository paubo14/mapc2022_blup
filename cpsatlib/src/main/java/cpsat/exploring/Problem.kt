package cpsat.exploring

import com.google.ortools.sat.*
import cpsat.*
import massim.common.Bounds
import massim.common.Position
import massim.common.subClosest
import kotlin.math.roundToLong

class Problem(
  private val seenCells: Map<Position, CellInfo>,
  private val agents: List<AgentPosInfo>,
  private val clearCells: Set<Position>,
  private val bounds: Bounds,
  private val timeSteps: Int,
  maxAmount: Int,
  private val goalRegion: Set<Position>? = null
) {
  private fun Position.neighboursAtMostBounded(distance: Int) = neighboursAtMost(distance).map { it.intoBounds(bounds) }
  private fun Position.neighboursLessBounded(distance: Int) = neighboursLess(distance).map { it.intoBounds(bounds) }

  // Agents can only go to cells that have already been seen
  private val candidateCells = agents.maxOf { it.info.vision }.let { maxVision ->
    seenCells.flatMapTo(mutableSetOf()) { (p, _) -> p.neighboursAtMostBounded(maxVision) }.apply {
      removeAll(seenCells.keys)
    }
  }

  private val model = CpModel()

  data class CellTime(val pos: Position, val timeStep: Int) {
    val suffix
      get() = "${pos.x}_${pos.y}_$timeStep"
  }

  data class AgentTime(val agentIndex: Int, val timeStep: Int) {
    val suffix
      get() = "${agentIndex}_$timeStep"

    fun addPos(pos: Position) = CellAgentTime(pos, agentIndex, timeStep)
  }

  data class AgentSubTime(val agentIndex: Int, val timeStep: Int, val subStep: Int) {
    fun addPos(pos: Position) = CellAgentSubTime(pos, agentIndex, timeStep, subStep)

    val suffix
      get() = "${agentIndex}_${timeStep}_${subStep}"
  }

  private val AgentSubTime.fullTimeStep
    get() = (timeStep - 1) * agents[agentIndex].info.stepDist + subStep

  data class CellAgentTime(val pos: Position, val agentIndex: Int, val timeStep: Int) {
    val suffix
      get() = "${pos.x}_${pos.y}_${agentIndex}_$timeStep"
  }

  data class CellAgentSubTime(
    val pos: Position, val agentIndex: Int, override val timeStep: Int, override val subStep: Int
  ) : SubTimeable<CellAgentSubTime> {
    override fun withSubTimeStep(timeStep: Int, subStep: Int) = copy(timeStep = timeStep, subStep = subStep)

    fun withPos(pos: Position) = copy(pos = pos)
    fun next(stepDist: Int) =
      if (subStep == stepDist) copy(timeStep = timeStep + 1, subStep = 1) else copy(subStep = subStep + 1)
  }

  private val CellAgentSubTime.next
    get() = nextSubTimeStep(agents[agentIndex].info)

  private fun subSteps(stepDist: Int, timeStep: Int) = if (timeStep == timeSteps) 1..1 else 1..stepDist

  private fun timeRange(excludeFirst: Boolean, excludeLast: Boolean) =
    1 + excludeFirst.toInt()..timeSteps - excludeLast.toInt()

  private fun subTimeSteps(
    stepDist: Int, excludeFirst: Boolean = false, excludeLast: Boolean = false
  ) = timeRange(false, excludeLast).asSequence().flatMap { t ->
    subSteps(stepDist, t).map { SubTimeStep(t, it) }
  }.let { r -> if (excludeFirst) r.drop(1) else r }

  private fun subTimeSteps(agInfo: AgentInfo, excludeFirst: Boolean = false, excludeLast: Boolean = false) =
    subTimeSteps(agInfo.stepDist, excludeFirst, excludeLast)

  private val isOnVars: Map<CellAgentSubTime, Literal>
  private val clearVars: Map<CellAgentTime, Literal>

  private val moveVars: Map<AgentSubTime, BoolVar>

  // F in the paper
  private val fullFactor = agents.asSequence().flatMap { agInfo ->
    (1 until timeSteps).asSequence().flatMap { t ->
      subSteps(agInfo.info.stepDist, t).map { (t - 1) * agInfo.info.stepDist + it }
    }
  }.sum() + agents.size * (1 until timeSteps).sum()

  init {
    CpLoader.load()

    // Create IsOn (AgOn)
    isOnVars = buildMap {
      for ((agIdx, agPosInfo) in agents.withIndex()) {
        val (agPos, agInfo) = agPosInfo
        for ((t, s) in subTimeSteps(agInfo, excludeFirst = false, excludeLast = false)) {
          if (t == 1 && s == 1) {
            // The agent is on its initial position at the beginning of the simulation
            put(CellAgentSubTime(agPos, agIdx, t, s), model.trueLiteral())
            continue
          }

          val key = AgentSubTime(agIdx, t, s)
          val cells = agPos.neighboursLessBounded(key.fullTimeStep).filter { p ->
            seenCells[p]?.let { it.cellType != CellType.FIXED_OBSTACLE } == true
          }.toMutableSet()
          // Exclude clear cells (i.e. marked cells)
          if (agPos !in clearCells) cells.removeAll(clearCells)

          for (p in cells) {
            put(key.addPos(p), model.newBoolVar("is_on_${key.suffix}"))
          }
        }
      }
    }

    // Create Clear
    clearVars = buildMap {
      for ((agIndex, agPosInfo) in agents.withIndex()) {
        val (agPos, agInfo) = agPosInfo
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val dist = agInfo.stepDist * (t - 1) + agInfo.clearDist
          for (pos in agPos.neighboursAtMostBounded(dist)) {
            val cellInfo = seenCells[pos] ?: continue
            if (cellInfo.cellType == CellType.MUTABLE_OBSTACLE) continue

            val key = CellAgentTime(pos, agIndex, t)
            put(key, model.newBoolVar("clear_${key.suffix}"))
          }
        }
      }
    }

    // Create New
    val newSeenVars = buildMap {
      for (canPos in candidateCells) {
        put(canPos, model.newBoolVar("new_${canPos.x}_${canPos.y}"))
      }
    }
    // Constraint 1.1
    for ((canPos, varNew) in newSeenVars) {
      val listIsOn = buildList {
        for ((agIndex, agInfo) in agents.withIndex()) {
          for (pos in canPos.neighboursAtMostBounded(agInfo.info.vision)) {
            if (pos !in seenCells) continue
            for (t in timeRange(excludeFirst = true, excludeLast = false)) {
              val varIsOn = isOnVars[CellAgentSubTime(pos, agIndex, t, 1)] ?: continue
              add(varIsOn)
            }
          }
        }
      }
      model.addOrEquality(varNew, listIsOn.asSequence())
    }

    // Create Move
    moveVars = buildMap {
      for ((agIndex, agPosInfo) in agents.withIndex()) {
        for ((t, s) in subTimeSteps(agPosInfo.info, excludeFirst = false, excludeLast = true)) {
          val key = AgentSubTime(agIndex, t, s)
          put(key, model.newBoolVar("move_${key.suffix}"))
        }
      }
    }
    // Constraint 1.2
    for ((info, varMove) in moveVars) {
      model.addOrAndEqualityExactlyOne(varMove, seenCells.keys.asSequence().mapNotNull { pos ->
        val key = info.addPos(pos)
        val varOn1 = isOnVars[key] ?: return@mapNotNull null
        val varOn2 = isOnVars.getValue(key.next)
        varOn1 to !varOn2
      })
    }

    // Create ClearAny
    val clearAnyVars = buildMap {
      for (agIdx in agents.indices) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val key = AgentTime(agIdx, t)
          put(key, model.newBoolVar("clear_any_${key.suffix}"))
        }
      }
    }
    // Constraint 1.3
    for ((info, varClearAny) in clearAnyVars) {
      val seq = sequenceOf(!varClearAny) + seenCells.keys.asSequence().mapNotNull { p -> clearVars[info.addPos(p)] }
      model.addExactlyOne(seq)
    }

    // Create ClearAmount (ClearedAmount)
    val clearAmountVars = buildMap {
      for (pos in seenCells.keys) {
        // Exclude first: nothing can be cleared in the first time step
        for (t in timeRange(excludeFirst = true, excludeLast = false)) {
          val key = CellTime(pos, t)
          // 2 * MaxAmount - 1: The maximum that can occur due to a permitted combination of clears
          put(key, model.newIntVar(0, 2 * maxAmount.toLong() - 1, "clear_amount_${key.suffix}"))
        }
      }
    }
    // Constraint 1.4
    for (pos in seenCells.keys) {
      for (t in timeRange(excludeFirst = true, excludeLast = false)) {
        model.addEquality(clearAmountVars.getValue(CellTime(pos, t)), LinearExpr.newBuilder().apply {
          for ((agIdx, agPosInfo) in agents.withIndex()) {
            val clearProb = (maxAmount * agPosInfo.info.clearProb).roundToLong()
            for (prevT in 1 until t) {
              val varClear = clearVars[CellAgentTime(pos, agIdx, prevT)] ?: continue
              addTerm(varClear, clearProb)
            }
          }
        })
      }
    }

    // Constraint 2.1
    for ((agIdx, agPosInfo) in agents.withIndex()) {
      for ((t, s) in subTimeSteps(agPosInfo.info, excludeFirst = true, excludeLast = false)) {
        model.addExactlyOne(seenCells.keys.asSequence().mapNotNull { isOnVars[CellAgentSubTime(it, agIdx, t, s)] })
      }
    }

    // Constraint 2.2
    for (p in seenCells.keys) {
      for (t in timeRange(excludeFirst = true, excludeLast = false)) {
        model.addAtMostOne(agents.indices.asSequence().mapNotNull { isOnVars[CellAgentSubTime(p, it, t, 1)] })
      }
    }

    // Constraint 2.3
    for ((pos, cellInfo) in seenCells) {
      for ((agIdx, agPosInfo) in agents.withIndex()) {
        for ((t, s) in subTimeSteps(agPosInfo.info, excludeFirst = true, excludeLast = false)) {
          if (cellInfo.cellType != CellType.MUTABLE_OBSTACLE) continue

          val varIsOn = isOnVars[CellAgentSubTime(pos, agIdx, t, s)] ?: continue
          val varClearAmount = clearAmountVars[CellTime(pos, t)]

          if (varClearAmount != null) {
            val exprIsOn = LinearExpr.newBuilder().apply { addTerm(varIsOn, maxAmount.toLong()) }
            model.addLessOrEqual(exprIsOn, varClearAmount)
          } else {
            model.addBoolOr(!varIsOn)
          }
        }
      }
    }

    // Constraint 3.1
    for (pos in seenCells.keys) {
      for ((agIdx, agPosInfo) in agents.withIndex()) {
        // Exclude last: we cannot move in the last time step
        for ((t, s) in subTimeSteps(agPosInfo.info, excludeFirst = false, excludeLast = true)) {
          val key1 = CellAgentSubTime(pos, agIdx, t, s)
          val varIsOn1 = isOnVars[key1] ?: continue
          val cast2 = key1.next
          val seqIsOn2 = pos.neighboursAtMostBounded(1).mapNotNull { isOnVars[cast2.withPos(it)] }
          model.addBoolOr(sequenceOf(!varIsOn1) + seqIsOn2)
        }
      }
    }

    // Constraint 3.2
    for ((agIdx, agPosInfo) in agents.withIndex()) {
      val agInfo = agPosInfo.info
      for (t in timeRange(excludeFirst = false, excludeLast = true)) {
        for (s in 1 until agInfo.stepDist) {
          val varMove1 = moveVars.getValue(AgentSubTime(agIdx, t, s + 1))
          val varMove0 = moveVars.getValue(AgentSubTime(agIdx, t, s))
          model.addImplication(varMove1, varMove0)
        }
      }
    }

    // Constraint 3.3
    for (agIdx in agents.indices) {
      for (t in timeRange(excludeFirst = false, excludeLast = true)) {
        val varClear = clearAnyVars.getValue(AgentTime(agIdx, t))
        val varMove = moveVars.getValue(AgentSubTime(agIdx, t, 1))
        model.addImplication(varClear, !varMove)
      }
    }

    // Constraint 4.1
    for (pos in seenCells.keys) {
      for ((agIdx, agPosInfo) in agents.withIndex()) {
        val neighbours = pos.neighboursAtMostBounded(agPosInfo.info.clearDist)
        // For time step 1, clearVars only contains variables for cells that can be cleared immediately →
        // no constraint necessary
        for (t in timeRange(excludeFirst = true, excludeLast = false)) {
          val varClear = clearVars[CellAgentTime(pos, agIdx, t)] ?: continue
          val seqIsOn = neighbours.mapNotNull { isOnVars[CellAgentSubTime(it, agIdx, t, 1)] }
          model.addBoolOr(sequenceOf(!varClear) + seqIsOn)
        }
      }
    }

    // Constraint 4.2
    for (pos in seenCells.keys) {
      for (agIdx in agents.indices) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val varClear = clearVars[CellAgentTime(pos, agIdx, t)] ?: continue

          val lhs = LinearExpr.newBuilder().apply { addTerm(varClear, maxAmount.toLong()) }
          val rhs = LinearExpr.newBuilder().apply {
            add(2 * maxAmount.toLong())
            // Clearing in previous time steps
            if (t > 1) {
              addTerm(clearAmountVars.getValue(CellTime(pos, t)), -1)
            }
            // Clearing in the current time step by other agents
            for ((otherAgIdx, otherAgPosInfo) in agents.withIndex()) {
              if (otherAgIdx == agIdx) continue
              val otherVarClear = clearVars[CellAgentTime(pos, otherAgIdx, t)] ?: continue
              addTerm(otherVarClear, -(maxAmount * otherAgPosInfo.info.clearProb).roundToLong())
            }
          }

          model.addLessThan(lhs, rhs)
        }
      }
    }

    // Objective Function
    model.maximize(LinearExpr.newBuilder().apply {
      for ((pos, variable) in newSeenVars) {
        var weight = fullFactor.toLong()
        if (goalRegion != null && pos in goalRegion) {
          weight *= 2
        }
        addTerm(variable, weight)
      }
      for ((ast, variable) in moveVars) {
        val agInfo = agents[ast.agentIndex].info
        addTerm(!variable, ((ast.timeStep - 1) * agInfo.stepDist + ast.subStep).toLong())
      }
      for ((at, variable) in clearAnyVars) {
        addTerm(!variable, at.timeStep.toLong())
      }
    })
  }

  inner class Solution(private val solver: CpSolver, val status: CpSolverStatus) {
    val agentActions
      get() = agents.withIndex().map { (agIndex, agInfo) ->
        val initPos = agInfo.position
        val clearPos = seenCells.keys.find { p ->
          clearVars[CellAgentTime(p, agIndex, 1)]?.let { solver.booleanValue(it) } == true
        }
        if (clearPos != null) {
          return@map AgentAction.Clear(subClosest(clearPos, initPos, bounds))
        }

        var prev = initPos
        val offsets = (1..agInfo.info.stepDist).map { s ->
          val newPos = seenCells.keys.find { p ->
            val key = CellAgentSubTime(p, agIndex, 1, s).next(agInfo.info.stepDist)
            isOnVars[key]?.let { solver.booleanValue(it) } == true
          }!!
          val offset = subClosest(newPos, prev, bounds)
          prev = newPos
          offset
        }
        when {
          !offsets.first().isZero -> AgentAction.Move(offsets)
          else -> AgentAction.Nothing
        }
      }
  }

  fun solve(log: Boolean = true, maxSeconds: Double? = null): Solution {
    val solver = CpSolver()
    solver.parameters.apply {
      maxSeconds?.let { maxTimeInSeconds = it }
      logSearchProgress = log
      symmetryLevel = 1
      maxPresolveIterations = 1
    }
    val status = solver.solve(model)
    return Solution(solver, status)
  }
}
