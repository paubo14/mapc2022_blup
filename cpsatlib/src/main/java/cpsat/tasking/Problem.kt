package cpsat.tasking

import com.google.ortools.sat.*
import cpsat.*
import massim.common.*
import org.jgrapht.Graph
import org.jgrapht.alg.shortestpath.DijkstraManyToManyShortestPaths
import org.jgrapht.graph.DefaultWeightedEdge
import org.jgrapht.graph.SimpleDirectedWeightedGraph
import kotlin.math.max
import kotlin.math.roundToInt
import kotlin.math.roundToLong

val <T> T?.sequence
  get() = if (this != null) sequenceOf(this) else sequenceOf()

class Problem(
  private val seenCells: Map<Position, CellType>,
  private val moveAgents: List<PositionInfo<MoveAgentInfo>>,
  constructors: List<PositionInfo<ConstructorInfo>>,
  private val dispensers: List<PositionInfo<DispenserInfo>>,
  private val clearCells: Set<Position>,
  private val bounds: Bounds,
  private val timeSteps: Int,
  private val maxAmount: Int,
) {
  companion object {
    const val FORCE_DIGGER_ON_FLOCK = false
    const val PRINT_TIMES = false
  }

  private fun Timer.resetOptPrint(msg: String) {
    if (PRINT_TIMES) printReset(msg)
  }

  private val MoveAgentInfo.clearProbLong
    get() = (clearProb * maxAmount).roundToLong()

  private fun distanceBounded(pos1: Position, pos2: Position) = distanceBounded(pos1, pos2, bounds)

  private fun Position.neighboursExactlyBounded(distance: Int) =
    neighboursExactly(distance).map { it.intoBounds(bounds) }

  private fun Position.neighboursLessBounded(distance: Int) = neighboursLess(distance).map { it.intoBounds(bounds) }

  private fun Position.neighboursAtMostBounded(distance: Int) = neighboursAtMost(distance).map { it.intoBounds(bounds) }

  private infix fun Position.addBounded(other: Position) = (this + other).intoBounds(bounds)
  private infix fun Position.subBounded(other: Position) = (this - other).intoBounds(bounds)
  private infix fun Position.subClosest(other: Position) = subClosest(this, other, bounds)

  private val workersWithIdx = moveAgents.withIndex().mapNotNull { iv ->
    (iv.value.info as? MoveAgentInfo.Worker)?.let {
      IndexedValue(iv.index, PositionInfo(iv.value.position, it))
    }
  }

  private val connectedConstructorCells = buildSet {
    for ((_, conInfo) in constructors) {
      for ((cellPos, cellInfo) in conInfo.cells) {
        if (!cellInfo.occupied) continue
        val isConnected = workersWithIdx.any { (_, agInfo) ->
          if (distanceBounded(cellPos, agInfo.position) != 1) return@any false
          val offset = cellPos subClosest agInfo.position
          agInfo.info.attachedSides.any { offset == it.offset }
        }
        if (isConnected) add(cellPos)
      }
    }
  }

  // Do not mark constructor cells that are still connected to an agent as occupied
  private val constructorInfos = constructors.map { (cPos, cInfo) ->
    ConInfo(
      cPos,
      cInfo.clearDist,
      cInfo.clearProb,
      cInfo.cells.associate { (pos, info) ->
        pos to info.copy(occupied = info.occupied && pos !in connectedConstructorCells)
      },
    )
  }

  private val gathererSet = buildSet {
    for ((agIdx, agPosInfo) in workersWithIdx) {
      if (agPosInfo.info.status == WorkerStatus.GATHERER) add(agIdx)
    }
  }
  private val delivererSet = buildSet {
    for ((agIdx, agPosInfo) in workersWithIdx) {
      val info = agPosInfo.info
      if (info.status == WorkerStatus.DELIVERER && info.constrIdx in constructorInfos.indices) add(agIdx)
    }
  }

  private val availableDispensers = dispensers.filter {
    when (seenCells.getValue(it.position)) {
      CellType.EMPTY -> true
      CellType.MUTABLE_OBSTACLE -> throw Exception("A dispenser cannot be on a mutable obstacle!")
      CellType.FIXED_OBSTACLE -> false
    }
  }

  private val constructorMap = constructorInfos.associate { it.pos to it.cells }
  private val dispenserMap = dispensers.makeMap()

  private val visitableCells = seenCells.filter {
    when (it.value) {
      CellType.EMPTY, CellType.MUTABLE_OBSTACLE -> it.key !in constructorMap && it.key !in dispenserMap
      CellType.FIXED_OBSTACLE -> false
    }
  }
  private val blockableCells = seenCells.asSequence().mapNotNull { (pos, cellType) ->
    when (cellType) {
      CellType.EMPTY, CellType.MUTABLE_OBSTACLE -> if (pos !in constructorMap) pos else null
      CellType.FIXED_OBSTACLE -> null
    }
  }.toSet()
  private val obstacleCells = visitableCells.asSequence().mapNotNull {
    if (it.value == CellType.MUTABLE_OBSTACLE) it.key else null
  }.toSet()

  private val model = CpModel()

  data class CellTime(val pos: Position, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${pos.x}_${pos.y}_$timeStep"
  }

  data class AgentTime(val moveAgentIdx: Int, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${moveAgentIdx}_$timeStep"

    fun addPos(pos: Position) = CellAgentTime(pos, moveAgentIdx, timeStep)
    fun addDir(dir: Direction) = AgentDirTime(moveAgentIdx, dir, timeStep)
  }

  private val AgentTime.moveAgentInfo
    get() = moveAgents[moveAgentIdx].info

  data class AgentSubTime(val moveAgentIdx: Int, override val timeStep: Int, override val subStep: Int) : Suffixable,
    SubTimeable<AgentSubTime> {
    override val suffix
      get() = "${moveAgentIdx}_${timeStep}_${subStep}"

    fun addPos(pos: Position) = CellAgentSubTime(pos, moveAgentIdx, timeStep, subStep)
    override fun withSubTimeStep(timeStep: Int, subStep: Int) = copy(timeStep = timeStep, subStep = subStep)
  }

  private val AgentSubTime.fullTimeStep
    get() = (timeStep - 1) * moveAgents[moveAgentIdx].info.stepDist + subStep
  val AgentSubTime.next
    get() = nextSubTimeStep(moveAgents[moveAgentIdx].info)

  data class CellAgentTime(val pos: Position, val moveAgentIdx: Int, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${pos.x}_${pos.y}_${moveAgentIdx}_${timeStep}"
  }

  data class CellAgentSubTime(
    val pos: Position, val moveAgentIdx: Int, override val timeStep: Int, override val subStep: Int
  ) : Suffixable, SubTimeable<CellAgentSubTime> {
    val cellAgentTime
      get() = CellAgentTime(pos, moveAgentIdx, timeStep)
    override val suffix
      get() = "${pos.x}_${pos.y}_${moveAgentIdx}_${timeStep}_${subStep}"

    override fun withSubTimeStep(timeStep: Int, subStep: Int) = copy(timeStep = timeStep, subStep = subStep)

    fun withPos(pos: Position) = copy(pos = pos)
    fun next(stepDist: Int) =
      if (subStep == stepDist) copy(timeStep = timeStep + 1, subStep = 1) else copy(subStep = subStep + 1)
  }

  private fun CellAgentSubTime.withPosOffsetBounded(offset: Position) = copy(pos = pos addBounded offset)

  private val CellAgentSubTime.moveAgentInfo
    get() = moveAgents[moveAgentIdx].info
  private val CellAgentSubTime.isLast
    get() = timeStep == timeSteps
  private val CellAgentSubTime.next
    get() = nextSubTimeStep(moveAgentInfo)

  data class AgentDirTime(val moveAgentIdx: Int, val dir: Direction, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${moveAgentIdx}_${dir}_${timeStep}"
    val next
      get() = copy(timeStep = timeStep + 1)
    val prev
      get() = copy(timeStep = timeStep - 1)
  }

  data class AgentRotTime(val moveAgentIdx: Int, val rot: Rotation, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${moveAgentIdx}_${rot}_${timeStep}"
  }

  data class ConstrTime(val constrIdx: Int, val timeStep: Int) : Suffixable {
    fun addPos(pos: Position) = CellConstrTime(pos, constrIdx, timeStep)

    override val suffix: String
      get() = "${constrIdx}_${timeStep}"
  }

  data class CellConstrTime(val pos: Position, val constrIdx: Int, val timeStep: Int) : Suffixable {
    override val suffix
      get() = "${pos.x}_${pos.y}_${constrIdx}_${timeStep}"
    val next
      get() = copy(timeStep = timeStep + 1)
    val cellTime
      get() = CellTime(pos, timeStep)
    val constrTime
      get() = ConstrTime(constrIdx, timeStep)

    fun withConstr(constrIdx: Int) = copy(constrIdx = constrIdx)
  }

  private fun Suffixable.newBoolVar(name: String): BoolVar = model.newBoolVar("${name}_${suffix}")
  private fun Suffixable.newIntVar(name: String, lb: Long, ub: Long): IntVar =
    model.newIntVar(lb, ub, "${name}_${suffix}")

  private inline fun <reified T : Suffixable> T.withBoolVar(name: String): Pair<T, BoolVar> = this to newBoolVar(name)
  private inline fun <reified T : Suffixable> T.withIntVar(name: String, lb: Long, ub: Long): Pair<T, IntVar> =
    this to newIntVar(name, lb, ub)

  private fun subSteps(stepDist: Int, timeStep: Int) = if (timeStep == timeSteps) 1..1 else 1..stepDist

  private fun timeRange(excludeFirst: Boolean, excludeLast: Boolean) =
    1 + excludeFirst.toInt()..timeSteps - excludeLast.toInt()

  private fun subTimeSteps(
    stepDist: Int, excludeFirst: Boolean = false, excludeLast: Boolean = false
  ) = timeRange(false, excludeLast).asSequence().flatMap { t ->
    subSteps(stepDist, t).map { SubTimeStep(t, it) }
  }.let { r -> if (excludeFirst) r.drop(1) else r }

  private fun subTimeSteps(moveAgentInfo: MoveAgentInfo, excludeFirst: Boolean = false, excludeLast: Boolean = false) =
    subTimeSteps(moveAgentInfo.stepDist, excludeFirst, excludeLast)

  private fun subTimeStepNum(moveAgentInfo: MoveAgentInfo) = (timeSteps - 1) * moveAgentInfo.stepDist + 1

  private val moveAgentOnCells: Map<AgentSubTime, Set<Position>>
  private val conClearCells: List<Set<Position>>

  private val moveAgentOnVars: Map<CellAgentSubTime, Literal>
  private val freeBlockOnVars: Map<CellTime, Literal>
  private val attachedBlockDirVars: Map<AgentDirTime, Literal>
  private val constructorBlockOnVars: Map<CellConstrTime, Literal>

  // Actions
  private val moveClearVars: Map<CellAgentTime, BoolVar>
  private val conClearVars: Map<CellConstrTime, BoolVar>
  private val requestVars: Map<CellAgentTime, BoolVar>
  private val attachVars: Map<AgentDirTime, BoolVar>
  private val joinVars: Map<AgentDirTime, BoolVar>
  private val detachVars: Map<AgentDirTime, Literal>
  private val rotateVars: Map<AgentRotTime, BoolVar>
  private val submitVars: Map<ConstrTime, BoolVar>

  // Derived
  private val moveVars: Map<AgentSubTime, BoolVar>
  private val moveClearAnyVars: Map<AgentTime, BoolVar>
  private val conClearAnyVars: Map<ConstrTime, BoolVar>
  private val clearAmountVars: Map<CellTime, IntVar>
  private val attachedBlockOnVars: Map<CellAgentSubTime, BoolVar>
  private val anyWorkerActionVars: Map<AgentTime, BoolVar>

  data class ConInfo(
    val pos: Position, val clearDist: Int, val clearProb: Double, val cells: Map<Position, ConstructorCellInfo>
  )

  private val ConInfo.clearProbLong
    get() = (clearProb * maxAmount).roundToLong()

  init {
    CpLoader.load()
    val fullTimer = Timer()
    val timer = Timer()

    val moveAgentMap = moveAgents.withIndex().associate { (index, api) -> api.position to index }

    /*
     * Helper sets
     */
    // step → positions that any move agent or its attached blocks can reach up to t → the agents that can reach it
    val anyOnTimeCells = timeRange(excludeFirst = false, excludeLast = false).associateWith { t ->
      moveAgents.withIndex().asSequence().flatMap { (agIdx, agInfo) ->
        agInfo.position.neighboursAtMostBounded(AgentSubTime(agIdx, t, 1).fullTimeStep).filter { it in blockableCells }
          .map { agIdx to it }
      }.groupBySet({ it.second }, { it.first })
    }
    // The same for workers only
    val workerAnyOnTimeCells = timeRange(excludeFirst = false, excludeLast = false).associateWith { t ->
      workersWithIdx.asSequence().flatMap { (agIdx, agInfo) ->
        agInfo.position.neighboursAtMostBounded(AgentSubTime(agIdx, t, 1).fullTimeStep).filter { it in blockableCells }
          .map { agIdx to it }
      }.groupBySet({ it.second }, { it.first })
    }
    // Agent → sub-step → positions that the agent or its attached blocks can reach up to the sub-step
    val anyOnSubTimeCells = moveAgents.withIndex().associate { (agIdx, agInfo) ->
      agIdx to subTimeSteps(agInfo.info).associateWith { (t, s) ->
        agInfo.position.neighboursAtMostBounded(AgentSubTime(agIdx, t, s).fullTimeStep).filter { it in blockableCells }
          .toSet()
      }
    }

    /*
     * Main Variables
     */

    // Create MoveAgentOn (AgOn)
    val isOnInit = { cat: CellAgentTime -> moveAgentMap[cat.pos] == cat.moveAgentIdx }
    // Agent and sub-step → the set of positions the agent might be positioned on in the sub-step
    moveAgentOnCells = buildMap {
      for ((agIdx, agPosInfo) in moveAgents.withIndex()) {
        val (agPos, agInfo) = agPosInfo
        for ((t, s) in subTimeSteps(agInfo)) {
          val key = AgentSubTime(agIdx, t, s)
          val cells = agPos.neighboursLessBounded(key.fullTimeStep).filter {
            it in visitableCells
          }.toMutableSet().apply {
            // Exclude clear cells
            if (agPos !in clearCells) removeAll(clearCells)
          }
          put(key, cells)
        }
      }
    }
    moveAgentOnVars = buildMap {
      for ((info, positions) in moveAgentOnCells) {
        for (p in positions) {
          val cast = info.addPos(p)
          val agInfo = cast.moveAgentInfo
          if (FORCE_DIGGER_ON_FLOCK && agInfo is MoveAgentInfo.Digger && p !in agInfo.flock) continue

          if (!cast.isFirst) {
            put(cast, cast.newBoolVar("move_agent_on"))
          } else if (isOnInit(cast.cellAgentTime)) {
            put(cast, model.trueLiteral())
          }
        }
      }
    }

    // Create FreeBlockOn (FBOn)
    freeBlockOnVars = buildMap {
      for ((disPos, disInfo) in availableDispensers) {
        for (t in timeRange(excludeFirst = false, excludeLast = false)) {
          val key = CellTime(disPos, t)
          if (t != 1 && disPos in workerAnyOnTimeCells.getValue(t)) {
            put(key, key.newBoolVar("free_block_on"))
          } else if (disInfo.occupied) {
            put(key, model.trueLiteral())
          }
        }
      }
    }

    // Create AttachedBlockDir (ABDir)
    attachedBlockDirVars = buildMap {
      for ((agIdx, agInfo) in workersWithIdx) {
        for (d in Direction.values()) {
          for (t in timeRange(excludeFirst = false, excludeLast = false)) {
            val key = AgentDirTime(agIdx, d, t)
            if (t != 1) {
              put(key, key.newBoolVar("attached_block_dir"))
            } else if (d in agInfo.info.attachedSides) {
              put(key, model.trueLiteral())
            }
          }
        }
      }
    }

    // Create ConstructorBlockOn (CBOn)
    constructorBlockOnVars = buildMap {
      for ((conIdx, conInfo) in constructorInfos.withIndex()) {
        for ((cellPos, cellInfo) in conInfo.cells) {
          for (t in timeRange(excludeFirst = false, excludeLast = false)) {
            val key = CellConstrTime(cellPos, conIdx, t)
            if (t != 1) {
              put(key, key.newBoolVar("constructor_block_on"))
            } else if (cellInfo.occupied) {
              put(key, model.trueLiteral())
            }
          }
        }
      }
    }

    timer.resetOptPrint("main variables")

    /*
     * Action variables
     */

    // Create Clear
    val moveClearCellsBase = timeRange(excludeFirst = false, excludeLast = true).asSequence().map { t ->
      t to moveAgents.withIndex().asSequence().flatMap { (agIdx, agPosInfo) ->
        val (agInitPos, agInfo) = agPosInfo
        agInitPos.neighboursAtMostBounded((t - 1) * agInfo.stepDist + agInfo.clearDist).filter { it in obstacleCells }
          .map { agIdx to it }
      }
    }
    val moveClearCellsPos = moveClearCellsBase.map { (k, v) -> k to v.groupBy({ it.second }, { it.first }) }.toMap()
    val moveClearCellsAgent = moveClearCellsBase.map { (k, v) -> k to v.groupBy({ it.first }, { it.second }) }.toMap()
    moveClearVars = moveClearCellsPos.asSequence().flatMap { (t, posMap) ->
      posMap.asSequence().flatMap { (pos, agIdxs) ->
        agIdxs.asSequence().filter { it !in delivererSet }.map { CellAgentTime(pos, it, t) }
      }
    }.associate { it.withBoolVar("move_clear") }

    conClearCells = constructorInfos.map { conInfo ->
      conInfo.pos.neighboursAtMostBounded(conInfo.clearDist).filter { it in obstacleCells }.toSet()
    }
    conClearVars = timeRange(excludeFirst = false, excludeLast = true).asSequence().flatMap { t ->
      conClearCells.withIndex().asSequence().flatMap { (conIdx, conInfo) ->
        conInfo.asSequence().map { CellConstrTime(it, conIdx, t).withBoolVar("constructor_clear") }
      }
    }.toMap()

    val allClearCells = timeRange(excludeFirst = false, excludeLast = true).associateWith { t ->
      val moveSeq = moveClearCellsPos.getValue(t).keys.asSequence()
      val conSeq = conClearCells.asSequence().flatMap { it.asSequence() }
      (moveSeq + conSeq).toSet()
    }

    val gathererDispensers = buildMap {
      for (agIdx in gathererSet) {
        val agInfo = moveAgents[agIdx].infoAs<MoveAgentInfo.Worker>()
        val value = agInfo.dispenserIdx?.let { listOf(dispensers[it]) }
          ?: availableDispensers.filter { it.info.type == agInfo.blockType }
        put(agIdx, value)
      }
    }
    val gathererDispenserTimes = buildMap {
      for ((agIdx, agDispensers) in gathererDispensers) {
        val (agPos, agInfo) = moveAgents[agIdx]
        val firstStep = agDispensers.minOf {
          (max(distanceBounded(agPos, it.position) - 1, 0) divCeil agInfo.stepDist) + 1
        }
        if (firstStep >= timeSteps) continue
        put(agIdx, firstStep)
      }
    }

    // Create Request (Req)
    requestVars = buildMap {
      for ((agIdx, startTime) in gathererDispenserTimes) {
        for (t in startTime until timeSteps) {
          for ((disPos, _) in gathererDispensers.getValue(agIdx)) {
            if (workerAnyOnTimeCells.getValue(t)[disPos]?.contains(agIdx) != true) continue
            val key = CellAgentTime(disPos, agIdx, t)
            put(key, key.newBoolVar("request"))
          }
        }
      }
    }

    // Create Attach
    attachVars = buildMap {
      for ((agIdx, startTime) in gathererDispenserTimes) {
        for (t in startTime until timeSteps) {
          for (d in Direction.values()) {
            val key = AgentDirTime(agIdx, d, t)
            put(key, key.newBoolVar("attach"))
          }
        }
      }
    }

    val delivererConstructorTimes = buildMap {
      for (agIdx in delivererSet) {
        val (agPos, agInfo) = moveAgents[agIdx].withInfoAs<MoveAgentInfo.Worker>()
        val conInfo = constructorInfos[agInfo.constrIdx]
        val firstStep = conInfo.cells.keys.minOf {
          (max(distanceBounded(agPos, it) - 1, 0) divCeil agInfo.stepDist) + 1
        }
        if (firstStep >= timeSteps) continue
        put(agIdx, firstStep)
      }
    }

    // Create Join
    joinVars = buildMap {
      for ((agIdx, startTime) in delivererConstructorTimes) {
        for (t in startTime until timeSteps) {
          for (d in Direction.values()) {
            val key = AgentDirTime(agIdx, d, t)
            put(key, key.newBoolVar("join"))
          }
        }
      }
    }

    // Create Detach
    detachVars = buildMap {
      for ((agIdx, startTime) in delivererConstructorTimes) {
        val (agPos, agInfo) = moveAgents[agIdx].withInfoAs<MoveAgentInfo.Worker>()
        for (t in startTime until timeSteps) {
          for (d in Direction.values()) {
            val key = AgentDirTime(agIdx, d, t)
            if (t == 1) {
              // Ensure that the agent detaches if it is currently connected to a constructor cell
              val attached = d in agInfo.attachedSides
              val connected = agPos addBounded d.offset in connectedConstructorCells
              if (attached && connected) put(key, model.trueLiteral())
            } else {
              // Constraint 2.17: Use Join directly
              joinVars[key.prev]?.let { put(key, it) }
            }
          }
        }
      }
    }

    // Create Rotate (Rot)
    rotateVars = buildMap {
      for ((agIdx, _) in workersWithIdx) {
        for (r in Rotation.values()) {
          for (t in timeRange(excludeFirst = false, excludeLast = true)) {
            val key = AgentRotTime(agIdx, r, t)
            put(key, key.newBoolVar("rotate"))
          }
        }
      }
    }

    // Create Submit
    submitVars = constructors.indices.asSequence().flatMap { conIdx ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence().map { t ->
        ConstrTime(conIdx, t).withBoolVar("submit")
      }
    }.toMap()

    timer.resetOptPrint("action variables")

    /*
     * Derived
     */

    // Create Move
    moveVars = moveAgents.withIndex().asSequence().flatMap { (agIdx, agInfo) ->
      subTimeSteps(agInfo.info, excludeLast = true).map { (t, s) ->
        AgentSubTime(agIdx, t, s).withBoolVar("move")
      }
    }.toMap()
    // Constraint 1.1
    for ((info, varMove) in moveVars) {
      model.addOrAndEqualityExactlyOne(varMove, moveAgentOnCells.getValue(info).asSequence().mapNotNull {
        val key = info.addPos(it)
        val varOn1 = moveAgentOnVars[key] ?: return@mapNotNull null
        val varOn2 = moveAgentOnVars.getValue(key.next)
        varOn1 to !varOn2
      })
    }

    // Create ClearAny
    moveClearAnyVars = moveAgents.indices.asSequence().flatMap { agIdx ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence().map {
        AgentTime(agIdx, it).withBoolVar("move_clear_any")
      }
    }.toMap()
    conClearAnyVars = constructors.indices.asSequence().flatMap { conIdx ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence().map {
        ConstrTime(conIdx, it).withBoolVar("constructor_clear_any")
      }
    }.toMap()
    // Constraint 1.2
    for ((info, varClearAny) in moveClearAnyVars) {
      val seqMoveClear = moveClearCellsPos.getValue(info.timeStep).keys.asSequence().mapNotNull {
        moveClearVars[info.addPos(it)]
      }
      model.addExactlyOne(sequenceOf(!varClearAny) + seqMoveClear)
    }
    for ((info, varClearAny) in conClearAnyVars) {
      val seqConClear = conClearCells[info.constrIdx].asSequence().mapNotNull { conClearVars[info.addPos(it)] }
      model.addExactlyOne(sequenceOf(!varClearAny) + seqConClear)
    }

    // Create ClearAmount (ClearedAmount) only for mutable obstacles
    clearAmountVars = allClearCells.asSequence().flatMap { (t, positions) ->
      positions.asSequence().map { pos ->
        CellTime(pos, t + 1).withIntVar("clear_amount", 0, 2 * maxAmount.toLong() - 1)
      }
    }.toMap()
    // Constraint 1.3
    for ((info, varClearAmount) in clearAmountVars) {
      model.addEquality(varClearAmount, LinearExpr.newBuilder().apply {
        for (t in 1 until info.timeStep) {
          for ((agIdx, agInfo) in moveAgents.withIndex()) {
            val varClear = moveClearVars[CellAgentTime(info.pos, agIdx, t)] ?: continue
            addTerm(varClear, agInfo.info.clearProbLong)
          }
          for ((conIdx, conInfo) in constructorInfos.withIndex()) {
            val varClear = conClearVars[CellConstrTime(info.pos, conIdx, t)] ?: continue
            addTerm(varClear, conInfo.clearProbLong)
          }
        }
      })
    }

    // Create AttachedBlockOn (ABOn)
    // Worker and sub-step → potential attached block positions
    val attachedBlockOnCells = workersWithIdx.asSequence().flatMap { (agIdx, agInfo) ->
      subTimeSteps(agInfo.info).map { (t, s) ->
        val key = AgentSubTime(agIdx, t, s)
        key to moveAgents[agIdx].position.neighboursAtMostBounded(key.fullTimeStep).filter {
          it in blockableCells
        }.toList()
      }
    }.toMap()
    attachedBlockOnVars = attachedBlockOnCells.asSequence().flatMap { (info, positions) ->
      positions.asSequence().map { info.addPos(it) }
    }.associate { it.withBoolVar("attached_block_on") }.toMap()
    // Constraint 1.4
    for ((info, varAttachedBlockOn) in attachedBlockOnVars) {
      model.addOrAndEqualityAtMostOne(varAttachedBlockOn, Direction.values().asSequence().mapNotNull { d ->
        val varMoveAgentOn = moveAgentOnVars[info.withPosOffsetBounded(-d.offset)] ?: return@mapNotNull null
        val varAttachedBlockDir =
          attachedBlockDirVars[AgentDirTime(info.moveAgentIdx, d, info.timeStep)] ?: return@mapNotNull null
        varMoveAgentOn to varAttachedBlockDir
      })
    }

    // Create AnyOn
    // Constraint 1.5
    val anyOn = { cell: Position, agIdx: Int, timeStep: Int, subStep: Int ->
      val key = CellAgentSubTime(cell, agIdx, timeStep, subStep)
      val seqMoveAgentOn = moveAgentOnVars[key].sequence
      when (moveAgents[agIdx].info) {
        is MoveAgentInfo.Worker -> seqMoveAgentOn + attachedBlockOnVars[key].sequence
        is MoveAgentInfo.Digger -> seqMoveAgentOn
      }
    }

    // Create AnyStepAction (AnyStepAct)
    // For workers
    anyWorkerActionVars = workersWithIdx.asSequence().flatMap { (agIdx, _) ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence()
        .map { t -> AgentTime(agIdx, t).withBoolVar("any_worker_action") }
    }.toMap()
    // Constraint 1.6a
    for ((info, varAnyWorkerAction) in anyWorkerActionVars) {
      val dispenserSeq = availableDispensers.filter {
        it.info.type == (info.moveAgentInfo as MoveAgentInfo.Worker).blockType
      }.mapNotNull { requestVars[info.addPos(it.position)] }
      val baseSeq = sequenceOf(!varAnyWorkerAction, moveClearAnyVars.getValue(info))
      val dirSeq = Direction.values().asSequence().flatMap { d ->
        val key = info.addDir(d)
        attachVars[key].sequence + joinVars[key].sequence + detachVars[key].sequence
      }
      val rotSeq = Rotation.values().asSequence().map {
        rotateVars.getValue(AgentRotTime(info.moveAgentIdx, it, info.timeStep))
      }
      model.addExactlyOne(baseSeq + dispenserSeq + dirSeq + rotSeq)
    }

    // Constraint 1.6c: Just Clear for diggers

    // Combined for move agents
    val anyMoveAgentAction = { agIdx: Int, timeStep: Int ->
      val key = AgentTime(agIdx, timeStep)
      when (moveAgents[agIdx].info) {
        is MoveAgentInfo.Worker -> anyWorkerActionVars.getValue(key)
        is MoveAgentInfo.Digger -> moveClearAnyVars.getValue(key)
      }
    }

    // For constructors
    val anyConstrActionVars = constructorInfos.indices.asSequence().flatMap { cIdx ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence().map { t ->
        ConstrTime(cIdx, t).withBoolVar("any_constructor_action")
      }
    }.toMap()
    // Constraint 1.6b
    for ((info, varAnyConstrAction) in anyConstrActionVars) {
      val baseSeq = sequenceOf(!varAnyConstrAction, submitVars.getValue(info), conClearAnyVars.getValue(info))
      val workerSeq = workersWithIdx.asSequence().filter {
        it.value.info.constrIdx == info.constrIdx
      }.flatMap { (agIdx, _) ->
        Direction.values().asSequence().flatMap { d ->
          val key = AgentDirTime(agIdx, d, info.timeStep)
          joinVars[key].sequence + detachVars[key].sequence
        }
      }
      model.addExactlyOne(baseSeq + workerSeq)
    }

    // Create FullyLoaded (FuLo)
    val fullyLoadedVars = workersWithIdx.asSequence().filter { it.index in gathererSet }.flatMap { (agIdx, _) ->
      timeRange(excludeFirst = false, excludeLast = false).asSequence().map { t ->
        AgentTime(agIdx, t).withBoolVar("fully_loaded")
      }
    }.toMap()
    // Constraint 1.7
    for ((info, varFullyLoaded) in fullyLoadedVars) {
      val workerInfo = info.moveAgentInfo as MoveAgentInfo.Worker
      val maxAttached = workerInfo.maxAttached.toLong()
      val sum = LinearExpr.newBuilder().apply {
        Direction.values().forEach { d -> attachedBlockDirVars[info.addDir(d)]?.let { add(it) } }
      }
      model.addEquality(sum, maxAttached).onlyEnforceIf(varFullyLoaded)
      model.addLessThan(sum, maxAttached).onlyEnforceIf(!varFullyLoaded)
    }

    // Create AnyLoaded (AnyLo)
    val anyLoadedVars = workersWithIdx.asSequence().filter { it.index in delivererSet }.flatMap { (agIdx, _) ->
      timeRange(excludeFirst = false, excludeLast = false).asSequence().map { t ->
        AgentTime(agIdx, t).withBoolVar("any_loaded")
      }
    }.toMap()
    // Constraint 1.8
    for ((info, varAnyLoaded) in anyLoadedVars) {
      model.addOrEquality(
        varAnyLoaded,
        Direction.values().asSequence().mapNotNull { attachedBlockDirVars[info.addDir(it)] },
      )
    }

    // Create DispenserAttach (DisAttach)
    val dispenserAttachVars = availableDispensers.asSequence().flatMap { (disPos, disInfo) ->
      workersWithIdx.asSequence().filter { (_, posInfo) ->
        posInfo.info.status == WorkerStatus.GATHERER && posInfo.info.blockType == disInfo.type
      }.flatMap { (agIdx, _) ->
        timeRange(excludeFirst = false, excludeLast = true).asSequence().map { t ->
          CellAgentTime(disPos, agIdx, t).withBoolVar("dispenser_attach")
        }
      }
    }.toMap()
    // Constraint 1.9
    for ((info, varDisAttach) in dispenserAttachVars) {
      model.addOrAndEqualityAtMostOne(varDisAttach, Direction.values().asSequence().mapNotNull { dir ->
        val pos = info.pos subBounded dir.offset
        if (pos !in visitableCells) return@mapNotNull null

        val key1 = CellAgentSubTime(pos, info.moveAgentIdx, info.timeStep, 1)
        val varMoveAgentOn = moveAgentOnVars[key1] ?: return@mapNotNull null

        val key2 = AgentDirTime(info.moveAgentIdx, dir, info.timeStep)
        val varAttach = attachVars[key2] ?: return@mapNotNull null

        varMoveAgentOn to varAttach
      })
    }

    timer.resetOptPrint("derived variables")

    /*
     * Position-related constraints
     */

    // Constraint 2.1: Each agent is on exactly one cell at all times
    for ((agIdx, agInfo) in moveAgents.withIndex()) {
      for ((t, s) in subTimeSteps(agInfo.info)) {
        model.addExactlyOne(moveAgentOnCells.getValue(AgentSubTime(agIdx, t, s)).mapNotNull {
          moveAgentOnVars[CellAgentSubTime(it, agIdx, t, s)]
        })
      }
    }

    // Constraint 2.2: At most one agent or block is on each cell
    for (t in timeRange(excludeFirst = false, excludeLast = false)) {
      for ((pos, posAgs) in anyOnTimeCells.getValue(t)) {
        val seqFreeBlockOn = freeBlockOnVars[CellTime(pos, t)].sequence
        val seqAnyOn = posAgs.asSequence().flatMap { agIdx -> anyOn(pos, agIdx, t, 1) }
        val seqConstructorBlockOn = constructorInfos.withIndex().asSequence().mapNotNull { (conIdx, _) ->
          constructorBlockOnVars[CellConstrTime(pos, conIdx, t)]
        }
        val vars = seqFreeBlockOn + seqAnyOn + seqConstructorBlockOn
        if (vars.moreThanSingle()) model.addAtMostOne(vars)
      }
    }

    // Constraint 2.3: An agent or block can only be on an obstacle if it is fully cleared already
    for ((pos, agIdx, t, s) in anyOnTimeCells.asSequence().flatMap { (t, posMap) ->
      posMap.asSequence().filter { it.key in obstacleCells }.flatMap { (pos, agIdxs) ->
        agIdxs.asSequence().flatMap { agIdx ->
          subSteps(moveAgents[agIdx].info.stepDist, t).asSequence().map { s -> CellAgentSubTime(pos, agIdx, t, s) }
        }
      }
    }) {
      val varClearAmount = clearAmountVars[CellTime(pos, t - 1)]
      val seqAnyOn = anyOn(pos, agIdx, t, s)
      if (seqAnyOn.isEmpty()) continue
      val exprIsOn = LinearExpr.newBuilder().apply { seqAnyOn.forEach { addTerm(it, maxAmount.toLong()) } }

      if (varClearAmount != null) model.addLessOrEqual(exprIsOn, varClearAmount)
      else model.addBoolAnd(seqAnyOn.map { !it })
    }
    for ((conIdx, conInfo) in constructorInfos.withIndex()) {
      for (cellPos in conInfo.cells.keys) {
        if (cellPos !in obstacleCells) continue
        for (t in timeRange(excludeFirst = false, excludeLast = false)) {
          val varConstructorBlockOn = constructorBlockOnVars[CellConstrTime(cellPos, conIdx, t)] ?: continue
          val exprConstructorBlockOn = LinearExpr.newBuilder().apply {
            addTerm(varConstructorBlockOn, maxAmount.toLong())
          }
          val varClearAmount = clearAmountVars[CellTime(cellPos, t - 1)]
          if (varClearAmount != null) model.addLessOrEqual(exprConstructorBlockOn, varClearAmount)
          else model.addBoolOr(!varConstructorBlockOn)
        }
      }
    }

    // Constraint 2.4: If a cell is occupied by an agent or its attached blocks, it cannot be occupied by another agent
    // in the next step
    // For move agents
    for (t in timeRange(excludeFirst = false, excludeLast = true)) {
      for ((pos, agents) in anyOnTimeCells.getValue(t)) {
        for (agIdx in agents) {
          val seqAnyOn0 = anyOn(pos, agIdx, t, 1)
          val agents1 = anyOnTimeCells.getValue(t + 1).getValue(pos)
          val seqAnyOn1 = agents1.asSequence().filter { it != agIdx }.flatMap { anyOn(pos, it, t + 1, 1) }
          model.addAtMostOne(seqAnyOn0 + seqAnyOn1)
        }
      }
    }
    // For constructors
    for (t in timeRange(excludeFirst = false, excludeLast = true)) {
      for ((conIdx, conInfo) in constructorInfos.withIndex()) {
        for (pos in conInfo.cells.keys) {
          val varConBlockOn0 = constructorBlockOnVars[CellConstrTime(pos, conIdx, t)] ?: continue
          val agents1 = anyOnTimeCells.getValue(t + 1)[pos] ?: continue
          val seqAnyOn1 = agents1.asSequence().flatMap { anyOn(pos, it, t + 1, 1) }
          // Constructor cells should not overlap!
          model.addAtMostOne(sequenceOf(varConBlockOn0) + seqAnyOn1)
        }
      }
    }

    // Constraint 2.5: No agent may move onto other agents or their attached blocks between steps
    for ((agIdx, subTimeMap) in anyOnSubTimeCells) {
      for ((subTime, cells) in subTimeMap) {
        val (tBase, s) = subTime
        if (s == 1) continue
        for (pos in cells) {
          val seqAnyOnSub = anyOn(pos, agIdx, tBase, s)
          for (t in sequenceOf(tBase, tBase + 1)) {
            val agentsT = anyOnTimeCells.getValue(t)[pos]?.asSequence() ?: emptySequence()
            val seqAnyOnT = agentsT.filter { it != agIdx }.flatMap { anyOn(pos, it, t, 1) }
            val seqConBlockOnT = constructorInfos.indices.asSequence().flatMap {
              constructorBlockOnVars[CellConstrTime(pos, it, t)].sequence
            }
            val seq0 = seqAnyOnSub + seqAnyOnT + seqConBlockOnT
            if (seq0.moreThanSingle()) model.addAtMostOne(seq0)
          }
        }
      }
    }

    timer.resetOptPrint("position-related constraints")

    /*
     * Move-related constraints
     */

    // Constraint 3.1: One cell can be traversed per sub-step
    for ((vaft1, isOn1) in moveAgentOnVars) {
      if (vaft1.isLast) continue
      val vaft2 = vaft1.next
      val seqOnVars = vaft1.pos.neighboursAtMostBounded(1).mapNotNull { moveAgentOnVars[vaft2.withPos(it)] }
      model.addOrImplication(isOn1, seqOnVars)
    }

    // Constraint 3.2: If an agent moves in the next sub-step, it has to move in this one, too
    for (ats in moveAgents.withIndex().flatMap { (agIdx, agInfo) ->
      timeRange(excludeFirst = false, excludeLast = true).asSequence().flatMap { t ->
        (1 until agInfo.info.stepDist).asSequence().map { s -> AgentSubTime(agIdx, t, s) }
      }
    }) {
      model.addImplication(moveVars.getValue(ats.next), moveVars.getValue(ats))
    }

    // Constraint 3.3: An agent can only move if it does not do anything else in that step
    for (vafs in moveAgents.withIndex().asSequence().flatMap { (agIdx, agInfo) ->
      subTimeSteps(agInfo.info, excludeLast = true).map { (t, s) -> AgentSubTime(agIdx, t, s) }
    }) {
      model.addImplication(anyMoveAgentAction(vafs.moveAgentIdx, vafs.timeStep), !moveVars.getValue(vafs))
    }

    timer.resetOptPrint("move-related constraints")

    /*
     * Clear-related constraints
     */

    // Constraint 4.1 and 4.2 for move agents
    for (t in timeRange(excludeFirst = false, excludeLast = true)) {
      for ((pos, agents) in moveClearCellsPos.getValue(t)) {
        for (agIdx in agents) {
          val varClear = moveClearVars[CellAgentTime(pos, agIdx, t)] ?: continue

          // Constraint 4.1: A cell can only be cleared if the agent is close enough
          val seqIsOn = pos.neighboursAtMostBounded(moveAgents[agIdx].info.clearDist).mapNotNull {
            moveAgentOnVars[CellAgentSubTime(it, agIdx, t, 1)]
          }
          model.addOrImplication(varClear, seqIsOn)

          // Constraint 4.2: A cell can only be cleared if the cell is not fully cleared already
          val lhs = LinearExpr.newBuilder().apply { addTerm(varClear, maxAmount.toLong()) }
          val rhs = LinearExpr.newBuilder().apply {
            add(2 * maxAmount.toLong())
            // Clearing in previous steps
            if (t > 1) {
              clearAmountVars[CellTime(pos, t)]?.let { addTerm(it, -1) }
            }
            // Clearing in the current step by other agents
            for ((otherAgIdx, otherAgPosInfo) in moveAgents.withIndex()) {
              if (otherAgIdx == agIdx) continue
              val varOtherClear = moveClearVars[CellAgentTime(pos, otherAgIdx, t)] ?: continue
              addTerm(varOtherClear, -otherAgPosInfo.info.clearProbLong)
            }
            for ((conIdx, conInfo) in constructorInfos.withIndex()) {
              val varOtherClear = conClearVars[CellConstrTime(pos, conIdx, t)] ?: continue
              addTerm(varOtherClear, -conInfo.clearProbLong)
            }
          }
          model.addLessThan(lhs, rhs)
        }
      }
    }

    // Constraint 4.1 is automatically fulfilled for constructors, as they cannot move and clear variables
    // are only created for cells they can clear

    // Constraint 4.2 for constructors
    for (cct in timeRange(excludeFirst = false, excludeLast = true).asSequence().flatMap { t ->
      conClearCells.withIndex().asSequence().flatMap { (conIdx, positions) ->
        positions.asSequence().map { pos -> CellConstrTime(pos, conIdx, t) }
      }
    }) {
      val varClear = conClearVars.getValue(cct)
      val lhs = LinearExpr.newBuilder().apply { addTerm(varClear, maxAmount.toLong()) }
      val rhs = LinearExpr.newBuilder().apply {
        add(2 * maxAmount.toLong())
        // Clearing in previous steps
        if (cct.timeStep > 1) {
          clearAmountVars[cct.cellTime]?.let { addTerm(it, -1) }
        }
        // Clearing in the current step by other agents
        for ((agIndex, agInfo) in moveAgents.withIndex()) {
          val varOtherClear = moveClearVars[CellAgentTime(cct.pos, agIndex, cct.timeStep)] ?: continue
          addTerm(varOtherClear, -agInfo.info.clearProbLong)
        }
        for ((conIdx, conInfo) in constructorInfos.withIndex().asSequence().filter { it.index != cct.constrIdx }) {
          val varOtherClear = conClearVars[cct.withConstr(conIdx)] ?: continue
          addTerm(varOtherClear, -conInfo.clearProbLong)
        }
      }
      model.addLessThan(lhs, rhs)
    }

    // Constraint 4.3: An agent can only clear if it has nothing connected to it
    // For move agents
    for ((agIdx, _) in workersWithIdx) {
      for (d in Direction.values()) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val varAttachedBlockDir = attachedBlockDirVars[AgentDirTime(agIdx, d, t)] ?: continue
          model.addImplication(moveClearAnyVars.getValue(AgentTime(agIdx, t)), !varAttachedBlockDir)
        }
      }
    }
    // For constructors
    for ((conIdx, conInfo) in constructorInfos.withIndex()) {
      for (pos in conInfo.cells.keys) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val varConBlockOn = constructorBlockOnVars[CellConstrTime(pos, conIdx, t)] ?: continue
          model.addImplication(conClearAnyVars.getValue(ConstrTime(conIdx, t)), !varConBlockOn)
        }
      }
    }

    timer.resetOptPrint("clear-related constraints")

    /*
     * Attach-related constraints
     */

    // Constraint 5.1: A block cannot be attached on a given side if its resulting position is not blockable
    for ((agIdx, agInfo) in workersWithIdx) {
      for ((t, s) in subTimeSteps(agInfo.info)) {
        for (pos in agInfo.position.neighboursLessBounded(AgentSubTime(agIdx, t, s).fullTimeStep)
          .filter { it in visitableCells }) {
          for (dir in Direction.values().filter { dir -> pos addBounded dir.offset !in blockableCells }) {
            val varMoveAgentOn = moveAgentOnVars[CellAgentSubTime(pos, agIdx, t, s)] ?: continue
            val varAttachedBlockDir = attachedBlockDirVars[AgentDirTime(agIdx, dir, t)] ?: continue
            model.addImplication(varMoveAgentOn, !varAttachedBlockDir)
          }
        }
      }
    }

    // Constraint 5.2: An agent can only request a block from a dispenser if the agent is adjacent to the dispenser
    // and the latter is free
    for ((info, varReq) in requestVars) {
      val (disPos, agIdx, t) = info

      val varFreeBlockOn = freeBlockOnVars[CellTime(disPos, t)]
      if (varFreeBlockOn != null) {
        model.addImplication(varReq, !varFreeBlockOn)
      }

      for ((otherAgIdx, _) in workersWithIdx) {
        val seqAnyOn = anyOn(info.pos, otherAgIdx, info.timeStep, 1)
        if (seqAnyOn.isEmpty()) continue
        model.addBoolAnd(seqAnyOn.map { !it }).onlyEnforceIf(varReq)
      }

      val seqMoveAgentOn = disPos.neighboursExactlyBounded(1).filter { it in visitableCells }.mapNotNull {
        moveAgentOnVars[CellAgentSubTime(it, agIdx, t, 1)]
      }
      model.addBoolOr(sequenceOf(!varReq) + seqMoveAgentOn)
    }

    // Constraint 5.3: An agent can only attach a free block that the agent is adjacent to and that exists
    for ((agIdx, _) in workersWithIdx) {
      for (d in Direction.values()) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val varAttach = attachVars[AgentDirTime(agIdx, d, t)] ?: continue
          val conjunctions = availableDispensers.asSequence().mapNotNull { (disPos, _) ->
            val offPos = disPos subBounded d.offset
            if (offPos !in visitableCells) return@mapNotNull null
            val varFreeBlockOn = freeBlockOnVars[CellTime(disPos, t)] ?: return@mapNotNull null
            val varMoveAgentOn = moveAgentOnVars[CellAgentSubTime(offPos, agIdx, t, 1)] ?: return@mapNotNull null
            varMoveAgentOn to varFreeBlockOn
          }
          if (conjunctions.isEmpty()) {
            model.addBoolOr(!varAttach)
            continue
          }
          model.addOrAndImplicationAtMostOne(varAttach, conjunctions)
        }
      }
    }

    timer.resetOptPrint("attach-related constraints")

    /*
     * Constructor-related constraints
     */

    // Constraint 6.1: When joining, the block needs to be attached to the agent, the agent has to be on
    // an appropriate cell, and a direct neighbour of the target cell has to be attached already if joining non-directly
    for ((agIdx, agPosInfo) in workersWithIdx) {
      for (d in Direction.values()) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val key = AgentDirTime(agIdx, d, t)
          val varJoin = joinVars[key] ?: continue
          val varAttachedBlockDir = attachedBlockDirVars[key]
          if (varAttachedBlockDir == null) {
            model.addBoolOr(!varJoin)
            continue
          }

          val agInfo = agPosInfo.info
          val constrIdx = agInfo.constrIdx
          val conInfo = constructorInfos[constrIdx]
          val blockType = agInfo.blockType

          // Part 2: Determine the variables for the direct neighbours
          val seqMoveAgentOnDirect = conInfo.pos.neighboursExactlyBounded(1).mapNotNull {
            val posInfo = conInfo.cells[it] ?: return@mapNotNull null
            if (posInfo.type != blockType) return@mapNotNull null
            val offPos = it subBounded d.offset
            if (offPos !in visitableCells) return@mapNotNull null
            moveAgentOnVars[CellAgentSubTime(offPos, agIdx, t, 1)]
          }

          // Part 3a: Determine the DNF
          val conjunctions = conInfo.cells.asSequence().mapNotNull { (cellPos, CellType) ->
            if (CellType.type != blockType) return@mapNotNull null
            if (distanceBounded(conInfo.pos, cellPos) <= 1) return@mapNotNull null
            val offPos = cellPos subBounded d.offset
            if (offPos !in visitableCells) return@mapNotNull null
            cellPos to offPos
          }.mapNotNull { (pos, offPos) ->
            val varMoveAgentOn = moveAgentOnVars[CellAgentSubTime(offPos, agIdx, t, 1)] ?: return@mapNotNull null
            val seqConBlocks = pos.neighboursExactlyBounded(1).filter { it in conInfo.cells }.mapNotNull {
              constructorBlockOnVars[CellConstrTime(it, constrIdx, t)]
            }
            if (seqConBlocks.isEmpty()) return@mapNotNull null
            varMoveAgentOn to seqConBlocks
          }

          // If both parts of the disjunction are empty, no join is possible
          if (seqMoveAgentOnDirect.isEmpty() && conjunctions.isEmpty()) {
            model.addBoolOr(!varJoin)
            continue
          }

          // Part 1
          model.addImplication(varJoin, varAttachedBlockDir)
          // Part 3b
          model.addOrOrAndOrImplicationAtMostOne(varJoin, seqMoveAgentOnDirect, conjunctions)
        }
      }
    }

    // Constraint 6.2: Integrated into the definition of DetachConnected

    // Constraint 6.3: Submit takes place if all constructor cells are filled
    for ((conIdx, conInfo) in constructorInfos.withIndex()) {
      for (t in timeRange(excludeFirst = false, excludeLast = true)) {
        val varSubmit = submitVars.getValue(ConstrTime(conIdx, t))

        if (conInfo.cells.keys.any { CellConstrTime(it, conIdx, t) !in constructorBlockOnVars }) {
          model.addBoolOr(!varSubmit)
          continue
        }

        val seqConBlockOn = conInfo.cells.keys.asSequence().map {
          constructorBlockOnVars.getValue(CellConstrTime(it, conIdx, t))
        }
        model.addAndEquality(varSubmit, seqConBlockOn)
      }
    }

    // Constraint 6.4: A constructor block exists if it existed in the previous step, or it has been attached,
    // or the agent has detached after connecting, and it has not been submitted
    for ((conIdx, conInfo) in constructorInfos.withIndex()) {
      for (p in conInfo.cells.keys) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val cct = CellConstrTime(p, conIdx, t)
          val varSubmit = submitVars.getValue(cct.constrTime)
          val conCells = constructorInfos[cct.constrIdx].cells
          val conjunctions = workersWithIdx.asSequence().filter { (_, agInfo) ->
            agInfo.info.constrIdx == cct.constrIdx && agInfo.info.blockType == conCells.getValue(cct.pos).type
          }.flatMap { (agIdx, _) ->
            Direction.values().asSequence().mapNotNull { dir ->
              val pos = cct.pos subBounded dir.offset
              if (pos !in visitableCells) return@mapNotNull null
              val key = CellAgentSubTime(pos, agIdx, cct.timeStep, 1)
              val varMoveAgentOn = moveAgentOnVars[key] ?: return@mapNotNull null
              val varDetach = detachVars[AgentDirTime(agIdx, dir, cct.timeStep)] ?: return@mapNotNull null
              varMoveAgentOn to varDetach
            }
          }
          val varConBlockOn0 = constructorBlockOnVars[cct]
          val varConBlockOn1 = constructorBlockOnVars[cct.next]
          if (varConBlockOn0 != null) {
            model.addBoolOr(sequenceOf(!varConBlockOn0) + varConBlockOn1.sequence + sequenceOf(varSubmit))
          }
          if (varConBlockOn0 != null && varConBlockOn1 != null) {
            model.addBoolOr(!varConBlockOn0, !varConBlockOn1, !varSubmit)
          }
          model.addOrAndEqualityAtMostOne(
            varConBlockOn1 ?: model.falseLiteral(), conjunctions, enforcementLiteral = varConBlockOn0?.not()
          )
        }
      }
    }

    timer.resetOptPrint("constructor-related constraints")

    /*
     * Dispenser-related constraints
     */

    // Constraint 7.1: A free block is on a dispenser if it was there already, or it has just been requested,
    // and it has not been attached
    for ((disPos, disInfo) in availableDispensers) {
      for (t in timeRange(excludeFirst = false, excludeLast = true)) {
        val varFreeBlockOn1 = freeBlockOnVars[CellTime(disPos, t + 1)]
        val varFreeBlockOn0 = freeBlockOnVars[CellTime(disPos, t)]
        val seqRequest = workersWithIdx.asSequence().filter {
          it.value.info.blockType == disInfo.type
        }.mapNotNull { (agIdx, _) -> requestVars[CellAgentTime(disPos, agIdx, t)] }
        val seqDisAttach = workersWithIdx.asSequence().filter { (_, posInfo) ->
          posInfo.info.status == WorkerStatus.GATHERER && posInfo.info.blockType == disInfo.type
        }.map { (agIdx, _) -> !dispenserAttachVars.getValue(CellAgentTime(disPos, agIdx, t)) }

        if (varFreeBlockOn1 != null && varFreeBlockOn0 != null) {
          model.addAndEquality(varFreeBlockOn1, seqDisAttach, enforcementLiteral = varFreeBlockOn0)
        } else if (varFreeBlockOn0 != null) {
          model.addBoolOr(seqDisAttach.map { !it }).onlyEnforceIf(varFreeBlockOn0)
        }

        if (seqRequest.isNotEmpty()) {
          if (varFreeBlockOn1 != null) {
            model.addOrEquality(varFreeBlockOn1, seqRequest, enforcementLiteral = varFreeBlockOn0?.not())
          } else {
            val constraint = model.addBoolAnd(seqRequest.map { !it })
            if (varFreeBlockOn0 != null) constraint.onlyEnforceIf(varFreeBlockOn0)
          }
        } else if (varFreeBlockOn0 != null && varFreeBlockOn1 != null) {
          model.addImplication(!varFreeBlockOn0, !varFreeBlockOn1)
        } else if (varFreeBlockOn1 != null) {
          model.addBoolOr(!varFreeBlockOn1)
        }
      }
    }

    // Constraints 7.2 and 7.3
    for ((disPos, disInfo) in availableDispensers) {
      for (t in timeRange(excludeFirst = false, excludeLast = true)) {
        val disType = disInfo.type

        // Constraint 7.2: At most one worker can request from each dispenser in each step
        model.addAtMostOne(workersWithIdx.asSequence().filter { it.value.info.blockType == disType }.mapNotNull {
          requestVars[CellAgentTime(disPos, it.index, t)]
        })

        // Constraint 7.3: Each free block may be attached at most once
        model.addAtMostOne(workersWithIdx.asSequence().filter { (_, posInfo) ->
          posInfo.info.status == WorkerStatus.GATHERER && posInfo.info.blockType == disType
        }.map {
          dispenserAttachVars.getValue(CellAgentTime(disPos, it.index, t))
        })
      }
    }

    timer.resetOptPrint("dispenser-related constraints")

    /*
     * Attached-block-related constraints
     */

    // Constraint 8.1: An attached block is on a given side of a worker after rotating if it was on the appropriate side
    // before rotating
    for ((agIdx, _) in workersWithIdx) {
      for (d in Direction.values()) {
        for (r in Rotation.values()) {
          for (t in timeRange(excludeFirst = false, excludeLast = true)) {
            val varRotate1 = rotateVars.getValue(AgentRotTime(agIdx, r, t))
            val varAttachedBlockDir2 = attachedBlockDirVars[AgentDirTime(agIdx, d.rotate(r), t + 1)]
            val varAttachedBlockDir1 = attachedBlockDirVars[AgentDirTime(agIdx, d, t)]

            when {
              varAttachedBlockDir1 != null && varAttachedBlockDir2 != null -> {
                model.addBoolOr(!varRotate1, !varAttachedBlockDir2, varAttachedBlockDir1)
                model.addBoolOr(!varRotate1, varAttachedBlockDir2, !varAttachedBlockDir1)
              }

              varAttachedBlockDir1 != null -> model.addBoolOr(!varRotate1, !varAttachedBlockDir1)
              varAttachedBlockDir2 != null -> model.addBoolOr(!varRotate1, !varAttachedBlockDir2)
            }
          }
        }
      }
    }

    // Constraint 8.2: An attached block is on a given side of a worker when not rotating if it was there before,
    // or a cell has been attached to this side, and it has not been detached
    for ((agIdx, _) in workersWithIdx) {
      for (d in Direction.values()) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val key = AgentDirTime(agIdx, d, t)
          val varAttachedBlockDir2 = attachedBlockDirVars[key.next]
          val varAttachedBlockDir1 = attachedBlockDirVars[key]
          val varAttach1 = attachVars[key]
          val varDetach1 = detachVars[key]
          val seqRotation = Rotation.values().asSequence().map { rotateVars.getValue(AgentRotTime(agIdx, it, t)) }

          if (varAttachedBlockDir1 != null) {
            model.addBoolOr(
              seqRotation + sequenceOf(!varAttachedBlockDir1) + varAttachedBlockDir2.sequence + varDetach1.sequence
            )
          }
          if (varAttachedBlockDir1 != null && varAttachedBlockDir2 != null && varDetach1 != null) {
            model.addBoolOr(seqRotation + sequenceOf(!varAttachedBlockDir1, !varAttachedBlockDir2, !varDetach1))
          }
          if (varAttach1 != null) {
            model.addBoolOr(
              seqRotation + varAttachedBlockDir1.sequence + varAttachedBlockDir2.sequence + sequenceOf(!varAttach1)
            )
          }
          if (varAttachedBlockDir2 != null) {
            model.addBoolOr(
              seqRotation + varAttachedBlockDir1.sequence + sequenceOf(!varAttachedBlockDir2) + varAttach1.sequence
            )
          }
        }
      }
    }

    timer.resetOptPrint("attached-block-related constraints")

    // Objective Function
    model.maximize(LinearExpr.newBuilder().apply {
      val typedMinDispenserDists by lazy {
        dispensers.groupBy { it.info.type }.mapValues { (_, disInfos) ->
          visitableCells.keys.associateWith { cPos ->
            disInfos.minOfOrNull { (disPos, _) -> distanceBounded(cPos, disPos) }!!
          }
        }
      }

      val timer1 = Timer()

      // All constructor cells
      val allConCells = buildSet {
        for ((_, conInfo) in constructors) for ((pos, _) in conInfo.cells) {
          add(pos)
        }
      }

      // Problem cells: candidates within a given radius of a dispenser or constructor if the number
      // of problem candidates there exceeds a given limit
      val problemCells = buildSet {
        // Problem candidates: obstacles, move agents, or their attached blocks
        val problemCandidates = buildSet {
          addAll(seenCells.asSequence().filter { it.value != CellType.EMPTY }.map { it.key })
          for ((agPos, agInfo) in moveAgents) {
            add(agPos)
            if (agInfo is MoveAgentInfo.Worker) {
              addAll(agInfo.attachedSides.asSequence().map { agPos addBounded it.offset })
            }
          }
        }

        val problemDistance = 5
        val problemCandidateLimit = 8
        for (conInfo in constructorInfos) {
          val problemNeighbours =
            conInfo.pos.neighboursAtMostBounded(problemDistance).filter { it in problemCandidates }.toSet()
          if (problemNeighbours.size >= problemCandidateLimit) {
            addAll(problemNeighbours)
          }
        }
      }

      // Edges pointing towards problem cells have higher weights
      val edgeWeight = { _: Position, p2: Position -> if (p2 in problemCells) 6.0 else 1.0 }

      val graph = SimpleDirectedWeightedGraph<Position, DefaultWeightedEdge>(DefaultWeightedEdge::class.java).apply {
        // Include all constructor cells
        val baseVertices = buildSet { addAll(visitableCells.keys); addAll(allConCells) }
        baseVertices.forEach { addVertex(it) }
        // No edges pointing towards constructor cells
        baseVertices.forEach { p1 ->
          p1.neighboursExactlyBounded(1).filter { it in visitableCells }.forEach { p2 ->
            setEdgeWeight(addEdge(p1, p2), edgeWeight(p1, p2))
          }
        }
      }
      val fallbackGraph by lazy {
        SimpleDirectedWeightedGraph<Position, DefaultWeightedEdge>(DefaultWeightedEdge::class.java).apply {
          val baseVertices = seenCells.keys.asSequence().filter { it !in constructorMap && it !in dispenserMap }.toSet()
          baseVertices.forEach { addVertex(it) }
          baseVertices.forEach { p1 ->
            p1.neighboursExactlyBounded(1).filter { it in baseVertices && it !in allConCells }.forEach { p2 ->
              setEdgeWeight(addEdge(p1, p2), edgeWeight(p1, p2))
            }
          }
        }
      }
      timer1.resetOptPrint("base graph")

      val makeMap = { g: Graph<Position, DefaultWeightedEdge>, visitable: Set<Position>, conCells: Set<Position> ->
        // Add the new edges and store them
        val conEdges = buildSet {
          val addEdges = { positions: Iterable<Position> ->
            positions.forEach { p1 ->
              p1.neighboursExactlyBounded(1).filter { it in conCells }.forEach { p2 ->
                g.addEdge(p1, p2)?.also { g.setEdgeWeight(it, edgeWeight(p1, p2)); add(it) }
              }
            }
          }

          // Add edges from conCells to conCells
          addEdges(conCells)
          // Add edges from neighbours to conCells
          val neighbours = conCells.flatMap { it.neighboursExactlyBounded(1) }.filter { it in visitableCells }.toSet()
          addEdges(neighbours)
        }

        // Determine the shortest distance of each visitable cell to an appropriate constructor cell
        val shortestPaths = DijkstraManyToManyShortestPaths(g).getManyToManyPaths(visitable, conCells)
        val map = visitable.associateWith { p1 -> conCells.minOf { p2 -> shortestPaths.getWeight(p1, p2) } }

        // Remove the new edges again
        g.removeAllEdges(conEdges)

        map
      }

      // Constructor → its block types → positions reachable by its agents of this block type
      // → the shortest path weight
      val conShortestPaths = constructors.withIndex().associate { (conIdx, conPosInfo) ->
        val conCells = conPosInfo.info.cells
        val blockTypes = conCells.asSequence().map { it.info.type }.toSet()
        conIdx to blockTypes.associateWith { blockType ->
          // The appropriate deliverers
          val conTypeDeliverers = delivererSet.asSequence().map { delivererIdx ->
            val posInfo = moveAgents[delivererIdx]
            Triple(delivererIdx, posInfo.position, posInfo.info as MoveAgentInfo.Worker)
          }.filter { it.third.constrIdx == conIdx && it.third.blockType == blockType }
          if (conTypeDeliverers.isEmpty()) return@associateWith emptyMap<Position, Int>()

          // All cells that these deliverers could reach
          val conTypeVisitable = conTypeDeliverers.flatMap { (_, pos, agInfo) ->
            pos.neighboursAtMostBounded(subTimeStepNum(agInfo)).filter { it in visitableCells }
          }.toSet()

          // The appropriate constructor cells.
          // Only unoccupied cells are selected, if those exist, otherwise all appropriate constructor cells are used
          val baseConCells = conCells.filter { it.info.type == blockType }.asSequence()
          val freeConTypeCells = baseConCells.filter { !it.info.occupied }
          val conTypeCells = freeConTypeCells.ifEmpty { baseConCells }.map { it.position }.toSet()

          val baseMap = makeMap(graph, conTypeVisitable, conTypeCells)
          if (baseMap.values.max().isFinite()) {
            baseMap
          } else {
            makeMap(fallbackGraph, conTypeVisitable, conTypeCells)
          }.mapValues { it.value.roundToInt() }
        }
      }
      timer1.resetOptPrint("graph")

      // 1e, 2e, and 3c
      for ((ast, varMove) in moveVars) {
        addTerm(!varMove, ast.fullTimeStep.toLong())
      }
      val weightMove = (timeSteps.toLong() * (timeSteps.toLong() - 1)) / 2
      // 1f, 2f, and 3d
      for (agIdx in moveAgents.indices) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          addTerm(!anyMoveAgentAction(agIdx, t), t.toLong())
        }
      }

      // The weights are computed separately for each agent — is that a good idea?
      var weight1a = 0L
      var weight2a = 0L
      var weight3a = 0L
      for ((agIdx, agInfo) in workersWithIdx) {
        val info = agInfo.info
        val minDispenserDists by lazy { typedMinDispenserDists.getValue(info.blockType) }
        val fullTimeStepNumEx = timeSteps.toLong() * info.stepDist.toLong()
        val weightMoveAndAction = (fullTimeStepNumEx * (fullTimeStepNumEx + 1)) / 2 + weightMove
        val workerFactor = timeSteps * timeSteps * timeSteps * timeSteps * timeSteps * timeSteps

        when (info.status) {
          WorkerStatus.GATHERER -> {
            // 1d: Minimize the distance from the nearest appropriate dispenser
            val factor1d = weightMoveAndAction * workerFactor
            var sum1d = 0L
            for ((t, s) in subTimeSteps(info)) {
              var subTimeMax = 0L
              for (cPos in visitableCells.keys) {
                val varMoveAgentOn = moveAgentOnVars[CellAgentSubTime(cPos, agIdx, t, s)] ?: continue
                val dispenserIdx = agInfo.info.dispenserIdx
                val distWeight = if (dispenserIdx != null) {
                  distanceBounded(cPos, dispensers[dispenserIdx].position)
                } else {
                  minDispenserDists.getValue(cPos)
                }
                val weight = factor1d * distWeight
                subTimeMax = max(subTimeMax, weight)
                addTerm(varMoveAgentOn, -weight)
              }
              sum1d += subTimeMax
            }
            val weight1d = max(sum1d, factor1d)

            val weightFor1a = weight1d * timeSteps * (availableDispensers.size + Direction.values().size)
            for (t in timeRange(excludeFirst = false, excludeLast = false)) {
              // 1c: Maximize the number of requests
              for ((disPos, _) in availableDispensers) {
                requestVars[CellAgentTime(disPos, agIdx, t)]?.let { addTerm(it, weight1d) }
              }
              // 1b: Maximize the number of attached blocks
              for (d in Direction.values()) {
                attachedBlockDirVars[AgentDirTime(agIdx, d, t)]?.let { addTerm(it, weight1d) }
              }
              // 1a: Maximize the number of fully loaded workers
              addTerm(fullyLoadedVars.getValue(AgentTime(agIdx, t)), weightFor1a)
            }

            weight1a += weight1d * timeSteps * Direction.values().size * 2 * timeSteps
          }

          WorkerStatus.DELIVERER -> {
            if (info.constrIdx !in constructorInfos.indices) continue

            // 2d: Minimize the distance from the appropriate constructor cells
            val factor2d = weightMoveAndAction * workerFactor
            var sum2d = 0L
            val conDists = conShortestPaths.getValue(info.constrIdx).getValue(info.blockType)
            for ((t, s) in subTimeSteps(info)) {
              var max2d = 0L
              for ((pos, dist) in conDists) {
                val key = CellAgentSubTime(pos, agIdx, t, s)
                val weight = factor2d * max(0, dist - 1)
                max2d = max(max2d, weight)
                val varMoveAgentOn = moveAgentOnVars[key] ?: continue
                addTerm(varMoveAgentOn, -weight)
              }
              sum2d += max2d
            }
            val weight2d = max(sum2d, factor2d)

            // 2c: Maximize the number of sides that do not have an attached block
            for (t in timeRange(excludeFirst = false, excludeLast = false)) {
              for (dir in Direction.values()) {
                attachedBlockDirVars[AgentDirTime(agIdx, dir, t)]?.let { addTerm(!it, weight2d) }
              }
            }
            val weight2c = weight2d * timeSteps * Direction.values().size

            // 2b: Maximize the number of agents that do not carry anything
            for (t in timeRange(excludeFirst = false, excludeLast = false)) {
              addTerm(!anyLoadedVars.getValue(AgentTime(agIdx, t)), weight2c)
            }
            val weight2b = weight2c * timeSteps

            // 2a: Maximize the number of blocks given to a constructor using both Join and Detach
            for (t in timeRange(excludeFirst = false, excludeLast = true)) {
              for (d in Direction.values()) {
                val arg = AgentDirTime(agIdx, d, t)
                joinVars[arg]?.let { addTerm(it, weight2b) }
                detachVars[arg]?.let { addTerm(it, weight2b) }
              }
            }
            weight2a += weight2b * (timeSteps - 1) * Direction.values().size * 3
          }
        }
      }

      /*
       * Part 3: Diggers
       */

      for ((agIdx, agPosInfo) in moveAgents.withIndex()) {
        val info = agPosInfo.info
        if (info !is MoveAgentInfo.Digger) continue

        val flock = info.flock
        val obstacleFlock = flock.filter { it in obstacleCells }
        val fullTimeStepNumEx = timeSteps.toLong() * info.stepDist.toLong()
        val weightMoveAndAction = (fullTimeStepNumEx * (fullTimeStepNumEx + 1)) / 2 + weightMove

        // 3c: Maximize the number of clears outside the flock
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val positions = moveClearCellsAgent.getValue(t)[agIdx] ?: continue
          for (p in positions.filter { it !in flock }) {
            addTerm(moveClearVars.getValue(CellAgentTime(p, agIdx, t)), weightMoveAndAction)
          }
        }
        val weight3c = weightMoveAndAction * (timeSteps - 1)

        // 3b: Minimize the distance from the closest obstacle in its flock
        var sum3b = 0L
        for ((t, s) in subTimeSteps(info, excludeFirst = false, excludeLast = false)) {
          var max3b = 0L
          for (cPos in visitableCells.keys) {
            val obstacleDists = obstacleFlock.map { distanceBounded(cPos, it) }
            if (obstacleDists.isEmpty()) continue

            val weight = weight3c * t * max(obstacleDists.min() - info.clearDist, 0)
            if (weight == 0L) continue
            max3b = max(max3b, weight)

            val varMoveAgentOn = moveAgentOnVars[CellAgentSubTime(cPos, agIdx, t, s)] ?: continue
            addTerm(varMoveAgentOn, -weight)
          }
          sum3b += max3b
        }
        val weight3b = max(sum3b, weightMoveAndAction)

        // 3a: Maximize the number of clears inside the flock
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          val positions = moveClearCellsAgent.getValue(t)[agIdx] ?: continue
          for (p in positions.filter { it in flock }) {
            addTerm(moveClearVars.getValue(CellAgentTime(p, agIdx, t)), weight3b)
          }
        }

        weight3a += weight3b * (timeSteps - 1)
      }

      /*
       * Part 4: Constructor
       */

      // 4b: Maximize the number of clears
      conClearAnyVars.values.forEach { add(it) }
      val weight3b = timeSteps - 1

      // 4a: Maximize the number of submissions
      val weightFor3a = (weight1a + weight2a + weight3a) * weight3b
      for (conIdx in constructorInfos.indices) {
        for (t in timeRange(excludeFirst = false, excludeLast = true)) {
          addTerm(submitVars.getValue(ConstrTime(conIdx, t)), weightFor3a)
        }
      }
    })

    timer.resetOptPrint("objective function")

    fullTimer.resetOptPrint("init time")
  }

  inner class Solution(private val solver: CpSolver, val status: CpSolverStatus) {
    private val Literal.boolValue: Boolean
      get() = solver.booleanValue(this)
    private val IntVar.value: Long
      get() = solver.value(this)

    val agentActions: AgentActions
      get() {
        val constructorsAttachMap = mutableMapOf<Int, Position>()
        // The last position is the position of the block to be connected
        val constructorsConnectMap = mutableMapOf<Int, Triple<Position, Int, Position>>()

        val actionForWorkers = moveAgents.withIndex().map { (agIndex, agInfo) ->
          val info = agInfo.info
          val initPos = agInfo.position

          obstacleCells.find { p -> moveClearVars[CellAgentTime(p, agIndex, 1)]?.boolValue == true }
            ?.let { return@map AgentAction.Clear(it subClosest initPos) }

          var prev = initPos
          val offsets = (1..info.stepDist).map { s ->
            val newPos = moveAgentOnCells.getValue(AgentSubTime(agIndex, 1, s).next).find { p ->
              val key = CellAgentSubTime(p, agIndex, 1, s).next
              moveAgentOnVars[key]?.boolValue == true
            }!!
            val offset = newPos subClosest prev
            prev = newPos
            offset
          }
          if (!offsets.first().isZero) {
            return@map AgentAction.Move(offsets)
          }

          if (info is MoveAgentInfo.Worker) {
            // Rotate
            Rotation.values().find { currRot ->
              rotateVars[AgentRotTime(agIndex, currRot, 1)]?.boolValue == true
            }?.let { return@map AgentAction.Rotate(it) }

            when (info.status) {
              WorkerStatus.GATHERER -> {
                // Request
                availableDispensers.filter { (_, disInfo) -> disInfo.type == info.blockType }.find { (disPos, _) ->
                  requestVars[CellAgentTime(disPos, agIndex, 1)]?.boolValue == true
                }?.let { return@map AgentAction.Request(it.position subClosest initPos) }

                // Attach
                Direction.values().find { dir ->
                  attachVars[AgentDirTime(agIndex, dir, 1)]?.boolValue == true
                }?.offset?.let { return@map AgentAction.Attach(it) }

                AgentAction.Nothing
              }

              WorkerStatus.DELIVERER -> {
                val dirKey = { dir: Direction -> AgentDirTime(agIndex, dir, 1) }

                // Join
                Direction.values().find { joinVars[dirKey(it)]?.boolValue == true }?.let { dir ->
                  val conIdx = info.constrIdx
                  val conPos = constructorInfos[conIdx].pos
                  val blockPos = initPos addBounded dir.offset

                  if (distanceBounded(conPos, blockPos) == 1) {
                    // Worker does nothing, constructor attaches
                    constructorsAttachMap[conIdx] = blockPos subClosest conPos
                    return@map AgentAction.Nothing
                  } else {
                    // Worker and constructor connect
                    val conBlockPos = blockPos.neighboursExactlyBounded(1).find { pos ->
                      constructorBlockOnVars[CellConstrTime(pos, conIdx, 1)]?.boolValue == true
                    }!!
                    constructorsConnectMap[conIdx] =
                      Triple((conBlockPos subClosest conPos), agIndex, (blockPos subClosest conPos))
                    return@map AgentAction.Connect(agInfo.infoAs<MoveAgentInfo.Worker>().constrIdx, dir.offset)
                  }
                }

                Direction.values().find { detachVars[dirKey(it)]?.boolValue == true }?.let { dir ->
                  return@map AgentAction.Detach(dir.offset)
                }

                AgentAction.Nothing
              }
            }
          } else AgentAction.Nothing
        }

        val actionForConstructors = constructorInfos.withIndex().map { (constrIndex, conInfo) ->
          if (submitVars.getValue(ConstrTime(constrIndex, 1)).boolValue) {
            return@map AgentAction.Submit
          }

          conClearCells[constrIndex].find { p ->
            conClearVars[CellConstrTime(p, constrIndex, 1)]?.boolValue == true
          }?.let { return@map AgentAction.Clear(it subClosest conInfo.pos) }

          constructorsAttachMap[constrIndex]?.let { return@map AgentAction.Attach(it) }
          constructorsConnectMap[constrIndex]?.let { return@map AgentAction.Connect(it.second, it.first, it.third) }

          AgentAction.Nothing
        }

        return AgentActions(actionForWorkers, actionForConstructors)
      }
  }

  fun solve(log: Boolean = true, maxSeconds: Double? = null): Problem.Solution {
    val solver = CpSolver()
    solver.parameters.apply {
      maxSeconds?.let { maxTimeInSeconds = it }
      logSearchProgress = log
    }
    val status = solver.solve(model)
    return Solution(solver, status)
  }
}
