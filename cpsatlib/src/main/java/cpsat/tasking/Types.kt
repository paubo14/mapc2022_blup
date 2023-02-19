package cpsat.tasking

import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.encodeToStream
import massim.common.Bounds
import massim.common.Position
import java.io.OutputStream

@Serializable
data class PositionInfo<Info>(val position: Position, val info: Info) {
  inline fun <reified T> infoAs() = info as T
  inline fun <reified T> withInfoAs() = PositionInfo(position, info as T)
}

inline fun <reified Info> Iterable<PositionInfo<Info>>.makeMap() = associate { it.position to it.info }

@Serializable
enum class CellType {
  @SerialName("empty")
  EMPTY,

  @SerialName("mutable_obstacle")
  MUTABLE_OBSTACLE,

  @SerialName("fixed_obstacle")
  FIXED_OBSTACLE
}

@Serializable
enum class Direction {
  @SerialName("north")
  NORTH,

  @SerialName("east")
  EAST,

  @SerialName("south")
  SOUTH,

  @SerialName("west")
  WEST
}

val Direction.offset
  get() = when (this) {
    Direction.NORTH -> Position(0, -1)
    Direction.EAST -> Position(1, 0)
    Direction.SOUTH -> Position(0, 1)
    Direction.WEST -> Position(-1, 0)
  }

val Position.direction: Direction?
  get() = when {
    x == 0 && y == -1 -> Direction.NORTH
    x == 1 && y == 0 -> Direction.EAST
    x == 0 && y == 1 -> Direction.SOUTH
    x == -1 && y == 0 -> Direction.WEST
    else -> null
  }

enum class Rotation { CLOCKWISE, ANTICLOCKWISE }

fun Direction.rotate(rot: Rotation) = when (rot) {
  Rotation.CLOCKWISE -> when (this) {
    Direction.NORTH -> Direction.EAST
    Direction.EAST -> Direction.SOUTH
    Direction.SOUTH -> Direction.WEST
    Direction.WEST -> Direction.NORTH
  }

  Rotation.ANTICLOCKWISE -> when (this) {
    Direction.NORTH -> Direction.WEST
    Direction.EAST -> Direction.NORTH
    Direction.SOUTH -> Direction.EAST
    Direction.WEST -> Direction.SOUTH
  }
}

sealed class AgentAction {
  object Nothing : AgentAction() {
    override fun toString() = "Nothing"
  }

  data class Move(val offsets: List<Position>) : AgentAction()
  data class Clear(val offset: Position) : AgentAction()
  data class Request(val offset: Position) : AgentAction()
  data class Attach(val offset: Position) : AgentAction()
  data class Detach(val offset: Position) : AgentAction()
  data class Connect(val agentIdx: Int, val offset: Position, val offsetNewAttached: Position? = null) : AgentAction()
  data class Rotate(val rotate: Rotation) : AgentAction()

  object Submit : AgentAction() {
    override fun toString() = "Submit"
  }
}

data class AgentActions(val moveAgentActions: List<AgentAction>, val constructorsActions: List<AgentAction>)

@Serializable
enum class WorkerStatus {
  @SerialName("gatherer")
  GATHERER,

  @SerialName("deliverer")
  DELIVERER
}

@Serializable
sealed class MoveAgentInfo : cpsat.MoveAgentInfo {
  @Serializable
  @SerialName("worker")
  data class Worker(
    @SerialName("status") val status: WorkerStatus,
    val vision: Int,
    @SerialName("step_dist") override val stepDist: Int,
    @SerialName("clear_dist") val clearDist: Int,
    @SerialName("clear_probability") val clearProb: Double,
    @SerialName("max_attached") val maxAttached: Int,
    @SerialName("block_type") val blockType: String,
    @SerialName("constr_idx") val constrIdx: Int,
    @SerialName("attached_sides") val attachedSides: Set<Direction>,
    @SerialName("dispenser_idx") val dispenserIdx: Int? = null,
  ) : MoveAgentInfo()

  @Serializable
  @SerialName("digger")
  data class Digger(
    val vision: Int,
    @SerialName("step_dist") override val stepDist: Int,
    @SerialName("clear_dist") val clearDist: Int,
    @SerialName("clear_probability") val clearProb: Double,
    val flock: Set<Position>,
  ) : MoveAgentInfo()
}

val MoveAgentInfo.clearDist
  get() = when (this) {
    is MoveAgentInfo.Digger -> clearDist
    is MoveAgentInfo.Worker -> clearDist
  }
val MoveAgentInfo.clearProb
  get() = when (this) {
    is MoveAgentInfo.Digger -> clearProb
    is MoveAgentInfo.Worker -> clearProb
  }

@Serializable
data class ConstructorCellInfo(val type: String, val occupied: Boolean)

@Serializable
data class ConstructorInfo(
  @SerialName("clear_dist") val clearDist: Int,
  @SerialName("clear_probability") val clearProb: Double,
  val cells: List<PositionInfo<ConstructorCellInfo>>,
)

@Serializable
data class DispenserInfo(val type: String, val occupied: Boolean)

@Serializable
data class ProblemInfo constructor(
  @SerialName("seen_cells") val seenCells: List<PositionInfo<CellType>>,
  @SerialName("move_agents") val moveAgents: List<PositionInfo<MoveAgentInfo>>,
  val constructors: List<PositionInfo<ConstructorInfo>>,
  val dispensers: List<PositionInfo<DispenserInfo>>,
  @SerialName("clear_cells") val clearCells: Set<Position>,
  val bounds: Bounds,
) {
  @OptIn(ExperimentalSerializationApi::class)
  fun encode(outputStream: OutputStream) = Json.encodeToStream(this, outputStream)
}
