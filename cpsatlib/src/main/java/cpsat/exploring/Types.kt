package cpsat.exploring

import cpsat.MoveAgentInfo
import kotlinx.serialization.ExperimentalSerializationApi
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json
import kotlinx.serialization.json.encodeToStream
import massim.common.Bounds
import massim.common.Position
import java.io.OutputStream

@Serializable
data class AgentInfo(
  val vision: Int,
  @SerialName("step_dist") override val stepDist: Int,
  @SerialName("clear_dist") val clearDist: Int,
  @SerialName("clear_probability") val clearProb: Double
) : MoveAgentInfo

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
data class CellInfo(
  @SerialName("cell_type") val cellType: CellType
)

@Serializable
data class AgentPosInfo(val position: Position, val info: AgentInfo)

data class ProblemInfo(
  val cells: Map<Position, CellInfo>, val agents: List<AgentPosInfo>, val clearCells: Set<Position>, val bounds: Bounds
) {
  val jsonInfo
    get() = JsonProblemInfo(cells.map { CellPosInfo(it.key, it.value) }, agents, clearCells, bounds)
}

data class SolutionInfo(val cells: Map<Position, CellInfo>, val agents: List<AgentPosInfo?>)

sealed class AgentAction {
  object Nothing : AgentAction() {
    override fun toString() = "Nothing"
  }

  data class Move(val offsets: List<Position>) : AgentAction()
  data class Clear(val offset: Position) : AgentAction()
}

@Serializable
data class CellPosInfo(val position: Position, val info: CellInfo)

@Serializable
data class JsonProblemInfo(
  val cells: List<CellPosInfo>,
  val agents: List<AgentPosInfo>,
  @SerialName("clear_cells") val clearCells: Set<Position>,
  val bounds: Bounds,
) {
  @OptIn(ExperimentalSerializationApi::class)
  fun encode(outputStream: OutputStream) = Json.encodeToStream(this, outputStream)
}
