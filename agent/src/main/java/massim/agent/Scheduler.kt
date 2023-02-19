package massim.agent

import eis.AgentListener
import eis.EnvironmentListener
import eis.exceptions.AgentException
import eis.exceptions.PerceiveException
import eis.exceptions.RelationException
import eis.iilang.EnvironmentState
import eis.iilang.Percept
import java.io.IOException
import java.nio.file.Files
import java.nio.file.Paths
import massim.agent.agents.Agent
import massim.eismassim.EnvironmentInterface
import org.json.JSONObject

/**
 * A scheduler for agent creation and execution. EISMASSim scheduling needs to be enabled (via
 * config), so that getAllPercepts() blocks until new percepts are available! (Also, queued and
 * notifications should be disabled)
 *
 * @param path to a java agents configuration file
 * @param eis the environment interface to connect to
 */
class Scheduler(path: String, private val eis: EnvironmentInterface) : AgentListener, EnvironmentListener {
  /** Holds configured agent data. */
  private data class AgentConf(val name: String, val entity: String, val team: String, val className: String)

  private val agentConfigurations = mutableListOf<AgentConf>()
  private val agentManagers = mutableMapOf<String, AgentManager>()

  init {
    parseConfig(path)
    setupEnvironment()
  }

  /**
   * Parses the java agents config.
   *
   * @param path the path to the config
   */
  private fun parseConfig(path: String) {
    try {
      val config = JSONObject(String(Files.readAllBytes(Paths.get(path, "javaagentsconfig.json"))))
      val agents = config.optJSONArray("agents")
      if (agents != null) {
        for (i in 0 until agents.length()) {
          val agentBlock = agents.getJSONObject(i)
          val count = agentBlock.getInt("count")
          val startIndex = agentBlock.getInt("start-index")
          val agentPrefix = agentBlock.getString("agent-prefix")
          val entityPrefix = agentBlock.getString("entity-prefix")
          val team = agentBlock.getString("team")
          val agentClass = agentBlock.getString("class")

          for (index in startIndex until startIndex + count) {
            agentConfigurations.add(AgentConf(agentPrefix + index, entityPrefix + index, team, agentClass))
          }
        }
      }
    } catch (e: IOException) {
      e.printStackTrace()
    }
  }

  private fun setupEnvironment() {
    for (agentConf in agentConfigurations) {
      var agent: Agent? = null
      when (agentConf.className) {
        "Agent" -> agent = Agent(agentConf.name)
        // TODO add further types here
        else -> println("Unknown agent type/class ${agentConf.className}")
      }
      if (agent == null) continue

      try {
        eis.registerAgent(agent.name)
      } catch (e: AgentException) {
        e.printStackTrace()
      }

      try {
        eis.associateEntity(agent.name, agentConf.entity)
        println("associated agent \"${agent.name}\" with entity \"${agentConf.entity}\"")
      } catch (e: RelationException) {
        e.printStackTrace()
      }

      eis.attachAgentListener(agent.name, this)
      agentManagers.getOrPut(agentConf.team) { AgentManager(eis, agentConf.team) }.agents[agentConf.name] = agent
    }
    eis.attachEnvironmentListener(this)
  }

  /** Steps all agents and relevant infrastructure. */
  fun step() {
    // retrieve percepts for all agents
    val newPerceptAgents = mutableListOf<Agent>()
    for (agentManager in agentManagers.values) {
      for (ag in agentManager.agents.values) {
        try {
          val addList = mutableListOf<Percept>()
          val delList = mutableListOf<Percept>()
          for (pUpdate in eis.getPercepts(ag.name).values) {
            addList.addAll(pUpdate.addList)
            delList.addAll(pUpdate.deleteList)
          }
          if (addList.isNotEmpty() || delList.isNotEmpty()) {
            newPerceptAgents.add(ag)
            agentManager.setPercepts(ag, addList, delList)
          }
        } catch (ignored: PerceiveException) {
        }
      }
    }

    if (newPerceptAgents.size == 0) {
      try {
        // wait a bit in case no agents have been executed
        Thread.sleep(100)
      } catch (ignored: InterruptedException) {
      }
    }
  }

  override fun handlePercept(agent: String, percept: Percept) = Unit

  override fun handleStateChange(newState: EnvironmentState) = Unit

  override fun handleFreeEntity(entity: String, agents: Collection<String>) = Unit

  override fun handleDeletedEntity(entity: String, agents: Collection<String>) = Unit

  override fun handleNewEntity(entity: String) = Unit
}
