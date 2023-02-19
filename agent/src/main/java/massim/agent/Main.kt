package massim.agent

import eis.exceptions.ManagementException
import eis.iilang.EnvironmentState
import java.io.File
import java.util.*
import kotlin.system.exitProcess
import massim.eismassim.EnvironmentInterface

fun main(args: Array<String>) {
  val configDir: String

  println("PHASE 1: INSTANTIATING SCHEDULER")
  if (args.isNotEmpty()) {
    configDir = args[0]
  } else {
    println("PHASE 1.2: CHOOSE CONFIGURATION")
    val confDir = File("conf")
    confDir.mkdirs()
    val confFiles = confDir.listFiles(File::isDirectory)
    if (confFiles == null || confFiles.isEmpty()) {
      println("No javaagents config files available - exit JavaAgents.")
      exitProcess(0)
    } else {
      println("Choose a number:")
      for (i in confFiles.indices) {
        println("$i ${confFiles[i]}")
      }
      val scanner = Scanner(System.`in`)
      var confNum: Int? = null
      while (confNum == null) {
        try {
          val c = Integer.parseInt(scanner.next())
          if (c < 0 || c > confFiles.size - 1) {
            println("No config for that number, try again:")
          } else {
            confNum = c
          }
        } catch (e: Exception) {
          println("Invalid number, try again:")
        }
      }
      configDir = confFiles[confNum].path
    }
  }

  println("PHASE 2: INSTANTIATING ENVIRONMENT")
  val ei = EnvironmentInterface("$configDir${File.separator}eismassimconfig.json")

  try {
    ei.start()
  } catch (e: ManagementException) {
    e.printStackTrace()
  }

  println("PHASE 3: CONNECTING SCHEDULER AND ENVIRONMENT")
  val scheduler = Scheduler(configDir, ei)

  println("PHASE 4: RUNNING")
  var step = 0
  while (ei.state == EnvironmentState.RUNNING) {
    // println("SCHEDULER STEP $step")
    scheduler.step()
    step++
  }
}
