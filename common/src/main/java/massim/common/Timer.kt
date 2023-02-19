package massim.common

import java.io.PrintWriter

class Timer {
  var time = System.currentTimeMillis()

  // Reset the time after printing it
  fun printReset(name: String) {
    val currentTime = System.currentTimeMillis()
    println("$name: ${"%.3f".format(1e-3 * (currentTime - time))} s")
    time = currentTime
  }

  // Reset the time after printing it to the given log
  fun printReset(log: PrintWriter, name: String) {
    val currentTime = System.currentTimeMillis()
    log.println ("$name: ${"%.3f".format(1e-3 * (currentTime - time))} s")
    log.flush()
    time = currentTime
  }
}
