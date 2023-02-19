package massim.agent

import java.io.PrintWriter

const val PRINT_TO_LOG = false
const val OUTPUT_JSON = false

// Print to log file and flush directly after it
fun PrintWriter.printFlush(msgGen: () -> String) {
  if (PRINT_TO_LOG) {
    println(msgGen())
    flush()
  }
}
