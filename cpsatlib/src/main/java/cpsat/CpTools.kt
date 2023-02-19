package cpsat

import com.google.ortools.Loader
import com.google.ortools.sat.*
import cpsat.tasking.sequence
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.InvocationKind
import kotlin.contracts.contract
import kotlin.system.measureTimeMillis

object CpLoader {
  init {
    withTiming("load native libraries") {
      // Necessary for CP-Sat
      Loader.loadNativeLibraries()
    }
  }

  fun load() {}
}

fun Boolean.toInt(): Int = if (this) 1 else 0

fun Constraint.onlyEnforceIfOpt(enforcementLiteral: Literal?) {
  if (enforcementLiteral != null) onlyEnforceIf(enforcementLiteral)
}

fun CpModel.addAtMostOne(seq: Sequence<Literal>): Constraint = addAtMostOne(seq.asIterable())
fun CpModel.addExactlyOne(seq: Sequence<Literal>): Constraint = addExactlyOne(seq.asIterable())
fun CpModel.addBoolAnd(seq: Sequence<Literal>): Constraint = addBoolAnd(seq.asIterable())
fun CpModel.addBoolOr(seq: Sequence<Literal>): Constraint = addBoolOr(seq.asIterable())

fun CpModel.addBoolOr(vararg literals: Literal): Constraint = addBoolOr(literals)

// Transformation 2.1
fun CpModel.addAndEquality(target: Literal, literals: Sequence<Literal>, enforcementLiteral: Literal? = null) {
  addBoolAnd(literals).apply { onlyEnforceIf(target); onlyEnforceIfOpt(enforcementLiteral) }
  addBoolOr(sequenceOf(target) + enforcementLiteral.sequence.map { !it } + literals.map { !it })
}

// Transformation 2.2
fun CpModel.addOrEquality(target: Literal, literals: Sequence<Literal>, enforcementLiteral: Literal? = null) {
  addBoolOr(sequenceOf(!target) + enforcementLiteral.sequence.map { !it } + literals)
  addBoolAnd(literals.map { !it }).apply { onlyEnforceIf(!target); onlyEnforceIfOpt(enforcementLiteral) }
}

// No longer a transformation in the thesis
fun CpModel.addOrImplication(target: Literal, literals: Sequence<Literal>) {
  addBoolOr(sequenceOf(!target) + literals)
}

// Transformation 2.3
fun CpModel.addOrAndEqualityExactlyOne(target: Literal, literals: Sequence<Pair<Literal, Literal>>) {
  literals.forEach { (l1, l2) -> addBoolOr(target, !l1, !l2); addBoolOr(!target, !l1, l2) }
}

// Transformation 2.4
fun CpModel.addOrAndEqualityAtMostOne(
  target: Literal, literals: Sequence<Pair<Literal, Literal>>, enforcementLiteral: Literal? = null
) {
  val enforceSeq = enforcementLiteral.sequence.map { !it }
  literals.forEach { (l1, l2) ->
    addBoolOr(enforceSeq + sequenceOf(target, !l1, !l2))
    addBoolOr(enforceSeq + sequenceOf(!target, !l1, l2))
  }
  addBoolOr(enforceSeq + sequenceOf(!target) + literals.map { it.first })
}

// Transformation 2.5
fun CpModel.addOrAndImplicationAtMostOne(target: Literal, literals: Sequence<Pair<Literal, Literal>>) {
  literals.forEach { (l1, l2) -> addBoolOr(!target, !l1, l2) }
  addBoolOr(sequenceOf(!target) + literals.map { it.first })
}

// Transformation 2.6
fun CpModel.addOrOrAndOrImplicationAtMostOne(
  target: Literal, lit1: Sequence<Literal>, lit2: Sequence<Pair<Literal, Sequence<Literal>>>
) {
  lit2.forEach { addBoolOr(sequenceOf(!target, !it.first) + it.second) }
  addBoolOr(sequenceOf(!target) + lit1 + lit2.map { it.first })
}

@OptIn(ExperimentalContracts::class)
inline fun withTiming(name: String, block: () -> Unit) {
  contract {
    callsInPlace(block, InvocationKind.EXACTLY_ONCE)
  }
  val time = measureTimeMillis(block)
  println("$name runtime: ${time.toDouble() / 1000} s")
}
