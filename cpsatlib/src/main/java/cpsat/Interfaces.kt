package cpsat

interface MoveAgentInfo {
  val stepDist: Int
}

interface Suffixable {
  val suffix: String
}

interface SubTimeable<Self : SubTimeable<Self>> {
  val timeStep: Int
  val subStep: Int

  fun withSubTimeStep(timeStep: Int, subStep: Int): Self
}

data class SubTimeStep(override val timeStep: Int, override val subStep: Int) : SubTimeable<SubTimeStep> {
  fun next(info: MoveAgentInfo) =
    if (subStep == info.stepDist) SubTimeStep(timeStep + 1, 1) else copy(subStep = subStep + 1)

  override fun withSubTimeStep(timeStep: Int, subStep: Int) = copy(timeStep = timeStep, subStep = subStep)
}

fun <S : SubTimeable<S>> S.nextSubTimeStep(info: MoveAgentInfo) = if (subStep == info.stepDist) {
  withSubTimeStep(timeStep + 1, 1)
} else {
  withSubTimeStep(timeStep, subStep + 1)
}

val <S : SubTimeable<S>> S.isFirst
  get() = timeStep == 1 && subStep == 1
