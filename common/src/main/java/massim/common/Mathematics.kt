package massim.common

infix fun Int.divCeil(other: Int) = this / other + if (this % other != 0) 1 else 0
