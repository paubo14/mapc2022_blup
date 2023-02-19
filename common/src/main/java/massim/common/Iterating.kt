package massim.common

// Create a map by mapping each element of the sequence to a key (keySelector)
// and a value (valueTransform). Values with the same key are grouped into sets.
inline fun <T, K, V> Sequence<T>.groupBySet(keySelector: (T) -> K, valueTransform: (T) -> V): Map<K, Set<V>> {
  val destination = mutableMapOf<K, MutableSet<V>>()
  forEach { destination.getOrPut(keySelector(it)) { mutableSetOf() }.add(valueTransform(it)) }
  return destination
}

fun <T> Sequence<T>.isNotEmpty() = iterator().hasNext()

fun <T> Sequence<T>.isEmpty() = !iterator().hasNext()

fun <T> Sequence<T>.moreThanSingle(): Boolean {
  val iter = iterator()
  if (!iter.hasNext()) return false
  iter.next()
  if (!iter.hasNext()) return false
  return true
}
