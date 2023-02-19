plugins {
  `java-library`
  kotlin("jvm")
}

java.sourceCompatibility = JavaVersion.VERSION_18
java.targetCompatibility = JavaVersion.VERSION_18
tasks.compileKotlin.get().kotlinOptions.jvmTarget = "18"

val protocol = project(":protocol")

repositories {
  mavenCentral()
  maven("https://raw.github.com/eishub/mvn-repo/master")
}
dependencies {
  api("org.json:json:20220924")
  api("eishub:eis:0.7.0")
  api(protocol)
}

tasks.jar {
  // Taken from https://stackoverflow.com/a/70864141
  dependsOn(protocol.tasks.jar)
  from(configurations.runtimeClasspath.get().map(::zipTree))
  duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}
