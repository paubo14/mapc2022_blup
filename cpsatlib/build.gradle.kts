plugins {
  `java-library`
  kotlin("jvm")
  kotlin("plugin.serialization")
}

java.sourceCompatibility = JavaVersion.VERSION_18
java.targetCompatibility = JavaVersion.VERSION_18
tasks.compileKotlin.get().kotlinOptions.jvmTarget = "18"

val common = project(":common")

repositories {
  mavenCentral()
}
dependencies {
  api("com.google.ortools:ortools-java:9.5.2237")
  api("org.jetbrains.kotlinx:kotlinx-serialization-json:1.4.1")
  implementation("org.jgrapht:jgrapht-core:1.5.1")
  api(common)
}

tasks.jar {
  // Taken from https://stackoverflow.com/a/70864141
  dependsOn(common.tasks.jar)
  from(configurations.runtimeClasspath.get().map(::zipTree))
  duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}
