plugins {
  `java-library`
  kotlin("jvm")
  kotlin("plugin.serialization")
}

java.sourceCompatibility = JavaVersion.VERSION_18
java.targetCompatibility = JavaVersion.VERSION_18
tasks.compileKotlin.get().kotlinOptions.jvmTarget = "18"

repositories {
  mavenCentral()
}
dependencies { api("org.jetbrains.kotlinx:kotlinx-serialization-json:1.4.1") }

tasks.jar {
  // Taken from https://stackoverflow.com/a/70864141
  from(configurations.runtimeClasspath.get().map(::zipTree))
  duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}
