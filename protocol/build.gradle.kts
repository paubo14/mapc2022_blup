plugins {
  `java-library`
  kotlin("jvm")
}

java.sourceCompatibility = JavaVersion.VERSION_18
java.targetCompatibility = JavaVersion.VERSION_18
tasks.compileKotlin.get().kotlinOptions.jvmTarget = "18"

repositories { mavenCentral() }
dependencies { api("org.json:json:20220924") }

tasks.jar {
  // Taken from https://stackoverflow.com/a/70864141
  from(configurations.runtimeClasspath.get().map(::zipTree))
  duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}
