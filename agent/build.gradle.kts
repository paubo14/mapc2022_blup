plugins {
  application
  kotlin("jvm")
}

java.sourceCompatibility = JavaVersion.VERSION_18

java.targetCompatibility = JavaVersion.VERSION_18

tasks.compileKotlin.get().kotlinOptions.jvmTarget = "18"

val eismassim = project(":eismassim")
val common = project(":common")
val cpsatLib = project(":cpsatlib")
val mainClassName = "massim.agent.MainKt"

repositories {
  mavenCentral()
  maven("https://raw.github.com/eishub/mvn-repo/master")
}

dependencies {
  implementation("org.jgrapht:jgrapht-core:1.5.1")
  implementation(common)
  implementation(cpsatLib)
  implementation(eismassim)
}

application.mainClass.set(mainClassName)

tasks.run.get().standardInput = System.`in`

tasks.jar {
  // Taken from https://stackoverflow.com/a/70864141
  dependsOn(eismassim.tasks.jar, common.tasks.jar, cpsatLib.tasks.jar)
  manifest.attributes["Main-Class"] = mainClassName
  from(configurations.runtimeClasspath.get().map(::zipTree))
  duplicatesStrategy = DuplicatesStrategy.EXCLUDE
}
