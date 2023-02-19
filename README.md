# Blup

This repository contains _Blup_, a system designed by Paula Böhm for the scenario of the [16th Multi-Agent Programming Contest](https://multiagentcontest.org/2022/) (MAPC).
It utilizes constraint optimization to determine the actions of the agents using the [Google OR-Tools CP-SAT solver](https://developers.google.com/optimization/cp/cp_solver).

# The License Situation

This work is derived from the [MASSim 2022 project](https://github.com/agentcontest/massim_2022) and is also licensed under the terms of the _GNU Affero General Public License Version 3_ (see the [`LICENSE`](LICENSE) file).
The following components are based on or taken from that repository, whose original license is contained in the file [`LICENSE-MASSim`](LICENSE-MASSim):

- [`agent`](agent): Some files are based on the [`javaagents` example](https://github.com/agentcontest/massim_2022/tree/main/javaagents).
- [`config`](config): Based on the [example configuration](https://github.com/agentcontest/massim_2022/tree/main/javaagents/conf/BasicAgents).
- [`eismassim`](eismassim): Added a build file and a one-line change.
- [`protocol`](protocol): Added a build file.

The project has the following dependencies that are resolved by Gradle by default (as detailed later):

- [Google OR-Tools](https://github.com/google/or-tools/) is licensed under the terms of the _Apache License 2.0_, which can be found [here](https://github.com/google/or-tools/blob/v9.4/LICENSE).
- [JGraphT](https://github.com/jgrapht/jgrapht) may be used under the terms of the _GNU Lesser General Public License 2.1_, which can be found [here](https://github.com/jgrapht/jgrapht/blob/jgrapht-1.5.1/license-LGPL.txt).

# How to Use

This project uses the Gradle build system, which builds all components using the following command:

```sh
gradle assemble
```

To execute the agents, the following commands suffice:

```sh
gradle :agent:jar
java -jar ./agent/build/libs/agent.jar conf/BasicAgents/
```
