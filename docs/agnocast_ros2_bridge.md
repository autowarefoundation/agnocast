# Bridge Internal Design

This document covers internal design notes for the Agnocast-ROS 2 Bridge. For user-facing documentation (bridge modes, configuration, plugins, QoS behavior), see [Agnocast-ROS 2 Bridge](https://autowarefoundation.github.io/agnocast_doc/migration-guide/bridge/) on the documentation site.

## Isolation & Safety

Because a single bridge manager process is shared across all Agnocast processes in the IPC namespace, a crash in the bridge manager will affect all bridged topics.

## Internal Structure

Each bridge direction creates a pair of internal publisher and subscriber.
The internal publisher's QoS is fixed to maximize compatibility, ensuring connectivity regardless of the external QoS settings.

**R2A Bridge (RosToAgnocastBridge)**:

```mermaid
flowchart LR
   classDef listNode text-align:left,stroke-width:1px;

    ExtPub[External ROS 2 Publisher]

    subgraph Bridge [R2A Bridge]
        direction TB

        R2Sub["<b>ROS 2 Subscription</b><br/>-QoS: <br/> inherited from Agnocast subscriber"]:::listNode

        AgPub["<b>Agnocast Publisher</b><br/>- Depth: 10 (DEFAULT_QOS_DEPTH)<br/>- Durability: TransientLocal"]:::listNode

        R2Sub -->|callback| AgPub
    end

    AgSub[External Agnocast Subscriber]

    ExtPub --> R2Sub
    AgPub --> AgSub
```

**A2R Bridge (AgnocastToRosBridge)**:

```mermaid
flowchart LR
   classDef listNode text-align:left,stroke-width:1px;

    AgPubExt[External Agnocast Publisher]

    subgraph Bridge [A2R Bridge]
        direction TB

        AgSub["<b>Agnocast Subscription</b><br/>- QoS: <br/> inherited from Agnocast publisher"]:::listNode

        R2Pub["<b>ROS 2 Publisher</b><br/>- Depth: 10 (DEFAULT_QOS_DEPTH)<br/>- Reliability: Reliable<br/>- Durability: TransientLocal"]:::listNode

        AgSub -->|callback| R2Pub
    end

    ExtSub[External ROS 2 Subscriber]

    AgPubExt --> AgSub
    R2Pub --> ExtSub
```
