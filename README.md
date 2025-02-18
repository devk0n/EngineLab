```mermaid
flowchart BT
  sm([Scene Manager])
  sl([Scene Loader])
  lsi([Load Scene Info])

  lsi -->|Load| sl
  lsi -->|Unload| sl
  lsi -->|Transition| sl
  sl -->|Load| sm
  sl -->|Unload| sm
```