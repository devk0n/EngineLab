```mermaid
flowchart TB
    usm([EngineLab])

    usm --> asm([Application])
    asm --> s_a["WindowManager"]
    asm --> s_b["InputManager"]
```