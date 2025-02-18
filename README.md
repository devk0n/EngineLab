```mermaid
flowchart TB
    usm([Unity Scene Manager])

    usm --> asm([Advanced Scene Manager])
    asm --> s_a["Scene [0]"]
    asm --> s_b["Scene [1]"]
    asm --> s_n["Scene [n]"]
```