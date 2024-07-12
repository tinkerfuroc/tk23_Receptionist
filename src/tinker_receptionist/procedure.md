| What to do                                                   | State Machine     |
| ------------------------------------------------------------ | ----------------- |
| Speak"Hello, I'm Tinker, xxxxx"                              | Init State -> TTS |
| Register Host                                                | Register          |
| Move to the door                                             | Move              |
| Register Guest 1                                             | Register          |
| Get Guest 1 Preferences                                      | ASR               |
| Go to first observation poses to find host                   | Move              |
| Detect the host                                              | DetectionState    |
| Go to second observation poses if nothing found / Go to Host | Move              |
| Introduce Guest1 to the host                                 | TTS               |
| Introduce Host to the Guest1                                 | TTS               |
| Move to the door                                             | Move              |
| Register Guest 2                                             | Register          |
| Get Guest 2 Preferences                                      | ASR               |
| Go to first observation poses to find host                   | Move              |
| Detect the host                                              | DetectionState    |
| Go to second observation poses if nothing found / Go to Host | Move              |
| Introduce Guest2 to other people                             | TTS               |
| Introduce Guest1 to Guest2                                   | TTS               |
| Introduce Host to other people                               | TTS               |
| Done                                                         |                   |
|                                                              |                   |

