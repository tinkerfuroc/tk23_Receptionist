| What to do                                                   | State Machine     |
| ------------------------------------------------------------ | ----------------- |
| Speak"Hello, I'm Tinker, xxxxx"                              | Init State -> TTS |
| Register Host                                                | Register          |
| Move to the door                                             | Move              |
| Register Guest 1                                             | Register          |
| Get Guest 1 Preferences                                      | ASR               |
| Go to first observation poses to find host                   | Move              |
| Go to second observation poses if nothing found / Go to Host | Move              |
| Introduce Guest1 to the host                                 | TTS               |
| Introduce Host to the Guest1                                 | TTS               |
|                                                              |                   |
|                                                              |                   |
|                                                              |                   |
|                                                              |                   |
|                                                              |                   |

