# CasparCG MAV Edition #
CasparCG MAV Edition is a replay system based on the CasparCG open video server, using libjpeg-turbo for encoding and decoding video frames. It's essentially a module consisting of a producer and a consumer that work together to offer variable-speed replay.

The system allows multiple input and output channels: f.g. 1-in & 1-out, 3-in & 1-out, 1-in & 2-out, etc.

You can find the source code on GitHub: [Casparcg-Mav on GitHub](https://github.com/jstarpl/casparcg-mav)

## Configuration ##

Please follow first the ordinary CasparCG configuration guide at [CasparCG Wiki](http://www.casparcg.com/wiki).

The minimal configuration for the MAV edition requires at least two channels: one for ingest and another one for output. This usually means two Decklink cards (or a Decklink DUO/QUAD): one for input, and another one for output. Beyond that you can have multiple input and output channels, in various configurations. Always have only one replay buffer per ingest channel.

In the configuration file:
  * Configure all ingest channels as empty channels with proper system modes (f.g. PAL or 1080i50000). Leave the consumers tag empty.
  * Configure all output channels as porper channels, like you normally would, f.g. with Decklink consumers.
  * Start the server
  * Play the input from your ingest decklink card on your ingest channel:
```
play 1 decklink 1
```
  * Add the replay consumer to your ingest channel:
```
add 1 replay test-replay
```

The "test-replay" is a file-name for your replay buffer. Two files will be created in your CasparCG "media" folder: a TEST-REPLAY.MAV file and a TEST-REPLAY.IDX file. The first one is the video essence, and the second one is a manifest and index file. Should you copy your replay buffers, always copy both files.

  * Play your replay buffer on your output channel:
```
play 2 test-replay
```
  * Voila! You have your own replay system.

## Usage ##

You can seek within your replay buffer using the CALL ACMP 2.0 command:
  * This will seek to the 100th frame in the replay buffer:
```
call 2 seek 100
```

  * This will seek to the 100th frame counting from the end of the replay buffer:
```
call 2 seek |100
```

  * This will move the playhead of your replay 50 frames forward:
```
call 2 seek +50
```

  * This will move the playhead of your replay 50 frames backward:
```
call 2 seek -50
```

You can also change the speed of your playback:
  * Will make the playback go at normal speed
```
call 2 speed 1
```

  * Will make the playback go backwards at twice the normal speed
```
call 2 speed -2
```

  * Will make the playback go at half the speed
```
call 2 speed 0.5
```

  * Will make the playback go at 3/4 of normal speed
```
call 2 speed 0.75
```

You can also use CasparCG's auto-play feature along with transitions by using the `LENGTH` param. You can combine `SEEK`, `LENGTH` and `SPEED` in the `PLAY/LOAD/LOADBG` command.
  * This will make the playback start at 100th frame from the begining, play at 0.75x speed, and end playback after 100 frames (buffer time). Any transition (if specified using a f.g. `LOADBG (...) AUTO MIX 25`) to another clip will happen in the last 25 frames (output time)
```
play 2 test-replay seek 100 speed 0.5 length 100
```