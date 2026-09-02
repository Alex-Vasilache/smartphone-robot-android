# Reinforcement Learning Applications

`basicAssembler` is the current RL-oriented demo app.

It keeps the policy on the phone. To train instead against an agent running
on a computer, with the robot as the environment, see
[DreamerV3 Bridge](dreamer-bridge.md).

## Where To Customize basicAssembler

The policy loop is in:

`apps/basicAssembler/src/main/java/jp/oist/abcvlib/basicassembler/MyTrial.kt`

Customize `MyTrial.forward(data)`.

`data` is the assembled `TimeStepData` for the current timestep. Read state from
`data`, choose an action, record the selected action, then send outputs.

```kotlin
override fun forward(data: TimeStepData) {
    // Read state from data.*
    // Select a MotionAction / CommAction
    // Record the selected action with data.actions.add(...)
    // Send wheel output with outputs.setWheelOutput(...)
}
```

The app setup is in:

`apps/basicAssembler/src/main/java/jp/oist/abcvlib/basicassembler/MainActivity.kt`

Use `MainActivity` for wiring publishers, timestep length/counts, action-space
definitions, and GUI setup. Keep policy logic in `MyTrial.forward(data)`.

## Assembler Or Subscriber

![Subscriber vs Assembler workflow](../media/SubscriberVsAssembler.png)

Use assembled `TimeStepData` when you want a simpler loop:

```text
state for this timestep -> policy -> action
```

Use direct subscriber callbacks when you need finer timing control, every raw
callback event, or custom handling for sensors that update at different rates.

## State Space

```text
State data
├── wheelData
│   ├── left
│   │   ├── timestamps[]
│   │   ├── counts[]
│   │   ├── distances[]
│   │   ├── speedsInstantaneous[]
│   │   ├── speedsBuffered[]
│   │   └── speedsExpAvg[]
│   └── right
│       ├── timestamps[]
│       ├── counts[]
│       ├── distances[]
│       ├── speedsInstantaneous[]
│       ├── speedsBuffered[]
│       └── speedsExpAvg[]
├── batteryData
│   ├── timestamps[]
│   └── voltage[]
├── chargerData
│   ├── timestamps[]
│   ├── chargerVoltage[]
│   └── coilVoltage[]
├── orientationData
│   ├── timestamps[]
│   ├── tiltAngle[]
│   └── angularVelocity[]
├── soundData
│   ├── startTime
│   ├── endTime
│   ├── totalTime
│   ├── sampleRate
│   ├── totalSamples
│   ├── totalSamplesCalculatedViaTime
│   └── levels[]
├── imageData
│   └── images[]
│       ├── timestamp
│       ├── width
│       ├── height
│       ├── bitmap
│       └── webpImage
├── qrCodeData
│   └── qrDataDecoded
└── objectDetectorData
    ├── labels/categories
    ├── confidence scores
    ├── bounding boxes
    ├── timestamps
    └── image dimensions
```

## Assembled TimeStepData

`TimeStepDataBuffer` writes callback data into the current timestep window.
When the trial advances, `nextTimeStep()` moves the read/write pointers and
clears the next write slot.

Represented sensor callbacks are generally accumulated during the timestep
window, not reduced to only the latest value.

Assembled `TimeStepData` follows the state tree above, plus selected actions,
except for the gaps listed below.

Known gaps:

- QR code data is not yet stored in `TimeStepData`: [#258](https://github.com/tekkura/sr-android/issues/258).
- Object detection data is not yet stored in `TimeStepData`: [#259](https://github.com/tekkura/sr-android/issues/259).
- Per-callback sound metadata is not yet preserved in `TimeStepData`: [#260](https://github.com/tekkura/sr-android/issues/260).

## Action Space

### Low-Level Outputs

```text
Low-level outputs
├── Wheel output
│   └── outputs.setWheelOutput(left, right, leftBrake, rightBrake)
│       ├── left: -1.0 to 1.0
│       ├── right: -1.0 to 1.0
│       ├── leftBrake: Boolean
│       └── rightBrake: Boolean
└── QR display
    ├── QRCode.generate(data2Encode, foregroundColor)
    └── QRCode.close()
```

### basicAssembler Action Wrapper

```text
ActionSpace
├── MotionActionSpace
│   └── MotionAction
│       ├── actionName
│       ├── actionByte
│       ├── leftWheelPWM
│       ├── rightWheelPWM
│       ├── leftWheelBrake
│       └── rightWheelBrake
└── CommActionSpace
    └── CommAction
        ├── actionName
        └── actionByte
```

Example `basicAssembler` motion actions:

```text
stop
forward
backward
left
right
```

### Higher-Level Behaviors

`comprehensiveDemo` contains app-level behaviors such as charging and mating
logic. These are not yet refactored into the shared library action API. See
[#228](https://github.com/tekkura/sr-android/issues/228).
