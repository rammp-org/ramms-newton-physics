# RammsNewtonPhysics Third-Party Layout

The plugin is structured to support **either** vendoring an upstream Newton
checkout as a nested submodule **or** shipping prebuilt Newton binaries for the
target platforms that RAMMS cares about.

## Preferred upstream source checkout

Use a nested submodule inside the plugin:

```text
Plugins/RammsNewtonPhysics/ThirdParty/newton/
```

The nested submodule currently added in this repository is:

```text
https://github.com/newton-physics/newton
```

The current `RammsNewtonPhysicsThirdParty.Build.cs` detects that checkout and
exposes compile-time macros so the runtime module can report availability.

> **Important:** `newton-physics/newton` is a **Python/Warp-based Newton stack**,
> not a traditional C/C++ SDK with a simple `include/` + `lib/` layout. That
> means the nested submodule is now consumed through the plugin's **external
> Python worker bridge** (`Scripts/ramms_newton_worker.py`) instead of directly
> as a native UE library. Prebuilt native Newton binaries can still be supplied
> through the `Prebuilt/` layout below.

## Python bridge runtime layout

The current selected-upstream path expects:

```text
Plugins/RammsNewtonPhysics/
  Scripts/ramms_newton_worker.py
  ThirdParty/newton/
```

At runtime, `FRammsNewtonNativeBackend` launches that worker as an external
process and communicates with it over stdin/stdout JSON messages.

The worker resolves the vendored checkout from `ThirdParty/newton/`, but you
still need to provide a Python environment with the required dependencies
installed. The expected starting point is the upstream checkout itself, for
example:

```text
cd Plugins/RammsNewtonPhysics/ThirdParty/newton
python -m pip install -e .[sim]
```

Then point the plugin settings at the desired interpreter with
`PythonExecutablePath` if the default `python` command is not sufficient.

For packaged builds, deploy the worker script and a compatible Python runtime
alongside the application, or point `PythonExecutablePath` and
`PythonWorkerScriptPath` at locations managed by your deployment pipeline.

Legacy path names like `ThirdParty/newton-dynamics/` are still recognized by
the detection code for compatibility with future experiments.

## Supported prebuilt SDK layout

```text
Plugins/RammsNewtonPhysics/ThirdParty/Prebuilt/
  Win64/
    include/
    lib/newton.lib
    bin/newton.dll
  Linux/
    include/
    lib/libnewton.so
  Mac/
    include/
    lib/libnewton.dylib
```

If the platform-specific prebuilt library is present, the plugin links it and
reports the Newton backend as runtime-ready.
