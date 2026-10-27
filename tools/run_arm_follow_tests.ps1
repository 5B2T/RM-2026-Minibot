$ErrorActionPreference='Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force build | Out-Null
    # Native GCC on PATH, not arm-none-eabi-gcc.
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I tools/host_stubs -I Modules -I UserApp -I Bsp `
        tools/test_arm_follow.c UserApp/robot/robot.c UserApp/components/command/remote_control.c `
        UserApp/components/arm/arm.c Modules/actuator/servo.c Modules/actuator/base_motor.c Modules/command/command_protocol.c `
        Modules/command/telemetry_protocol.c -lm -o build/test_arm_follow.exe
    if($LASTEXITCODE -ne 0){throw 'Host build failed'}
    & ./build/test_arm_follow.exe
    if($LASTEXITCODE -ne 0){throw 'Arm follow tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I ArmController/UserApp `
        -I RemoteController/Modules -I RemoteController/UserApp tools/test_arm_targets.c `
        ArmController/UserApp/arm_targets.c Modules/command/command_protocol.c -o build/test_arm_targets.exe
    if($LASTEXITCODE -ne 0){throw 'Sender host build failed'}
    & ./build/test_arm_targets.exe
    if($LASTEXITCODE -ne 0){throw 'Sender tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I UserApp tools/test_base_motor.c `
        Modules/actuator/base_motor.c -lm -o build/test_base_motor.exe
    if($LASTEXITCODE -ne 0){throw 'Base PI host build failed'}
    & ./build/test_base_motor.exe
    if($LASTEXITCODE -ne 0){throw 'Base PI tests failed'}
} finally {Pop-Location}
