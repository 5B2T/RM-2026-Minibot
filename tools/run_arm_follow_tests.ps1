$ErrorActionPreference='Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    New-Item -ItemType Directory -Force build | Out-Null
    # Native GCC on PATH, not arm-none-eabi-gcc.
    & gcc -std=c11 -Wall -Wextra -Werror `
        -I tools/host_stubs -I Modules -I UserApp -I Bsp `
        tools/test_arm_follow.c UserApp/robot/robot.c UserApp/components/command/remote_control.c `
        UserApp/components/arm/arm.c UserApp/components/host/arm_debug.c Modules/actuator/servo.c Modules/actuator/base_motor.c Modules/actuator/base_position.c Modules/command/command_protocol.c `
        Modules/command/telemetry_protocol.c Modules/command/arm_debug_protocol.c -lm -o build/test_arm_follow.exe
    if($LASTEXITCODE -ne 0){throw 'Host build failed'}
    & ./build/test_arm_follow.exe
    if($LASTEXITCODE -ne 0){throw 'Arm follow tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules tools/test_pot_protocol.c `
        Modules/command/command_protocol.c Modules/command/telemetry_protocol.c -o build/test_pot_protocol.exe
    if($LASTEXITCODE -ne 0){throw 'Protocol host build failed'}
    & ./build/test_pot_protocol.exe
    if($LASTEXITCODE -ne 0){throw 'Protocol tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I ArmController/UserApp `
        -I RemoteController/Modules -I RemoteController/UserApp tools/test_arm_targets.c `
        ArmController/UserApp/arm_targets.c Modules/command/command_protocol.c Modules/command/telemetry_protocol.c -o build/test_arm_targets.exe
    if($LASTEXITCODE -ne 0){throw 'Sender host build failed'}
    & ./build/test_arm_targets.exe
    if($LASTEXITCODE -ne 0){throw 'Sender tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I ArmController/UserApp `
        -I RemoteController/Modules -I RemoteController/UserApp tools/test_pot_entry.c `
        RemoteController/Modules/control/remote_control.c ArmController/UserApp/arm_targets.c -lm -o build/test_pot_entry.exe
    if($LASTEXITCODE -ne 0){throw 'Pot entry host build failed'}
    & ./build/test_pot_entry.exe
    if($LASTEXITCODE -ne 0){throw 'Pot entry integration tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I Bsp -I ArmController/UserApp `
        -I RemoteController/Modules -I RemoteController/UserApp tools/test_radio_recovery.c `
        RemoteController/UserApp/components/radio/remote_radio.c Modules/command/command_protocol.c `
        Modules/command/telemetry_protocol.c -o build/test_radio_recovery.exe
    if($LASTEXITCODE -ne 0){throw 'Radio recovery host build failed'}
    & ./build/test_radio_recovery.exe
    if($LASTEXITCODE -ne 0){throw 'Radio recovery tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I tools/arm_bsp_stubs -I Bsp `
        tools/test_arm_bsp_recovery.c Bsp/arm/bsp_arm.c -o build/test_arm_bsp_recovery.exe
    if($LASTEXITCODE -ne 0){throw 'Arm BSP recovery host build failed'}
    & ./build/test_arm_bsp_recovery.exe
    if($LASTEXITCODE -ne 0){throw 'Arm BSP recovery tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I UserApp tools/test_base_motor.c `
        Modules/actuator/base_motor.c -lm -o build/test_base_motor.exe
    if($LASTEXITCODE -ne 0){throw 'Base PI host build failed'}
    & ./build/test_base_motor.exe
    if($LASTEXITCODE -ne 0){throw 'Base PI tests failed'}
    & gcc -std=c11 -Wall -Wextra -Werror -I Modules -I UserApp tools/test_base_position.c `
        Modules/actuator/base_position.c Modules/actuator/base_motor.c -lm -o build/test_base_position.exe
    if($LASTEXITCODE -ne 0){throw 'Base position host build failed'}
    & ./build/test_base_position.exe
    if($LASTEXITCODE -ne 0){throw 'Base position tests failed'}
} finally {Pop-Location}
