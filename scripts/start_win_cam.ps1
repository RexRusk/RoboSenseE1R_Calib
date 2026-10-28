param(
    [int]$CaptureWidth = 1280,
    [int]$CaptureHeight = 720,
    [int]$Width = 1024,
    [int]$Height = 576,
    [string]$Out = 'C:\Users\nonew\cam_frames\live.jpg',
    [string]$Python = 'C:\Users\nonew\camcalib-venv\Scripts\python.exe',
    [int]$Device = 2
)

$script = Join-Path $PSScriptRoot 'win_cam_relay.py'
& $Python $script `
    --device $Device `
    --capture-width $CaptureWidth `
    --capture-height $CaptureHeight `
    --width $Width `
    --height $Height `
    --out $Out
exit $LASTEXITCODE
